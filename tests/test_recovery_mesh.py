import pathlib, subprocess, tempfile, unittest, zlib

SELF_CHECKED = {1: 0xF3, 2: 0xF5, 3: 0xF6, 4: 0xF9, 5: 0xFA, 6: 0xFC}


def crc(data):
    return zlib.crc32(data).to_bytes(4, 'little')


def crc_form(kind, payload):
    body = bytes([0xE0 + kind]) + payload
    return body + crc(body)


class RecoveryMesh(unittest.TestCase):
    def test_ground_envelopes_preserve_uplink_and_drop_foreign_or_corrupt_frames(self):
        header = pathlib.Path(__file__).resolve().parents[1] / 'Core/Inc/recovery_mesh.h'
        payload = b'opaque SDT fragment\x00\xa5\x5a'
        sdk = b'\x42SDK frame with its own CRC'
        sdk += crc(sdk)
        # (frame, command channel, accepted, command out, data out)
        vectors = []
        for kind in range(17):
            vectors.append((crc_form(kind, payload), 1, kind in (5, 6), kind == 6, payload))
        for kind, marker in SELF_CHECKED.items():
            vectors.append((bytes([marker]) + sdk, 1, kind in (5, 6), kind == 6, sdk))
        for marker in range(0xF1, 0x100):
            if marker not in SELF_CHECKED.values():
                vectors.append((bytes([marker]) + sdk, 1, False, False, b''))
        for marker in (0xFA, 0xFC):
            for bit in range(8):
                # Below 0xE0 it is no envelope: passed on unchanged, and the
                # UART CRC check (which covers that byte) has already failed.
                frame = bytes([marker ^ (1 << bit)]) + sdk
                vectors.append((frame, 1, frame[0] < 0xE0, True, frame))
        valid = vectors[5][0]
        self_valid = bytes([0xFA]) + sdk
        for good in (valid, self_valid):
            for i in range(1, len(good)):
                vectors.append((good[:i] + bytes([good[i] ^ 0x10]) + good[i + 1:], 1, False, False, b''))
            for prefix in range(len(good)):
                vectors.append((good[:prefix], 1, prefix < 1, True, good[:prefix]))
        # Not an envelope (an SDK frame, an RS window) or the data channel: unchanged.
        vectors += [(sdk, 1, True, True, sdk), (b'RS\x01\x00\x00\x00\x00', 1, True, True, b'RS\x01\x00\x00\x00\x00'),
                    (valid, 0, True, False, valid)]
        lines = ['#include "recovery_mesh.h"', '#include <assert.h>', '#include <string.h>', 'int main(void) {']
        for i, (frame, channel, accepted, command, expected) in enumerate(vectors):
            values = ','.join(map(str, frame)) or '0'
            lines += [f'const uint8_t a{i}[]={{ {values} }};',
                      f'const uint8_t *p{i}=a{i}; size_t n{i}={len(frame)}; uint8_t c{i}={channel};',
                      f'assert(recovery_mesh_uplink(&p{i},&n{i},&c{i})=={int(accepted)});']
            if accepted:
                data = ','.join(map(str, expected)) or '0'
                lines += [f'const uint8_t e{i}[]={{ {data} }};',
                          f'assert(n{i}=={len(expected)} && memcmp(p{i},e{i},n{i})==0);',
                          f'assert(c{i}=={int(command)});']
        for marker in range(256):
            lines.append(f'assert(recovery_mesh_self_checked({marker}U)=={int(marker in SELF_CHECKED.values())});')
        lines += ['return 0;}']
        with tempfile.TemporaryDirectory() as tmp:
            src = pathlib.Path(tmp) / 'mesh.c'
            exe = pathlib.Path(tmp) / 'mesh'
            src.write_text('\n'.join(lines))
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(header.parent), str(src), '-o', str(exe)],
                           check=True, capture_output=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
