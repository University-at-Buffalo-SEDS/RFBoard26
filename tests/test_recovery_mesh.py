import pathlib, subprocess, tempfile, unittest, zlib

class RecoveryMesh(unittest.TestCase):
    def test_ground_envelopes_preserve_uplink_and_drop_foreign_or_corrupt_frames(self):
        header=pathlib.Path(__file__).resolve().parents[1]/'Core/Inc/recovery_mesh.h'
        payload=b'opaque SDT fragment\x00\xa5\x5a'
        vectors=[]
        for kind in range(8):
            body=b'RR\x01'+bytes([kind])+payload
            vectors.append((body+zlib.crc32(body).to_bytes(4,'little'),kind in (5,6),kind==6,payload))
        valid=vectors[5][0]
        vectors += [(valid[:-1]+bytes([valid[-1]^1]),False,False,b''), (b'RR\x01',False,False,b'')]
        for prefix in range(len(valid)):
            vectors.append((valid[:prefix], prefix<2, True,valid[:prefix]))
        lines=['#include "recovery_mesh.h"','#include <assert.h>','#include <string.h>','int main(void) {']
        for i,(frame,accepted,command,expected) in enumerate(vectors):
            values=','.join(map(str,frame)) or '0'
            lines += [f'const uint8_t a{i}[]={{ {values} }};',f'const uint8_t *p{i}=a{i}; size_t n{i}={len(frame)}; uint8_t c{i}=1;',f'assert(recovery_mesh_uplink(&p{i},&n{i},&c{i})=={int(accepted)});']
            if accepted:
                data=','.join(map(str,expected)) or '0'
                lines += [f'const uint8_t e{i}[]={{ {data} }};',f'assert(n{i}=={len(expected)} && memcmp(p{i},e{i},n{i})==0);',f'assert(c{i}=={int(command)});']
        lines += ['return 0;}']
        with tempfile.TemporaryDirectory() as tmp:
            src=pathlib.Path(tmp)/'mesh.c'; exe=pathlib.Path(tmp)/'mesh'
            src.write_text('\n'.join(lines))
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(header.parent),str(src),'-o',str(exe)],check=True,capture_output=True)
            subprocess.run([str(exe)],check=True)

if __name__=='__main__': unittest.main()
