import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
from dataclasses import replace
import build
from scripts import rfd_profile


def profile():
    return {'version': 1, 'name': 'rfd-async-64k-low-latency',
            'modem': {'serial_baud': 115200, 'air_bps': 64000, 'max_data_bytes': 3968,
                      'max_retries': 2, 'broadcast_retries': 1, 'serial_break_ms': 10,
                      'tx_encapsulation': 0, 'rx_encapsulation': 0},
            'transport': {'usable_air_bps': 32000, 'frame_overhead_bytes': 16,
                          'packet_burst_ms': 100, 'packet_gap_ms': 15, 'idle_margin_ms': 5,
                          'rf_share_percent': 40, 'host_default_share_percent': 25}}


def readbacks():
    result = {}
    for role, node in [('gs', 4), ('rf', 1), ('rrls', 2)]:
        result[role] = {'identity': ['RFD ASYNC 4.03 on RFD900X2'], 'forwarding_table': [],
                        'public_settings': {'serial_speed': 115, 'air_speed': 64, 'max_data': 3968,
                            'max_retries': 2, 'broadcast_retries': 1, 'serial_break_ms': 10,
                            'tx_encapsulation': 0, 'rx_encapsulation': 0, 'net_id': 25,
                            'node_id': node, 'destination_id': 32768, 'rtscts': 0}}
    return result


class RfdProfileTests(unittest.TestCase):
    def test_numeric_and_framing_contracts(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'profile.json'
            path.write_text(json.dumps(profile()))
            self.assertEqual(rfd_profile.load(path), profile())
            self.assertEqual(rfd_profile.cmake_values(path)['RADIO_PACKET_GAP_MS'], 15)
            for section, key, value in [
                ('modem', 'serial_baud', 57600), ('modem', 'air_bps', 48000),
                ('modem', 'serial_break_ms', 0), ('modem', 'serial_break_ms', 50),
                ('modem', 'tx_encapsulation', 2), ('modem', 'max_retries', 0),
                ('modem', 'max_data_bytes', 1028), ('transport', 'usable_air_bps', 64001),
                ('transport', 'packet_gap_ms', 10), ('transport', 'packet_burst_ms', 500),
                ('transport', 'rf_share_percent', True), ('transport', 'extra', 0)]:
                candidate = profile(); candidate[section][key] = value
                path.write_text(json.dumps(candidate))
                with self.subTest(field=(section, key, value)), self.assertRaises(ValueError):
                    rfd_profile.load(path)

    def test_profile_flags_and_legacy_cache_reset_are_explicit(self):
        ui = mock.Mock()
        cfg = build.build_cfg_from_args(ui, build.make_parser().parse_args(['build', '--release']))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'profile.json'; path.write_text(json.dumps(profile()))
            for selected, gap, bps in [(path, 15, 32000), (None, 55, 32000)]:
                for preset in [True, False]:
                    with mock.patch.object(build, 'run') as execute, mock.patch.object(build, 'assert_embedded_cache'), \
                         mock.patch.object(build, 'pick_elf', return_value=Path('/tmp/profile.elf')):
                        build.configure_and_build(ui, replace(cfg, rfd_profile=selected, use_preset=preset))
                    args = execute.call_args_list[0].args[1]
                    self.assertIn(f'-DRADIO_PACKET_GAP_MS={gap}', args)
                    self.assertIn(f'-DRADIO_AIR_BIT_RATE_BPS={bps}', args)
                    expected_path = selected or Path(build.__file__).resolve().parent / 'config/rfd-profile.json'
                    self.assertIn(f'-DRFD_PROFILE_FILE={expected_path.resolve()}', args)
                    for key in rfd_profile.DEFAULTS:
                        self.assertEqual(sum(arg.startswith(f'-D{key}=') for arg in args), 1)
            with self.assertRaises(build.FriendlyError):
                build.configure_and_build(ui, replace(cfg, rfd_profile=path, radio_mode='lora'))

    def test_all_three_live_readbacks_and_forwarding_are_required(self):
        self.assertTrue(rfd_profile.audit(profile(), readbacks())['matched'])
        for key, value in [('max_retries', 10), ('broadcast_retries', 10), ('serial_break_ms', 50),
                           ('node_id', 4), ('net_id', 42), ('rtscts', 1), ('max_retries', True)]:
            candidate = readbacks(); candidate['rf']['public_settings'][key] = value
            self.assertFalse(rfd_profile.audit(profile(), candidate)['matched'], key)
        for role in ['gs', 'rf', 'rrls']:
            candidate = readbacks(); del candidate[role]['forwarding_table']
            self.assertFalse(rfd_profile.audit(profile(), candidate)['matched'])
            candidate[role]['forwarding_table'] = [[1, 4]]
            self.assertFalse(rfd_profile.audit(profile(), candidate)['matched'])
        candidate = readbacks(); del candidate['rf']
        with self.assertRaises(ValueError): rfd_profile.audit(profile(), candidate)

    def test_only_public_whitelisted_values_are_reported(self):
        candidate = readbacks(); candidate['rf']['encryption_key'] = 'never-echo-this'
        candidate['rf']['public_settings']['unrelated_secret'] = 'never-echo-this'
        candidate['rf']['public_settings']['max_retries'] = {'private': 'never-echo-this'}
        result = rfd_profile.audit(profile(), candidate)
        self.assertFalse(result['matched'])
        self.assertNotIn('never-echo-this', json.dumps(result))
