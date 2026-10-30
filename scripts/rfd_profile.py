"""Validate a shared raw-ASYNC profile and public readbacks; never opens a port."""
import argparse
import json
from pathlib import Path

MODEM = {'serial_baud', 'air_bps', 'max_data_bytes', 'max_retries', 'broadcast_retries',
         'serial_break_ms', 'tx_encapsulation', 'rx_encapsulation'}
TRANSPORT = {'usable_air_bps', 'frame_overhead_bytes', 'packet_burst_ms', 'packet_gap_ms',
             'idle_margin_ms', 'rf_share_percent', 'host_default_share_percent'}
DEFAULTS = {'RADIO_AIR_BIT_RATE_BPS': 64000, 'RADIO_AIR_SHARE_PERCENT': 40,
            'RADIO_AIR_FRAME_OVERHEAD_BYTES': 16, 'RADIO_PACKET_BURST_MS': 100,
            'RADIO_PACKET_GAP_MS': 55}


def load(path):
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict) or set(value) != {'version', 'name', 'modem', 'transport'}:
        raise ValueError('Profile must contain exactly version, name, modem and transport')
    if type(value['version']) is not int or value['version'] != 1 or not isinstance(value['name'], str) or not 1 <= len(value['name'].encode('utf-8')) <= 80:
        raise ValueError('Unsupported or unnamed RFD profile')
    m, t = value['modem'], value['transport']
    for obj, keys in [(m, MODEM), (t, TRANSPORT)]:
        if not isinstance(obj, dict) or set(obj) != keys or any(type(v) is not int for v in obj.values()):
            raise ValueError('Unknown, missing or non-integer RFD setting')
    if m['serial_baud'] != 115200 or m['air_bps'] != 64000:
        raise ValueError('This profile supports raw ASYNC at 115200 UART / 64000 air bps')
    if not 1028 <= m['max_data_bytes'] <= 4068 or not 1 <= m['max_retries'] <= 100 or not 0 <= m['broadcast_retries'] <= 100:
        raise ValueError('Invalid modem capacity or retries')
    if not 1 <= m['serial_break_ms'] <= 500 or m['tx_encapsulation'] or m['rx_encapsulation']:
        raise ValueError('Finite break detection and raw encapsulation are required')
    if not (1200 <= t['usable_air_bps'] <= m['air_bps'] and 0 <= t['frame_overhead_bytes'] <= 256
            and 1 <= t['packet_burst_ms'] <= 500 and 1 <= t['packet_gap_ms'] <= 1000
            and 1 <= t['rf_share_percent'] <= 100 and 1 <= t['host_default_share_percent'] <= 100):
        raise ValueError('Invalid transport capacity')
    if not 5 <= t['idle_margin_ms'] <= 100 or t['packet_gap_ms'] < m['serial_break_ms'] + t['idle_margin_ms']:
        raise ValueError('UART idle gap must exceed serial break with a guard margin')
    if (m['serial_baud'] * t['packet_burst_ms'] + 9999) // 10000 + 1028 > m['max_data_bytes']:
        raise ValueError('Burst could cross MAX_DATA inside an owned UART frame')
    return value


def cmake_values(path=None):
    if path is None:
        return dict(DEFAULTS)
    t = load(path)['transport']
    return dict(zip(DEFAULTS, [t['usable_air_bps'], t['rf_share_percent'],
                              t['frame_overhead_bytes'], t['packet_burst_ms'], t['packet_gap_ms']]))


def audit(profile, snapshots):
    """Only whitelisted public settings are reported; no secret/key handling."""
    if set(snapshots) != {'gs', 'rf', 'rrls'}:
        raise ValueError('Supply one public readback each for gs, rf and rrls')
    fields = {'max_data_bytes': 'max_data', 'max_retries': 'max_retries',
              'broadcast_retries': 'broadcast_retries', 'serial_break_ms': 'serial_break_ms',
              'tx_encapsulation': 'tx_encapsulation', 'rx_encapsulation': 'rx_encapsulation'}
    mismatches, nodes, networks = [], [], []
    for role, snapshot in snapshots.items():
        if not isinstance(snapshot, dict):
            raise ValueError('Readback must be a JSON object')
        identity = snapshot.get('identity', [])
        if not isinstance(identity, list) or not any(isinstance(s, str) and s.startswith('RFD ASYNC 4.03 ') for s in identity):
            mismatches.append({'node': role, 'field': 'firmware', 'reason': 'RFD ASYNC 4.03 readback required'})
        public = snapshot.get('public_settings', {})
        if not isinstance(public, dict):
            raise ValueError('Public settings must be a JSON object')
        for target, source in fields.items():
            if type(public.get(source)) is not int or public.get(source) != profile['modem'][target]:
                mismatches.append({'node': role, 'field': source, 'expected': profile['modem'][target], 'observed': public.get(source) if type(public.get(source)) is int else None})
        for field, wanted in [('serial_speed', 115), ('air_speed', 64), ('destination_id', 32768)]:
            if type(public.get(field)) is not int or public.get(field) != wanted:
                mismatches.append({'node': role, 'field': field, 'expected': wanted, 'observed': public.get(field) if type(public.get(field)) is int else None})
        node, network = public.get('node_id'), public.get('net_id')
        nodes.append(node if type(node) is int else None); networks.append(network if type(network) is int else None)
        if type(node) is not int or not 1 <= node <= 32767:
            mismatches.append({'node': role, 'field': 'node_id', 'reason': 'valid unique ID required'})
        if type(network) is not int or not 0 <= network <= 255:
            mismatches.append({'node': role, 'field': 'net_id', 'reason': 'valid shared network ID required'})
        # Software RRLS failover owns forwarding. A hidden modem table can
        # repeat traffic even when the application reports standby.
        if snapshot.get('forwarding_table') != []:
            mismatches.append({'node': role, 'field': 'forwarding_table', 'reason': 'verified empty table required for software relay'})
        if public.get('rtscts') not in (0, 1) or type(public.get('rtscts')) is not int:
            mismatches.append({'node': role, 'field': 'rtscts', 'reason': 'explicit public flow-control readback required'})
        if public.get('rtscts') == 1 and snapshot.get('host_hardware_flow_control_verified') is not True:
            mismatches.append({'node': role, 'field': 'rtscts', 'reason': 'matching host wiring/flow-control verification required'})
    if len(set(nodes)) != 3:
        mismatches.append({'field': 'node_id', 'reason': 'duplicate node IDs'})
    if len(set(networks)) != 1:
        mismatches.append({'field': 'net_id', 'reason': 'different network IDs'})
    return {'profile': profile['name'], 'matched': not mismatches, 'mismatches': mismatches,
            'scope': 'Offline public-readback comparison only; no serial opens, writes, resets or hardware validation'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--readback', action='append', default=[], metavar='ROLE=JSON')
    args = parser.parse_args()
    profile = load(args.profile)
    if not args.readback:
        print(json.dumps({'profile': profile['name'], 'cmake': cmake_values(args.profile),
                          'desired_public_modem_settings': profile['modem']}, indent=2))
        return 0
    snapshots = {}
    for value in args.readback:
        role, path = value.split('=', 1)
        if role in snapshots:
            raise ValueError('Duplicate node role')
        snapshots[role] = json.loads(Path(path).read_text())
    result = audit(profile, snapshots)
    print(json.dumps(result, indent=2))
    return 0 if result['matched'] else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError) as error:
        raise SystemExit(str(error)) from error
