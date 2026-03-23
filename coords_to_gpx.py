"""
coords_to_gpx.py
Converts a NEO-M9N GPS debug log directly into a .gpx file
for use in QGIS, GPSPrune, Google Earth, uMap, etc.

Usage:
    python3  coords_to_gpx.py <input.log> [output.gpx]
"""

import re
import sys
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, ElementTree, indent

# Regex patterns
RE_LAT = re.compile(r'Lat:\s*([\d.]+)[^\d\s]*\s*([NS])')
RE_LON = re.compile(r'Lon:\s*(-?[\d.]+)[^\d\s]*\s*([EW])')
RE_ALT = re.compile(r'Alt:\s*([\d.]+)\s*m MSL')
RE_DATE = re.compile(r'Date:\s*(\d{2})/(\d{2})/(\d{4})')  
RE_TIME = re.compile(r'Time:\s*(\d{2}):(\d{2}):(\d{2})\.(\d{3})\s*UTC')
RE_FIX = re.compile(r'Fix:\s*(VALID|NO FIX)')


def parse_log(log_path: Path) -> list[dict]:
    records = []
    current = {}

    with open(log_path, 'r', errors='replace') as f:
        for line in f:
            line = line.strip()

            if '========== GPS DATA ==========' in line:
                current = {}
                continue

            if '==============================' in line:
                if 'lat' in current and 'lon' in current:
                    records.append(current)
                current = {}
                continue

            m = RE_LAT.search(line)
            if m:
                val = float(m.group(1))
                current['lat'] = val if m.group(2) == 'N' else -val
                continue

            m = RE_LON.search(line)
            if m:
                val = float(m.group(1))
                current['lon'] = abs(val) if m.group(2) == 'E' else -abs(val)
                continue

            m = RE_ALT.search(line)
            if m:
                current['alt_m'] = float(m.group(1))
                continue

            m = RE_DATE.search(line)
            if m:
                #store as yyyy-mm-dd for ISO format
                current['date'] = f"{m.group(3)}-{m.group(2)}-{m.group(1)}"
                continue

            m = RE_TIME.search(line)
            if m:
                current['time'] = f"{m.group(1)}:{m.group(2)}:{m.group(3)}.{m.group(4)}Z"
                continue

            m = RE_FIX.search(line)
            if m:
                current['fix'] = m.group(1)
                continue

    return records


def build_gpx(records: list[dict]) -> ElementTree:
    gpx = Element('gpx', attrib={
        'version': '1.1',
        'creator': 'John Welgoss',
        'xmlns': 'http://www.topografix.com/GPX/1/1',
        'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
        'xsi:schemaLocation': (
            'http://www.topografix.com/GPX/1/1 '
            'http://www.topografix.com/GPX/1/1/gpx.xsd'
        )
    })

    #metadata
    meta = SubElement(gpx, 'metadata')
    SubElement(meta, 'name').text = 'NEO-M9N GPS Track'
    if records and 'date' in records[0]:
        SubElement(meta, 'time').text = f"{records[0]['date']}T{records[0].get('time', '00:00:00Z')}"

    #track
    trk = SubElement(gpx, 'trk')
    SubElement(trk, 'name').text = 'GPS Track'
    trkseg = SubElement(trk, 'trkseg')

    for r in records:
        #skip points without a valid fix
        if r.get('fix', 'VALID') != 'VALID':
            continue

        trkpt = SubElement(trkseg, 'trkpt', attrib={
            'lat': str(r['lat']),
            'lon': str(r['lon'])
        })

        if 'alt_m' in r:
            SubElement(trkpt, 'ele').text = str(r['alt_m'])

        if 'date' in r and 'time' in r:
            SubElement(trkpt, 'time').text = f"{r['date']}T{r['time']}"

    return ElementTree(gpx)


def write_gpx(tree: ElementTree, out_path: Path):
    indent(tree.getroot(), space='  ')  #pprint
    with open(out_path, 'wb') as f:
        f.write(b'<?xml version="1.0" encoding="UTF-8"?>\n')
        tree.write(f, encoding='utf-8', xml_declaration=False)
    print(f"Wrote {out_path}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 coords_to_gpx.py <input.log> [output.gpx]")
        sys.exit(1)

    log_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else log_path.with_suffix('.gpx')

    if not log_path.exists():
        print(f"File not found: {log_path}")
        sys.exit(1)

    records = parse_log(log_path)
    if not records:
        print("No valid GPS records found.")
        sys.exit(1)

    print(f"Parsed {len(records)} records")
    tree = build_gpx(records)
    write_gpx(tree, out_path)