"""
coords_to_csv.py
Converts a NEO-M9N GPS debug log into a CSV of lat/lon pairs.
Optionally includes altitude, date, time, and fix status.

Usage:
    python3 coords_to_csv.py <input.log> [output.csv]
"""

import re
import csv
import sys
from pathlib import Path

# Regex patterns matching the GPS_TEST_MODE print format
RE_LAT  = re.compile(r'Lat:\s*([\d.]+)[^\d\s]*\s*([NS])')
RE_LON  = re.compile(r'Lon:\s*(-?[\d.]+)[^\d\s]*\s*([EW])')
RE_ALT  = re.compile(r'Alt:\s*([\d.]+)\s*m MSL')
RE_DATE = re.compile(r'Date:\s*(\d{2}/\d{2}/\d{4})')
RE_TIME = re.compile(r'Time:\s*(\d{2}:\d{2}:\d{2}\.\d{3} UTC)')
RE_FIX  = re.compile(r'Fix:\s*(VALID|NO FIX)')


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
                # End of a block — only keep records with at least lat/lon
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
                # Value is already signed in the log; E/W is redundant but kept for safety
                current['lon'] = abs(val) if m.group(2) == 'E' else -abs(val)
                continue

            m = RE_ALT.search(line)
            if m:
                current['alt_m'] = float(m.group(1))
                continue

            m = RE_DATE.search(line)
            if m:
                current['date'] = m.group(1)
                continue

            m = RE_TIME.search(line)
            if m:
                current['time'] = m.group(1)
                continue

            m = RE_FIX.search(line)
            if m:
                current['fix'] = m.group(1)
                continue

    return records


def write_csv(records: list[dict], out_path: Path):
    if not records:
        print("No valid GPS records found.")
        return

    fieldnames = ['lat', 'lon', 'alt_m', 'date', 'time', 'fix']

    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        for r in records:
            writer.writerow({k: r.get(k, '') for k in fieldnames})

    print(f"Wrote {len(records)} records to {out_path}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python gps_log_to_csv.py <input.log> [output.csv]")
        sys.exit(1)

    log_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else log_path.with_suffix('.csv')

    if not log_path.exists():
        print(f"File not found: {log_path}")
        sys.exit(1)

    records = parse_log(log_path)
    write_csv(records, out_path)












