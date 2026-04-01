"""
GPS Log Parser and Plotter
Usage: python parse_gps_log.py <input.log>
Outputs graphs to the same directory as the input file.
"""
 
import re
import sys
import os
from datetime import datetime
import matplotlib.pyplot as plt #type: ignore
import matplotlib.dates as mdates  #type: ignore
 
 
# ── Parsing ───────────────────────────────────────────────────────────────────
 
def parse_log(filepath):
    """Parse GPS log file and return a list of record dicts."""
    block_pattern = re.compile(
        r"={10} GPS DATA ={10}(.*?)={30}",
        re.DOTALL
    )
    records = []
 
    with open(filepath, "r") as f:
        content = f.read()
 
    for match in block_pattern.finditer(content):
        block = match.group(1)
 
        def get(pattern, cast=str, default=None):
            m = re.search(pattern, block)
            if m:
                try:
                    return cast(m.group(1))
                except (ValueError, TypeError):
                    return default
            return default
 
        lat_val   = get(r"Lat:\s*([\-\d.]+)", float)
        lat_dir   = get(r"Lat:\s*[\d.]+°\s*([NS])")
        lon_val   = get(r"Lon:\s*([\-\d.]+)", float)
        lon_dir   = get(r"Lon:\s*[\-\d.]+°\s*([EW])")
        alt       = get(r"Alt:\s*([\d.]+)\s*m", float)
        date_str  = get(r"Date:\s*(\d{2}/\d{2}/\d{4})")
        time_str  = get(r"Time:\s*([\d:.]+)\s*UTC")
        fc_time   = get(r"FC Time:\s*(\d+)\s*ms", int)
        sats      = get(r"Satellites:\s*(\d+)", int)
        fix       = get(r"Fix:\s*(\w+)")
        last_upd  = get(r"Last Update:\s*(\d+)\s*ticks", int)
 
        # Apply N/S and E/W sign conventions
        if lat_val is not None and lat_dir == "S":
            lat_val = -lat_val
        if lon_val is not None and lon_dir == "W" and lon_val > 0:
            lon_val = -lon_val
 
        # Parse UTC datetime
        dt = None
        if date_str and time_str:
            try:
                dt = datetime.strptime(f"{date_str} {time_str}", "%d/%m/%Y %H:%M:%S.%f")
            except ValueError:
                try:
                    dt = datetime.strptime(f"{date_str} {time_str}", "%d/%m/%Y %H:%M:%S")
                except ValueError:
                    pass
 
        records.append({
            "lat":                lat_val,
            "lon":                lon_val,
            "alt":                alt,
            "datetime":           dt,
            "fc_time_ms":         fc_time,
            "satellites":         sats,
            "fix":                fix,
            "last_update_ticks":  last_upd,
        })
 
    return records
 
 
# Helpers
def make_tick_times(records):
    """Return elapsed seconds from first FC tick for each record."""
    ticks = [r["fc_time_ms"] for r in records if r["fc_time_ms"] is not None]
    if not ticks:
        return []
    t0 = ticks[0]
    return [(t - t0) / 1000.0 for t in ticks]
 
 
def make_datetimes(records):
    return [r["datetime"] for r in records if r["datetime"] is not None]
 
 
def style_ax(ax, xlabel, ylabel, title):
    ax.set_xlabel(xlabel, fontsize=11)
    ax.set_ylabel(ylabel, fontsize=11)
    ax.set_title(title, fontsize=13, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.tick_params(labelsize=9)
 
 
# Plotting 
def plot_altitude(records, out_dir):
    alts = [r["alt"] for r in records]
 
    # Tick-time version
    tick_x = make_tick_times(records)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(tick_x, alts, marker="o", markersize=3, linewidth=1.5, color="#1f77b4")
    style_ax(ax, "Elapsed Time (s, from first tick)", "Altitude (m MSL)",
             "Altitude over Time  [Tick-Based]")
    fig.tight_layout()
    path = os.path.join(out_dir, "altitude_vs_tick_time.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {path}")
 
    # UTC datetime version
    dt_x = make_datetimes(records)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(dt_x, alts, marker="o", markersize=3, linewidth=1.5, color="#1f77b4")
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    fig.autofmt_xdate()
    style_ax(ax, "UTC Time", "Altitude (m MSL)", "Altitude over Time  [UTC]")
    fig.tight_layout()
    path = os.path.join(out_dir, "altitude_vs_utc_time.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {path}")
 
 
def plot_satellites(records, out_dir):
    sats = [r["satellites"] for r in records]
 
    # Tick-time version
    tick_x = make_tick_times(records)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(tick_x, sats, marker="s", markersize=4, linewidth=1.5,
            color="#2ca02c", drawstyle="steps-post")
    ax.set_yticks(range(0, max(sats) + 3))
    style_ax(ax, "Elapsed Time (s, from first tick)", "Satellites in View",
             "Satellite Count over Time  [Tick-Based]")
    fig.tight_layout()
    path = os.path.join(out_dir, "satellites_vs_tick_time.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {path}")
 
    # UTC datetime version
    dt_x = make_datetimes(records)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(dt_x, sats, marker="s", markersize=4, linewidth=1.5,
            color="#2ca02c", drawstyle="steps-post")
    ax.set_yticks(range(0, max(sats) + 3))
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    fig.autofmt_xdate()
    style_ax(ax, "UTC Time", "Satellites in View",
             "Satellite Count over Time  [UTC]")
    fig.tight_layout()
    path = os.path.join(out_dir, "satellites_vs_utc_time.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {path}")
 
 
def plot_position_trace(records, out_dir):
    lons = [r["lon"] for r in records]
    lats = [r["lat"] for r in records]
 
    def _draw_trace(fig, ax, color_values, cmap, cbar_label):
        sc = ax.scatter(lons, lats, c=color_values, cmap=cmap,
                        s=25, zorder=3)
        ax.plot(lons, lats, color="gray", linewidth=0.8, alpha=0.5, zorder=2)
        ax.scatter([lons[0]], [lats[0]], color="limegreen", s=100, zorder=4,
                   label="Start", edgecolors="black", linewidths=0.7)
        ax.scatter([lons[-1]], [lats[-1]], color="red", s=100, zorder=4,
                   label="End", edgecolors="black", linewidths=0.7)
        cbar = fig.colorbar(sc, ax=ax, pad=0.02)
        cbar.set_label(cbar_label, fontsize=9)
        ax.legend(fontsize=9)
 
    # Tick-time version
    tick_x = make_tick_times(records)
    fig, ax = plt.subplots(figsize=(7, 6))
    _draw_trace(fig, ax, tick_x, "viridis", "Elapsed Time (s)")
    style_ax(ax, "Longitude (°)", "Latitude (°)",
             "GPS Position Trace  [color = elapsed tick-time]")
    fig.tight_layout()
    path = os.path.join(out_dir, "position_trace_tick_time.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {path}")
 
    # UTC datetime version
    dt_x   = make_datetimes(records)
    t0     = dt_x[0].timestamp()
    dt_sec = [d.timestamp() - t0 for d in dt_x]
    fig, ax = plt.subplots(figsize=(7, 6))
    _draw_trace(fig, ax, dt_sec, "plasma", "Elapsed UTC Time (s)")
    style_ax(ax, "Longitude (°)", "Latitude (°)",
             "GPS Position Trace  [color = elapsed UTC time]")
    fig.tight_layout()
    path = os.path.join(out_dir, "position_trace_utc_time.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {path}")
 
 
def main():
    if len(sys.argv) != 2:
        print("Usage: python parse_gps_log.py <input.log>")
        sys.exit(1)
 
    log_path = sys.argv[1]
    if not os.path.isfile(log_path):
        print(f"Error: File not found: {log_path}")
        sys.exit(1)
 
    log_stem = os.path.splitext(os.path.basename(log_path))[0]
    out_dir  = os.path.join(os.path.dirname(os.path.abspath(log_path)), f"{log_stem}_plots")
    os.makedirs(out_dir, exist_ok=True)
    print(f"Parsing: {log_path}")
 
    records = parse_log(log_path)
    if not records:
        print("No GPS records found in log file.")
        sys.exit(1)
 
    print(f"Found {len(records)} GPS records.\nGenerating plots in: {out_dir}\n")
 
    plot_altitude(records, out_dir)
    plot_satellites(records, out_dir)
    plot_position_trace(records, out_dir)
 
    print("\nDone.")
 
 
if __name__ == "__main__":
    main()
 