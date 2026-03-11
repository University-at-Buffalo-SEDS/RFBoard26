# GPS Log Plotting Guide

Convert a `gps.log` from the NEO-M9N debug output into a map visualization using QGIS.

---

## Prerequisites

- Python 3.x
- [QGIS](https://qgis.org/download/) installed

---

## Step 1 — Convert the log to GPX

Run the conversion script from the directory containing your `gps.log`:

```bash
python3 gps_log_to_gpx.py gps.log
```

This produces `gps.gpx` in the same directory.

> **NOTE:** You can name `.log` anything you want, at any path, and `.gpx` will have same name and path.

---

## Step 2 — Load the GPX into QGIS

1. Open QGIS
2. Go to **Layer → Add Layer → Add Vector Layer**
3. Set source type to **File** and browse to `gps.gpx`
4. Click **Add**
5. When prompted, select **track_points** or **tracks** (Which ever has more detail)

You should now see your route plotted, but with no map underneath.

---

## Step 3 — Add a base map

1. Go to **Plugins → Manage and Install Plugins**
2. Search for **QuickMapServices** and install it
3. Go to **Web → QuickMapServices → OSM → OSM Standard**

Your route will now appear over an OpenStreetMap base layer.

> **Tip:** If the route disappears behind the map, drag your GPX layer above the OSM layer in the **Layers** panel.

> **Tip:** For satellite imagery, go to **Web → QuickMapServices → Settings → More Services → Get Contributed Pack**, then select Google Satellite.