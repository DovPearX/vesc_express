# Offline Maps

## Build a map

```sh
python -m pip install -r main/navigation/map_builder/requirements.txt
curl -L -o poland.osm.pbf https://download.geofabrik.de/europe/poland-latest.osm.pbf
python main/navigation/map_builder/map_builder.py poland.osm.pbf NAV/POL.NVA
```

Copy `NAV/POL.NVA` to the SD card under `NAV/`.
Roads and street names are in the same file.

## LispBM

```clj
(nav-open "/sdcard/NAV/POL.NVA")
(def frame (img-buffer 'rgb565 400 160))

(if (nav-render-async frame 52.2297f64 21.0122f64 15 'night) {
    (var status 0)
    (loopwhile (= (setq status (nav-render-ready frame)) 0)
        (sleep 0.02))
    (if (= status 1) (disp-render frame 70 45))
})
(nav-close)
```

- Zoom range is 14–18. Local street names appear from zoom 15.
- Palette defaults to `'day`. Use `'night` or `'(background outline major-road local-road)` in RGB888.
- `nav-render-ready` returns `1` for a copied frame, `0` while pending or idle, and `-1` on failure.

© OpenStreetMap contributors. [Open Database License](https://www.openstreetmap.org/copyright).
