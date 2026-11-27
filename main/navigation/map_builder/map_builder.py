"""Pack roads and street names from a Geofabrik OSM extract into one NVA file."""

import argparse
import math
import sqlite3
import struct
import tempfile
import unicodedata
import zlib
from contextlib import closing
from pathlib import Path

import osmium
from shapely.geometry import LineString

CELL = 16384
QUANTUM = 4
MAX_MAP_BYTES = 512_000_000
MAX_CELL_BYTES = 8 * 1024 * 1024
HEADER = struct.Struct("<4sII")
ENTRY = struct.Struct("<iiIIII")
MAJOR = {"motorway", "trunk", "primary", "secondary", "tertiary"}
CLOSED = {"abandoned", "construction", "disused", "planned", "proposed", "razed"}

def project(lon, lat):
    lat = max(-85.05112878, min(85.05112878, lat))
    return (
        6378137 * math.radians(lon),
        6378137 * math.asinh(math.tan(math.radians(lat))),
    )


def varint(data, value):
    value = (value << 1) ^ (value >> 31)
    while value >= 128:
        data.append((value & 127) | 128)
        value >>= 7
    data.append(value)


def encode_road(points, major, x, y):
    data = bytearray([major])
    varint(data, len(points))
    last_x, last_y = x * (CELL // QUANTUM), y * (CELL // QUANTUM)
    for px, py in points:
        varint(data, px - last_x)
        varint(data, py - last_y)
        last_x, last_y = px, py
    return data


def street_name(name):
    text = unicodedata.normalize("NFKD", name.replace("Ł", "L").replace("ł", "l"))
    return "".join(ch for ch in text if " " <= ch <= "~").strip().encode("ascii")[:28]


class Roads(osmium.SimpleHandler):
    def __init__(self, database, tolerance):
        super().__init__()
        self.database = database
        self.tolerance = tolerance
        self.roads = []
        self.labels = []

    def way(self, way):
        highway = way.tags["highway"].removesuffix("_link")
        if highway in CLOSED or way.tags.get("area") == "yes":
            return
        nodes = [project(n.lon, n.lat) for n in way.nodes if n.location.valid()]
        if len(nodes) < 2 or len(nodes) != len(way.nodes):
            return
        major = highway in MAJOR
        road = LineString(nodes)
        points = []
        for x, y in road.simplify(self.tolerance, preserve_topology=False).coords:
            point = round(x / QUANTUM), round(y / QUANTUM)
            if not points or points[-1] != point:
                points.append(point)
        for start in range(0, len(points) - 1, 127):
            self.add_road(points[start : start + 128], major)
        self.add_label(road, street_name(way.tags.get("name", "")), major)
        if len(self.roads) >= 8192:
            self.flush()

    def add_road(self, points, major):
        xs, ys = zip(*points)
        step = CELL // QUANTUM
        for x in range(min(xs) // step, max(xs) // step + 1):
            for y in range(min(ys) // step, max(ys) // step + 1):
                self.roads.append((x, y, encode_road(points, major, x, y)))

    def add_label(self, road, name, major):
        if not name or road.length < 80:
            return
        point = road.interpolate(0.5, normalized=True)
        x, y = int(point.x // CELL), int(point.y // CELL)
        self.labels.append((
            x, y, name, int(point.x - x * CELL), int(point.y - y * CELL),
            14 if major else 15, road.length,
        ))

    def flush(self):
        self.database.executemany("INSERT INTO roads VALUES (?, ?, ?)", self.roads)
        self.database.executemany(
            """
            INSERT INTO labels VALUES (?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT (x, y, name) DO UPDATE SET
                dx=excluded.dx, dy=excluded.dy, zoom=excluded.zoom, length=excluded.length
            WHERE excluded.length > labels.length
            """,
            self.labels,
        )
        self.roads.clear()
        self.labels.clear()


def read_roads(database, source, tolerance):
    database.executescript("""
        CREATE TABLE roads (x INTEGER, y INTEGER, data BLOB);
        CREATE TABLE labels (
            x INTEGER, y INTEGER, name BLOB, dx INTEGER, dy INTEGER,
            zoom INTEGER, length REAL, PRIMARY KEY (x, y, name)
        ) WITHOUT ROWID;
    """)
    reader = Roads(database, tolerance)
    with database:
        reader.apply_file(str(source), locations=True, filters=[osmium.filter.KeyFilter("highway")])
        reader.flush()
        database.execute("CREATE INDEX road_cells ON roads(x, y)")


def pack_cell(database, x, y):
    roads = database.execute("SELECT data FROM roads WHERE x=? AND y=?", (x, y)).fetchall()
    geometry = b"".join(data for (data,) in roads)
    labels = database.execute(
        "SELECT dx, dy, zoom, name FROM labels WHERE x=? AND y=? "
        "ORDER BY zoom, length DESC, name LIMIT 256",
        (x, y),
    ).fetchall()
    data = bytearray(struct.pack("<III", len(roads), len(geometry), len(labels)))
    data.extend(geometry)
    for dx, dy, zoom, name in labels:
        data.extend(struct.pack("<HHBB", dx, dy, zoom, len(name)))
        data.extend(name)
    if len(data) > MAX_CELL_BYTES:
        raise ValueError(f"Cell {x}, {y} exceeds 8 MiB")
    encoder = zlib.compressobj(6, wbits=-15)
    packed = encoder.compress(data) + encoder.flush()
    if len(packed) > MAX_CELL_BYTES:
        raise ValueError(f"Compressed cell {x}, {y} exceeds 8 MiB")
    return packed, len(data), zlib.crc32(data)


def write_map(database, output):
    cells = database.execute(
        "SELECT DISTINCT x, y FROM roads UNION SELECT x, y FROM labels ORDER BY x, y"
    ).fetchall()
    if not 0 < len(cells) <= 200_000:
        raise ValueError("Map must contain 1 to 200,000 cells")

    directory = bytearray()
    with output.open("wb") as stream:
        stream.seek(HEADER.size + len(cells) * ENTRY.size)
        for x, y in cells:
            packed, raw_size, checksum = pack_cell(database, x, y)
            offset = stream.tell()
            if offset + len(packed) > MAX_MAP_BYTES:
                raise ValueError("Map exceeds 512 MB. Increase --simplify or use a smaller extract.")
            stream.write(packed)
            directory.extend(ENTRY.pack(x, y, offset, len(packed), raw_size, checksum))
        stream.seek(0)
        stream.write(HEADER.pack(b"NVA\0", len(cells), zlib.crc32(directory)))
        stream.write(directory)
    return len(cells)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="OpenStreetMap .osm.pbf extract")
    parser.add_argument("output", type=Path, help="Output .NVA file")
    parser.add_argument("--simplify", type=float, default=12, help="Tolerance in projected metres")
    args = parser.parse_args()
    if not math.isfinite(args.simplify) or args.simplify < 0:
        parser.error("--simplify must be a finite non-negative number")
    if args.source.resolve() == args.output.resolve():
        parser.error("The input and output must be different files")
    if args.output.suffix.lower() != ".nva":
        parser.error("The output filename must end in .NVA")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    try:
        with tempfile.TemporaryDirectory(dir=args.output.parent) as folder:
            temporary = Path(folder)
            archive = temporary / "map.nva"
            with closing(sqlite3.connect(temporary / "roads.db")) as database:
                print(f"Reading {args.source.name}", flush=True)
                read_roads(database, args.source, args.simplify)
                print("Packing roads and labels", flush=True)
                count = write_map(database, archive)
            archive.replace(args.output)
    except (OSError, RuntimeError, ValueError, sqlite3.Error) as exc:
        parser.exit(1, f"{exc}\n")
    print(f"{args.output}: {count:,} cells, {args.output.stat().st_size:,} bytes")


if __name__ == "__main__":
    main()
