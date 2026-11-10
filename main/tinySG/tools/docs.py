import argparse
import base64
import json
import math
import re
import subprocess
import tempfile
from pathlib import Path

from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"


def catalog():
    examples = []
    shapes = ["Rectangle", "Circle", "Line", "Rectangle outline", "Circle outline",
              "Ellipse", "Arc", "Pie", "Triangle", "Rounded rectangle", "Rounded outline", "Text"]
    geometry = {
        1: [160, 90, 54, 0], 2: [40, 130, 280, 50], 4: [160, 90, 54, 0],
        5: [160, 90, 92, 48], 6: [160, 90, 54, 0], 7: [160, 90, 54, 0],
        8: [160, 30, 40, 145], 11: [55, 66, 0, 0], 12: [32, 58, 256, 64],

    }

    def add(kind, title, group, overrides=None, text=None, tags=""):
        scale = 6 if kind == 11 else 1
        value = 0
        p = [kind, 0, 255, 0, 0, 0, 256, 0, 0, 0, 6, 0, 0, 5, scale,
             0x07ff, 0xfd20,
             value, 0, 0, *geometry.get(kind, [30, 36, 260, 108]),
             135 if kind == 104 else -135, 270, 18, 280, 145, 0, 72, 0,
             0, -1, 0, 0, -1, 0, 0x94b5, 0xfd20, 0xffff, 5, 0, 80,
             0, 74, 0, 1, 2, 0, 0, 1, 1, 1, 0, 24]
        for index, value in (overrides or {}).items():
            p[index] = value
        if text is None:
            text = "tinySG"
        examples.append(dict(id=len(examples), title=title, group=group,
                             tags=tags, parameters=p, text=text, code=""))

    for i, title in enumerate(shapes):
        add(i, title, "primitives")
    add(12, "Imported image", "images", tags="RGB565 BIN JPEG")
    add(2, "Dashed line", "effects", {10: 4, 11: 12, 12: 7}, tags="dash gap stroke")
    add(11, "Rotated text", "effects", {4: -18, 3: 1}, tags="rotation antialias")
    add(11, "Skew and wave", "effects", {5: 64, 7: 6, 30: 72, 31: 20},
        tags="text skew wave transform")
    add(0, "Gradient angle", "effects", {1: 3, 29: 35}, tags="gradient rotation")
    add(9, "Opacity", "effects", {2: 128}, tags="transparency alpha")
    order = {"primitives": 0, "images": 1, "effects": 2}
    examples.sort(key=lambda item: order[item["group"]])
    for index, item in enumerate(examples):
        item["id"] = index
    return examples


def build_wasm(clang):
    with tempfile.TemporaryDirectory(prefix="tinysg-docs-") as directory:
        shim = Path(directory)
        (shim / "string.h").write_text(
            "#include <stddef.h>\nvoid *memcpy(void *,const void *,size_t);\n"
            "void *memset(void *,int,size_t);\nsize_t strlen(const char *);\n"
            "int strcmp(const char *,const char *);\nint memcmp(const void *,const void *,size_t);\n"
            "void *memchr(const void *,int,size_t);\n")
        exports = ["parameters", "text", "image", "font", "load_font", "fill_allowed"]
        subprocess.run([
            clang, "--target=wasm32", "-std=c99", "-O3", "-fno-builtin", "-nostdlib",
            "-I" + str(shim), "-I" + str(ROOT / "include"),
            *map(str, sorted((ROOT / "src").glob("*.c"))), str(DOCS / "preview.c"),
            "-Wl,--no-entry", "-Wl,--export-memory", "-Wl,--initial-memory=1048576",
            "-Wl,--export=tinysg_preview",
            *["-Wl,--export=tinysg_preview_" + name for name in exports],
            "-o", str(DOCS / "renderer.wasm"),
        ], check=True)


def build_gallery(examples, node):
    script = r"""
const fs = require('fs');
(async () => {
  const items = JSON.parse(fs.readFileSync(process.argv[1], 'utf8'));
  const wasm = (await WebAssembly.instantiate(fs.readFileSync(process.argv[2]))).instance.exports;
  const image = fs.readFileSync(process.argv[3]).subarray(5);
  const frames = [];
  for (const item of items) {
    new Int32Array(wasm.memory.buffer, wasm.tinysg_preview_parameters(), 56).set(item.parameters);
    const text = new Uint8Array(wasm.memory.buffer, wasm.tinysg_preview_text(), 128);
    text.fill(0); text.set(new TextEncoder().encode(item.text).subarray(0, 127));
    new Uint8Array(wasm.memory.buffer, wasm.tinysg_preview_image(), image.length).set(image);
    const pixels = wasm.tinysg_preview();
    if (!pixels) throw Error(item.title);
    frames.push(Buffer.from(new Uint8Array(wasm.memory.buffer, pixels, 320 * 180 * 4)));
  }
  fs.writeFileSync(process.argv[4], Buffer.concat(frames));
})().catch(error => { console.error(error); process.exit(1); });
"""
    with tempfile.TemporaryDirectory(prefix="tinysg-gallery-") as directory:
        data, pixels = Path(directory) / "examples.json", Path(directory) / "frames.rgba"
        data.write_text(json.dumps(examples))
        subprocess.run([node, "-e", script, str(data), str(DOCS / "renderer.wasm"),
                        str(DOCS / "assets/logo-256x64.bin"), str(pixels)], check=True)
        frames = pixels.read_bytes()
    rows = math.ceil(len(examples) / 4)
    atlas = Image.new("RGB", (1280, rows * 180), (8, 12, 18))
    for i in range(len(examples)):
        frame = Image.frombytes("RGBA", (320, 180), frames[i * 230400:(i + 1) * 230400])
        atlas.paste(frame.convert("RGB"), ((i % 4) * 320, (i // 4) * 180))
    atlas.save(DOCS / "gallery.png", optimize=True)
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--clang", default="clang")
    parser.add_argument("--node", default="node")
    parser.add_argument("--skip-wasm", action="store_true")
    args = parser.parse_args()
    if not args.skip_wasm:
        build_wasm(args.clang)
    examples = catalog()
    rows = build_gallery(examples, args.node)
    page = (DOCS / "index.html").read_text(encoding="utf-8")
    page = re.sub(r"const examples=\[[^\n]*\];", lambda _: "const examples=" + json.dumps(examples) + ";", page)
    page = re.sub(r"const rows=\d+;", f"const rows={rows};", page)
    payload = base64.b64encode((DOCS / "renderer.wasm").read_bytes()).decode()
    page = re.sub(r'(<script id="renderer-wasm"[^>]*>)[^<]*(</script>)',
                  lambda match: match[1] + payload + match[2], page)
    (DOCS / "index.html").write_text(page, encoding="utf-8")
    print(f"{len(examples)} examples, gallery 1280x{rows * 180}, WebAssembly rebuilt: {not args.skip_wasm}")


if __name__ == "__main__":
    main()
