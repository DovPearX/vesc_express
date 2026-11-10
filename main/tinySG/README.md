# tinySG

## Lisp

Initialize the display and touch driver before creating the GUI.

```clj
(tinysg 'init 320 180 '(buffer 1024))
(def panel (tinysg 'rect 16 16 288 148 0x1082 '(filled) '(rounded 8)))
(def title (tinysg 'text 32 30 0xffff "SPEED" '(font 5 2)))
(def value (tinysg 'text 32 68 0x07ff "42" '(font 5 6)))
(tinysg 'render 0 (list panel title value))
```

The binding registers one LispBM extension: `tinysg`. Operation names are
symbols such as `init`, `rect` and `render`. The previous `tinysg-*`
extensions are removed. VESC tinySG builds reserve 352 built-in extension slots
plus the existing package/hardware reserves. The build checks the required capacity.

## Building indicators from primitives

Scene nodes are the base API. Compose arcs, lines, shapes, text and images into
application-specific controls. Input handling and control state belong to the
application.

```clj
(defun arc-meter (value) {
  (var level (if (< value 0) 0 (if (> value 100) 100 value)))
  (var text (str-from-n (to-i level) "%d"))
  (var width (ix (tinysg 'text-size text 5 4) 0))
  (list
    (tinysg 'arc 160 90 70 0x2128 135 270 '(thickness 8))
    (tinysg 'arc 160 90 70 0x05d9 135 (to-i (* level 2.7)) '(thickness 8))
    (tinysg 'text (- 160 (/ width 2)) 106 0xffff text '(font 5 4)))
})
(tinysg 'render 0 (arc-meter 65))
```

This indicator is three nodes. Reuse static decorations and update the dynamic
parts when telemetry changes. Dirty-region rendering compares the resulting scene.

## Appearance

Filled shapes support solid colors, horizontal/vertical gradients and angled
gradients. Dashes apply to lines. Text uses glyph coverage rather than pattern fills.
Use FAST on C3; select `aa2` or `aa4` where smoother edges justify the CPU cost.
Drawing attributes include `filled`, `rounded`, `thickness`, `dashed`, `fill`,
`opacity`, `quality`, `rotate`, `skew`, `zoom`, `pivot` and `wave`.

## Fonts

Import one bitmap font and choose a size wherever it is used:

```clj
(import "assets/dashboard-font/font.bin" 'ui-font)
(tinysg 'text 20 30 0xffff "Speed" (list 'font ui-font 24))
(tinysg 'text 20 70 0xffff "42" (list 'font ui-font 90))
(tinysg 'text-size "Speed" ui-font 24)
```

The same asset supplies all sizes. Glyph coverage is stored as A4 (16 alpha
levels), read directly from the imported data, and scaled while drawing. No
separate font bitmap is allocated per size. Built-in 3x5 and 5x7 fonts remain
available through the existing numeric `font` option.

Convert TTF/OTF on the host with Pillow:

```sh
python tools/font.py Roboto-Bold.ttf font.bin --size 90
```

The source raster size sets the detail available when scaling. `--characters`
can limit the included glyphs to reduce flash usage. Retain the source font's
license when distributing a converted asset.
The firmware does not load or rasterize TTF/OTF files.

TSF1 layout: four magic bytes, first character, character count, raster height,
nominal size, then eight-byte glyph records. Each record contains a big-endian
32-bit pixel offset, pixel width, advance and two reserved zero bytes. Glyphs
store consecutive A4 pixels, high nibble first.

## Images

Import RGB `.bin` assets in the tinyGFX format, or baseline JPEG files.
Dimensions are read from the file.

```clj
(import "assets/logo.jpg" 'logo)
(def image (tinysg 'image '(40 80) logo))
(tinysg 'render 0x0862 (list image))
```

BIN files have a five-byte header: width and height as big-endian 16-bit integers,
then bits per pixel (8, 16 or 24). Pixels use RGB332, big-endian RGB565 or RGB888.
Indexed/palette BIN files are not supported. RGB565 BIN assets are read directly
without allocating a decoded image, making them the preferred format on C3.

The tinySG JPEG decoder supports 8-bit, single-scan baseline grayscale and YCbCr images, including
chroma subsampling and restart intervals. JPEG is decoded once into RGB565 when
creating the image and needs `width * height * 2` bytes for the decoded pixels,
plus temporary decoder storage. Progressive JPEG, CMYK and PNG are not supported
by the firmware. The browser reference accepts PNG/JPEG and exports RGB565 BIN.

Images accept `'(opacity value)` and `'(revision number)`.
Masks use header byte 0 for A8 coverage or 0x81 for packed A1 coverage
(most significant bit first). `'(tint color)` sets their color.
`'(source byte-offset)` selects a BIN image within a packed asset.
For mutable image buffers, change the revision after modifying pixels.
Opaque image rows are copied into the output tile; no framebuffer is required.

## Memory and updates

```clj
(tinysg 'init 172 320 '(offset 34 0) '(buffer 1024)
  '(nodes 64) '(text 512) '(dirty 512) '(layout 16) '(quality fast))
```

| Option | Default | Range |
| --- | ---: | ---: |
| `buffer` | 1024 bytes | 2..1048576 |
| `nodes` | 64 per snapshot | 1..4096 |
| `text` | 512 bytes per snapshot | 1..65535 |
| `dirty` | 512 bitmap bytes | 1..65535 |
| `layout` | 16 items | 1..4096 |
| `offset` | 0, 0 | Visible area plus offset must fit 32767 |
| `quality` | `fast` | `fast`, `aa2`, `aa4` |

Initialization may be repeated to change limits. `(tinysg 'deinit)` releases storage.
The pixel buffer is only part of the RAM cost: scene snapshots, Lisp objects,
driver buffers and stacks also need memory.

Rendering compares scene snapshots and redraws dirty regions. An unchanged scene
sends no pixels. The first frame or a background-color change redraws the display.
`(tinysg 'stats)` returns `(pixel-bytes tiles nodes storage-bytes)`.
Keep initialization, input and rendering in one application task.

## Drawing and layout

```clj
(tinysg 'rect 8 12 100 24 0x07ff '(filled) '(rounded 4))
(tinysg 'circle 80 110 28 0xfd20 '(thickness 3))
(tinysg 'line 8 160 160 210 0xffff '(thickness 3) '(dashed 6 4))
(tinysg 'text 12 240 0xffff "tinySG" '(font 5 2))

(tinysg 'grid-cell 8 40 156 180 3 2 4 2 0)
(tinysg 'flex 8 40 156 80 'row-wrap 4 2 'space-between 'center
  '((40 24 0) (32 20 1) (24 28 2)))
```

Rectangles and circles default to outlines; `'(filled)` selects a filled shape.
Text supports built-in 3x5 and 5x7 fonts, ASCII 32..126.
Drawing attributes include opacity, quality, gradients, rotation and zoom.
Angles are degrees; positive rotation is clockwise. Zoom uses Q8, where 256 is 1x.
Grid returns one area; flex returns an area per item. Flex supports row/column,
wrapping, gap, padding, grow/shrink and alignment.

## C API and build

```sh
cmake -S main/tinySG -B build-tinysg
cmake --build build-tinysg
```

Header: `include/tinysg.h`. CMake target: `tinysg`, alias `tinySG::tinySG`.
The caller owns scenes, styles and scratch memory. Keep node data valid until
rendering and dirty comparison finish.

The synchronous output callback receives big-endian RGB565 tiles and must consume
them before returning. Default-display registration:

```c
tinysg_display_extensions_set_callbacks(panel_write, panel_clear, panel_reset);
```

VESC selects tinySG through `HW_USE_TINYSG` in hwconf and registers existing
panel renderers through `lispif_disp_set_callbacks`.

The P4 port supports blocking PPA fill and ST7701 tile rotation.
`(tinysg 'accel 'auto)` enables eligible operations; `'cpu` selects software.
`(tinysg 'accel-stats)` returns `(fills rotations failures)`.
Small or unaligned operations use the CPU path. PPA setup and cache synchronization
can cost more than the operation saves on small regions.

## Visual reference

Open [docs/index.html](docs/index.html) for the gallery and live preview. Each
card covers a distinct primitive, image or effect. Gradients, quality and
opacity remain adjustable without separate cards for every combination.

The preview covers primitive geometry, text transforms, image export and optional
TSF1 font import. The generated Lisp imports the same font file.

To rebuild the WebAssembly renderer and gallery from the tinySG directory:

```sh
python tools/docs.py
```

Requires Python with Pillow, Node.js and Clang with WebAssembly support. Use
`--clang /path/to/clang` or `--node /path/to/node` when needed. `--skip-wasm`
regenerates the gallery using the current renderer. Gallery images and the live
preview use the same C entry point in `docs/preview.c`.

## Further work

- Lisp control composition, scrolling and focus.
- Unicode and text layout.
- Image compression and asynchronous PPA composition.
