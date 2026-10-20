# Scenes

Scenes redraw changed areas through a reusable RGB565 tile.

## Functions

| Function | Purpose |
| --- | --- |
| `(scene-create width height slots [tile-width tile-height [background]])` | Create a scene. Default tile: 32 x 16, background: 0. |
| `(scene-set scene slot object)` | Replace an object; `nil` removes it. |
| `(scene-get scene slot)` | Return the object in a slot. |
| `(scene-invalidate scene [x y width height])` | Mark an area, or the entire scene, for redraw. |
| `(scene-background scene color)` | Set the background and mark the entire scene for redraw. |
| `(scene-render scene)` | Draw dirty areas; an unchanged scene sends nothing. |

Slots start at zero. Higher slots are drawn over lower slots. `scene-set` marks
both old and new bounds for redraw. Changing an image, palette or string in place
requires explicit invalidation. Keep scene updates and rendering in one context,
or protect them with a mutex.

## Objects

```lisp
(list 'rect x y width height color)
(list 'roundrect x y width height radius color)
(list 'circle center-x center-y radius color)
(list 'text x y font string color scale)
(list 'image x y buffer transparent-color palette alpha)
(list 'ttf x y width height pen-x baseline-y prepared-font string palette)
```

Shapes are filled. Colors are RGB888 numbers or regular/gradient `img-color`
values. Bitmap text uses a built-in font ID or font array; scale is 1-32.

Indexed images require 2, 4 or 16 palette entries; RGB images use `nil`.
`transparent-color` is a source index or RGB888 value; `-1` disables it.
Alpha is 0-255.

`ttf` uses a `ttf-prepare` font and printable ASCII. Its rectangle clips the text;
pen and baseline coordinates are absolute. Palette entry zero fills the box;
use `(img-color 'regular 0 0)` there for transparency. Imported fonts may have a
trailing NUL byte, as added by VESC Tool.

## Example

```lisp
(disp-init)
(def scene (scene-create 240 320 8))
(scene-set scene 0 (list 'rect 40 40 100 30 0x00BFFF))
(scene-render scene)
(scene-set scene 0 (list 'rect 40 80 100 30 0x00BFFF))
(scene-render scene)
```

Use the dimensions and coordinate offsets expected by the display driver.
The default pixel tile uses 1024 bytes; scene state and object data are additional.
Limits: 256 slots and 16 dirty rectangles. Dirty overflow redraws the entire scene;
a failed render retains dirty areas for retry. The driver must consume each tile
before its callback returns.
