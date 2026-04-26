
# Documentation for display drivers

# Introduction

Drivers for a specific display is loaded with the associated `disp-load-x` function, where x is the name of the driver. Each disp-load function takes arguments specific to the display. These are
explained below.

Each display driver provides a "rendering", "clearing" and "reset"
function that is connected to the LispBM extensions `disp-render`,
`disp-clear` and `disp-reset` upon loading. The render, clear and
reset functions are the display dependent interface that are
implemented per display.

# Adding new panels with esp_lcd

New display drivers should use the `esp_lcd` component-based flow (see
`disp_axs15231.c` for a complete example). In this project, adding a
panel consists of two parts: the low-level display driver and the
LispBM loader extension.

## 1. Add dependency

If your panel has a dedicated esp_lcd component, add it to
`main/idf_component.yml`.

Example:

```yaml
espressif/esp_lcd_axs15231b: "^1.0.1"
```

## 2. Create display driver file

Create a new `disp_<panel>.c/.h` and implement:

* `disp_<panel>_init(...)`
* `disp_<panel>_render_image(...)`
* `disp_<panel>_clear(...)`
* `disp_<panel>_reset(...)`

Inside `init` use the esp_lcd sequence:

1. Configure and initialize the bus (`spi_bus_initialize(...)` for SPI/QSPI).
2. Create panel IO (`esp_lcd_new_panel_io_spi(...)` or other IO backend).
3. Configure panel device settings (`esp_lcd_panel_dev_config_t`).
4. Create panel handle (`esp_lcd_new_panel_<controller>(...)`).
5. Call `esp_lcd_panel_reset(...)` and `esp_lcd_panel_init(...)`.

Use `esp_lcd_panel_draw_bitmap(...)` in render/clear routines.

## 3. Register LispBM load function

In `lispif_disp_extensions.c`:

1. Add `ext_disp_load_<panel>(...)` argument parsing and validation.
2. Call your `disp_<panel>_init(...)`.
3. Connect callbacks with `lbm_display_extensions_set_callbacks(...)`.
4. Register loader name with `lbm_add_extension("disp-load-<panel>", ...)`.

After this, LispBM can switch to the panel at runtime with
`disp-load-<panel>`.

# Displays

## sh8501b

* Resolution: 194 * 368
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.


### disp-load-sh8501b

```clj
(disp-load-sh8501b gpio-sd0 gpio-clk gpio-cs gpio-reset spi-mhz)
```

Loads the sh8501b driver. The driver uses hardware-SPI at rate `spi-mhz` on the
`gpio-sd0` and `gpio-clk` GPIO pins.

Example using GPIO pins 6,5,7 and 8 for sd0,clk,cs and reset running the
SPI clock at 40MHz:

```clj
(disp-load-sh8501b 6 5 7 8 40)
```

## sh8601

* Resolution: 170 * 320
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-sh8601

```clj
(disp-load-sh8601 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the sh8601 driver. The driver uses hardware-SPI at rate `spi-mhz` on the
`gpio-sd0` and `gpio-clk` GPIO pins.

Example using GPIO pins 6,5,7 and 8 for sd0,clk,cs and reset running the
SPI clock at 40MHz and DC on GPIO 9:

```clj
(disp-load-sh8601 6 5 7 8 9 40)
```

## ili9341

* Resolution: 320 * 240
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-ili9341

```clj
(disp-load-ili9341 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the ili9341 driver. The driver uses hardware-SPI at rate
`spi-mhz` on the `gpio-sd0` and `gpio-clk` GPIO pins. In addition, the
ili9341 uses a data/command signal to discern between commands and
data. The data/command signal is mapped to GPIO `gpio-cs`.

Example using GPIO pins 6,5,19,18 and 7 for sd0,clk,cs,reset and dc.
The SPI clock is set to 40MHz.

```clj
(disp-load-ili9341 6 5 19 18 7 40)
```


## ssd1306

* Resolution: 128 * 64
* Colors: B/W  (1bpp)
* Interface: I2C

Can display images of `indexed2` format and is limited to displaying
only full screen images starting at position (0, 0).

### disp-load-ssd1306

```clj
(disp-load-ssd1306 gpio-sda gpio-scl i2c-hz)
```

Load the ssd1306 driver. The ssd1306 talks I2C over the GPIOs
`gpio-sda` (serial data) and `gpio-scl` (clock).

Example using GPIO pins 7 and 6 for serial data and clock.

```clj
(disp-load-ssd1306 7 6 700000)
```

## st7789

* Resolution: up to 320 * 240
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-st7789

```clj
(disp-load-st7789 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the st7789 driver. The driver uses hardware-SPI at rate
`spi-mhz` on the `gpio-sd0` and `gpio-clk` GPIO pins. In addition, the
st7789 uses a data/command signal to discern between commands and
data. The data/command signal is mapped to GPIO `gpio-cs`.

Example using GPIO pins 6,5,19,18 and 7 for sd0,clk,cs,reset and dc.
The SPI clock is set to 40MHz.

```clj
(disp-load-st7789 6 5 19 18 7 40)
```

**Note**
Many st7789-based displays do not have the full resolution that the driver supports in the panel. Some of them also have an offset where the panel starts. The panel size and offset has to be taken into account when using disp-render.

## st7735

* Resolution: up to 162 * 132
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-st7735

```clj
(disp-load-st7735 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the st7735 driver. The driver uses hardware-SPI at rate
`spi-mhz` on the `gpio-sd0` and `gpio-clk` GPIO pins. In addition, the
st7789 uses a data/command signal to discern between commands and
data. The data/command signal is mapped to GPIO `gpio-cs`.

Example using GPIO pins 6,5,19,18 and 7 for sd0,clk,cs,reset and dc.
The SPI clock is set to 40MHz.

```clj
(disp-load-st7789 6 5 19 18 7 40)
```

**Note**
Many st7735-based displays do not have the full resolution that the driver supports in the panel. Some of them also have an offset where the panel starts. The panel size and offset has to be taken into account when using disp-render.

## ili9488

* Resolution: 480 ** 320
* Colors: 24Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-ili9488

```clj
(disp-load-ili9488 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the ili9488 driver. The driver uses hardware-SPI at rate
`spi-mhz` on the `gpio-sd0` and `gpio-clk` GPIO pins. In addition, the
ili9488 uses a data/command signal to discern between commands and
data. The data/command signal is mapped to GPIO `gpio-cs`.

Example using GPIO pins 6,5,19,18 and 7 for sd0,clk,cs,reset and dc.
The SPI clock is set to 40MHz.

```clj
(disp-load-ili9488 6 5 19 18 7 40)
```

## ssd1351

* Resolution: 128 * 128
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-ssd1351

```clj
(disp-load-ssd1351 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the ssd1351 driver. The driver uses hardware-SPI at rate
`spi-mhz` on the `gpio-sd0` and `gpio-clk` GPIO pins. In addition, the
ssd1351 uses a data/command signal to discern between commands and
data. The data/command signal is mapped to GPIO `gpio-cs`.

Example using GPIO pins 6,5,19,18 and 7 for sd0,clk,cs,reset and dc.
The SPI clock is set to 40MHz.

```clj
(disp-load-ssd1351 6 5 19 18 7 40)
```

## icna3306

* Resolution: 194 * 368
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.


### disp-load-icna3306

```clj
(disp-load-icna3306 gpio-sd0 gpio-clk gpio-cs gpio-reset spi-mhz)
```

Loads the icna3306 driver. The driver uses hardware-SPI at rate `spi-mhz` on the
`gpio-sd0` and `gpio-clk` GPIO pins.

Example using GPIO pins 6,5,7 and 8 for sd0,clk,cs and reset running the
SPI clock at 40MHz:

```clj
(disp-load-icna3306 6 5 7 8 40)
```

## axs15231

* Resolution: 320 * 480
* Colors: 16Bit
* Interface: QSPI (SD0, SD1, SD2, SD3, CLK, CS)

### disp-load-axs15231

```clj
(disp-load-axs15231 gpio-sd0 gpio-sd1 gpio-sd2 gpio-sd3 gpio-clk gpio-cs gpio-reset spi-mhz)
```

Loads the axs15231 driver using esp_lcd with QSPI panel IO.

For best performance and stability on this panel, prefer rendering images in `rgb565` format.
The esp_lcd pipeline for this panel is configured as 16-bit (`bits_per_pixel = 16`), so non-`rgb565` image formats require extra per-pixel conversion before transfer.

* `gpio-reset` can be set to `-1` if reset is not connected.
* `spi-mhz` must be in range `1..80`.

Example using GPIO pins 21,48,40,39,47,45, reset=-1, and 40MHz:

```clj
(disp-load-axs15231 21 48 40 39 47 45 -1 40)
```

After loading this driver, these helper extensions are available:

* `ext-disp-cmd` - send a raw display command with optional parameters.
* `ext-disp-orientation` - set rotation (`0`, `1`, `2`, `3`).

## gc9a01

* Resolution: 240 * 240
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-gc9a01

```clj
(disp-load-gc9a01 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the gc9a01 driver using `hwspi` over SPI.

* `gpio-reset` can be set to `-1` if reset is not connected.
* `spi-mhz` must be in range `1..40`.

Example:

```clj
(disp-load-gc9a01 6 5 19 -1 7 40)
```
