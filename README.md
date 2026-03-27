# VESC Express

The is the codebase for the VESC Express, which is a WiFi and Bluetooth-enabled logger and IO-board. At the moment it is tested and runs on the ESP32C3 and ESP32S3 but other ESP32 devices can be added.

## Toolchain

This project now targets ESP-IDF 6.0.0.

Instructions for setting up the toolchain can be found in the official ESP-IDF documentation:
[https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32c3/get-started/](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32c3/get-started/)

**Note**  
ESP-IDF 6.0.0 or later is required for building this project.

### Get Release 6.0.0

The instructions linked above can install the latest branch of ESP-IDF. To use the stable 6.0.0 release, clone that version explicitly:

```bash
git clone -b v6.0.0 --recursive https://github.com/espressif/esp-idf.git esp-idf-v6.0.0
cd esp-idf-v6.0.0/
./install.sh esp32c3 esp32s3
```

At the moment development is done using the stable 6.0.0 release.

## Building

After installing the toolchain, source/export the ESP-IDF environment for your shell and set the target chip/architecture with

```bash
idf.py set-target <target> 
```

where target is `esp32c3` or `esp32s3`. You will need to run `idf.py fullclean` or remove the build directory when changing targets.

Once the toolchain is active in the current shell, the project can be built with

```bash
idf.py build
```

That will create `vesc_express.bin` in the build directory, which can be used with the bootloader in VESC Tool. If the target does not come with firmware preinstalled, the USB port can be used for flashing firmware using the built-in bootloader. In that case `bootloader.bin` and `partition-table.bin` from the build directory are also required. This can be done from VESC Tool or with `idf.py flash`.

All supported targets can be built with

```bash
python build_all.py
```

That will create all required firmware files under the `build_output` directory, with hardware names as child directories. Target selection is handled automatically by `build_all.py`.

### Custom Hardware Targets

If you wish to build the project with custom hardware config files, add the hardware config files to `main/hwconf` and use the `HW_NAME` build flag:

```bash
idf.py build -DHW_NAME="VESC Express T"
```

**Note:** If you change environment variables or switch to a different ESP-IDF installation, run `idf.py reconfigure` before building. Running `idf.py fullclean` has the same effect, as it forces CMake to regenerate the build configuration.
