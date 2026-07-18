idf_component_register(SRCS "bootloader_start.c"
                    REQUIRES bootloader bootloader_support spi_flash)

# ESP-IDF 6 registers and preprocesses the target bootloader linker scripts
# from its bootloader subproject. Registering the removed pre-6.0 .ld files
# here would bypass that pipeline and break the bootloader build.
