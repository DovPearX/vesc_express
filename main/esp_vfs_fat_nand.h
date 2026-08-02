#pragma once

#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "spi_nand_flash.h"

esp_err_t esp_vfs_fat_nand_mount(const char *base_path,
		spi_nand_flash_device_t *nand_device,
		const esp_vfs_fat_mount_config_t *mount_config);
esp_err_t esp_vfs_fat_nand_unmount(const char *base_path,
		spi_nand_flash_device_t *nand_device);
