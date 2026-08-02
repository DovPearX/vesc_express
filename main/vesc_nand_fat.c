#include <stdlib.h>

#include "esp_vfs_fat_nand.h"
#include "diskio.h"
#include "diskio_impl.h"
#include "esp_check.h"
#include "esp_log.h"
#include "vfs_fat_internal.h"

static const char *TAG = "vfs_fat_nand";
static spi_nand_flash_device_t *nand_handles[FF_VOLUMES];

static DSTATUS nand_initialize(BYTE pdrv) {
	(void)pdrv;
	return 0;
}

static DSTATUS nand_status(BYTE pdrv) {
	(void)pdrv;
	return 0;
}

static DRESULT nand_read(BYTE pdrv, BYTE *buffer, DWORD sector, UINT count) {
	spi_nand_flash_device_t *device = nand_handles[pdrv];
	uint32_t sector_size;

	if (!device || spi_nand_flash_get_sector_size(device, &sector_size) != ESP_OK) {
		return RES_ERROR;
	}

	for (UINT i = 0; i < count; i++) {
		if (spi_nand_flash_read_sector(device, buffer + i * sector_size, sector + i) != ESP_OK) {
			return RES_ERROR;
		}
	}

	return RES_OK;
}

static DRESULT nand_write(BYTE pdrv, const BYTE *buffer, DWORD sector, UINT count) {
	spi_nand_flash_device_t *device = nand_handles[pdrv];
	uint32_t sector_size;

	if (!device || spi_nand_flash_get_sector_size(device, &sector_size) != ESP_OK) {
		return RES_ERROR;
	}

	for (UINT i = 0; i < count; i++) {
		if (spi_nand_flash_write_sector(device, buffer + i * sector_size, sector + i) != ESP_OK) {
			return RES_ERROR;
		}
	}

	return RES_OK;
}

static DRESULT nand_ioctl(BYTE pdrv, BYTE cmd, void *buffer) {
	spi_nand_flash_device_t *device = nand_handles[pdrv];
	uint32_t value;

	if (!device) {
		return RES_ERROR;
	}

	if (cmd == CTRL_SYNC) {
		return spi_nand_flash_sync(device) == ESP_OK ? RES_OK : RES_ERROR;
	}

	if (cmd == GET_SECTOR_COUNT) {
		if (spi_nand_flash_get_capacity(device, &value) != ESP_OK) {
			return RES_ERROR;
		}

		*(DWORD *)buffer = value;
		return RES_OK;
	}

	if (cmd == GET_SECTOR_SIZE) {
		if (spi_nand_flash_get_sector_size(device, &value) != ESP_OK || value > UINT16_MAX) {
			return RES_ERROR;
		}

		*(WORD *)buffer = (WORD)value;
		return RES_OK;
	}

	return RES_PARERR;
}

static esp_err_t ff_diskio_register_nand(BYTE pdrv, spi_nand_flash_device_t *device) {
	if (pdrv >= FF_VOLUMES || !device) {
		return ESP_ERR_INVALID_ARG;
	}

	static const ff_diskio_impl_t impl = {
		.init = nand_initialize,
		.status = nand_status,
		.read = nand_read,
		.write = nand_write,
		.ioctl = nand_ioctl,
	};

	nand_handles[pdrv] = device;
	ff_diskio_register(pdrv, &impl);

	return ESP_OK;
}

static BYTE ff_diskio_get_pdrv_nand(const spi_nand_flash_device_t *device) {
	for (BYTE i = 0; i < FF_VOLUMES; i++) {
		if (nand_handles[i] == device) {
			return i;
		}
	}

	return FF_DRV_NOT_USED;
}

static void ff_diskio_clear_pdrv_nand(const spi_nand_flash_device_t *device) {
	for (BYTE i = 0; i < FF_VOLUMES; i++) {
		if (nand_handles[i] == device) {
			nand_handles[i] = NULL;
		}
	}
}

esp_err_t esp_vfs_fat_nand_mount(const char *base_path,
		spi_nand_flash_device_t *nand_device,
		const esp_vfs_fat_mount_config_t *mount_config) {
	if (!base_path || !nand_device || !mount_config) {
		return ESP_ERR_INVALID_ARG;
	}

	BYTE pdrv = FF_DRV_NOT_USED;
	FATFS *fs = NULL;
	void *workbuf = NULL;
	esp_err_t ret = ff_diskio_get_drive(&pdrv);
	if (ret != ESP_OK) {
		return ret;
	}

	char drive[3] = {(char)('0' + pdrv), ':', 0};
	ESP_GOTO_ON_ERROR(ff_diskio_register_nand(pdrv, nand_device), fail, TAG, "diskio");

	uint32_t sector_size;
	ESP_GOTO_ON_ERROR(spi_nand_flash_get_sector_size(nand_device, &sector_size), fail, TAG, "geometry");

	esp_vfs_fat_conf_t config = {
		.base_path = base_path,
		.fat_drive = drive,
		.max_files = mount_config->max_files,
	};
	ESP_GOTO_ON_ERROR(esp_vfs_fat_register(&config, &fs), fail, TAG, "vfs");

	FRESULT fr = f_mount(fs, drive, 1);
	if (fr == FR_OK) {
		return ESP_OK;
	}

	if ((fr != FR_NO_FILESYSTEM && fr != FR_INT_ERR) || !mount_config->format_if_mount_failed) {
		ret = ESP_FAIL;
		goto fail;
	}

	workbuf = ff_memalloc(4096);
	if (!workbuf) {
		ret = ESP_ERR_NO_MEM;
		goto fail;
	}

	MKFS_PARM opt = {
		(BYTE)FM_ANY,
		0,
		0,
		0,
		esp_vfs_fat_get_allocation_unit_size(sector_size, mount_config->allocation_unit_size),
	};
	if (f_mkfs(drive, &opt, workbuf, 4096) != FR_OK || f_mount(fs, drive, 0) != FR_OK) {
		ret = ESP_FAIL;
		goto fail;
	}

	free(workbuf);
	return ESP_OK;

fail:
	free(workbuf);
	if (fs) {
		esp_vfs_fat_unregister_path(base_path);
	}
	ff_diskio_unregister(pdrv);
	ff_diskio_clear_pdrv_nand(nand_device);

	return ret;
}

esp_err_t esp_vfs_fat_nand_unmount(const char *base_path, spi_nand_flash_device_t *nand_device) {
	BYTE pdrv = ff_diskio_get_pdrv_nand(nand_device);
	if (pdrv == FF_DRV_NOT_USED) {
		return ESP_ERR_INVALID_STATE;
	}

	char drive[3] = {(char)('0' + pdrv), ':', 0};
	f_mount(NULL, drive, 0);
	ff_diskio_unregister(pdrv);
	ff_diskio_clear_pdrv_nand(nand_device);

	return esp_vfs_fat_unregister_path(base_path);
}
