#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"

#include "sdcard.h"

static sdmmc_card_t *sdcard = NULL;

int sdcard_init(const char *mount_path)
{
	sdmmc_host_t host = SDMMC_HOST_DEFAULT();
	host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;

	esp_vfs_fat_sdmmc_mount_config_t mount_config = {0};
	mount_config.format_if_mount_failed = false;
	mount_config.max_files = 5;

	return esp_vfs_fat_sdmmc_mount(mount_path, &host, &slot_config, &mount_config, &sdcard);
}

int sdcard_deinit()
{
	if (!sdcard) {
		return ESP_FAIL;
	}

	return esp_vfs_fat_sdmmc_unmount();
}

bool sdcard_present(void) { return sdcard != NULL; }
