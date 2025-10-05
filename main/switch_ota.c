/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * Tests for switching between partitions: factory, OTAx, test.
 */

#include <esp_types.h>
#include <stdio.h>
#include "string.h"
#include <inttypes.h>
#include "sdkconfig.h"

#include "esp_rom_spiflash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "unity.h"

#include "bootloader_common.h"
#include "../bootloader_flash/include/bootloader_flash_priv.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_flash_partitions.h"
#include "esp_image_format.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "esp_sleep.h"

#define BOOT_COUNT_NAMESPACE "boot_count"

static const char *TAG = "swith_ota";

/* @brief Copies a current app to next partition using handle.
 *
 * @param[in] update_handle - Handle of API ota.
 * @param[in] cur_app - Current app.
 */
static void copy_app_partition(esp_ota_handle_t update_handle, const esp_partition_t *curr_app)
{
    const void *partition_bin = NULL;
    esp_partition_mmap_handle_t data_map;
    ESP_LOGI(TAG, "start the copy process");
    TEST_ESP_OK(esp_partition_mmap(curr_app, 0, curr_app->size, ESP_PARTITION_MMAP_DATA, &partition_bin, &data_map));
    TEST_ESP_OK(esp_ota_write(update_handle, (const void *)partition_bin, curr_app->size));
    esp_partition_munmap(data_map);
    ESP_LOGI(TAG, "finish the copy process");
}

/* @brief Get the next partition of OTA for the update.
 *
 * @return The next partition of OTA(OTA0-15).
 */
static const esp_partition_t * get_next_update_partition(void)
{
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    TEST_ASSERT_NOT_EQUAL(NULL, update_partition);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%"PRIx32, update_partition->subtype, update_partition->address);
    return update_partition;
}

/* @brief Copies a current app to next partition (OTA0-15) and then configure OTA data for a new boot partition.
 *
 * @param[in] cur_app_partition - Current app.
 * @param[in] next_app_partition - Next app for boot.
 */
static void copy_current_app_to_next_part(const esp_partition_t *cur_app_partition, const esp_partition_t *next_app_partition)
{
    esp_ota_get_next_update_partition(NULL);
    TEST_ASSERT_NOT_EQUAL(NULL, next_app_partition);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%"PRIx32, next_app_partition->subtype, next_app_partition->address);

    esp_ota_handle_t update_handle = 0;
    TEST_ESP_OK(esp_ota_begin(next_app_partition, OTA_SIZE_UNKNOWN, &update_handle));

    copy_app_partition(update_handle, cur_app_partition);

    TEST_ESP_OK(esp_ota_end(update_handle));
    TEST_ESP_OK(esp_ota_set_boot_partition(next_app_partition));
}

/* @brief Erase otadata partition
 */
static void erase_ota_data(void)
{
    const esp_partition_t *data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, NULL);
    TEST_ASSERT_NOT_EQUAL(NULL, data_partition);
    TEST_ESP_OK(esp_partition_erase_range(data_partition, 0, 2 * SPI_FLASH_SEC_SIZE));
}

/* @brief Reboots ESP using mode deep sleep. This mode guaranty that RTC_DATA_ATTR variables is not reset.
 */
void reboot_as_deep_sleep(void)
{
    ESP_LOGI(TAG, "reboot as deep sleep");
    esp_deep_sleep(20000);
    TEST_FAIL_MESSAGE("Should never be reachable except when sleep is rejected, abort");
}

/* @brief Get running app.
 *
 * @return The next partition of OTA(OTA0-15).
 */
static const esp_partition_t* get_running_firmware(void)
{
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08"PRIx32")",
            running->type, running->subtype, running->address);
    ESP_LOGI(TAG, "Configured partition type %d subtype %d (offset 0x%08"PRIx32")",
            configured->type, configured->subtype, configured->address);
    TEST_ASSERT_NOT_EQUAL(NULL, configured);
    TEST_ASSERT_NOT_EQUAL(NULL, running);
    if (running->subtype != ESP_PARTITION_SUBTYPE_APP_TEST) {
        TEST_ASSERT_EQUAL_PTR(running, configured);
    }
    return running;
}

// type of a corrupt ota_data
typedef enum {
    CORR_CRC_1_SECTOR_OTA_DATA       = (1 << 0),   /*!< Corrupt CRC only 1 sector of ota_data */
    CORR_CRC_2_SECTOR_OTA_DATA       = (1 << 1),   /*!< Corrupt CRC only 2 sector of ota_data */
} corrupt_ota_data_t;

void ota_factory_reload(void)
{
    const esp_partition_t *cur_app = get_running_firmware();
    TEST_ASSERT_EQUAL(ESP_PARTITION_SUBTYPE_APP_FACTORY, cur_app->subtype);
    ESP_LOGI(TAG, "Copy Factory app to next part");
    copy_current_app_to_next_part(cur_app, get_next_update_partition());
    reboot_as_deep_sleep();
}

void prepare_factory_reload(void)
{
    erase_ota_data();
}

bool prepare_rollback(void)
{
    return esp_ota_check_rollback_is_possible();
}

void ota_rollback(void)
{
    if (esp_ota_mark_app_invalid_rollback_and_reboot() == ESP_OK)
        reboot_as_deep_sleep();
}