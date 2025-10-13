/* 
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "esp_image_format.h"
#include "ota_ws_update_esp.h"
#include "config.h"

static const char *TAG = "ota_ws_esp";

static const esp_partition_t *update_partition = NULL;
static bool image_header_was_checked = false;
static esp_ota_handle_t update_handle = 0;

//static int tstc=0;

esp_err_t start_ota_ws(void)
{
    //return ESP_OK; // debug return
    //tstc=0;

    esp_err_t err;
    ESP_LOGI(TAG, "Starting OTA");

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();
    if(configured==NULL || running == NULL)
    {
        ESP_LOGE(TAG,"OTA data not found");
        return ESP_FAIL;
    }

    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08lx, but running from offset 0x%08lx",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08lx)",
             running->type, running->subtype, running->address);

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%lx",
             update_partition->subtype, update_partition->address);

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed ");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded");

    image_header_was_checked = false;
    return ESP_OK;
}
esp_err_t write_ota_ws(int data_read, uint8_t *ota_write_data)
{
    if (image_header_was_checked == false) // first segment
    {
        esp_app_desc_t new_app_info;
        if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
        {
            // check current version with downloading
            memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
            ESP_LOGI(TAG, "New firmware project name: %s", new_app_info.project_name);
            ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);
            if (!strcmp(ProjectName, new_app_info.project_name))
            {
                if (VersionNum < atoi(new_app_info.version))
                {
                    image_header_was_checked = true;
                    goto write_next;
                }
            }
            ESP_LOGE(TAG, "Binary File Corrupt!");
            return ESP_ERR_INVALID_VERSION;
        }
        else
        {
            ESP_LOGE(TAG, "Received package is not fit len");
            return ESP_ERR_INVALID_SIZE;
        }
    }
write_next:
    esp_err_t err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
    //tstc+=data_read;
    if (err != ESP_OK)
    {
        return ESP_FAIL;
    }
    //ESP_LOGI("tstc","%d",tstc);
    return ESP_OK;
}
esp_err_t end_ota_ws(void)
{
        //return ESP_OK; // debug return

    esp_err_t err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        }
        ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        return ESP_FAIL;
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        return ESP_FAIL;
    }
    return  ESP_OK;
}
esp_err_t abort_ota_ws(void)
{
    return esp_ota_abort(update_handle);
}
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
// false - rollback disable
// true - rollback enable
bool check_ota_ws_rollback_enable(void)
{
    esp_ota_img_states_t ota_state_running_part;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(running, &ota_state_running_part) == ESP_OK) {
        if (ota_state_running_part == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "Running app has ESP_OTA_IMG_PENDING_VERIFY state");
            return true;
        }
    }
    return false;    
}
// rollback == true - rollback
// rollback == false - app valid? confirm update -> no rollback
esp_err_t rollback_ota_ws(bool rollback)
{
    if(rollback == false)
    {
        return esp_ota_mark_app_valid_cancel_rollback(); // app valid
    }
    else
    {
        return esp_ota_mark_app_invalid_rollback_and_reboot(); // app rolback & reboot
    }
    return ESP_FAIL;
}
#endif