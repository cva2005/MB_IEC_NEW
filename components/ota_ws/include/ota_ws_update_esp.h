#pragma once

#include <esp_log.h>
#include "esp_http_server.h"

#define OTA_RESTART_ESP "oRes"
#define OTA_SIZE_START "oSz"
#define OTA_SET_CHUNK_SIZE "pLen"
#define OTA_GET_CHUNK "pGet"
#define OTA_END "oEnd"
#define OTA_ERROR "oErr"
#define OTA_CANCEL "oCnl"

esp_err_t start_ota_ws(void);
esp_err_t write_ota_ws(int data_read, uint8_t *ota_write_data);
esp_err_t end_ota_ws(void);
esp_err_t abort_ota_ws(void);
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
#define OTA_CHECK_ROLLBACK "chRlb"
#define OTA_PROCESS_ROLLBACK "prRlb"
bool check_ota_ws_rollback_enable(void);
esp_err_t rollback_ota_ws(bool rollback);
#endif