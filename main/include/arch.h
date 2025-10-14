#pragma once

#include "nvs.h"
#include "esp_system.h"

#define SYS_EVT_SHIFFT  4
typedef enum
{
    NO_SYS_EVT = 0,                      /* No System Event */
    CH_CHANGE_RST = 1 << SYS_EVT_SHIFFT, /* Change to Reserved IEC Channel */
    FW_UPD = 2 << SYS_EVT_SHIFFT,        /* Firmware update */
    CFG_WR = 3 << SYS_EVT_SHIFFT,        /* Write Configuration data */
    RST_WEB = 4 << SYS_EVT_SHIFFT,       /* Reset Command from Browser */
    SN_LOAD = 5 << SYS_EVT_SHIFFT,       /* Load Serial in Work Mode */
    CLR_ARC = 6 << SYS_EVT_SHIFFT,       /* Clear System Events Archive */
    FW_RLB = 7 << SYS_EVT_SHIFFT,        /* Firmware roll back */
    FW_FCT = 8 << SYS_EVT_SHIFFT,        /* Firmware reset to Factory */
    CFG_DF = 9 << SYS_EVT_SHIFFT,        /* Reset to Default Configuration */
    CFG_MD = 10 << SYS_EVT_SHIFFT,       /* Load Device in to Configuration Mode */
    CLR_CNT = 11 << SYS_EVT_SHIFFT,      /* Reset Modbus IO, ERR counts */
    CON_104 = 12 << SYS_EVT_SHIFFT,      /* Connected to IEC 104 Client */
    CON_101 = 13 << SYS_EVT_SHIFFT,      /* Connected to IEC 101 Master */
    DCN_IEC = 14 << SYS_EVT_SHIFFT,      /* Disconnected from IEC 101/104 Master/Client */
} system_event_t;

typedef struct
{
    uint32_t rtc_count;
    uint32_t arc_event;
} arc_rec_t;

esp_err_t init_events_arch(nvs_handle_t nvs_handle);
void save_arch_event(uint32_t sys_event);
esp_err_t clear_events_arch(void);
char *get_arch_json(char *p);
esp_err_t save_utc_copy(void);
esp_err_t read_utc_copy(void);