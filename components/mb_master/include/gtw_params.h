#ifndef _DEVICE_PARAMS
#define _DEVICE_PARAMS

#include <stdint.h>
#include "sdkconfig.h"
#include "mbcontroller.h"
#include "cs101_information_objects.h"
#include "mb_proto.h"
#include "esp_err.h"
#include "config.h"

typedef enum
{
    SER1_CONN = 0,
    SER2_CONN = 1,
    SERIAL_NUM = 2,
    ETH_CONN = 2,
    MB_NUM = 3,
} mb_connect_t;

typedef enum
{
    GTW_RESET_STATE = 0,
    MB_INIT_DELAY = 1,
    IEC_CONNECTION_WAIT = 2,
    MB_FAULT_WAIT = 3,
    MB_FAULT_PROCESS = 4,
    IEC_CONNECTION_ESTABLISHED = 5,
    MB_FAULT_STATE = 6,
} gtw_state_t;

typedef enum
{
    COT_PERIODIC = 1,
    COT_BACKGROUND_SCAN = 2,
    COT_SPONTANEOUS = 3,
    COT_INITIALIZED = 4,
    COT_REQUEST = 5,
} gtw_cause_t;

typedef enum
{
    QUALITY_GOOD = 0,
    QUALITY_OVERFLOW = 0x01,
    QUALITY_BLOCKED = 0x10,
    QUALITY_NON_TOPICAL = 0x40,
    QUALITY_INVALID = 0x80,
} gtw_quality_t;

#pragma pack(1)
typedef struct
{
    uint8_t init;
    uint8_t quality;
    uint8_t to_cnt;
    uint8_t cause;
    uint8_t *old_val;
    uint8_t *tmp_val;
    uint32_t sel_time;
    uint64_t pool_time;
    uint64_t send_cnt;
} gtw_elm_t;

typedef struct
{
    uint8_t pid;
    uint8_t time_out;
    uint8_t cause;
    uint8_t quality;
    uint16_t reserved;
    uint16_t idx;
    int32_t data;
    uint64_t time_ms;
} queue_elm_t;
#pragma pack()

#define DP_BIT_SIZE     2
#define DP_BIT_MASK     0x03
#define BIT_REG_LEN     8
#define DP_REG_LEN      (BIT_REG_LEN / DP_BIT_SIZE)
#define BYTES_IN_REG    2
#define REG_IN_CDU      123 /* Write Multiple Registers: 1 - 123 */
#define BYTES_IN_CDU    (REG_IN_CDU * BYTES_IN_REG)
#define BITS_IN_CDU     (BYTES_IN_CDU * BIT_REG_LEN) /* Write Multiple Coils: 1 - 1968 */
#define MB_QUEUE_LEN    (32)
#define IEC_QUEUE_LEN   (32)
#define MB_DL           (REG_IN_CDU * BYTES_IN_REG)
#define DSC_LEN         (CFG_MAX_LEN)
#define D_LEN           (DSC_LEN * BYTES_IN_CDU)
#define ARC_LEN         (DSC_LEN * REG_IN_CDU * sizeof(float))
#define ETH_MB_CONNECT  4096
#define WIFI_MB_CONNECT 8192
#define BY_STATION      20
#define CNT_GENERAL     5
#define PRM_NOT_FOUND   (-1)

extern QueueHandle_t mbQueueHdl;
extern QueueHandle_t iecQueueHdl;
extern SemaphoreHandle_t gtwSemaphore;

esp_err_t gtw_param_init(void);
int get_obj_elm(int pid);
float get_old_val(int cid, int idx);
void set_old_val(int cid, int idx, float val);
bool get_old_state(int cid, int idx);
void set_old_state(int cid, int idx, bool state);
void set_old_bits(int cid, int idx, uint8_t bits);
float get_old_val(int cid, int idx);
float *get_old_val_ptr(int cid, int idx);
void set_old_val(int cid, int idx, float val);
bool is_val_init(int cid, int pid);
mb_commands_t get_mb_command(mb_descr_type_t param_type, int num);
int iec_to_mb(mb_descr_type_t param_type, uint8_t *buff, uint32_t *pdata, int num, bool common_data, float *step_data);
bool is_poll_time(int cid, int pid);
bool is_send_time(int cid, int pid);
bool is_mb_timeout(int cid);
void clr_mb_error(int cid);
bool inc_mb_error(int cid);
void set_gtw_state(gtw_state_t state);
gtw_state_t get_gtw_state(void);
float get_fault_val(int pid);
void set_init_time(void);
bool is_init_time_out(void);
void set_fault_time(void);
bool is_fault_time_out(void);
void set_fault_state(int cid);
bool is_fault_state(int cid);
bool is_fault_none(int pid);
bool is_floating(int pid);
bool is_normalized(int pid);
bool is_bitstring32(int pid);
bool is_scaled_val(int pid);
bool is_step_position(int pid);
bool is_int_totals(int pid);
bool is_scaled_val(int pid);
uint32_t get_old_bit_string_32(int cid, int idx);
void set_old_bit_string_32(int cid, int idx, uint32_t bs32);
bool is_double_point(int pid);
DoublePointValue get_old_dpoint(int cid, int idx);
void set_old_dpoint(int cid, int idx, DoublePointValue dp_val);
DoublePointValue get_tmp_dpoint(int cid, int idx);
void set_tmp_dpoint(int cid, int idx, DoublePointValue dp_val);
bool is_mb_connect_use(int conn_type);
int get_param_num(int conn_type);
int get_param_first(int conn_type);
mb_parameter_descriptor_t *get_param_desc(int conn_type);
mb_connect_t get_connect_type(int pid);
char **get_ip_table(void);
int get_db_len(void);
iec_data_t get_iec_data_type(int pid);
int get_iec_obj_addr(int pid);
int get_iec_obj_num(int pid);
uint8_t get_iec_group(int pid);
iec_time_t interrogation_time_type(void);
iec_time_t measured_time_type(void);
iec_time_t spontanius_time_type(void);
uint8_t get_iec_cnt_group(int pid);
void set_cause(int cid, gtw_cause_t cause);
gtw_cause_t get_cause(int cid);
void set_quality(int cid, gtw_quality_t quality);
gtw_quality_t get_quality(int cid);
int get_common_addr(void);
bool is_cmd_term(void);
bool is_cse_term(void);
bool get_obj_id(int obj, int *pid, int *idx);
void set_select_time(int cid);
void clr_select_time(int cid);
bool is_select_time(int cid);
bool is_select_time_out(int cid);
uint64_t get_pool_time(int cid);
void set_pool_time(int cid, uint64_t time_ms);
void end_of_init_set(void);
bool is_end_of_init(void);
uint32_t get_tmp_bit_string_32(int cid, int idx);
void set_tmp_bit_string_32(int cid, int idx, uint32_t bs32);
float get_tmp_val(int cid, int idx);
float *get_tmp_val_ptr(int cid, int idx);
void set_tmp_val(int cid, int idx, float val);
bool get_tmp_state(int cid, int idx);
void set_tmp_state(int cid, int idx, bool state);
void set_tmp_bits(int cid, int idx, uint8_t bits);
uint32_t get_time_sync(void);
uint32_t get_time_swith(void);
int get_prm_id(int pid);
int get_prm_cid(int pid);
TickType_t get_between_del(void);

#endif // !defined(_DEVICE_PARAMS)
