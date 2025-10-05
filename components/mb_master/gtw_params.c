#include "gtw_params.h"
#include "timer_1ms.h"
#include "mbcontroller.h"
#include "protocol_examples_common.h"
#include "tcp_slave_iec.h"
#include "ser_slave_iec.h"

#define MB_SLAVE_ID_MASK    0xff
#define IP_STR_LEN          32
#define CMD_MASK            0x01
#define CSE_MASK            0x02
#define SER2_MASK           0x0100
#define MAX_CONN            (CONFIG_FMB_TCP_PORT_MAX_CONN)

QueueHandle_t mbQueueHdl;
QueueHandle_t iecQueueHdl;
static StaticSemaphore_t xMutexBuffer;
SemaphoreHandle_t gtwSemaphore = NULL;
static gtw_elm_t gtw_elm[DSC_LEN] = {0};
static uint8_t mb_old[ARC_LEN];
static uint8_t mb_tmp[ARC_LEN];
static uint32_t o_cnt = 0;
static mb_parameter_descriptor_t prm_dsc[DSC_LEN];
static gtw_state_t gtw_state = GTW_RESET_STATE;
static int eth_dsc_n = 0, ser1_dsc_n = 0, ser2_dsc_n = 0;
static char *uid_ip_eth_table[MAX_CONN + 1] = {NULL};
static int conn_num = 0;
static bool end_of_init = false;
static uint64_t fault_time = 0;
static uint64_t init_time = 0;
const mb_descr_type_t SCALED_IEC[] = {PARAM_TYPE_U16_AB, PARAM_TYPE_U16_BA};
const mb_descr_type_t NORMAL_IEC[] = {PARAM_TYPE_I16_AB, PARAM_TYPE_I16_BA};
const mb_descr_type_t FLOAT_IEC[] = {PARAM_TYPE_FLOAT_ABCD, PARAM_TYPE_FLOAT_CDAB, PARAM_TYPE_FLOAT_BADC, PARAM_TYPE_FLOAT_DCBA};
const mb_descr_type_t TOTAL_IEC[] = {PARAM_TYPE_U32_ABCD, PARAM_TYPE_U32_CDAB, PARAM_TYPE_U32_BADC, PARAM_TYPE_U32_DCBA};
static const char *TAG = "GTW_PARAM";

static uint32_t get_connect_ip(int pid)
{
    int idx = RamCfg.CfgA[pid].mb_dev_id >> 8;
    idx &= 0x0f;
    uint32_t *ip_addr = &RamCfg.iP0;
    return ip_addr[idx];
}

static uint16_t get_connect_port(int pid)
{
    int idx = RamCfg.CfgA[pid].mb_dev_id >> 8;
    idx &= 0x0f;
    uint16_t *port = &RamCfg.eP0;
    return port[idx];
}

static mb_descr_type_t get_mb_type(int pid)
{
    int swap_idx = RamCfg.CfgA[pid].swap_data;
    iec_data_t iec_type = RamCfg.CfgA[pid].iec_data_type;
    switch (iec_type)
    {
    case NORMALIZED_VAL:
    case STEP_POSITION:
        return NORMAL_IEC[swap_idx];
    case SCALED_VAL:
        return SCALED_IEC[swap_idx];
        break;
    case FLOATING_VAL:
        return FLOAT_IEC[swap_idx];
    case INTEGR_TOTALS:
    case BIT_STRING_32:
        return TOTAL_IEC[swap_idx];
    default:
        return PARAM_TYPE_U8;
    }
}

esp_err_t gtw_param_init(void)
{
    int len = RamCfg.tLen;
    if (!len)
        return ESP_ERR_NOT_FOUND;
    for (int i = 0; i < len; i++)
    {
        mb_connect_t mb_connect = get_connect_type(i);
        if (mb_connect == ETH_CONN)
            eth_dsc_n++;
        else if (mb_connect == SER2_CONN)
            ser2_dsc_n++;
        else
            ser1_dsc_n++;
    }
    int s1i = 0;
    int s2i = ser1_dsc_n;
    int ei = s2i + ser2_dsc_n;
    for (int i = 0; i < len; i++)
    {
        int j, id;
        mb_connect_t mb_connect = get_connect_type(i);
        if (mb_connect == SER1_CONN)
        {
            id = j = s1i;
            s1i++;
        }
        else if (mb_connect == SER2_CONN)
        {
            j = s2i;
            id = j - ser1_dsc_n;
            s2i++;
        }
        else // if (mb_connect == ETH_CONN)
        {
            j = ei;
            id = j - ser1_dsc_n - ser2_dsc_n;
            ei++;
        }
        mb_parameter_descriptor_t *pprm = &prm_dsc[j];
        pprm->cid = id;
        pprm->param_offset = i;
        int obj_num = RamCfg.CfgA[i].iec_obj_num;
        pprm->mb_slave_addr = RamCfg.CfgA[i].mb_dev_id;
        if (mb_connect > SER2_CONN) /* Modbus TCP connection */
        {
            if (conn_num == MAX_CONN)
                return ESP_ERR_INVALID_ARG;
            uint32_t ip = get_connect_ip(i);
            uint16_t port = get_connect_port(i);
            char *uid_ip_str = NULL;
            asprintf(&uid_ip_str, "%d;%lu.%lu.%lu.%lu;%d", pprm->mb_slave_addr,
                ip & 0xff, (ip >> 8) & 0xff, (ip >> 16) & 0xff, (ip >> 24) & 0xff, port);
            ESP_LOGI(TAG, "Slave Addr: %d IP: %s", pprm->mb_slave_addr, uid_ip_str);
            int conn;
            for (conn = 0; conn < MAX_CONN; conn++)
            {
                if (uid_ip_eth_table[conn] == NULL)
                {
                    uid_ip_eth_table[conn] = uid_ip_str;
                    conn_num++;
                    break;
                }
                if (strcmp(uid_ip_str, uid_ip_eth_table[conn]) == 0)
                    break;
            }
            if (conn == MAX_CONN)
                return ESP_ERR_INVALID_ARG;
        }
        pprm->mb_reg_start = RamCfg.CfgA[i].mb_reg_addr;
        *((float *)&pprm->param_opts.min) = RamCfg.CfgA[i].low_limit;
        *((float *)&pprm->param_opts.max) = RamCfg.CfgA[i].high_limit;
        *((float *)&pprm->param_opts.step) = RamCfg.CfgA[i].threshold;
        switch (RamCfg.CfgA[i].mb_function)
        {
        case COIL_STATUS_RW:
            pprm->access = PAR_PERMS_READ_WRITE;
            goto coil_status;
        case COIL_STATUS_RO:
            pprm->access = PAR_PERMS_READ;
            goto coil_status;
        case COIL_STATUS_WO:
            pprm->access = PAR_PERMS_WRITE;
        coil_status:
            pprm->param_key = "COIL_STATUS";
            pprm->mb_param_type = MB_PARAM_COIL;
            goto bin_var_common;
        case INPUT_STATUS_RO:
            pprm->param_key = "INPUT_STATUS";
            pprm->mb_param_type = MB_PARAM_DISCRETE;
            pprm->access = PAR_PERMS_READ;
        bin_var_common:
            size_t b_size;
            if (RamCfg.CfgA[i].iec_data_type == SINGLE_POINT)
            {
                pprm->param_size = b_size = obj_num / BIT_REG_LEN + ((obj_num % BIT_REG_LEN) > 0);
                pprm->param_units = "IEC_TYPE_SINGLE_POINT";
                pprm->param_type = PARAM_TYPE_BIN;
                pprm->mb_size = obj_num;
            }
            else
            {
                pprm->param_size = obj_num / DP_REG_LEN + ((obj_num % DP_REG_LEN) > 0);
                b_size = obj_num;
                pprm->param_units = "IEC_TYPE_DOUBLE_POINT";
                pprm->param_type = PARAM_TYPE_U8;
                pprm->mb_size = obj_num * DP_BIT_SIZE;
            }
            gtw_elm[j].old_val = &mb_old[o_cnt];
            gtw_elm[j].tmp_val = &mb_tmp[o_cnt];
            o_cnt += b_size;
            goto next_id;
        case HOLDING_REG_RW:
            pprm->access = PAR_PERMS_READ_WRITE;
            goto holding_status;
        case HOLDING_REG_RO:
            pprm->access = PAR_PERMS_READ;
            goto holding_status;
        case HOLDING_REG_WO:
            pprm->access = PAR_PERMS_WRITE;
        holding_status:
            pprm->param_key = "HOLDING_REG";
            pprm->mb_param_type = MB_PARAM_HOLDING;
            pprm->access = PAR_PERMS_READ_WRITE;
            goto reg_var_common;
        case INPUT_REG_RO:
            pprm->param_key = "INPUT_REG";
            pprm->mb_param_type = MB_PARAM_INPUT;
            pprm->access = PAR_PERMS_READ;
        reg_var_common:
            mb_descr_type_t mb_type = get_mb_type(i);
            pprm->param_type = mb_type;
            switch (mb_type)
            {
            case PARAM_TYPE_I16_AB:
                pprm->param_units = "PARAM_TYPE_I16_AB";
                goto i16_common;
            case PARAM_TYPE_I16_BA:
                pprm->param_units = "PARAM_TYPE_I16_BA";
            i16_common:
                pprm->param_size = PARAM_SIZE_I16;
                break;
            case PARAM_TYPE_U16_AB:
                pprm->param_units = "PARAM_TYPE_U16_AB";
                goto u16_common;
            case PARAM_TYPE_U16_BA:
                pprm->param_units = "PARAM_TYPE_U16_BA";
            u16_common:
                pprm->param_size = PARAM_SIZE_U16;
                break;
            case PARAM_TYPE_U32_ABCD:
                pprm->param_units = "PARAM_TYPE_U32_ABCD";
                goto u32_common;
            case PARAM_TYPE_U32_CDAB:
                pprm->param_units = "PARAM_TYPE_U32_CDAB";
                goto u32_common;
            case PARAM_TYPE_U32_BADC:
                pprm->param_units = "PARAM_TYPE_U32_BADC";
                goto u32_common;
            case PARAM_TYPE_U32_DCBA:
                pprm->param_units = "PARAM_TYPE_U32_DCBA";
            u32_common:
                pprm->param_size = PARAM_SIZE_U32;
                break;
            case PARAM_TYPE_FLOAT_ABCD:
                pprm->param_units = "PARAM_TYPE_FLOAT_ABCD";
                goto f_common;
            case PARAM_TYPE_FLOAT_CDAB:
                pprm->param_units = "PARAM_TYPE_FLOAT_CDAB";
                goto f_common;
            case PARAM_TYPE_FLOAT_BADC:
                pprm->param_units = "PARAM_TYPE_FLOAT_BADC";
                goto f_common;
            case PARAM_TYPE_FLOAT_DCBA:
                pprm->param_units = "PARAM_TYPE_FLOAT_DCBA";
            f_common:
                pprm->param_size = PARAM_SIZE_FLOAT;
                break;
            default:
                goto param_err;
            }
            gtw_elm[j].old_val = &mb_old[o_cnt];
            gtw_elm[j].tmp_val = &mb_tmp[o_cnt];
            o_cnt += sizeof(float) * obj_num;
            if (pprm->param_size > BYTES_IN_REG)
                b_size = pprm->param_size;
            else
                b_size = BYTES_IN_REG;
            b_size *= obj_num;
            pprm->param_size = b_size;
            b_size /= BYTES_IN_REG;
            if (b_size > REG_IN_CDU)
                goto param_err;
            pprm->mb_size = b_size;
            break;
        default:
            goto param_err;
        }
    next_id:
        ESP_LOGI(TAG, "pprm->cid: %d, prm_dsc [j]: %u, Slave ID: %d, pprm->mb_param_type: %d, pprm->mb_reg_start: #%d, obj_num: %d, Connect Type: %d",
                 pprm->cid,
                 j,
                 pprm->mb_slave_addr,
                 pprm->mb_param_type,
                 pprm->mb_reg_start,
                 obj_num,
                 mb_connect);
    }
    if ((iecQueueHdl = xQueueCreate(IEC_QUEUE_LEN, sizeof(queue_elm_t))) != NULL)
        vQueueAddToRegistry(iecQueueHdl, "IecTcpEventQueue");
    else
        goto param_err;
    if ((mbQueueHdl = xQueueCreate(MB_QUEUE_LEN, sizeof(queue_elm_t))) != NULL)
        vQueueAddToRegistry(mbQueueHdl, "MBEventQueue");
    else
        goto param_err;
    gtwSemaphore = xSemaphoreCreateMutexStatic(&xMutexBuffer);
    return ESP_OK;
param_err:
    return ESP_ERR_INVALID_ARG;
}

int get_obj_elm(int pid)
{
    return RamCfg.CfgA[pid].iec_obj_num;
}

float get_old_val(int cid, int idx)
{
    return *((float *)(gtw_elm[cid].old_val) + idx);
}

float *get_old_val_ptr(int cid, int idx)
{
    return ((float *)(gtw_elm[cid].old_val) + idx);
}

void set_old_val(int cid, int idx, float val)
{
    *((float *)(gtw_elm[cid].old_val) + idx) = val;
}

DoublePointValue get_old_dpoint(int cid, int idx)
{
    return gtw_elm[cid].old_val[idx];
}

void set_old_dpoint(int cid, int idx, DoublePointValue dp_val)
{
   gtw_elm[cid].old_val[idx] = dp_val;
}

bool get_tmp_state(int cid, int idx)
{
    int byte_n = idx / BIT_REG_LEN;
    int mask = 1 << (idx % BIT_REG_LEN);
    int byte_val = gtw_elm[cid].tmp_val[byte_n];
    return (byte_val & mask) != 0;
}

void set_tmp_state(int cid, int idx, bool state)
{
    int byte_n = idx / BIT_REG_LEN;
    int mask = 1 << (idx % BIT_REG_LEN);
    int byte_val = gtw_elm[cid].tmp_val[byte_n];
    if (state)
        byte_val |= mask;
    else
        byte_val &= ~mask;
    gtw_elm[cid].tmp_val[byte_n] = byte_val;
}

bool get_old_state(int cid, int idx)
{
    int byte_n = idx / BIT_REG_LEN;
    int mask = 1 << (idx % BIT_REG_LEN);
    int byte_val = gtw_elm[cid].old_val[byte_n];
    return (byte_val & mask) != 0;
}

void set_old_state(int cid, int idx, bool state)
{
    int byte_n = idx / BIT_REG_LEN;
    int mask = 1 << (idx % BIT_REG_LEN);
    int byte_val = gtw_elm[cid].old_val[byte_n];
    if (state)
        byte_val |= mask;
    else
        byte_val &= ~mask;
    gtw_elm[cid].old_val[byte_n] = byte_val;
}

void set_old_bits(int cid, int idx, uint8_t bits)
{
    gtw_elm[cid].old_val[idx] = bits;
}

bool is_val_init(int cid, int pid)
{
    if (gtw_elm[cid].init == false)
    {
        gtw_elm[cid].init = true;
        if (is_normalized(pid) || is_scaled_val(pid) || is_floating(pid))
            gtw_elm[cid].send_cnt = get_time_ms();
        return false;
    }
    return true;
}

bool is_fault_time_out(void)
{
    if ((get_time_ms() - fault_time) >= RamCfg.tFault)
        return true;
    return false;
}

void set_init_time(void)
{
    init_time = get_time_ms();
}

bool is_init_time_out(void)
{
    if ((get_time_ms() - init_time) >= RamCfg.mbInDel)
        return true;
    return false;
}

void set_fault_time(void)
{
    fault_time = get_time_ms();
}

bool is_poll_time(int cid, int pid)
{
    if ((get_time_ms() - gtw_elm[cid].pool_time) >= RamCfg.CfgA[pid].mb_poll_ms)
        return true;
    return false;
}

bool is_send_time(int cid, int pid)
{
    uint32_t cycle_time;
    if (is_normalized(pid))
        cycle_time = RamCfg.tNorm;
    else if (is_scaled_val(pid))
        cycle_time = RamCfg.tScal;
    else if (is_floating(pid))
        cycle_time = RamCfg.tFloat;
    else
        return false;
    if (cycle_time == 0)
        return false;
    uint64_t time_new = get_time_ms();
    uint64_t time_ms = time_new - gtw_elm[cid].send_cnt;
    if (time_ms > cycle_time * 1000)
    {
        gtw_elm[cid].send_cnt = time_new;
        return true;
    }
    return false;
}

mb_commands_t get_mb_command(mb_descr_type_t param_type, int num)
{
    if (param_type == PARAM_TYPE_U8)
        return MB_FUNC_WRITE_MULTIPLE_COILS;
    if (param_type == PARAM_TYPE_BIN)
    {
        if (num > 1)
            return MB_FUNC_WRITE_MULTIPLE_COILS;
        else
            return MB_FUNC_WRITE_SINGLE_COIL;
    }
    if (num > 1)
        return MB_FUNC_WRITE_MULTIPLE_REGISTERS;
    return MB_FUNC_WRITE_REGISTER;
}

int iec_to_mb(mb_descr_type_t param_type, uint8_t *buff, uint32_t *pdata, int num, bool common_data, float *step_data)
{
    void *pdest, *psrc;
    int len = 0, ret = 0;
    uint32_t bin_val;
    int16_t step_val;
    for (int i = 0; i < num; i++)
    {
        pdest = &buff[len];
        int k = common_data ? 0 : i;
        psrc = &pdata[k];
        if ((step_data != NULL) && (!common_data))
        {
            StepCommandValue step_cmd = (StepCommandValue)pdata[i];
            step_val = step_data[i];
            if (step_cmd == IEC60870_STEP_LOWER)
            {
                if (step_val > INT16_MIN)
                    step_val--;
            }
            else if (step_cmd == IEC60870_STEP_HIGHER)
            {
                if (step_val < INT16_MAX)
                    step_val++;
            }
            else
                return 0;
            pdata[i] = step_val;
        }
        int shift, idx;
        switch (param_type)
        {
        case PARAM_TYPE_I16_AB:
            mb_set_int16_ab((val_16_arr *)pdest, *(int16_t *)psrc);
            goto add_16_bit;
        case PARAM_TYPE_I16_BA:
            mb_set_int16_ba((val_16_arr *)pdest, *(int16_t *)psrc);
            goto add_16_bit;
        case PARAM_TYPE_U16_AB:
            mb_set_uint16_ab((val_16_arr *)pdest, *(uint16_t *)psrc);
            goto add_16_bit;
        case PARAM_TYPE_U16_BA:
            mb_set_uint16_ba((val_16_arr *)pdest, *(uint16_t *)psrc);
        add_16_bit:
            ret += sizeof(int16_t) / BYTES_IN_REG;
            len += sizeof(int16_t);
            break;
        case PARAM_TYPE_U32_ABCD:
            mb_set_uint32_abcd((val_32_arr *)pdest, *(uint32_t *)psrc);
            goto add_32_bit;
        case PARAM_TYPE_U32_CDAB:
            mb_set_uint32_cdab((val_32_arr *)pdest, *(uint32_t *)psrc);
            goto add_32_bit;
        case PARAM_TYPE_U32_BADC:
            mb_set_uint32_badc((val_32_arr *)pdest, *(uint32_t *)psrc);
            goto add_32_bit;
        case PARAM_TYPE_U32_DCBA:
            mb_set_uint32_dcba((val_32_arr *)pdest, *(uint32_t *)psrc);
            goto add_32_bit;
        case PARAM_TYPE_FLOAT_ABCD:
            mb_set_float_abcd((val_32_arr *)pdest, *(float *)psrc);
            goto add_32_bit;
        case PARAM_TYPE_FLOAT_CDAB:
            mb_set_float_cdab((val_32_arr *)pdest, *(float *)psrc);
            goto add_32_bit;
        case PARAM_TYPE_FLOAT_BADC:
            mb_set_float_badc((val_32_arr *)pdest, *(float *)psrc);
            goto add_32_bit;
        case PARAM_TYPE_FLOAT_DCBA:
            mb_set_float_dcba((val_32_arr *)pdest, *(float *)psrc);
        add_32_bit:
            ret += sizeof(int32_t) / BYTES_IN_REG;
            len += sizeof(int32_t);
            break;
        case PARAM_TYPE_U8: /* Double point IEC object */
            if (common_data)
                bin_val = pdata[0] ? IEC60870_DOUBLE_POINT_ON : IEC60870_DOUBLE_POINT_OFF;
            else
                bin_val = pdata[i];
            idx = i / DP_REG_LEN;
            shift = (i % DP_REG_LEN) * DP_BIT_SIZE;
            ret = len = (i + 1) * DP_BIT_SIZE;
            goto bits_load;
        case PARAM_TYPE_BIN: /* Single point IEC object */
            if (num == 1)
            {
                *(uint16_t *)pdest = *(int *)psrc ? 0xff00 : 0x0000;
                return sizeof(uint16_t);
            }
            bin_val = pdata[k] ? true : false;
            idx = i / BIT_REG_LEN;
            shift = i % BIT_REG_LEN;
            ret = len = i + 1;
        bits_load:
            if (!shift)
                buff[idx] = 0;
            buff[idx] |= bin_val << shift;
            break;
        default: /* Error */
            return 0;
        }
    }
    return ret;
}

bool is_mb_timeout(int cid)
{
    return (gtw_elm[cid].to_cnt >= RamCfg.mbRetry);
}

void clr_mb_error(int cid)
{
    if (gtw_state != MB_FAULT_PROCESS)
        gtw_elm[cid].to_cnt = 0;
    gtw_elm[cid].quality = QUALITY_GOOD;
    gtw_elm[cid].pool_time = get_time_ms();
}

bool inc_mb_error(int cid)
{
    if (gtw_elm[cid].to_cnt < RamCfg.mbRetry)
    {
        gtw_elm[cid].to_cnt++;
        return false;
    }
    gtw_elm[cid].quality = QUALITY_NON_TOPICAL;
    return true;
}

void set_gtw_state(gtw_state_t state)
{
    gtw_state = state;
}

gtw_state_t get_gtw_state(void)
{
    return gtw_state;
}

float get_fault_val(int pid)
{
    return RamCfg.CfgA[pid].fault_val;
}

bool is_normalized(int pid)
{
    return (RamCfg.CfgA[pid].iec_data_type == NORMALIZED_VAL);
}

bool is_bitstring32(int pid)
{
    return (RamCfg.CfgA[pid].iec_data_type == BIT_STRING_32);
}

bool is_scaled_val(int pid)
{
    return (RamCfg.CfgA[pid].iec_data_type == SCALED_VAL);
}

bool is_step_position(int pid)
{
    return (RamCfg.CfgA[pid].iec_data_type == STEP_POSITION);
}

bool is_int_totals(int pid)
{
    return (RamCfg.CfgA[pid].iec_data_type == INTEGR_TOTALS);
}

bool is_floating(int pid)
{
    return (RamCfg.CfgA[pid].iec_data_type == FLOATING_VAL);
}

uint32_t get_old_bit_string_32(int cid, int idx)
{
    return *((uint32_t *)(gtw_elm[cid].old_val) + idx);
}
void set_old_bit_string_32(int cid, int idx, uint32_t bs32)
{
    *((uint32_t *)(gtw_elm[cid].old_val) + idx) = bs32;
}

bool is_double_point(int pid)
{
    return (RamCfg.CfgA[pid].iec_data_type == DOUBLE_POINT);
}

int get_param_num(int conn_type)
{
    if (conn_type == SER1_CONN)
        return ser1_dsc_n;
    return eth_dsc_n;
}

int get_param_first(int conn_type)
{
    if (conn_type == SER1_CONN)
        return 0;
    return ser1_dsc_n + ser2_dsc_n;
}

mb_parameter_descriptor_t *get_param_desc(int conn_type)
{
    if (conn_type == SER1_CONN)
        return prm_dsc;
    return &prm_dsc[ser1_dsc_n + ser2_dsc_n];
}

int get_prm_id(int pid)
{
    int conn_type = get_connect_type(pid);
    int param_num = get_param_num(conn_type);
    int id = get_param_first(conn_type);
    for (int i = 0; i < param_num; i++, id++)
    {
        if (prm_dsc[id].param_offset == pid)
            return i;
    }
    return PRM_NOT_FOUND;
}

int get_prm_cid(int pid)
{
    int conn_type = get_connect_type(pid);
    int param_num = get_param_num(conn_type);
    int id = get_param_first(conn_type);
    for (int i = 0; i < param_num; i++, id++)
    {
        if (prm_dsc[id].param_offset == pid)
            return id;
    }
    return PRM_NOT_FOUND;
}

bool is_mb_connect_use(int conn_type)
{
    if (conn_type == SER1_CONN)
        return ser1_dsc_n > 0;
    return eth_dsc_n > 0;
}

mb_connect_t get_connect_type(int pid)
{
    uint16_t dev_id = RamCfg.CfgA[pid].mb_dev_id;
    if (dev_id >= ETH_MB_CONNECT)
        return ETH_CONN;
    if (dev_id & SER2_MASK)
        return SER2_CONN;
    return SER1_CONN;
}

char **get_ip_table(void)
{
    return uid_ip_eth_table;
}

int get_db_len(void)
{
    return RamCfg.tLen;
}

iec_data_t get_iec_data_type(int pid)
{
    return RamCfg.CfgA[pid].iec_data_type;
}

int get_iec_obj_addr(int pid)
{
    return RamCfg.CfgA[pid].iec_obj_addr;
}

int get_iec_obj_num(int pid)
{
    return RamCfg.CfgA[pid].iec_obj_num;
}

uint8_t get_iec_group(int pid)
{
    return RamCfg.CfgA[pid].iec_group + BY_STATION;
}

uint8_t get_iec_cnt_group(int pid)
{
    if (!RamCfg.CfgA[pid].iec_group)
        return CNT_GENERAL;
    return RamCfg.CfgA[pid].iec_group;
}

iec_time_t interrogation_time_type(void)
{
    return RamCfg.tIrr;
}

iec_time_t measured_time_type(void)
{
    return RamCfg.tMval;
}

iec_time_t spontanius_time_type(void)
{
    return RamCfg.tEvt;
}

void set_cause(int cid, gtw_cause_t cause)
{
    gtw_elm[cid].cause = cause;
}

gtw_cause_t get_cause(int cid)
{
    return gtw_elm[cid].cause;
}

void set_quality(int cid, gtw_quality_t quality)
{
    gtw_elm[cid].quality = quality;
}

gtw_quality_t get_quality(int cid)
{
    uint8_t quality = gtw_elm[cid].quality;
    if ((get_time_ms() - gtw_elm[cid].pool_time) / 1000 >= RamCfg.tPSt)
        quality |= QUALITY_NON_TOPICAL;
    return quality;
}

int get_common_addr(void)
{
    return RamCfg.AsduAdr;
}

bool is_cmd_term(void)
{
    return (RamCfg.actTerm & CMD_MASK) != 0;
}
bool is_cse_term(void)
{
    return (RamCfg.actTerm & CSE_MASK) != 0;
}

bool get_obj_id(int obj, int *pid, int *idx)
{
    for (int i = 0; i < RamCfg.tLen; i++)
    {
        int obj_addr = RamCfg.CfgA[i].iec_obj_addr;
        if (obj >= obj_addr)
        {
            if (obj < obj_addr + RamCfg.CfgA[i].iec_obj_num)
            {
                *pid = i;
                *idx = obj - obj_addr;
                return true;
            }
        }
    }
    return false;
}

void set_select_time(int cid)
{
    gtw_elm[cid].sel_time = get_time_ms() / 1000;
}

void clr_select_time(int cid)
{
    gtw_elm[cid].sel_time = 0;
}

bool is_select_time(int cid)
{
    return gtw_elm[cid].sel_time > 0;
}

bool is_select_time_out(int cid)
{
    if (!gtw_elm[cid].sel_time) return true;
    if ((get_time_ms() / 1000 - gtw_elm[cid].sel_time) >= RamCfg.tSel)
    {
        gtw_elm[cid].sel_time = 0;
        return true;
    }
    return false;
}

uint64_t get_pool_time(int cid)
{
    return gtw_elm[cid].pool_time;
}

void set_pool_time(int cid, uint64_t time_ms)
{
    gtw_elm[cid].pool_time = time_ms;
}

void end_of_init_set(void)
{
    end_of_init = true;
}

bool is_end_of_init(void)
{
    return end_of_init;
}

DoublePointValue get_tmp_dpoint(int cid, int idx)
{
    return gtw_elm[cid].tmp_val[idx];
}

void set_tmp_dpoint(int cid, int idx, DoublePointValue dp_val)
{
    gtw_elm[cid].tmp_val[idx] = dp_val;
}

uint32_t get_tmp_bit_string_32(int cid, int idx)
{
    return *((uint32_t *)(gtw_elm[cid].tmp_val) + idx);
}

void set_tmp_bit_string_32(int cid, int idx, uint32_t bs32)
{
    *((uint32_t *)(gtw_elm[cid].tmp_val) + idx) = bs32;
}

void set_tmp_bits(int cid, int idx, uint8_t bits)
{
    gtw_elm[cid].tmp_val[idx] = bits;
}

float get_tmp_val(int cid, int idx)
{
    return *((float *)(gtw_elm[cid].tmp_val) + idx);
}

float *get_tmp_val_ptr(int cid, int idx)
{
    return ((float *)(gtw_elm[cid].tmp_val) + idx);
}

void set_tmp_val(int cid, int idx, float val)
{
    *((float *)(gtw_elm[cid].tmp_val) + idx) = val;
}

void set_fault_state(int cid)
{
    gtw_elm[cid].to_cnt = MB_FAULT_STATE;
}

bool is_fault_state(int cid)
{
    return (gtw_elm[cid].to_cnt == MB_FAULT_STATE);
}

bool is_fault_none(int pid)
{
    return (RamCfg.CfgA[pid].fault_val > HIGH_VAL_MAX);
}

uint32_t get_time_sync(void)
{
    return RamCfg.tSync;
}

uint32_t get_time_swith(void)
{
    return RamCfg.tSwith;
}

TickType_t get_between_del(void)
{
    return (RamCfg.tBetw / portTICK_PERIOD_MS);
}