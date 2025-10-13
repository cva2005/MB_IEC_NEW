#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "mbcontroller.h"
#include "mb_proto.h"
#include "sdkconfig.h"
#include "config.h"
#include "timer_1ms.h"
#include "gtw_params.h"
#include "webserver.h"
#include "gtw_mb_master.h"
#include "iec_controller.h"
#include "port_common.h"
#include "esp_sleep.h"
#include "gpio_drv.h"

#define MS_IN_SEC 						(1000)
#define WAIT_CONNECT_MS 				(10)
#define	INIT_DELAY_TICS					(WAIT_CONNECT_MS / portTICK_PERIOD_MS)
#define UPDATE_CIDS_TIMEOUT_MS          (1000)
#define UPDATE_CIDS_TIMEOUT_TICS        (UPDATE_CIDS_TIMEOUT_MS / portTICK_PERIOD_MS)
#define CHANGE_MASK						(0xff)
#define FMB_EXT_TYPE_SUPPORT			(CONFIG_FMB_EXT_TYPE_SUPPORT)
#define ELM_MAX_SIZE					(4)
#define MB_BUFF_LEN 					(BYTES_IN_CDU)
#if MB_BUFF_LEN > 250
#error "ModBus Data Buffer Size Error!"
#endif
#define THRESHOLD (*((float *)&pprm->param_opts.step))
#define MAX_LIMIT (*((float *)&pprm->param_opts.max))
#define MIN_LIMIT (*((float *)&pprm->param_opts.min))
#define IS_THRESHOLD_USE() (THRESHOLD > 0)
#define IS_MAX_LIMIT_USE() (MAX_LIMIT <= HIGH_VAL_MAX)
#define IS_MIN_LIMIT_USE() (MIN_LIMIT >= LOW_VAL_MIN)

static const char *TAG = "MB_MASTER_DRIVER";
static void *master_handle[MB_NUM] = {NULL};
static uint64_t reload_time = 0;
static const char *gtw_state_str[] = {
	"GTW_RESET_STATE",
	"MB_INIT_DELAY",
	"IEC_CONNECTION_WAIT",
	"MB_FAULT_WAIT",
	"MB_FAULT_PROCESS",
	"IEC_CONNECTION_ESTABLISHED",
	"MB_FAULT_STATE"};

static void reload_time_set(void)
{
	if (get_time_swith())
		reload_time = get_time_ms();
}

static bool is_reload_time_out(void)
{
	uint32_t ts = get_time_swith();
	if (ts == 0)
		return false;
	if ((get_time_ms() - reload_time) > (get_db_len() * WAIT_CONNECT_MS + ts * MS_IN_SEC))
		return true;
	return false;
}

static void load_from_queue(void)
{
	int mess = uxQueueMessagesWaiting(mbQueueHdl);
	if (!mess)
		return;
	queue_elm_t first_elm, next_elm;
	uint32_t iec_data[MB_QUEUE_LEN];
	xQueueReceive(mbQueueHdl, &first_elm, QUEUE_NO_WAIT);
	iec_data[0] = first_elm.data;
	ESP_LOGI(TAG, "IEC message from Queue by cid #%u.", first_elm.pid);
	int num = 1;
	for (; num < mess; num++)
	{
		xQueuePeek(mbQueueHdl, &next_elm, QUEUE_NO_WAIT);
		if (next_elm.pid != first_elm.pid)
			break;
		if (next_elm.idx != (first_elm.idx + num))
			break;
		iec_data[num] = next_elm.data;
		xQueueReceive(mbQueueHdl, &next_elm, QUEUE_NO_WAIT);
	}
	ESP_LOGI(TAG, "Load %u ModBus Objects", num);
	const mb_parameter_descriptor_t *pprm = NULL;
	int pid = first_elm.pid;
	void *handle = master_handle[get_connect_type(pid)];
	esp_err_t err = mbc_master_get_cid_info(handle, get_prm_id(pid), &pprm);
	if ((err != ESP_ERR_NOT_FOUND) && (pprm != NULL) && (pprm->access != PAR_PERMS_READ))
	{
		uint8_t mb_buff[MB_BUFF_LEN];
		mb_param_request_t request;
		int cid = pprm->cid;
		float *step_cmd_data = NULL;
		if (is_step_position(pid))
			step_cmd_data = get_tmp_val_ptr(cid, 0);
		request.reg_size = iec_to_mb(pprm->param_type, mb_buff, iec_data, num, false, step_cmd_data);
		if (!request.reg_size)
			return;
		request.slave_addr = pprm->mb_slave_addr;
		int elm = get_obj_elm(pid);
		int mb_size = pprm->mb_size / elm;
		request.command = get_mb_command(pprm->param_type, num);
		request.reg_start = pprm->mb_reg_start;
		if (request.command == MB_FUNC_WRITE_MULTIPLE_COILS) /* Double point IEC object */
			request.reg_start += DP_BIT_SIZE * first_elm.idx;
		else
			request.reg_start += mb_size * first_elm.idx;
		ESP_LOGD(TAG, "IEC Addr: %d, Objects: %d, Slave ID: %d, Function: %d, Register: #%d, Size: %d, Data: %ld",
				 pid,
				 num,
				 request.slave_addr,
				 request.command,
				 request.reg_start,
				 request.reg_size,
				 iec_data[0]);
		if (mbc_master_send_request(handle, &request, mb_buff) == ESP_OK)
		{
			if (first_elm.time_ms)
				set_pool_time(cid, first_elm.time_ms);
			clr_mb_error(cid);
		}
		else
			inc_mb_error(cid);
	}
}

void mb_master_register(void *handle, mb_connect_t conn_idx)
{
	master_handle[conn_idx] = handle;
}

static bool set_mb_fault(void *handle, int cid, const mb_parameter_descriptor_t *pprm)
{
	int pid = pprm->param_offset;
	float fval = get_fault_val(pid);
	uint32_t data_common = fval;
	uint8_t mb_buff[MB_BUFF_LEN];
	mb_param_request_t request;
	int elm = get_obj_elm(pid);
	float *step_cmd_data = NULL;
	if (is_step_position(pid))
		step_cmd_data = get_old_val_ptr(cid, 0);
	request.reg_size = iec_to_mb(pprm->param_type, mb_buff, &data_common, elm, true, step_cmd_data);
	request.slave_addr = pprm->mb_slave_addr;
	request.command = get_mb_command(pprm->param_type, elm);
	request.reg_start = pprm->mb_reg_start;
	ESP_LOGI(TAG, "IEC Addr: %d, Objects: %u, Slave ID: %d, Function: %d, Register: #%d, Size: %d, Data: %ld",
				cid,
				elm,
				request.slave_addr,
				request.command,
				request.reg_start,
				request.reg_size,
				data_common);
	vTaskDelay(get_between_del());
	if (mbc_master_send_request(handle, &request, mb_buff) == ESP_OK)
	{
		for (int i = 0; i < elm; i++)
		{
			if (request.command == MB_FUNC_WRITE_SINGLE_COIL ||
				request.command == MB_FUNC_WRITE_MULTIPLE_COILS)
			{
				if (pprm->param_type == PARAM_TYPE_U8) /* Double point IEC object */
					set_old_dpoint(cid, i, data_common ? IEC60870_DOUBLE_POINT_ON : IEC60870_DOUBLE_POINT_OFF);
				else
					set_old_state(cid, i, data_common);
			}
			else
				set_old_val(cid, i, fval);
		}
		return true;
	}
	return false;
}

static float data_to_float(mb_descr_type_t param_type, uint8_t *pdata)
{
	switch (param_type)
	{
	case PARAM_TYPE_I16_AB:
	case PARAM_TYPE_I16_BA:
		return *(int16_t *)pdata;
	case PARAM_TYPE_U16_AB:
	case PARAM_TYPE_U16_BA:
		return *(uint16_t *)pdata;
	case PARAM_TYPE_I32_ABCD:
	case PARAM_TYPE_I32_CDAB:
	case PARAM_TYPE_I32_BADC:
	case PARAM_TYPE_I32_DCBA:
		return *(int32_t *)pdata;
	case PARAM_TYPE_U32_ABCD:
	case PARAM_TYPE_U32_CDAB:
	case PARAM_TYPE_U32_BADC:
	case PARAM_TYPE_U32_DCBA:
		return *(uint32_t *)pdata;
	case PARAM_TYPE_FLOAT_ABCD:
	case PARAM_TYPE_FLOAT_CDAB:
	case PARAM_TYPE_FLOAT_BADC:
	case PARAM_TYPE_FLOAT_DCBA:
		return *(float *)pdata;
	default:
		return NAN;
	}
}

void mb_master_operation_func(void)
{
	while (true)
	{
		init_delay_wait:
		for (int dev = SER1_CONN; dev < MB_NUM; dev++)
		{
			int cid = get_param_first(dev);
			int desc_num = get_param_num(dev);
			void *handle = master_handle[dev];
			if (handle == NULL)
				continue;
			for (int id = 0; id < desc_num; id++, cid++)
			{
				const mb_parameter_descriptor_t *pprm = NULL;
				mbc_master_get_cid_info(handle, id, &pprm);
				int pid = pprm->param_offset;
				gtw_state_t state = get_gtw_state();
				if (iec_slave_is_connected())
				{
					set_gtw_state(IEC_CONNECTION_ESTABLISHED);
					goto master_pool;
				}
				else if (state == IEC_CONNECTION_ESTABLISHED)
				{
					state = IEC_CONNECTION_WAIT;
				}
				switch (state)
				{
				case GTW_RESET_STATE:
					set_gtw_state(MB_INIT_DELAY);
					set_init_time();
				case MB_INIT_DELAY:
					if (is_init_time_out())
					{
						state = IEC_CONNECTION_WAIT;
						goto set_state;
					}
					vTaskDelay(INIT_DELAY_TICS);
					goto init_delay_wait;
				case IEC_CONNECTION_WAIT:
					set_fault_time();
					state = MB_FAULT_WAIT;
					goto set_state;
				case MB_FAULT_WAIT:
					if (is_fault_time_out())
					{
						state = MB_FAULT_PROCESS;
						goto set_state;
					}
					break;
				case MB_FAULT_PROCESS:
					if (!is_fault_state(cid))
					{
						if (is_fault_none(pid))
							goto save_fault_state;
						else if (set_mb_fault(handle, cid, pprm))
						{
						save_fault_state:
							set_fault_state(cid);
							reload_time_set();
						}
					}
					if (is_reload_time_out())
					{
						if (slave_select == SLAVE_IEC_101_SERIAL)
							slave_select = SLAVE_IEC_104_TCP;
						else
							slave_select = SLAVE_IEC_101_SERIAL;
						ESP_LOGI(TAG, "Switching to a reserved channel!");
						esp_deep_sleep(20000);
					}
					break;
				default:
				set_state:
					set_gtw_state(state);
					ESP_LOGI(TAG, "Current State: %s.", gtw_state_str[state]);
				}
			master_pool:
				if (is_end_of_init() && !is_poll_time(cid, pid))
					continue;
				vTaskDelay(get_between_del());
				bool resp_state = false, var_binary;
				mb_param_type_t mb_param_type;
				uint8_t alarm_matrix[BITS_IN_CDU] = {false};
				uint8_t data_ptr[BYTES_IN_CDU];
				uint8_t type = 0;
				int b_size = 0;
				set_quality(cid, QUALITY_GOOD);
				ESP_LOGD(TAG, "get_parameter: id=%d cid=%d dev=%d", id, cid, dev);
				led_data_tx();
				esp_err_t get_error = mbc_master_get_parameter(handle, id, data_ptr, &type);
				mb_io_count++;
				if (get_error == ESP_OK)
				{
					led_data_rx();
					clr_mb_error(cid);
					int elm = get_obj_elm(pid);
					b_size = pprm->param_size / elm;
					bool val_init = is_val_init(cid, pid);
					mb_param_type = pprm->mb_param_type;
					var_binary = (mb_param_type == MB_PARAM_COIL || mb_param_type == MB_PARAM_DISCRETE);
					if (var_binary)
					{
						if (is_double_point(pid))
						{
							for (int i = 0; i < elm; i++)
							{
								DoublePointValue new_dpoint = (data_ptr[i / DP_REG_LEN] >> ((i % DP_REG_LEN) * DP_BIT_SIZE)) & DP_BIT_MASK;
								set_tmp_dpoint(cid, i, new_dpoint);
								if (val_init)
								{
									DoublePointValue old_dpoint = get_old_dpoint(cid, i);
									if (new_dpoint != old_dpoint)
									{
										alarm_matrix[i] = resp_state = true;
										ESP_LOGI(TAG, "Characteristic #%d[%d] %s %s(%d) value = (%d) read successful.",
												 (int)pprm->cid,
												 i,
												 (char *)pprm->param_key,
												 (char *)pprm->param_units,
												 pprm->param_type,
												 new_dpoint);
										goto init_dp_value;
									}
								}
								else
								{
								init_dp_value:
									set_old_dpoint(cid, i, new_dpoint);
								}
							}
						}
						else /* Single Point Type */
						{
							if (val_init)
							{
								for (int i = 0; i < elm; i++)
								{
									int byte_n = i / BIT_REG_LEN;
									int bit_msk = 1 << (i % BIT_REG_LEN);
									bool new_state = (data_ptr[byte_n] & bit_msk) != 0;
									set_tmp_state(cid, i, new_state);
									if (new_state != get_old_state(cid, i))
									{
										alarm_matrix[i] = resp_state = true;
										set_old_state(cid, i, new_state);
										const char *rw_str = new_state ? "ON" : "OFF";
										const char *m_type = (dev == SER1_CONN) ? "SERIAL #1" : (dev == SER2_CONN) ? "SERIAL #2"
															: (dev == ETH_CONN) ? "ETHERNET TCP/IP" : "WI-FI TCP/IP";
										ESP_LOGI(TAG, "[%s] Characteristic #%d[%d] elements = %d %s (%s) new state = %s",
												 (const char *)m_type,
												 (int)pprm->cid,
												 i, elm,
												 (char *)pprm->param_key,
												 (char *)pprm->param_units,
												 (const char *)rw_str);
									}
								}
							}
							else
							{ /* first read of discrete values */
								int n_bytes = (elm / BIT_REG_LEN) + (elm % BIT_REG_LEN > 0);
								for (int i = 0; i < n_bytes; i++)
								{
									set_old_bits(cid, i, data_ptr[i]);
									set_tmp_bits(cid, i, data_ptr[i]);
								}
							}
						}
					}
					else /* ModBus Registry Values */
					{
						for (int i = 0; i < elm; i++)
						{
							if (is_bitstring32(pid))
							{
								int32_t bs32_new = *(int *)(data_ptr + i * b_size);
								set_tmp_bit_string_32(cid, i, bs32_new);
								if (val_init)
								{
									if (bs32_new != get_old_bit_string_32(cid, i))
									{
										alarm_matrix[i] = resp_state = true;
										ESP_LOGI(TAG, "Characteristic #%d[%d] %s %s(%d) value = (0x%" PRIx32 ") read successful.",
												 (int)pprm->cid,
												 i,
												 (char *)pprm->param_key,
												 (char *)pprm->param_units,
												 pprm->param_type,
												 bs32_new);
										goto init_bit_string_32;
									}
								}
								else
								{
								init_bit_string_32:
									set_old_bit_string_32(cid, i, bs32_new);
								}
							}
							else
							{
								float fdata = data_to_float(pprm->param_type, data_ptr + i * b_size);
								if (is_normalized(pid))
									fdata /= 32767.f;
								set_tmp_val(cid, i, fdata);
								if (val_init)
								{
									float old = get_old_val(cid, i);
									if (IS_THRESHOLD_USE())
									{
										if (fabs(fdata - old) > THRESHOLD)
											goto change_value;
									}
									if (IS_MAX_LIMIT_USE())
									{
										if ((fdata > MAX_LIMIT && old <= MAX_LIMIT) ||
											(fdata < MAX_LIMIT && old > MAX_LIMIT))
											goto change_value;
									}
									if (IS_MIN_LIMIT_USE())
									{
										if ((fdata < MIN_LIMIT && old >= MIN_LIMIT) ||
											(fdata > MIN_LIMIT && old < MIN_LIMIT))
										{
										change_value:
											set_quality(cid, QUALITY_OVERFLOW);
											alarm_matrix[i] = resp_state = true;
											ESP_LOGI(TAG, "Characteristic #%d[%d] %s %s(%d) value = (%f) read successful.",
													 (int)pprm->cid,
													 i,
													 (char *)pprm->param_key,
													 (char *)pprm->param_units,
													 pprm->param_type,
													 fdata);
											goto init_value;
										}
									}
								}
								else
								{
								init_value:
									set_old_val(cid, i, fdata);
								}
							}
						}
						if (resp_state)
						{
							set_cause(cid, COT_SPONTANEOUS);
							goto send_responce;
						}
						if (is_send_time(cid, pid))
						{
							memset(alarm_matrix, true, BYTES_IN_CDU);
							set_cause(cid, COT_PERIODIC);
							resp_state = true;
						}
						send_responce:
					}
				}
				else
				{
					mb_err_count++;
					led_data_off();
					ESP_LOGE(TAG, "get_parameter: id=%d cid=%d dev=%d error=(0x%x) (%s).",
							 id, cid, dev, (uint16_t)get_error, esp_err_to_name(get_error));
					inc_mb_error(cid);
				}
				if (resp_state)
				{
					ESP_LOGI(TAG, "To IEC Responde by pid #%u.", pid);
					queue_elm_t queue_elm;
					queue_elm.pid = pid;
					queue_elm.time_out = is_mb_timeout(cid);
					queue_elm.cause = get_cause(cid);
					queue_elm.quality = get_quality(cid);
					int elm = get_obj_elm(pid);
					for (int i = 0; i < elm; i++)
					{
						if (alarm_matrix[i])
						{
							queue_elm.idx = i;
							if (var_binary)
							{
								if (is_double_point(pid))
									queue_elm.data = get_tmp_dpoint(cid, i);
								else
									queue_elm.data = get_tmp_state(cid, i);
							}
							else
							{
								if (is_bitstring32(pid) || is_int_totals(pid))
								{
									queue_elm.data = (uint32_t)get_tmp_bit_string_32(cid, i);
								}
								else
								{
									float fdata = get_tmp_val(cid, i);
									queue_elm.data = *(uint32_t *)&fdata;
								}
							}
							queue_elm_t queue_del;
							if (!uxQueueSpacesAvailable(iecQueueHdl))
							{
								xQueueReceive(iecQueueHdl, &queue_del, QUEUE_NO_WAIT);
								ESP_LOGW(TAG, "Delete Old Message from Queue by TCP/IP IEC 104.");
							}
							if (xQueueSend(iecQueueHdl, &queue_elm, QUEUE_NO_WAIT) != pdTRUE)
								ESP_LOGE(TAG, "Send message in to Queue by TCP/IP IEC 104 failure.");
							vTaskDelay(get_between_del());
						}
					}
				}
				load_from_queue();
			}
		}
		end_of_init_set();
	}
	ESP_LOGI(TAG, "Destroy masters...");
	for (int i = 0; i < MB_NUM; i++)
	{
		if (master_handle[i] !=NULL)
		{
			ESP_ERROR_CHECK(mbc_master_delete(master_handle[i]));
		}
	}
}