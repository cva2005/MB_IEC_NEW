#include "dbgu_printf.h"
#include <stdio.h>
#include "cs101_slave.h"
#include "cs104_slave.h"
#include "hal_time.h"
#include "gtw_params.h"
#include "gtw_iec_slave.h"
#include "arch.h"

static sCS101_StaticASDU _asdu;
static uint8_t ioBuf[250];
static const char *TAG = "IEC_SLAVE_DRIVER";
RTC_DATA_ATTR static uint64_t iv_time_ms = 0;
static bool utc_save_need = false;

static bool is_time_IV(void)
{
	if (get_time_sync() == 0)
		return false;
	return (get_time_ms() - iv_time_ms) > (get_time_sync() * 1000 * 60);
}

void check_utc_save(void)
{
	if (utc_save_need)
	{
		utc_save_need = false;
		save_utc_copy();
		ESP_LOGI(TAG, "Save UTC Copy in to Storage");
	}
}

static void setCP24ime2a_fromCP56Time2a(CP24Time2a cp24t, CP56Time2a cp56t)
{
	CP24Time2a_setMillisecond(cp24t, CP56Time2a_getMillisecond(cp56t));
	CP24Time2a_setMinute(cp24t, CP56Time2a_getMinute(cp56t));
	CP24Time2a_setInvalid(cp24t, CP56Time2a_isInvalid(cp56t));
	CP24Time2a_setSubstituted(cp24t, CP56Time2a_isSubstituted(cp56t));
}

static InformationObject create_obj_info(int pid, int i, void *data, iec_time_t time_add)
{
	int cid = get_prm_cid(pid);
	struct sCP56Time2a cp56time;
	struct sCP24Time2a cp24time;
	int addr = get_iec_obj_addr(pid);
	bool tmp_use;
	uint32_t inp_data;
	if (data == NULL)
	{
		inp_data = 0;
		tmp_use = true;
	}
	else
	{
		tmp_use = false;
		inp_data = *(uint32_t *)data;
	}
	QualityDescriptor quality = get_quality(cid);
	InformationObject obj_info = NULL;
	if (time_add != TIME_NONE)
	{
		CP56Time2a_createFromMsTimestamp(&cp56time, get_pool_time(cid));
		CP56Time2a_setInvalid(&cp56time, is_time_IV());
	}
	if (time_add == TIME_24_BITS)
		setCP24ime2a_fromCP56Time2a(&cp24time, &cp56time);
	switch (get_iec_data_type(pid))
	{
	case SINGLE_POINT:
		bool state = tmp_use ? get_tmp_state(cid, i) : inp_data;
		if (time_add == TIME_24_BITS)
			obj_info = (InformationObject)SinglePointWithCP24Time2a_create((SinglePointWithCP24Time2a)&ioBuf,
																		   addr + i,
																		   state,
																		   quality,
																		   &cp24time);
		else if (time_add == TIME_56_BITS)
			obj_info = (InformationObject)SinglePointWithCP56Time2a_create((SinglePointWithCP56Time2a)&ioBuf,
																		   addr + i,
																		   state,
																		   quality,
																		   &cp56time);
		else /* without time stamp */
			obj_info = (InformationObject)SinglePointInformation_create((SinglePointInformation)&ioBuf,
																		addr + i,
																		state,
																		quality);
		break;
	case DOUBLE_POINT:
		DoublePointValue dp_value = tmp_use ? get_tmp_dpoint(cid, i) : inp_data;
		if (time_add == TIME_24_BITS)
			obj_info = (InformationObject)DoublePointWithCP24Time2a_create((DoublePointWithCP24Time2a)&ioBuf,
																		   addr + i,
																		   dp_value,
																		   quality,
																		   &cp24time);
		else if (time_add == TIME_56_BITS)
			obj_info = (InformationObject)DoublePointWithCP56Time2a_create((DoublePointWithCP56Time2a)&ioBuf,
																		   addr + i,
																		   dp_value,
																		   quality,
																		   &cp56time);
		else /* without time stamp */
			obj_info = (InformationObject)DoublePointInformation_create((DoublePointInformation)&ioBuf,
																		addr + i,
																		dp_value,
																		quality);
		break;
	case STEP_POSITION:
		float step_value = tmp_use ? get_tmp_val(cid, i) : *(float *)data;
		if (time_add == TIME_24_BITS)
			obj_info = (InformationObject)StepPositionWithCP24Time2a_create((StepPositionWithCP24Time2a)&ioBuf,
																			addr + i,
																			step_value,
																			false,
																			quality,
																			&cp24time);
		else if (time_add == TIME_56_BITS)
			obj_info = (InformationObject)StepPositionWithCP56Time2a_create((StepPositionWithCP56Time2a)&ioBuf,
																			addr + i,
																			step_value,
																			false,
																			quality,
																			&cp56time);
		else /* without time stamp */
			obj_info = (InformationObject)StepPositionInformation_create((StepPositionInformation)&ioBuf,
																		 addr + i,
																		 step_value,
																		 false,
																		 quality);
		break;
	case NORMALIZED_VAL:
		float norm_value = tmp_use ? get_tmp_val(cid, i) : *(float *)data;
		if (time_add == TIME_24_BITS)
			obj_info = (InformationObject)MeasuredValueNormalizedWithCP24Time2a_create((MeasuredValueNormalizedWithCP24Time2a)&ioBuf,
																					   addr + i,
																					   norm_value,
																					   quality,
																					   &cp24time);
		else if (time_add == TIME_56_BITS)
			obj_info = (InformationObject)MeasuredValueNormalizedWithCP56Time2a_create((MeasuredValueNormalizedWithCP56Time2a)&ioBuf,
																					   addr + i,
																					   norm_value,
																					   quality,
																					   &cp56time);
		else /* without time stamp */
			obj_info = (InformationObject)MeasuredValueNormalized_create((MeasuredValueNormalized)&ioBuf,
																		 addr + i,
																		 norm_value,
																		 quality);
		break;
	case SCALED_VAL:
		float scaled_value = tmp_use ? get_tmp_val(cid, i) : *(float *)data;
		if (time_add == TIME_24_BITS)
			obj_info = (InformationObject)MeasuredValueScaledWithCP24Time2a_create((MeasuredValueScaledWithCP24Time2a)&ioBuf,
																				   addr + i,
																				   scaled_value,
																				   quality,
																				   &cp24time);
		else if (time_add == TIME_56_BITS)
			obj_info = (InformationObject)MeasuredValueScaledWithCP56Time2a_create((MeasuredValueScaledWithCP56Time2a)&ioBuf,
																				   addr + i,
																				   scaled_value,
																				   quality,
																				   &cp56time);
		else /* without time stamp */
			obj_info = (InformationObject)MeasuredValueScaled_create((MeasuredValueScaled)&ioBuf,
																	 addr + i,
																	 scaled_value,
																	 quality);
		break;
	case FLOATING_VAL:
		float float_value = tmp_use ? get_tmp_val(cid, i) : *(float *)data;
		if (time_add == TIME_24_BITS)
			obj_info = (InformationObject)MeasuredValueShortWithCP24Time2a_create((MeasuredValueShortWithCP24Time2a)&ioBuf,
																				  addr + i,
																				  float_value,
																				  quality,
																				  &cp24time);
		else if (time_add == TIME_56_BITS)
			obj_info = (InformationObject)MeasuredValueShortWithCP56Time2a_create((MeasuredValueShortWithCP56Time2a)&ioBuf,
																				  addr + i,
																				  float_value,
																				  quality,
																				  &cp56time);
		else /* without time stamp */
			obj_info = (InformationObject)MeasuredValueShort_create((MeasuredValueShort)&ioBuf,
																	addr + i,
																	float_value,
																	quality);
		break;
	case BIT_STRING_32:
		uint32_t bit32str_value = tmp_use ? get_tmp_bit_string_32(cid, i) : inp_data;
		if (time_add == TIME_24_BITS)
			obj_info = (InformationObject)Bitstring32WithCP24Time2a_createEx((Bitstring32WithCP24Time2a)&ioBuf,
																			 addr + i,
																			 bit32str_value,
																			 quality,
																			 &cp24time);
		else if (time_add == TIME_56_BITS)
			obj_info = (InformationObject)Bitstring32WithCP56Time2a_createEx((Bitstring32WithCP56Time2a)&ioBuf,
																			 addr + i,
																			 bit32str_value,
																			 quality,
																			 &cp56time);
		else /* without time stamp */
			obj_info = (InformationObject)BitString32_createEx((BitString32)&ioBuf,
															   addr + i,
															   bit32str_value,
															   quality);
		break;
	case INTEGR_TOTALS:
		struct sBinaryCounterReading vBCR;
		uint32_t cnt_val = tmp_use ? get_tmp_val(cid, i) : inp_data;
		vBCR.encodedValue[0] = cnt_val & 0xff;
		vBCR.encodedValue[1] = (cnt_val >> 8) & 0xff;
		vBCR.encodedValue[2] = (cnt_val >> 16) & 0xff;
		vBCR.encodedValue[3] = (cnt_val >> 24) & 0xff;
		vBCR.encodedValue[4] = 0;
		if (time_add == TIME_24_BITS)
			obj_info = (InformationObject)IntegratedTotalsWithCP24Time2a_create((IntegratedTotalsWithCP24Time2a)&ioBuf,
																				addr + i,
																				&vBCR,
																				&cp24time);
		else if (time_add == TIME_56_BITS)
			obj_info = (InformationObject)IntegratedTotalsWithCP56Time2a_create((IntegratedTotalsWithCP56Time2a)&ioBuf,
																				addr + i,
																				&vBCR,
																				&cp56time);
		else /* without time stamp */
			obj_info = (InformationObject)IntegratedTotals_create((IntegratedTotals)&ioBuf, addr + i, &vBCR);
		break;
	default:
		break;
	}
	return obj_info;
}

void printCP56Time2a(CP56Time2a time)
{
	ESP_LOGI(TAG, "%02i:%02i:%02i %02i/%02i/%04i",
			 CP56Time2a_getHour(time),
			 CP56Time2a_getMinute(time),
			 CP56Time2a_getSecond(time),
			 CP56Time2a_getDayOfMonth(time),
			 CP56Time2a_getMonth(time),
			 CP56Time2a_getYear(time) + 2000);
}

/* Callback handler to log sent or received messages (optional) */
void rawMessageHandler(void *parameter, uint8_t *msg, int msgSize, bool sent)
{
	if (sent)
		printf("SEND: ");
	else
		printf("RCVD: ");
	int i;
	for (i = 0; i < msgSize; i++)
		printf("%02x ", msg[i]);
	printf("\n");
}

bool clockSyncHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, CP56Time2a newTime)
{
	ESP_LOGI(TAG, "Sync command with UTC Time [GMT+00]:");
	printCP56Time2a(newTime);
	uint64_t newSystemTimeInMs = CP56Time2a_toMsTimestamp(newTime);
	/* Set time for ACT_CON message */
	CP56Time2a_setFromMsTimestamp(newTime, /*Hal_getTimeInMs()*/ newSystemTimeInMs); // ToDo: system or new time send?
	/* update system time here */
	set_time_ms(newSystemTimeInMs);
	iv_time_ms = newSystemTimeInMs;
	ESP_LOGI(TAG, "Total memory: CAP_8BIT: %d CAP_32BIT: %d",
			 heap_caps_get_free_size(MALLOC_CAP_8BIT),
			 heap_caps_get_free_size(MALLOC_CAP_32BIT));
	utc_save_need = true;
	return true;
}

bool interrogationHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu, uint8_t qoi)
{
	xSemaphoreTake(gtwSemaphore, 0);
	ESP_LOGI(TAG, "Received interrogation for group %i", qoi);
	int db_len = get_db_len();
	iec_time_t time_add;
	if ((int)parameter == IEC_104_TCP)
		time_add = interrogation_time_type();
	else /* The CS101 specification only allows information objects without timestamp in GI responses */
		time_add = TIME_NONE;
	CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);
	IMasterConnection_sendACT_CON(connection, asdu, false);
	for (int pid = 0; pid < db_len; pid++)
	{
		if (qoi != get_iec_group(pid) && qoi != IEC60870_QOI_STATION)
			continue;
		int num = get_iec_obj_num(pid);
		CS101_ASDU newAsdu = CS101_ASDU_initializeStatic(&_asdu, alParams, false, CS101_COT_INTERROGATED_BY_STATION,
														 alParams->originatorAddress, get_common_addr(), false, false);
		for (int i = 0; i < num; i++)
		{
			InformationObject io = create_obj_info(pid, i, NULL, time_add);
			if (io == NULL)
				continue;
			if (CS101_ASDU_addInformationObject(newAsdu, io) == false)
			{
				IMasterConnection_sendASDU(connection, newAsdu);
				newAsdu = CS101_ASDU_initializeStatic(&_asdu, alParams, false, CS101_COT_INTERROGATED_BY_STATION,
													  alParams->originatorAddress, get_common_addr(), false, false);
				CS101_ASDU_addInformationObject(newAsdu, io);
			}
		}
		IMasterConnection_sendASDU(connection, newAsdu);
	}
	IMasterConnection_sendACT_TERM(connection, asdu);
	xSemaphoreGive(gtwSemaphore);
	return true;
}

bool CounterInterrogationHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, QualifierOfCIC qcc)
{
	ESP_LOGI(TAG, "Received Counter interrogation command");
	int db_len = get_db_len();
	iec_time_t time_add = interrogation_time_type();
	CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);
	IMasterConnection_sendACT_CON(connection, asdu, false);
	for (int pid = 0; pid < db_len; pid++)
	{
		if (INTEGR_TOTALS != get_iec_data_type(pid))
			continue;
		if (qcc != get_iec_cnt_group(pid) && qcc != IEC60870_QCC_RQT_GENERAL)
			continue;
		int num = get_iec_obj_num(pid);
		CS101_ASDU newAsdu = CS101_ASDU_initializeStatic(&_asdu, alParams, false, CS101_COT_REQUESTED_BY_GENERAL_COUNTER,
														 alParams->originatorAddress, get_common_addr(), false, false);
		for (int i = 0; i < num; i++)
		{
			InformationObject io = create_obj_info(pid, i, NULL, time_add);
			if (io == NULL)
				continue;
			CS101_ASDU_addInformationObject(newAsdu, io);
		}
		IMasterConnection_sendASDU(connection, newAsdu);
	}
	IMasterConnection_sendACT_TERM(connection, asdu);
	return true;
}

bool ReadHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, int ioa)
{
	bool ret = false;
	int pid, idx;
	if (get_obj_id(ioa, &pid, &idx))
	{
		CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);
		IMasterConnection_sendACT_CON(connection, asdu, false);
		InformationObject io = create_obj_info(pid, idx, NULL, TIME_NONE);
		if (io == NULL)
			goto exit;
		CS101_ASDU newAsdu = CS101_ASDU_initializeStatic(&_asdu, alParams, false, CS101_COT_REQUEST,
														 alParams->originatorAddress, get_common_addr(), false, false);
		CS101_ASDU_addInformationObject(newAsdu, io);
		IMasterConnection_sendASDU(connection, newAsdu);
		ret = true;
	}
	exit:
	return ret;
}

bool asduHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu)
{
	bool ret = false, time_add = false;
	CP56Time2a cp56time;
	IEC60870_5_TypeID type_id = CS101_ASDU_getTypeID(asdu);
	CS101_CauseOfTransmission cause = CS101_ASDU_getCOT(asdu);
	int elm_num = CS101_ASDU_getNumberOfElements(asdu);
	InformationObject io;
	ESP_LOGI(TAG, "RECVD ASDU type: %s(%i) elements: %i",
			 TypeID_toString(type_id), type_id, elm_num);
	for (int i = 0; i < elm_num; i++)
	{
		io = CS101_ASDU_getElement(asdu, i);
		if (!io)
			continue;
		int ioa = InformationObject_getObjectAddress(io);
		int pid, idx;
		if (get_obj_id(ioa, &pid, &idx) == false)
		{
			InformationObject_destroy(io);
			continue;
		}
		queue_elm_t queue_elm = {0};
		switch (type_id)
		{
		case C_SC_TA_1: /* 58 - Single command with CP56Time2a */
			time_add = true;
			cp56time = SinglePointWithCP56Time2a_getTimestamp((SinglePointWithCP56Time2a)io);
		case C_SC_NA_1:
			if (cause == CS101_COT_ACTIVATION)
			{
				SingleCommand sc = (SingleCommand)io;
				if (is_cmd_term())
				{
					if (SingleCommand_isSelect(sc))
						goto activation_con;
					else /* command of execution */
					{
						if (is_select_time_out(pid))
							goto time_out_con;
						goto single_exec;
					}
				}
				else /* execution of single set point */
				{
				single_exec:
					SingleCommand_getQU(sc); // ToDo: check for need
					queue_elm.data = SingleCommand_getState(sc);
					ESP_LOGI(TAG, "IOA: %i switch to %li", ioa, queue_elm.data);
					goto queue_send;
				}
			}
			else if (cause == CS101_COT_DEACTIVATION)
				goto deactivation_con;
			else
				goto unknown_cot;
			break;
		case C_DC_TA_1: /* 59 - Double command with CP56Time2a */
			time_add = true;
			cp56time = DoublePointWithCP56Time2a_getTimestamp((DoublePointWithCP56Time2a)io);
		case C_DC_NA_1:
			if (cause == CS101_COT_ACTIVATION)
			{
				DoubleCommand dc = (DoubleCommand)io;
				if (is_cmd_term())
				{
					if (DoubleCommand_isSelect(dc))
						goto activation_con;
					else /* command of execution */
					{
						if (is_select_time_out(pid))
							goto time_out_con;
						goto double_exec;
					}
				}
				else /* execution of double set point */
				{
				double_exec:
					DoubleCommand_getQU(dc); // ToDo: check for need
					queue_elm.data = DoubleCommand_getState(dc);
					ESP_LOGI(TAG, "IOA: %i switch to %li", ioa, queue_elm.data);
					goto queue_send;
				}
			}
			else if (cause == CS101_COT_DEACTIVATION)
				goto deactivation_con;
			else
				goto unknown_cot;
			break;
		case C_RC_TA_1: /* 60 - Step command with CP56Time2a */
			time_add = true;
			cp56time = StepPositionWithCP56Time2a_getTimestamp((StepPositionWithCP56Time2a)io);
		case C_RC_NA_1:
			if (cause == CS101_COT_ACTIVATION)
			{
				StepCommand sc = (StepCommand)io;
				if (is_cmd_term())
				{
					if (StepCommand_isSelect(sc))
						goto activation_con;
					else /* command of execution */
					{
						if (is_select_time_out(pid))
							goto time_out_con;
						goto step_exec;
					}
				}
				else /* execution of step set point */
				{
				step_exec:
					StepCommand_getQU(sc); // ToDo: check for need
					queue_elm.data = StepCommand_getState(sc);
					ESP_LOGI(TAG, "IOA: %i step to %li", ioa, queue_elm.data);
					goto queue_send;
				}
			}
			else if (cause == CS101_COT_DEACTIVATION)
				goto deactivation_con;
			else
				goto unknown_cot;
			break;
		case C_BO_TA_1: /* 64 - Bitstring command with CP56Time2a */
			time_add = true;
			cp56time = Bitstring32WithCP56Time2a_getTimestamp((Bitstring32WithCP56Time2a)io);
		case C_BO_NA_1:
			if (cause == CS101_COT_ACTIVATION)
			{
				Bitstring32Command boc = (Bitstring32Command)io;
				uint32_t value = Bitstring32Command_getValue(boc);
				ESP_LOGI(TAG, "IOA: %i step to %li", ioa, value);
				if (is_cmd_term())
				{
					if (!is_select_time(pid))
						goto activation_con;
					else /* command of execution */
					{
						if (is_select_time_out(pid))
							goto time_out_con;
						goto bitstr32_exec;
					}
				}
				else /* execution of set point */
				{
				bitstr32_exec:
					queue_elm.data = value;
					goto queue_send;
				}
			}
			else if (cause == CS101_COT_DEACTIVATION)
				goto deactivation_con;
			else
				goto unknown_cot;
			break;
		case C_SE_TA_1: /* 61 - Setpoint command, normalized value with CP56Time2a */
			time_add = true;
			cp56time = MeasuredValueNormalizedWithCP56Time2a_getTimestamp((MeasuredValueNormalizedWithCP56Time2a)io);
		case C_SE_NA_1:
			if (cause == CS101_COT_ACTIVATION)
			{
				SetpointCommandNormalized sc = (SetpointCommandNormalized)io;
				float value = SetpointCommandNormalized_getValue(sc);
				ESP_LOGI(TAG, "IOA: %i step to %f", ioa, value);
				if (is_cse_term())
				{
					if (SetpointCommandNormalized_isSelect(sc))
						goto activation_con;
					else /* command of execution */
					{
						if (is_select_time_out(pid))
							goto time_out_con;
						goto normal_exec;
					}
				}
				else /* execution of step set point */
				{
				normal_exec:
					SetpointCommandNormalized_getQL(sc); // ToDo: check for need
					queue_elm.data = (int16_t)(value * 32767);
					goto queue_send;
				}
			}
			else if (cause == CS101_COT_DEACTIVATION)
				goto deactivation_con;
			else
				goto unknown_cot;
			break;
		case C_SE_TB_1: /* 62 - Setpoint command, scaled value with CP56Time2a */
			time_add = true;
			cp56time = MeasuredValueScaledWithCP56Time2a_getTimestamp((MeasuredValueScaledWithCP56Time2a)io);
		case C_SE_NB_1:
			if (cause == CS101_COT_ACTIVATION)
			{
				SetpointCommandScaled sc = (SetpointCommandScaled)io;
				int value = SetpointCommandScaled_getValue(sc);
				ESP_LOGI(TAG, "IOA: %i step to %i", ioa, value);
				if (is_cse_term())
				{
					if (SetpointCommandScaled_isSelect(sc))
						goto activation_con;
					else /* command of execution */
					{
						if (is_select_time_out(pid))
							goto time_out_con;
						goto scaled_exec;
					}
				}
				else /* execution of step set point */
				{
				scaled_exec:
					SetpointCommandScaled_getQL(sc); // ToDo: check for need
					queue_elm.data = *(uint32_t *)&value;
					goto queue_send;
				}
			}
			else if (cause == CS101_COT_DEACTIVATION)
				goto deactivation_con;
			else
				goto unknown_cot;
			break;
		case C_SE_TC_1: /* 63 - Setpoint command, short value with CP56Time2a */
			time_add = true;
			cp56time = MeasuredValueShortWithCP56Time2a_getTimestamp((MeasuredValueShortWithCP56Time2a)io);
		case C_SE_NC_1:
			if (cause == CS101_COT_ACTIVATION)
			{
				SetpointCommandShort sc = (SetpointCommandShort)io;
				float value = SetpointCommandShort_getValue(sc);
				ESP_LOGI(TAG, "IOA: %i step to %f\n", ioa, value);
				if (is_cse_term())
				{
					if (SetpointCommandShort_isSelect(sc))
					{
					activation_con:
						set_select_time(pid);
						CS101_ASDU_setCOT(asdu, CS101_COT_ACTIVATION_CON);
					}
					else /* command of execution */
					{
						if (is_select_time_out(pid))
							goto time_out_con;
						goto short_exec;
					}
				}
				else /* execution of step set point */
				{
				short_exec:
					SetpointCommandShort_getQL(sc); // ToDo: check for need
					queue_elm.data = *(uint32_t *)&value;
				queue_send:
					queue_elm.pid = pid;
					queue_elm.idx = idx;
					if (time_add)
						queue_elm.time_ms = CP56Time2a_toMsTimestamp(cp56time);
					if (xQueueSend(mbQueueHdl, &queue_elm, QUEUE_NO_WAIT) != pdTRUE)
						ESP_LOGE(TAG, "Send message in to Queue by MB driver failure.");
					CS101_ASDU_setCOT(asdu, CS101_COT_ACTIVATION_TERMINATION);
				}
			}
			else if (cause == CS101_COT_DEACTIVATION)
			{
			deactivation_con:
				clr_select_time(pid);
			time_out_con:
				CS101_ASDU_setCOT(asdu, CS101_COT_DEACTIVATION_CON);
			}
			else
			{
			unknown_cot:
				CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_COT);
			}
			break;
		default:
			InformationObject_destroy(io);
			ret = false;
			goto exit;
		}
		InformationObject_destroy(io);
		vTaskDelay(get_between_del());
	}
	ret = true;
exit:
	IMasterConnection_sendASDU(connection, asdu);
	return ret;
}

/* Handler for reset process command (C_RP_NA_1 - 105) */
bool resetProcessHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, uint8_t qrp)
{
	ESP_LOGI(TAG, "Received Reset process command (C_RP_NA_1 - 105)");
	switch (qrp)
	{
	case IEC60870_QRP_GENERAL_RESET:
		ESP_LOGI(TAG, "QRP_GENERAL_RESET");
		break;
	case IEC60870_QRP_RESET_PENDING_INFO_WITH_TIME_TAG:
		ESP_LOGI(TAG, "QRP_RESET_PENDING_INFO_WITH_TIME_TAG");
		break;
	default:
		break;
	}
	IMasterConnection_sendACT_CON(connection, asdu, false);
	return true;
}

/* Handler for delay acquisition command (C_CD_NA:1 - 106) */
bool delayAcquisitionHandler(void *parameter, IMasterConnection connection, CS101_ASDU asdu, CP16Time2a delayTime)
{
	ESP_LOGI(TAG, "Received Delay acquisition command (C_CD_NA:1 - 106)");
	ESP_LOGI(TAG, "delay Time: %0i ms", CP16Time2a_getEplapsedTimeInMs(delayTime));
	IMasterConnection_sendACT_CON(connection, asdu, false);
	return true;
}

static void enqueueASDU(void *slave, InformationObject io, CS101_CauseOfTransmission cot, conn_t conn)
{
	if ((io == NULL) || (slave == NULL))
		return;
	CS101_AppLayerParameters alParams = (conn == IEC_101_SER) ? CS101_Slave_getAppLayerParameters(slave) :
																CS104_Slave_getAppLayerParameters(slave);
	CS101_ASDU newAsdu = CS101_ASDU_initializeStatic(&_asdu, alParams, false, cot,
													 alParams->originatorAddress, get_common_addr(), false, false);
	CS101_ASDU_addInformationObject(newAsdu, io);
	(conn == IEC_101_SER) ? CS101_Slave_enqueueUserDataClass2(slave, newAsdu) :
							CS104_Slave_enqueueASDU(slave, newAsdu);
}

static void iec_obj_send(queue_elm_t *obj, int num, void *slave, conn_t conn)
{
	iec_time_t time_add;
	if (obj->cause == COT_SPONTANEOUS)
		time_add = spontanius_time_type();
	else if (obj->cause == COT_PERIODIC)
		time_add = measured_time_type();
	else
		time_add = TIME_NONE;
	CS101_AppLayerParameters alParams = (conn == IEC_101_SER) ? CS101_Slave_getAppLayerParameters(slave) :
																CS104_Slave_getAppLayerParameters(slave);
	CS101_CauseOfTransmission cot = obj[0].cause;
	uint8_t pid = obj[0].pid;
	CS101_ASDU newAsdu = CS101_ASDU_initializeStatic(&_asdu, alParams, false, cot,
													 alParams->originatorAddress, get_common_addr(), false, false);
	for (int i = 0; i < num; i++)
	{
		InformationObject io = create_obj_info(pid, obj[i].idx, &(obj[i].data), time_add);
		if (io == NULL)
			continue;
		if (CS101_ASDU_addInformationObject(newAsdu, io) == false)
		{
			(conn == IEC_101_SER) ? CS101_Slave_enqueueUserDataClass2(slave, newAsdu) :
									CS104_Slave_enqueueASDU(slave, newAsdu);
			newAsdu = CS101_ASDU_initializeStatic(&_asdu, alParams, false, cot, alParams->originatorAddress,
												  get_common_addr(), false, false);
			CS101_ASDU_addInformationObject(newAsdu, io);
		}
	}
	(conn == IEC_101_SER) ? CS101_Slave_enqueueUserDataClass2(slave, newAsdu) :
							CS104_Slave_enqueueASDU(slave, newAsdu);
}

void end_of_init_send(void *slave, conn_t conn)
{
	InformationObject io = (InformationObject)EndOfInitialization_create(NULL, 0);
	enqueueASDU(slave, io, CS101_COT_INITIALIZED, conn);
}

void iec_queue_send(void *slave, conn_t conn)
{
	queue_elm_t mb_data[IEC_QUEUE_LEN];
	int mess = uxQueueMessagesWaiting(iecQueueHdl);
	if (!mess)
		return;
	int n = 0;
	for (int i = 0; i < mess; i++)
	{
		xQueuePeek(iecQueueHdl, &mb_data[i], QUEUE_NO_WAIT);
		ESP_LOGI(TAG, "ModBus Message from Queue by pid #%u.", mb_data[i].pid);
		if (mb_data[i].pid == mb_data[0].pid)
		{
			n++;
			xQueueReceive(iecQueueHdl, &mb_data[i], QUEUE_NO_WAIT);
		}
	}
	ESP_LOGI(TAG, "Send ASDU with %u IO", n);
	iec_obj_send(mb_data, n, slave, conn);
}