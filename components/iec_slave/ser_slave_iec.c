#include <stdio.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h" 
#include "sdkconfig.h"
#include "hal_time.h"
#include "esp_system.h"
#include "iec_controller.h"
#include "cs101_slave.h"
#include "config.h"
#include "gtw_params.h"
#include "gtw_iec_slave.h"
#include "link_layer.h"

#define RX_BUFF_LEN 263

static const char *TAG = "SERIAL SLAVE IEC-101";
static void *ser_slave_handle = NULL;
static CS101_Slave slave =  NULL;
static iec_event_group_t event = IEC_EVENT_NO_EVENTS;
static const uint32_t B_R[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 128000, 230400, 256000, 460800, 921600};
static const uart_parity_t P_R[] = {UART_PARITY_DISABLE, UART_PARITY_ODD, UART_PARITY_EVEN};
static bool slave_avalaible = false;
static uint64_t idle_time = 0;

static void set_idle_time(void)
{
	idle_time = get_time_ms();
}

static bool is_idle_time_out(void)
{
	if ((get_time_ms() - idle_time) >= RamCfg.tFault)
		return true;
	return false;
}

static void resetCUHandler(void *parameter)
{
	DEBUG_PRINT("Received reset CU\n");
	event |= IEC_EVENT_RESET_CU;

	CS101_Slave_flushQueues((CS101_Slave) parameter);
}

static void
linkLayerStateChanged(void* parameter, int address, LinkLayerState state)
{
	DEBUG_PRINT("Link layer state: ");
	event |= IEC_EVENT_LL_CHANGE;

	switch (state) {
	case LL_STATE_IDLE:
		DEBUG_PRINT("IDLE\n");
		set_idle_time();
		slave_avalaible = false;
		break;
	case LL_STATE_ERROR:
		DEBUG_PRINT("ERROR\n");
		break;
	case LL_STATE_BUSY:
		DEBUG_PRINT("BUSY\n");
		break;
	case LL_STATE_AVAILABLE:
		DEBUG_PRINT("AVAILABLE\n");
		slave_avalaible = true;
		break;
	}
}

void ser_iec_slave_init(int connect)
{
	ESP_LOGI(TAG, "IEC 101 Slave stack START Initialization...");
	/* create a new slave/server instance with default link layer and application layer parameters */
	slave = CS101_Slave_create(NULL, NULL, NULL,
		(RamCfg.llMode == UNBALANCED) ? IEC60870_LINK_LAYER_UNBALANCED : IEC60870_LINK_LAYER_BALANCED);
	/* get the application layer parameters - we need them to create correct ASDUs */
	CS101_AppLayerParameters alParameters = CS101_Slave_getAppLayerParameters(slave);
	alParameters->sizeOfCOT = RamCfg.cotSize + 1;
	alParameters->sizeOfCA = RamCfg.alAdrSize + 1;
	alParameters->sizeOfIOA = RamCfg.ioaSize + 1;
	LinkLayerParameters llParameters = CS101_Slave_getLinkLayerParameters(slave);
	llParameters->timeoutForAck = RamCfg.tAck;
	llParameters->useSingleCharACK = RamCfg.usACK;
	llParameters->addressLength = RamCfg.llAdrSize + 1;
	llParameters->timeoutRepeat = RamCfg.tRep;
	CS101_Slave_setLinkLayerAddress(slave, RamCfg.llAdr);
	CS101_Slave_setLinkLayerAddressOtherStation(slave, 1);

	/* set the callback handler for the clock synchronization command */
	CS101_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);
	/* set the callback handler for the interrogation command */
	CS101_Slave_setInterrogationHandler(slave, interrogationHandler, (void *)IEC_101_SER);
	/* set the callback handler for the Counter interrogation command */
	CS101_Slave_setCounterInterrogationHandler(slave, CounterInterrogationHandler, NULL);
	/* set handler for ASDU command */
	CS101_Slave_setASDUHandler(slave, asduHandler, NULL);
	/* set handler for set handler for read request (C_RD_NA_1 - 102) */
	CS101_Slave_setReadHandler(slave, ReadHandler, NULL);
	/* set handler for reset CU (reset communication unit) message */
	CS101_Slave_setResetCUHandler(slave, resetCUHandler, (void*) slave);
	/* set timeout for detecting connection loss */
	CS101_Slave_setIdleTimeout(slave, 1500);
	/* set handler for link layer state changes */
	CS101_Slave_setLinkLayerStateChanged(slave, linkLayerStateChanged, NULL);
	/* uncomment to log messages */
	CS101_Slave_setRawMessageHandler(slave, rawMessageHandler, NULL);
	/* set the callback handler for Reset command */
	CS101_Slave_setResetProcessHandler(slave, resetProcessHandler, NULL);
	/* set the callback handler for delay Acquisition command */
	CS101_Slave_setDelayAcquisitionHandler(slave, delayAcquisitionHandler, NULL);

	ESP_LOGI(TAG, "IEC 101 Slave stack initialized.");

	/* Set UART log level */
	esp_log_level_set(TAG, ESP_LOG_INFO);

	iec_communication_info_t comm_config = {
		.ser_opts.port = CONFIG_UART1_PORT_NUM,
		.ser_opts.mode = IEC_MODE_101,
		.ser_opts.baudrate = B_R[RamCfg.Baud],
		.ser_opts.parity = P_R[RamCfg.Parity], // ToDo: stop bits/word len ???
		//.ser_opts.uid = RamCfg.llAdr,
		.ser_opts.data_bits = UART_DATA_8_BITS,
		.ser_opts.stop_bits = UART_STOP_BITS_1};
	ESP_ERROR_CHECK(iecc_slave_create_serial(&comm_config, &ser_slave_handle));

	/* Set UART pin nuiec_ers */
	ESP_ERROR_CHECK(uart_set_pin(
		CONFIG_UART1_PORT_NUM,
		CONFIG_UART1_TXD,
		CONFIG_UART1_RXD,
		CONFIG_UART1_RTS,
		UART_PIN_NO_CHANGE));

	/* Set UART driver mode to Half Duplex */
	ESP_ERROR_CHECK(uart_set_mode(CONFIG_UART1_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX));
}

void ser_iec_slave_task(void *arg)
{
	while (true)
	{
		vTaskDelay(1);
		if (is_end_of_init())
		{
			end_of_init_send(slave, IEC_101_SER);
			/* Starts of iec controller and stack */
			ESP_ERROR_CHECK(iecc_slave_start(ser_slave_handle));
			ESP_LOGI(TAG, "Start slave...");
			break;
		}
	}
	while (true)
	{
		vTaskDelay(1);
		CS101_Slave_run(slave);
		iec_queue_send(slave, IEC_101_SER);
		check_utc_save();
	}
}

bool ser_frame_parse(uint16_t *len, uint8_t **buf)
{
	ESP_LOG_BUFFER_HEX_LEVEL("IEC SLAVE RECIEVED:", *buf,
							 (uint16_t)*len, ESP_LOG_DEBUG);
	if (RamCfg.llMode == UNBALANCED)
		ParserHeaderSecondaryUnbalanced(slave->unbalancedLinkLayer, *buf, *len);
	else
		HandleMessageBalancedAndPrimaryUnbalanced(slave->balancedLinkLayer->linkLayer, *buf, *len);
	*buf = LinkTxNeed(len);
	if (*buf == NULL)
		return false;
	ESP_LOG_BUFFER_HEX_LEVEL("IEC SLAVE SEND:", *buf,
							 (uint16_t)*len, ESP_LOG_DEBUG);
	return true;
}

bool is_iec_ser_start(void)
{
	if (is_idle_time_out() && !slave_avalaible)
		return false;
	return true;
}