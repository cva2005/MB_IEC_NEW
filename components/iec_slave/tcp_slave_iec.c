#include <stdio.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "netdb.h"
#include "esp_netif.h"
#include "iec_controller.h"
#include "protocol_examples_common.h"
#include "cs104_slave.h"
#include "hal_time.h"
#include "hal_socket.h"
#include "config.h"
#include "gtw_params.h"
#include "gtw_iec_slave.h"

#define PERIODIC_TASK_WAIT_MS	5
#define PERIODIC_TASK_WAIT		pdMS_TO_TICKS(PERIODIC_TASK_WAIT_MS)
#define RX_BUFF_LEN 			263

static const char *TAG = "TCP SLAVE IEC-104";
static void* tcp_slave_handle = NULL;
static CS104_Slave slave = NULL;
static char* slave_ip_address_table[] = {NULL, NULL};
extern struct sCS104_APCIParameters defaultConnectionParameters;
static bool frame_rcv = false;
static uint8_t rx_buf[RX_BUFF_LEN];
static uint16_t rx_len;

static bool
connectionRequestHandler(void* parameter, const char* ipAddress)
{
	ESP_LOGI(TAG, "New connection request from %s\n", ipAddress);
	return true;
}

static void
connectionEventHandler(void* parameter, IMasterConnection con, CS104_PeerConnectionEvent event)
{
	if (event == CS104_CON_EVENT_CONNECTION_OPENED) {
		ESP_LOGI(TAG, "Connection opened (%p)", con);
	}
	else if (event == CS104_CON_EVENT_CONNECTION_CLOSED) {
		ESP_LOGI(TAG, "Connection closed (%p)", con);
	}
	else if (event == CS104_CON_EVENT_ACTIVATED) {
		ESP_LOGI(TAG, "Connection activated (%p)", con);
	}
	else if (event == CS104_CON_EVENT_DEACTIVATED) {
		ESP_LOGI(TAG, "Connection deactivated (%p)", con);
	}
}

void tcp_iec_slave_init(void)
{
	ESP_LOGI(TAG, "IEC 104 TCP Slave stack START Initialization...");
	/* create a new slave/server instance with default connection parameters and
	 * default message queue size */
	defaultConnectionParameters.t0 = RamCfg.t0;
	defaultConnectionParameters.t1 = RamCfg.t1;
	defaultConnectionParameters.t2 = RamCfg.t2;
	defaultConnectionParameters.t3 = RamCfg.t3;
	defaultConnectionParameters.k = RamCfg.K;
	defaultConnectionParameters.w = RamCfg.W;
	slave = CS104_Slave_create(20, 20);
	/* Set mode to a single redundancy group
	 * NOTE: library has to be compiled with CONFIG_CS104_SUPPORT_SERVER_MODE_SINGLE_REDUNDANCY_GROUP enabled (=1)
	 */
	CS104_Slave_setServerMode(slave, CS104_MODE_SINGLE_REDUNDANCY_GROUP);
	/* get the connection parameters - we need them to create correct ASDUs */
	CS101_AppLayerParameters alParams = CS104_Slave_getAppLayerParameters(slave);
	alParams->sizeOfCOT = RamCfg.cotSize + 1;
	alParams->sizeOfCA = RamCfg.alAdrSize + 1;
	alParams->sizeOfIOA = RamCfg.ioaSize + 1;
	alParams->originatorAddress = RamCfg.orgAdr;
	ESP_LOGI(TAG, "sizeOfTypeId: %i, sizeOfVSQ: %i, sizeOfCOT: %i, originatorAddress: %i, sizeOfCA: %i, sizeOfIOA: %i, maxSizeOfASDU: %i",
			 alParams->sizeOfTypeId,	  /* size of the type id (default = 1 - don't change) */
			 alParams->sizeOfVSQ,		  /* don't change */
			 alParams->sizeOfCOT,		  /* size of COT (1/2 - default = 2 -> COT includes OA) */
			 alParams->originatorAddress, /* originator address (OA) to use (0-255) */
			 alParams->sizeOfCA,		  /* size of common address (CA) of ASDU (1/2 - default = 2) */
			 alParams->sizeOfIOA,		  /* size of information object address (IOA) (1/2/3 - default = 3) */
			 alParams->maxSizeOfASDU);	  /* maximum size of the ASDU that is generated - the maximum maximum value is 249 for IEC 104 and 254 for IEC 101 */
	/* APCI parameters (t0-t3, k, w) */
	CS104_APCIParameters apciParams = CS104_Slave_getConnectionParameters(slave);
	ESP_LOGI(TAG, "t0: %i, t1: %i, t2: %i, t3: %ik: %i, w: %i", apciParams->t0,
			 apciParams->t1, apciParams->t2, apciParams->t3, apciParams->k, apciParams->w);
	/* set the callback handler for the clock synchronization command */
	CS104_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);
	/* set the callback handler for the interrogation command */
	CS104_Slave_setInterrogationHandler(slave, interrogationHandler, (void *)IEC_104_TCP);
	/* set the callback handler for the Counter interrogation command */
	CS104_Slave_setCounterInterrogationHandler(slave, CounterInterrogationHandler, NULL);
	/* set handler for ASDU command */
	CS104_Slave_setASDUHandler(slave, asduHandler, NULL);
	/* set handler for set handler for read request (C_RD_NA_1 - 102) */
	CS104_Slave_setReadHandler(slave, ReadHandler, NULL);
	/* set handler to handle connection requests (optional) */
	CS104_Slave_setConnectionRequestHandler(slave, connectionRequestHandler, NULL);
	/* set handler to track connection events (optional) */
	CS104_Slave_setConnectionEventHandler(slave, connectionEventHandler, NULL);
	/* uncomment to log messages */
	//CS104_Slave_setRawMessageHandler(slave, rawMessageHandler, NULL);
	/* set the callback handler for delay Acquisition command */
	CS104_Slave_setDelayAcquisitionHandler(slave, delayAcquisitionHandler, NULL);
	/* set the callback handler for Reset command */
	CS104_Slave_setResetProcessHandler(slave, resetProcessHandler, NULL);

	CS104_Slave_start(slave);
	CS104_Slave_startThreadless(slave);
	/* Set log level */
	esp_log_level_set(TAG, ESP_LOG_INFO);

	uint32_t ip = RamCfg.ipAdr;
	asprintf(&slave_ip_address_table[0], "%lu.%lu.%lu.%lu",
			 ip & 0xff, (ip >> 8) & 0xff, (ip >> 16) & 0xff, (ip >> 24) & 0xff);
	iec_communication_info_t tcp_slave_config = {
		.tcp_opts.port = RamCfg.tcpPort,
		.tcp_opts.mode = IEC_MODE_104,
		.tcp_opts.addr_type = MB_IPV4,
		.tcp_opts.ip_addr_table = (void *)slave_ip_address_table,
		.tcp_opts.ip_netif_ptr = (void *)get_example_netif_from_desc(EXAMPLE_NETIF_DESC_ETH)
	};
	ESP_ERROR_CHECK(iecc_slave_create_tcp(&tcp_slave_config, &tcp_slave_handle));
	Socket_ready(&tcp_slave_handle);
}

void tcp_iec_slave_task(void *arg)
{
	while (true) {
		vTaskDelay(1);
		if (is_end_of_init())
		{
			end_of_init_send(slave, IEC_104_TCP);
			/* Starts of iec controller and stack */
			ESP_ERROR_CHECK(iecc_slave_start(tcp_slave_handle));
			ESP_LOGI(TAG, "Start slave...");
			break;
		}
	}
	while (true) {
		vTaskDelay(1);
		if (frame_rcv)
		{
			frame_rcv = false;
			ESP_LOGD(TAG, "ParseMessage rx_buf: %p, rx_len: %d", rx_buf, rx_len);
			CS104_ParseMessage(slave, rx_buf, rx_len);
		}
		if (CS104_Slave_getNumberOfHighQueueEntries(slave))
		{
			if (xSemaphoreTake(gtwSemaphore, 0) == pdTRUE)
			{
				CS104_Slave_executePeriodicTasks(slave);
				xSemaphoreGive(gtwSemaphore);
			}
		}
		iec_queue_send(slave, IEC_104_TCP);
		int entr = CS104_Slave_getNumberOfQueueEntries(slave, NULL);
		while (entr--)
		{
			if (xSemaphoreTake(gtwSemaphore, 0) == pdTRUE)
			{
				CS104_Slave_executePeriodicTasks(slave);
				xSemaphoreGive(gtwSemaphore);
			}
			else
				break;
		}
		check_utc_save();
	}
}

uint8_t *get_tcp_buff_ptr(void)
{
	return rx_buf;
}

void tcp_frame_ready(uint16_t len)
{
	rx_len = len;
	frame_rcv = true;
}

void *get_tcp_slave_handle(void)
{
	return tcp_slave_handle;
}

void tcp_new_connect(void)
{
	CS104_MasterConnectionReset(slave);
}