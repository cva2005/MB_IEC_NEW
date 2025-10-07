#include <string.h>
#include "esp_log.h"
#include "mbcontroller.h"
#include "sdkconfig.h"
#include "config.h"
#include "gtw_params.h"
#include "gtw_mb_master.h"

static const char *TAG = "MB Serial Master";
static void *master_handle = NULL;
static const uint32_t B_R[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 128000, 230400, 256000, 460800, 921600};
static const uart_parity_t P_R[] = {UART_PARITY_DISABLE, UART_PARITY_ODD, UART_PARITY_EVEN};
static const uart_word_length_t D_B[] = {UART_DATA_7_BITS, UART_DATA_8_BITS};
static const uart_stop_bits_t S_B[] = {UART_STOP_BITS_1, UART_STOP_BITS_2, UART_STOP_BITS_1_5};

/* Modbus master initialization */
static esp_err_t serial_master_init(void)
{
	// Initialize Modbus controller
	mb_communication_info_t comm = {
		.ser_opts.port = CONFIG_UART1_PORT_NUM,
		.ser_opts.mode = RamCfg.mbMode,
		.ser_opts.baudrate = B_R[RamCfg.mbBaud],
		.ser_opts.parity = P_R[RamCfg.mbPrt],
		.ser_opts.uid = 0,
		.ser_opts.response_tout_ms = RamCfg.mbRespT,
		.ser_opts.data_bits = D_B[RamCfg.mbDtb],
		.ser_opts.stop_bits = S_B[RamCfg.mbStb]};

	esp_err_t err = mbc_master_create_serial(&comm, &master_handle);
	MB_RETURN_ON_FALSE((master_handle != NULL), ESP_ERR_INVALID_STATE, TAG,
					   "mb controller initialization fail.");
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller initialization fail, returns(0x%x).", (int)err);

    // Set UART pin numbers
	err = uart_set_pin(CONFIG_UART1_PORT_NUM, CONFIG_UART1_TXD, CONFIG_UART1_RXD,
					   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                        "mb serial set pin failure, uart_set_pin() returned (0x%x).", (int)err);

	err = mbc_master_set_descriptor(master_handle, get_param_desc(SER1_CONN), get_param_num(SER1_CONN));
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
					   "mb serial controller set descriptor fail, returns(0x%x).", (int)err);

	err = mbc_master_start(master_handle);
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller start fail, returned (0x%x).", (int)err);

    // Set driver mode to Half Duplex
	err = uart_set_mode(CONFIG_UART1_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (int)err);

    vTaskDelay(5);
	ESP_LOGI(TAG, "Modbus Serial Master Initialized...");
	return err;
}

void mb_serial_master_register(void)
{
	if (is_mb_connect_use(SER1_CONN))
	{
		ESP_ERROR_CHECK(serial_master_init());
		vTaskDelay(10);
		mb_master_register(master_handle, SER1_CONN);
	}
}