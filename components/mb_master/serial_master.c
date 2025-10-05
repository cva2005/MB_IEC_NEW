#include <string.h>
#include "esp_log.h"
#include "mbcontroller.h"
#include "sdkconfig.h"
#include "config.h"
#include "gtw_params.h"
#include "gtw_mb_master.h"

static const char *TAG = "MB Serial Master";
static void *master_handle[SERIAL_NUM] = {NULL};
static const uint32_t B_R[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 128000, 230400, 256000, 460800, 921600};
static const uart_parity_t P_R[] = {UART_PARITY_DISABLE, UART_PARITY_ODD, UART_PARITY_EVEN};
static const uart_word_length_t D_B[] = {UART_DATA_7_BITS, UART_DATA_8_BITS};
static const uart_stop_bits_t S_B[] = {UART_STOP_BITS_1, UART_STOP_BITS_2, UART_STOP_BITS_1_5};

/* Modbus master initialization */
static esp_err_t serial_master_init(int idx)
{
	uart_port_t uart_num = (idx == SER1_CONN) ? CONFIG_UART1_PORT_NUM : CONFIG_UART2_PORT_NUM;
	int tx_io_num = (idx == SER1_CONN) ? CONFIG_UART1_TXD : CONFIG_UART2_TXD;
	int rx_io_num = (idx == SER1_CONN) ? CONFIG_UART1_RXD : CONFIG_UART2_RXD;
	int rts_io_num = (idx == SER1_CONN) ? CONFIG_UART1_RTS : CONFIG_UART2_RTS;
	// Initialize Modbus controller
	mb_communication_info_t comm = {
		.ser_opts.port = uart_num,
		.ser_opts.mode = (mb_mode_type_t)(idx == SER1_CONN) ? RamCfg.mbM0 : RamCfg.mbM1,
		.ser_opts.baudrate = (idx == SER1_CONN) ? B_R[RamCfg.mbB0] : B_R[RamCfg.mbB1],
		.ser_opts.parity = (idx == SER1_CONN) ? P_R[RamCfg.mbP0] : P_R[RamCfg.mbP1],
		.ser_opts.uid = 0,
		.ser_opts.response_tout_ms = RamCfg.mbRespT,
		.ser_opts.data_bits = (idx == SER1_CONN) ? D_B[RamCfg.mbD0] : D_B[RamCfg.mbD1],
		.ser_opts.stop_bits = (idx == SER1_CONN) ? S_B[RamCfg.mbS0] : S_B[RamCfg.mbS1]};

	esp_err_t err = mbc_master_create_serial(&comm, &master_handle[idx]);
	MB_RETURN_ON_FALSE((master_handle[idx] != NULL), ESP_ERR_INVALID_STATE, TAG,
					   "mb controller initialization fail.");
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller initialization fail, returns(0x%x).", (int)err);

    // Set UART pin numbers
	err = uart_set_pin(uart_num, tx_io_num, rx_io_num,
					   rts_io_num, UART_PIN_NO_CHANGE);
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                        "mb serial set pin failure, uart_set_pin() returned (0x%x).", (int)err);

	err = mbc_master_set_descriptor(master_handle[idx], get_param_desc(idx), get_param_num(idx));
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
					   "mb serial controller set descriptor fail, returns(0x%x).", (int)err);

	err = mbc_master_start(master_handle[idx]);
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                            "mb controller start fail, returned (0x%x).", (int)err);

    // Set driver mode to Half Duplex
	err = uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (int)err);

    vTaskDelay(5);
	ESP_LOGI(TAG, "Modbus Serial Master #%u Initialized...", idx + 1);
	return err;
}

void mb_serial_master_register(void)
{
	for (int i = 0; i < SERIAL_NUM; i++)
	{
		if (is_mb_connect_use(i))
		{
			ESP_ERROR_CHECK(serial_master_init(i));
			vTaskDelay(10);
			mb_master_register(master_handle[i], i);
		}
	}
}