#include <string.h>
#include "esp_log.h"
#include "mbcontroller.h"
#include "sdkconfig.h"
#include "config.h"
#include "gtw_params.h"
#include "gtw_mb_master.h"
#include "protocol_examples_common.h"

static const char *TAG = "MB TC Master";
static void *eth_master_handle = NULL;

static esp_err_t tcp_master_init(void)
{
	void **master_handle;
	uint32_t resp_to;
	int param_num;
	mb_parameter_descriptor_t *mb_desc;
	master_handle = &eth_master_handle;
	resp_to = RamCfg.eRespT;
	mb_desc = get_param_desc(ETH_CONN);
	param_num = get_param_num(ETH_CONN);
	mb_communication_info_t comm = {0};
	ESP_LOGI(TAG, "Modbus tcp start initialization...");
	comm.tcp_opts.port = CONFIG_FMB_TCP_PORT_DEFAULT;
	comm.tcp_opts.addr_type = MB_IPV4;
	comm.tcp_opts.mode = MB_TCP;
	comm.tcp_opts.ip_addr_table = get_ip_table();
	comm.tcp_opts.uid = 0;
	comm.tcp_opts.start_disconnected = false;
	comm.tcp_opts.response_tout_ms = resp_to;
	esp_netif_t *esp_netif = get_example_netif_from_desc(EXAMPLE_NETIF_DESC_ETH);
	comm.tcp_opts.ip_netif_ptr = (void *)esp_netif;
	esp_err_t err = mbc_master_create_tcp(&comm, master_handle);
	MB_RETURN_ON_FALSE((*master_handle != NULL), ESP_ERR_INVALID_STATE, TAG,
					   "mb controller initialization fail.");
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
					   "mb controller initialization fail, returns(0x%x).", (int)err);

	err = mbc_master_set_descriptor(*master_handle, mb_desc, param_num);
	MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
					   "mb tcp controller set descriptor fail, returns(0x%x).", (int)err);

	err = mbc_master_start(*master_handle);
	vTaskDelay(5);
	if (err != ESP_OK)
	{
		mbc_master_delete(*master_handle);
		MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
						   "mb tcp controller start fail, returned (0x%x).", (int)err);
	}
	else
	{
		ESP_LOGI(TAG, "Start tcp modbus master...");
	}
	return err;
}

void mb_tcp_master_register(void)
{
		ESP_ERROR_CHECK(tcp_master_init());
		vTaskDelay(10);
		mb_master_register(eth_master_handle, ETH_CONN);
}