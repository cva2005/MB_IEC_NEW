#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "webserver.h"
#include "timer_1ms.h"
#include "config.h"
#include "protocol_examples_common.h"
#include "serial_master.h"
#include "tcp_master.h"
#include "gtw_mb_master.h"
#include "tcp_slave_iec.h"
#include "ser_slave_iec.h"
#include "gtw_params.h"
#include "sdkconfig.h"
#include "gpio_drv.h"
#include "esp_ota_ops.h"
#include "switch_ota.h"
#include "arch.h"

const char *TAG = "MB_IEC_GTW";
RTC_DATA_ATTR static bool ota_key = false;
RTC_DATA_ATTR slave_select_t slave_select = SLAVE_IEC_104_TCP;

#define WIFI_SSID CONFIG_ESP_WIFI_SSID
#define WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define WIFI_CHANNEL CONFIG_ESP_WIFI_CHANNEL
#define MAX_STA_CONN CONFIG_ESP_MAX_STA_CONN

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
							   int32_t event_id, void *event_data)
{
	if (event_id == WIFI_EVENT_AP_STACONNECTED)
	{
		wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
		ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
				 MAC2STR(event->mac), event->aid);
	}
	else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
	{
		wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
		ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d, reason=%d",
				 MAC2STR(event->mac), event->aid, event->reason);
	}
}

static void wifi_init_softap(void)
{
	esp_netif_create_default_wifi_ap();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
														ESP_EVENT_ANY_ID,
														&wifi_event_handler,
														NULL,
														NULL));
	char ser_n[8];
	sprintf(ser_n, "-%u", RamCfg.SerN);
	wifi_config_t wifi_config = {
		.ap = {
			.ssid = WIFI_SSID,
			//.ssid_len = strlen(WIFI_SSID),
			.channel = WIFI_CHANNEL,
			.password = WIFI_PASS,
			.max_connection = MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
			.authmode = WIFI_AUTH_WPA3_PSK,
			.sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
			.authmode = WIFI_AUTH_WPA2_PSK,
#endif
			.pmf_cfg = {
				.required = true,
			},
		},
	};
	strcat((char *)wifi_config.ap.ssid, ser_n);
	wifi_config.ap.ssid_len = strlen((char *)wifi_config.ap.ssid);
	if (strlen(WIFI_PASS) == 0)
	{
		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	}

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
			 WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
}

void app_main(void)
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	ESP_ERROR_CHECK(read_config());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	reset_state_t reset_state = get_rst_state();
	clr_rst_state();
	switch (reset_state)
	{
		case FACTORY_LOAD:
			ESP_LOGI(TAG, "Copy Factory app to next part");
			save_arch_event(FW_FCT);
			ota_factory_reload();
			break;
		case ROLLBACK_CMD:
			ESP_LOGI(TAG, "Rollback Firmware to preview part");
			save_arch_event(FW_RLB);
			ota_rollback();
			break;
		case RST_ONLY_CMD:
			ESP_LOGI(TAG, "Reset Command from Browser");
			save_arch_event(RST_WEB);
			break;
		case RST_DEFL_CFG:
			ESP_LOGI(TAG, "Reset to Default Configuration");
			save_arch_event(CFG_DF);
			break;
		case CLR_ARCH_CMD:
			ESP_LOGI(TAG, "Clear Events Archive");
			save_arch_event(CLR_ARC);
			break;
		default:
			if (esp_reset_reason() == ESP_RST_POWERON)
				read_utc_copy();
			save_arch_event(NO_SYS_EVT);
			break;
	}
	gpio_init();
	if (!ota_key)
	{
		if (is_factory_button())
		{
			ota_key = true;
			prepare_factory_reload();
			reboot_as_deep_sleep();
		}
		if (is_rollback_on())
		{
			ota_key = true;
			if (prepare_rollback())
				reboot_as_deep_sleep();
		}
	}
	ota_key = false;
	timer_1ms_init();
	if ((gtw_param_init() == ESP_ERR_NOT_FOUND) || is_web_cfg())
	{
		ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
		wifi_init_softap();
		webserver_init();
		webserver_start();
		while (true)
		{
			if (is_reset_time_out())
			{
				esp_err_t err;
				switch (get_rst_state())
				{
				case FACTORY_LOAD:
					prepare_factory_reload();
					ESP_LOGI(TAG, "Factory Reload Command");
					break;
				case ROLLBACK_CMD:
					if (prepare_rollback() == false)
					{
						ESP_LOGE(TAG, "Rollback Firmware Command: Error");
						goto cmd_false;
					}
					ESP_LOGI(TAG, "Rollback Firmware Command");
					break;
				case RST_ONLY_CMD:
					ESP_LOGI(TAG, "Reset Device Command");
					break;
				case RST_DEFL_CFG:
					err = write_config_default();
					if (err != ESP_OK)
					{
						ESP_LOGE(TAG, "Reset to Default Configuration: Error %d", err);
						goto cmd_false;
					}
					ESP_LOGI(TAG, "Reset to Default Command");
					break;
				case CLR_ARCH_CMD:
					err = clear_events_arch();
					if (err != ESP_OK)
					{
						ESP_LOGE(TAG, "Clear Events Archive: Error %d", err);
						goto cmd_false;
					}
					ESP_LOGI(TAG, "Clear Events Archive");
					break;
				case LD_UTC_CMD:
					save_utc_copy();
					clr_rst_state();
					ESP_LOGI(TAG, "Correct UTC Counter: OK!");
				default:
					goto cmd_false;
				}
				reboot_as_deep_sleep();
			}
		cmd_false:
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
	else
	{
		example_connect();
	}
	mb_serial_master_register();
	mb_tcp_master_register();
	if (slave_select == SLAVE_IEC_101_SERIAL)
	{
		int conn_iec;
		if (!is_mb_connect_use(SER1_CONN))
			conn_iec = SER1_CONN;
		else if (!is_mb_connect_use(SER2_CONN))
			conn_iec = SER2_CONN;
		else
		{
			slave_select = SLAVE_IEC_104_TCP;
			goto tcp_only;
		}
		ser_iec_slave_init(conn_iec);
		xTaskCreatePinnedToCore(ser_iec_slave_task,
								"ser_iec_slave_task",
								IEC_TASK_STACK_SIZE,
								NULL,
								1,
								NULL,
								tskNO_AFFINITY);
	}
	else /* slave_select == SLAVE_IEC_104_TCP */
	{
	tcp_only:
		tcp_iec_slave_init();
		xTaskCreatePinnedToCore(tcp_iec_slave_task,
								"tcp_iec_slave_task",
								IEC_TASK_STACK_SIZE,
								NULL,
								1,
								NULL,
								tskNO_AFFINITY);
	}
	mb_master_operation_func();
}