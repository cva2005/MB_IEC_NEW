#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include "esp_tls_crypto.h"
#include <esp_http_server.h>
#include "config.h"
#include "esp_vfs.h"
#include "cJSON.h"
#include "rtc.h"
#include "timer_1ms.h"
#include "webserver.h"
#include "switch_ota.h"
#include "ota_ws_update_esp.h"
#include "jsmn.h"
#include "arch.h"

typedef enum
{
	ROW_BODY = 0,
	ROW_END = 1,
	ROW_NEXT = 2,
} tparse_state_t;

#define STR_LEN 30
#define RST_ID "Res"
#define CLA_ID "clrA"
#define RBFW_ID "rbFW"
#define FACT_ID "fcFW"
#define IAPS_ID "IAPs"
#define SER_ID "SerN"
#define DEF_ID "dCfg"
#define CDT_ID "cDT"

#define DELAY_BEFORE_RESET 	100
#define READ_ONLY_PRM 		2
#define OTA_CHUNK_SIZE 		(0x2000)

static const char * TAG = "webserver";
static httpd_handle_t server = NULL;
static int table_len = 0;
RTC_DATA_ATTR static reset_state_t reset_state = RST_NOT_ACTIVE;
static stime_t res_time;
static int ota_size;		// ota firmware size
static int ota_start_chunk; // start address of http chunk
static int ota_started;		// ota download started

void start_reset_delay(reset_state_t new_state)
{
	reset_state = new_state;
	set_finish_time(DELAY_BEFORE_RESET, &res_time);
}

void clr_rst_state(void)
{
	reset_state = RST_NOT_ACTIVE;
}

reset_state_t get_rst_state(void)
{
	return reset_state;
}

bool is_reset_time_out(void)
{
	return ((reset_state != RST_NOT_ACTIVE) && is_time_out(&res_time));
}

static esp_err_t save_reg_table(char *p)
{
	if (table_len)
	{
		int i = RamCfg.tLen - table_len;
		table_len--;
		if (p[0] != '[')
			return ESP_FAIL;
		tparse_state_t pstate = ROW_BODY;
		for (int j = 0; j <= TBL_RW; j++)
		{
			char d[STR_LEN] = {0};
			int str_cnt = 0;
		next_el:
			p++;
			char a = *p;
			if (pstate == ROW_END)
				return ESP_OK;
			else if (pstate == ROW_BODY)
			{
				/*"CfgA":"[[1,2,2,4,0.1,1,0,1,16,1],[0,2,2,4,0.1,0,0,1,16,1]]"*/
				if (a == ']')
				{
					pstate = ROW_END;
					goto parse_item;
				}
				else if (a == ',')
				{
				parse_item:
					int ival = atoi(d);
					float fval = atof(d);
					ESP_LOGI(TAG, "J: %u, VAL: %s = %u = %f", j, d, ival, fval);
					switch (j)
					{
					case RW_REG_ADDR:
						RamCfg.CfgA[i].mb_reg_addr = ival;
						break;
					case RW_REG_TYPE:
						RamCfg.CfgA[i].mb_function = ival;
						break;
					case RW_IEC_TYPE:
						RamCfg.CfgA[i].iec_data_type = ival;
						break;
					case RW_SWAP_TYPE:
						RamCfg.CfgA[i].swap_data = ival;
						break;
					case RW_TRESHOLD:
						RamCfg.CfgA[i].threshold = fval;
						break;
					case RW_OBJ_ADDR:
						RamCfg.CfgA[i].iec_obj_addr = ival;
						break;
					case RW_HIGH_LIM:
						RamCfg.CfgA[i].high_limit = fval;
						break;
					case RW_SLAVE_ID:
						RamCfg.CfgA[i].mb_dev_id = ival;
						break;
					case RW_LOW_LIM:
						RamCfg.CfgA[i].low_limit = fval;
						break;
					case RW_OBJ_NUM:
						RamCfg.CfgA[i].iec_obj_num = ival;
						break;
					case RW_POLL_MS:
						RamCfg.CfgA[i].mb_poll_ms = ival;
						break;
					case RW_FAULT_VAL:
						RamCfg.CfgA[i].fault_val = fval;
						break;
					case RW_FAULT_TO:
						RamCfg.CfgA[i].fault_sec = ival;
						break;
					case RW_GROUP_NUM:
						RamCfg.CfgA[i].iec_group = ival;
						return ESP_OK;
					default:
						return ESP_FAIL;
					}
					continue;
				}
				else
				{
					d[str_cnt] = a;
					if (++str_cnt >= STR_LEN)
						return ESP_FAIL;
					goto next_el;
				}
			}
		}
	}
	return ESP_OK;
}

static esp_err_t cfg_rd_handler(httpd_req_t *req)
{
	static char json_response[1024] = { 0 };
	char *p = json_response;
	*p++ = '{';
	for (int i = 0; i < PARSE_TAB_LEN; i++)
	{
		int var;
		void *ptr = ParseTab[i].ptr;
		switch (ParseTab[i].size)
		{
		case sizeof(uint8_t):
			var = *((uint8_t *)ptr);
			break;
		case sizeof(uint16_t):
			var = *((uint16_t *)ptr);
			break;
		default: // sizeof(uint32_t)
			var = *((uint32_t *)ptr);
		}
		p += sprintf(p, "\"%s\":%u,", ParseTab[i].key, var);
		ESP_LOGD(TAG, "\"%s\":%u,", ParseTab[i].key, var);
	}
	p--; // remove last coma
	*p++ = '}';
	*p++ = 0;
	table_len = RamCfg.tLen;
	httpd_resp_set_type(req, "application/json");
	httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
	return httpd_resp_send(req, json_response, strlen(json_response));
}

httpd_uri_t cfg_rd = {
	.uri = "/cfg_rd",
	.method = HTTP_GET,
	.handler = cfg_rd_handler,
	.user_ctx = NULL
 };

static esp_err_t tbl_rd_handler(httpd_req_t *req)
{
	static char json_response[1024] = {0};
	char *p = json_response;
	*p++ = '{';
	if (table_len)
	{
		int i = RamCfg.tLen - table_len;
		table_len--;
		p += sprintf(p, "\"CfgA\":[");
		for (int j = 0; j < TBL_RW; j++)
		{
			switch (j)
			{
			case RW_REG_ADDR:
				p += sprintf(p, "%u,", RamCfg.CfgA[i].mb_reg_addr);
				break;
			case RW_REG_TYPE:
				p += sprintf(p, "%u,", RamCfg.CfgA[i].mb_function);
				break;
			case RW_IEC_TYPE:
				p += sprintf(p, "%u,", RamCfg.CfgA[i].iec_data_type);
				break;
			case RW_SWAP_TYPE:
				p += sprintf(p, "%u,", RamCfg.CfgA[i].swap_data);
				break;
			case RW_TRESHOLD:
				p += sprintf(p, "%f,", RamCfg.CfgA[i].threshold);
				break;
			case RW_OBJ_ADDR:
				p += sprintf(p, "%lu,", RamCfg.CfgA[i].iec_obj_addr);
				break;
			case RW_HIGH_LIM:
				p += sprintf(p, "%f,", RamCfg.CfgA[i].high_limit);
				break;
			case RW_SLAVE_ID:
				p += sprintf(p, "%u,", RamCfg.CfgA[i].mb_dev_id);
				break;
			case RW_LOW_LIM:
				p += sprintf(p, "%f,", RamCfg.CfgA[i].low_limit);
				break;
			case RW_OBJ_NUM:
				p += sprintf(p, "%u,", RamCfg.CfgA[i].iec_obj_num);
				break;
			case RW_POLL_MS:
				p += sprintf(p, "%lu,", RamCfg.CfgA[i].mb_poll_ms);
				break;
			case RW_FAULT_TO:
				p += sprintf(p, "%u,", RamCfg.CfgA[i].fault_sec);
				break;
			case RW_FAULT_VAL:
				p += sprintf(p, "%f,", RamCfg.CfgA[i].fault_val);
				break;
			case RW_GROUP_NUM:
				p += sprintf(p, "%u", RamCfg.CfgA[i].iec_group);
				break;
			}
		}
		*p++ = ']'; // end of array
	}
	*p++ = '}';
	*p++ = 0;
	httpd_resp_set_type(req, "application/json");
	httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
	return httpd_resp_send(req, json_response, strlen(json_response));
}

httpd_uri_t tbl_rd = {
	.uri = "/tbl_rd",
	.method = HTTP_GET,
	.handler = tbl_rd_handler,
	.user_ctx = NULL
};

static esp_err_t arc_rd_handler(httpd_req_t *req)
{
	static char json_response[1024] = {0};
	char *p = json_response;
	*p++ = '{';
	p = get_arch_json(p);
	*p++ = '}';
	*p++ = 0;
	httpd_resp_set_type(req, "application/json");
	httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
	return httpd_resp_send(req, json_response, strlen(json_response));
}

httpd_uri_t arc_rd = {
	.uri = "/arc_rd",
	.method = HTTP_GET,
	.handler = arc_rd_handler,
	.user_ctx = NULL
};

static esp_err_t cfg_wr_handler(httpd_req_t *req)
{
	char *buf = NULL;
	int ret, remaining = req->content_len;
	if (remaining > PARSE_BUF_LEN) return ESP_FAIL;
	buf = (char *)malloc(PARSE_BUF_LEN);
	while (remaining > 0) {
		/* Read the data for the request */
		if ((ret = httpd_req_recv(req,
			buf, remaining)) <= 0) {
			if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
				/* Retry receiving if timeout occurred */
				continue;
			}
			free(buf);
			return ESP_FAIL;
		}
		remaining -= ret;
		/* Log data received */
		ESP_LOGI(TAG, "%.*s", ret, buf);
	}
	cJSON *root = cJSON_Parse(buf);
	if (cJSON_HasObjectItem(root, FACT_ID))
	{
		start_reset_delay(FACTORY_LOAD);
	}
	else if (cJSON_HasObjectItem(root, RBFW_ID))
	{
		start_reset_delay(ROLLBACK_CMD);
	}
	else if (cJSON_HasObjectItem(root, RST_ID))
	{
		start_reset_delay(RST_ONLY_CMD);
	}
	else if (cJSON_HasObjectItem(root, DEF_ID))
	{
		start_reset_delay(RST_DEFL_CFG);
	}
	else if (cJSON_HasObjectItem(root, CLA_ID))
	{
		start_reset_delay(CLR_ARCH_CMD);
	}
	else if (cJSON_HasObjectItem(root, CDT_ID))
	{
		uint64_t time_ms = (uint64_t)atoi(cJSON_GetObjectItemCaseSensitive(root, CDT_ID)->valuestring);
		ESP_LOGD(TAG, "new utc = %llu, sec", time_ms);
		time_ms *= 1000;
		ESP_LOGD(TAG, "new utc = %llu, ms", time_ms);
		set_time_ms(time_ms);
		start_reset_delay(LD_UTC_CMD);
	}
	else if (cJSON_HasObjectItem(root, SER_ID))
	{
		int ser = atoi(cJSON_GetObjectItemCaseSensitive(root, SER_ID)->valuestring);
		save_serial_key(ser);
	}
	else
	{
		for (int i = 0; i < (PARSE_TAB_LEN - READ_ONLY_PRM); i++)
		{
			char *valuestr = cJSON_GetObjectItemCaseSensitive(root, ParseTab[i].key)->valuestring;
			int var = atoi(valuestr);
			ESP_LOGI(TAG, "JSON parse: %s = %d", ParseTab[i].key, var);
			void *ptr = ParseTab[i].ptr;
			switch (ParseTab[i].size)
			{
			case sizeof(uint8_t):
				*((uint8_t *)ptr) = var;
				break;
			case sizeof(uint16_t):
				*((uint16_t *)ptr) = var;
				break;
			default: // sizeof(uint32_t)
				*((uint32_t *)ptr) = var;
			}
		}
		table_len = RamCfg.tLen;
	}
	ESP_LOGI(TAG, "JSON parse END");
	cJSON_Delete(root);
	/* End of response */
	httpd_resp_send_chunk(req, NULL, 0);
	free(buf);
	ESP_LOGI(TAG, "Responde Send Complette");
	if (!RamCfg.tLen)
		return write_config();
	return ESP_OK;
}

static const httpd_uri_t cfg_wr = {
	.uri = "/cfg_wr",
	.method = HTTP_POST,
	.handler = cfg_wr_handler,
	.user_ctx = NULL
};

static esp_err_t tbl_wr_handler(httpd_req_t *req)
{
	char *buf = NULL;
	int ret, remaining = req->content_len;
	if (remaining > PARSE_BUF_LEN) return ESP_FAIL;
	buf = (char *)malloc(PARSE_BUF_LEN);
	while (remaining > 0) {
		/* Read the data for the request */
		if ((ret = httpd_req_recv(req,
			buf, remaining)) <= 0) {
			if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
				/* Retry receiving if timeout occurred */
				continue;
			}
			free(buf);
			return ESP_FAIL;
		}
		remaining -= ret;
		/* Log data received */
		ESP_LOGI(TAG, "%.*s", ret, buf);
	}
	cJSON *root = cJSON_Parse(buf);
	if (RamCfg.tLen) save_reg_table(cJSON_GetObjectItemCaseSensitive(root, "CfgA")->valuestring);
	ESP_LOGI(TAG, "JSON parse END");
	cJSON_Delete(root);
	/* End of response */
	httpd_resp_send_chunk(req, NULL, 0);
	free(buf);
	ESP_LOGI(TAG, "Responde Send Complette");
	if (table_len == 0)
		return write_config();
	return ESP_OK;
}

static const httpd_uri_t tbl_wr = {
	.uri = "/tbl_wr",
	.method = HTTP_POST,
	.handler = tbl_wr_handler,
	.user_ctx = NULL
};

static esp_err_t index_handler(httpd_req_t *req)
{
	extern const unsigned char index_html_gz_start[] asm("_binary_index_html_gz_start");
	extern const unsigned char index_html_gz_end[] asm("_binary_index_html_gz_end");
	size_t index_html_gz_len = index_html_gz_end - index_html_gz_start;

	httpd_resp_set_type(req, "text/html");
	httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
	esp_err_t err = httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_len);
	return err;
}

static const httpd_uri_t start = {
	.uri = "/",
	.method = HTTP_GET,
	.handler = index_handler,
	.user_ctx = NULL
};

static esp_err_t send_json_string(char *str, httpd_req_t *req)
{
	httpd_ws_frame_t ws_pkt;
	memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
	ws_pkt.type = HTTPD_WS_TYPE_TEXT;
	ws_pkt.payload = (uint8_t *)str;
	ws_pkt.len = strlen(str);
	return httpd_ws_send_frame(req, &ws_pkt);
}

// abort OTA, send error/cancel msg to ws
static void ota_error(httpd_req_t *req, char *code, char *msg)
{
	char json_str[128];
	ota_size = ota_start_chunk = ota_started = 0;
	abort_ota_ws();
	ESP_LOGE(TAG, "%s %s", code, msg);
	snprintf(json_str, sizeof(json_str), "{\"name\":\"%s\",\"value\":\"%s\"}", code, msg);
	send_json_string(json_str, req);
}

// simple json parse -> only one parametr name/val
static esp_err_t json_to_str_parm(char *jsonstr, char *nameStr, char *valStr) // распаковать строку json в пару  name/val
{
	int r; // количество токенов
	jsmn_parser p;
	jsmntok_t t[5]; // только 2 пары параметров и obj

	jsmn_init(&p);
	r = jsmn_parse(&p, jsonstr, strlen(jsonstr), t, sizeof(t) / sizeof(t[0]));
	if (r < 2)
	{
		valStr[0] = 0;
		nameStr[0] = 0;
		return ESP_FAIL;
	}
	strncpy(nameStr, jsonstr + t[2].start, t[2].end - t[2].start);
	nameStr[t[2].end - t[2].start] = 0;
	if (r > 3)
	{
		strncpy(valStr, jsonstr + t[4].start, t[4].end - t[4].start);
		valStr[t[4].end - t[4].start] = 0;
	}
	else
		valStr[0] = 0;
	return ESP_OK;
}

static esp_err_t ota_ws_handler(httpd_req_t *req)
{
	char json_key[64] = {0};
	char json_value[64] = {0};
	char json_str[128] = {0};

	httpd_ws_frame_t ws_pkt;
	uint8_t *buf = NULL;

	if (req->method == HTTP_GET)
	{
		ESP_LOGI(TAG, "Handshake done, the new connection was opened");
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
		if (check_ota_ws_rollback_enable()) // check rollback enable, send cmd to enable rollback dialog on html
		{
			snprintf(json_str, sizeof(json_str), "{\"name\":\"%s\",\"value\":\"%s\" }", OTA_CHECK_ROLLBACK, "true");
			send_json_string(json_str, req);
		}
#endif
		return ESP_OK;
	}
	memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
	// Set max_len = 0 to get the frame len
	esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
	if (ret != ESP_OK)
	{
		ota_error(req, OTA_ERROR, "httpd_ws_recv_frame failed to get frame len");
		return ret;
	}
	if (ws_pkt.len)
	{
		// ws_pkt.len + 1 is for NULL termination as we are expecting a string
		buf = calloc(1, ws_pkt.len + 1);
		if (buf == NULL)
		{
			ota_error(req, OTA_ERROR, "Failed to calloc memory for buf");
			return ESP_ERR_NO_MEM;
		}
		ws_pkt.payload = buf;
		// Set max_len = ws_pkt.len to get the frame payload
		ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
		if (ret != ESP_OK)
		{
			ota_error(req, OTA_ERROR, "httpd_ws_recv_frame failed");
			goto _recv_ret;
		}
	}
	ret = ESP_OK;
	if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) // process json cmd
	{
		if (json_to_str_parm((char *)buf, json_key, json_value)) // decode json to key/value parm
		{
			ota_error(req, OTA_ERROR, "Error json str");
			goto _recv_ret;
		}
		if (strncmp(json_key, OTA_SIZE_START, sizeof(OTA_SIZE_START)) == 0) // start ota
		{
			ota_size = atoi(json_value);
			if (ota_size == 0)
			{
				ota_error(req, OTA_ERROR, "Error ota size = 0");
				goto _recv_ret;
			}
			ret = start_ota_ws();
			if (ret)
			{
				ota_error(req, OTA_ERROR, "Error start ota");
				goto _recv_ret;
			}
			ota_started = 1;
			ota_start_chunk = 0;
			snprintf(json_str, sizeof(json_str), "{\"name\":\"%s\",\"value\":%d}",
					 OTA_SET_CHUNK_SIZE, OTA_CHUNK_SIZE); // set download chunk
			send_json_string(json_str, req);
			snprintf(json_str, sizeof(json_str), "{\"name\":\"%s\",\"value\":%d}",
					 OTA_GET_CHUNK, ota_start_chunk); // cmd -> send first chunk with start addresss = 0
			send_json_string(json_str, req);
		}
		if (strncmp(json_key, OTA_CANCEL, sizeof(OTA_CANCEL)) == 0) // cancel ota
		{
			ota_error(req, OTA_CANCEL, "Cancel command");
			ret = ESP_OK;
			goto _recv_ret;
		}
		if (strncmp(json_key, OTA_ERROR, sizeof(OTA_ERROR)) == 0) // error ota
		{
			ota_error(req, OTA_ERROR, "Error command");
			ret = ESP_OK;
			goto _recv_ret;
		}
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
		if (strncmp(json_key, OTA_PROCESS_ROLLBACK, sizeof(OTA_PROCESS_ROLLBACK)) == 0) // process rollback &
		{
			if (strncmp(json_value, "true", sizeof("true")) == 0)
			{
				ESP_LOGI(TAG, "Rollback and restart");
				ret = rollback_ota_ws(true); // rollback and restart
			}
			else
			{
				ESP_LOGI(TAG, "App veryfied, fix ota update");
				ret = rollback_ota_ws(false); // app veryfied
			}
			goto _recv_ret;
		}
#endif
		if (strncmp(json_key, OTA_RESTART_ESP, sizeof(OTA_RESTART_ESP)) == 0) // cancel ota
		{
			save_arch_event(FW_UPD);
			esp_restart();
		}
	}
	else if (ws_pkt.type == HTTPD_WS_TYPE_BINARY && ota_started) // download OTA firmware with chunked part
	{

		if (ota_start_chunk + ws_pkt.len < ota_size) // read chuk of ota
		{
			ret = write_ota_ws(ws_pkt.len, buf); // write chunk of ota
			if (ret != ESP_OK)
			{
				char *msg;
				if (ret == ESP_FAIL)
					msg = "Error write ota";
				else
					msg = "Binary File Corrupt!";
				ota_error(req, OTA_ERROR, msg);
				goto _recv_ret;
			}
			ota_start_chunk += ws_pkt.len;
			snprintf(json_str, sizeof(json_str), "{\"name\":\"%s\",\"value\": %d }",
					 OTA_GET_CHUNK, ota_start_chunk); // cmd -> next chunk
			send_json_string(json_str, req);
		}
		else // last chunk and end ota
		{
			ret = write_ota_ws(ws_pkt.len, buf); // write last chunk of ota
			if (ret)
			{
				ota_error(req, OTA_ERROR, "Error write ota");
				goto _recv_ret;
			}
			ret = end_ota_ws(); // end ota
			if (ret)
			{
				ota_error(req, OTA_ERROR, "Error end ota");
				goto _recv_ret;
			}
			ota_size = 0;
			ota_start_chunk = 0;
			ota_started = 0;
			ESP_LOGI(TAG, "OTA END OK");
			snprintf(json_str, sizeof(json_str), "{\"name\":\"%s\",\"value\":\"%s\" }",
					 OTA_END, "OK"); // send ota end cmd ( ota ok )
			send_json_string(json_str, req);
		}
	}
_recv_ret:
	free(buf);
	return ret;
}

static const httpd_uri_t ws = {
	.uri = "/ws",
	.method = HTTP_GET,
	.handler = ota_ws_handler,
	.user_ctx = NULL,
	.is_websocket = true
};

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/hello", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        /* Return ESP_FAIL to close underlying socket */
        return ESP_FAIL;
    }
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
	    httpd_register_uri_handler(server, &start);
		httpd_register_uri_handler(server, &cfg_wr);
		httpd_register_uri_handler(server, &tbl_wr);
		httpd_register_uri_handler(server, &cfg_rd);
		httpd_register_uri_handler(server, &tbl_rd);
		httpd_register_uri_handler(server, &arc_rd);
		httpd_register_uri_handler(server, &ws);
#if CONFIG_EXAMPLE_BASIC_AUTH
		httpd_register_basic_auth(server);
#endif
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
		httpd_stop(*server);
		*server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

void webserver_init(void)
{
	/* Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
	 * and re-start it upon connection.
	 */
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
}

void webserver_start(void)
{
    /* Start the server for the first time */
    server = start_webserver();
}