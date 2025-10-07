#include <esp_system.h>
#include <esp_crc.h>
#include <sys/param.h>
#include <esp_log.h>
#include "config.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "arch.h"

#define STORAGE_NAMESPACE "storage"
static const char *TAG = "config.c";
static const uint16_t VersionNum = 100;
const config_t CfgDefault = {
	/* Application Layer */
	3,			  /* ASDU Address */
	60,			  /* Point Status Timeout */
	ASDU_ADDR_2B, /* ASDU Address Size */
	IOA_SIZE_3B,  /* IOA Size */
	COT_SIZE_2B,  /* COT Size */
	DISABLE,	  /* Active Termination */
	ENABLE,		  /* Enable/disable spontaneous feature */
	TIME_NONE,	  /* General interrogation response time stamp */
	TIME_56_BITS, /* Event with/without time stamp */
	TIME_NONE,	  /* Measured value cyclic time stamp */
	0,			  /* Cyclic send measured value (normalized) 0-2073600 s; 0 for disable */
	0,			  /* Cyclic send measured value (scaled) */
	0,			  /* Cyclic send measured value (short floating point number) */
	10,			  /* Select timeout 0-600 s; 0 for executing only */
	0,			  /* Originator Address 0-255 */
	/* IEC 870-5-104 */
	30,			/* Timeout 0 */
	15,			/* Timeout 1 */
	10,			/* Timeout 2 */
	20,			/* Timeout 3 */
	12,			/* K Parameter */
	8,			/* W Parameter */
	2404,		/* TCP Port */
	0x1001A8C0, /* IP Address */
	/* IEC 870-5-101 Serial Connection */
	BAUD_115200, /* Baud Rate */
	PARITY_ODD,	 /* Parity */
	STOPBITS_1,	 /* Stop Bits */
	DATABITS_8,	 /* Data Bits */
	30,			 /* Timeout Frames Rx */
	3,			 /* Link layer retries */
	/* Link Layer [FT1.2] */
	UNBALANCED,	  /* Link Layer Mode */
	ASDU_ADDR_2B, /* Link Address Size */
	DISABLE,	  /* Use Single Char ACK */
	0,			  /* Link Address */
	5000,		  /* Timeout for ACK,ms */
	5000,		  /* Timeout Repead,ms */
	/* ModBus Serial CH#0/CH#1 */
	BAUD_115200, /* Baud Rate */
	BAUD_115200, /* Baud Rate */
	PARITY_NONE, /* Parity */
	PARITY_NONE, /* Parity */
	STOPBITS_1,	 /* Stop Bits */
	STOPBITS_1,	 /* Stop Bits */
	DATABITS_8,	 /* Data Bits */
	DATABITS_8,	 /* Data Bits */
	MB_RTU,		 /* RTU transmission mode */
	MB_RTU,		 /* RTU transmission mode */
	3,			 /* Retry before Timeout set */
	0,			 /* Initial delay,ms */
	500,		 /* Responde Timeout,ms */
	/* ModBus Ethernet */
	502,		/* TCP Port */
	502,		/* TCP Port */
	502,		/* TCP Port */
	502,		/* TCP Port */
	502,		/* TCP Port */
	502,		/* TCP Port */
	502,		/* TCP Port */
	502,		/* TCP Port */
	0x0A01A8C0, /* IP Address */
	0x0B01A8C0, /* IP Address */
	0x0C01A8C0, /* IP Address */
	0x0D01A8C0, /* IP Address */
	0x0E01A8C0, /* IP Address */
	0x0F01A8C0, /* IP Address */
	0x1001A8C0, /* IP Address */
	0x1101A8C0, /* IP Address */
	500,		/* Ethernet Responde Timeout,ms */
	3,			/* Channel Retry */
	/*  Gateway Configuration Table */
	0,			/* Configuration table lengt */
	{{0}},		/*  Gateway Configuration Table */
	6000,		/* Fault timeout 100 - 65535 ms def:6000 */
	30,			/* Sync time interval 0 - 65535 [min] default - 30 (0 - OFF) */
	100,		/* Swith Reserwed Channel delay time 0 - 65535 [sec] default - 100 (0 - OFF) */
	100,		/* Delay Between Polls, 0 - 65535 [ms] default - 100 */
	25000,		/* Serial Number */
	VersionNum, /* Version FW */
	0			/* False of Control Summ */
};

config_t RamCfg;
#define cfg_id_make(field) {#field, &RamCfg.field, sizeof(RamCfg.field)}
parsed_t ParseTab[PARSE_TAB_LEN] = {
	cfg_id_make(alAdrSize),
	cfg_id_make(AsduAdr),
	cfg_id_make(ioaSize),
	cfg_id_make(cotSize),
	cfg_id_make(tPSt),
	cfg_id_make(actTerm),
	cfg_id_make(tSel),
	cfg_id_make(tIrr),
	cfg_id_make(tEvt),
	cfg_id_make(tMval),
	cfg_id_make(spVal),
	cfg_id_make(tNorm),
	cfg_id_make(tScal),
	cfg_id_make(tFloat),
	cfg_id_make(orgAdr),
	cfg_id_make(t0),
	cfg_id_make(t1),
	cfg_id_make(t2),
	cfg_id_make(t3),
	cfg_id_make(K),
	cfg_id_make(W),
	cfg_id_make(ipAdr),
	cfg_id_make(tcpPort),
	cfg_id_make(Baud),
	cfg_id_make(Parity),
	cfg_id_make(stopBits),
	cfg_id_make(dataBits),
	cfg_id_make(chRetry),
	cfg_id_make(tRx),
	cfg_id_make(mbB0),
	cfg_id_make(mbB1),
	cfg_id_make(mbP0),
	cfg_id_make(mbP1),
	cfg_id_make(mbS0),
	cfg_id_make(mbS1),
	cfg_id_make(mbD0),
	cfg_id_make(mbD1),
	cfg_id_make(mbM0),
	cfg_id_make(mbM1),
	cfg_id_make(mbInDel),
	cfg_id_make(mbRetry),
	cfg_id_make(mbRespT),
	cfg_id_make(eP0),
	cfg_id_make(eP1),
	cfg_id_make(eP2),
	cfg_id_make(eP3),
	cfg_id_make(eP4),
	cfg_id_make(eP5),
	cfg_id_make(eP6),
	cfg_id_make(eP7),
	cfg_id_make(iP0),
	cfg_id_make(iP1),
	cfg_id_make(iP2),
	cfg_id_make(iP3),
	cfg_id_make(iP4),
	cfg_id_make(iP5),
	cfg_id_make(iP6),
	cfg_id_make(iP7),
	cfg_id_make(eRetry),
	cfg_id_make(eRespT),
	cfg_id_make(llMode),
	cfg_id_make(llAdrSize),
	cfg_id_make(llAdr),
	cfg_id_make(tAck),
	cfg_id_make(tRep),
	cfg_id_make(usACK),
	cfg_id_make(tLen),
	cfg_id_make(tFault),
	cfg_id_make(tSync),
	cfg_id_make(tSwith),
	cfg_id_make(tBetw),
	cfg_id_make(SerN), // Read Only
	cfg_id_make(VerFW) // Read Only
};

esp_err_t read_config(void)
{
    nvs_handle_t nvs_handle;
    size_t cfg_size = sizeof(RamCfg);
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
	if (err != ESP_OK)
	{
		ESP_LOGI(TAG, "read_config nvs_open error: %d", err);
		return err;
	}
	err = nvs_get_blob(nvs_handle, "cfg", &RamCfg, &cfg_size);
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
	{
		nvs_close(nvs_handle);
		ESP_LOGI(TAG, "CFG nvs_get_blob error: %d", err);
		goto write_default;
	}
	else
	{
		err = init_events_arch(nvs_handle);
		if (err != ESP_OK)
		{
			nvs_close(nvs_handle);
			ESP_LOGI(TAG, "init_events_arch() error: %d", err);
			return clear_events_arch();
		}
	}
	uint16_t ser;
	nvs_get_u16(nvs_handle, "serN", &ser);
	nvs_close(nvs_handle);
	if (RamCfg.crc16 != esp_crc16_le(0, (uint8_t const *)&RamCfg, sizeof(config_t) - sizeof(RamCfg.crc16)))
	{
	write_default:
		return write_config_default();
	}
#if 0
	RamCfg.SerN = 25000;
#else
	if (ser > 25000 && ser <= 30000)
		RamCfg.SerN = ser;
#endif
	RamCfg.VerFW = VersionNum;
	return ESP_OK;
}

esp_err_t write_config_default(void)
{
	RamCfg = CfgDefault;
	return write_config();
}

esp_err_t write_config(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
	{
		ESP_LOGI(TAG, "write_config nvs_open error: %d", err);
		goto err_exit;
	}
#if 0
	err = nvs_erase_all(nvs_handle);
#else
	err = nvs_erase_key(nvs_handle, "cfg");
#endif
	if (err != ESP_OK)
	{
		ESP_LOGI(TAG, "nvs_erase error: %d", err);
		goto err_exit;
	}
	RamCfg.crc16 = esp_crc16_le(0, (uint8_t const *)&RamCfg, sizeof(config_t) - sizeof(RamCfg.crc16));
	err = nvs_set_blob(nvs_handle, "cfg", &RamCfg, sizeof(RamCfg));
	if (err != ESP_OK)
	{
		ESP_LOGI(TAG, "nvs_set_blob error: %d", err);
		goto err_exit;
	}
	err = nvs_commit(nvs_handle);
	if (err != ESP_OK)
	{
		ESP_LOGI(TAG, "nvs_commit error: %d", err);
		goto err_exit;
	}
err_exit:
	nvs_close(nvs_handle);
	return err;
}

esp_err_t save_serial_key(uint16_t value)
{
	nvs_handle_t nvs_handle;
	esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
	if (err != ESP_OK)
	{
		ESP_LOGI(TAG, "write_config nvs_open error: %d", err);
		goto err_exit;
	}
	err = nvs_set_u16(nvs_handle, "serN", value);
	if (err != ESP_OK)
	{
		ESP_LOGI(TAG, "nvs_set_u16 error: %d", err);
		goto err_exit;
	}
	ESP_LOGI(TAG, "Serial: %d Save Complette", value);
	err = nvs_commit(nvs_handle);
	if (err != ESP_OK)
	{
		ESP_LOGI(TAG, "nvs_commit error: %d", err);
		goto err_exit;
	}
	RamCfg.SerN = value;
err_exit:
	nvs_close(nvs_handle);
	return err;
}