/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "bt_app_core.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
#include "esp_pbac_api.h"
#include "bt_app_hf.h"
#include "gpio_pcm_config.h"
#include "esp_console.h"
#include "app_hf_msg_set.h"
#include "bt_app_pbac.h"

// add by nishi 2025.9.14
#include "i2s_acc_std.h"

esp_bd_addr_t peer_addr = {0};
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static uint8_t peer_bdname_len;
static const char remote_device_name[] = CONFIG_EXAMPLE_PEER_DEVICE_NAME;

// add by nishi
const char *c_gap_evt_str[] = {
    "ESP_BT_GAP_DISC_RES_EVT",                    /*!< Device discovery result event */
    "ESP_BT_GAP_DISC_STATE_CHANGED_EVT",              /*!< Discovery state changed event */
    "ESP_BT_GAP_RMT_SRVCS_EVT",                       /*!< Get remote services event */
    "ESP_BT_GAP_RMT_SRVC_REC_EVT",                    /*!< Get remote service record event */
    "ESP_BT_GAP_AUTH_CMPL_EVT",                       /*!< Authentication complete event */
    "ESP_BT_GAP_PIN_REQ_EVT",                         /*!< Legacy Pairing Pin code request */
    "ESP_BT_GAP_CFM_REQ_EVT",                         /*!< Security Simple Pairing User Confirmation request. */
    "ESP_BT_GAP_KEY_NOTIF_EVT",                       /*!< Security Simple Pairing Passkey Notification */
    "ESP_BT_GAP_KEY_REQ_EVT",                         /*!< Security Simple Pairing Passkey request */
    "ESP_BT_GAP_READ_RSSI_DELTA_EVT",                 /*!< Read rssi event */
    "ESP_BT_GAP_CONFIG_EIR_DATA_EVT",                 /*!< Config EIR data event */
    "ESP_BT_GAP_SET_AFH_CHANNELS_EVT",                /*!< Set AFH channels event */
    "ESP_BT_GAP_READ_REMOTE_NAME_EVT",                /*!< Read Remote Name event */
    "ESP_BT_GAP_MODE_CHG_EVT",
    "ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT",         /*!< remove bond device complete event */
    "ESP_BT_GAP_QOS_CMPL_EVT",                        /*!< QOS complete event */
    "ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT",              /*!< ACL connection complete status event */
    "ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT",           /*!< ACL disconnection complete status event */
    "ESP_BT_GAP_SET_PAGE_TO_EVT",                     /*!< Set page timeout event */
    "ESP_BT_GAP_GET_PAGE_TO_EVT",                     /*!< Get page timeout event */
    "ESP_BT_GAP_ACL_PKT_TYPE_CHANGED_EVT",            /*!< Set ACL packet types event */
    "ESP_BT_GAP_ENC_CHG_EVT",                         /*!< Encryption change event */
    "ESP_BT_GAP_SET_MIN_ENC_KEY_SIZE_EVT",            /*!< Set minimum encryption key size */
    "ESP_BT_GAP_GET_DEV_NAME_CMPL_EVT",               /*!< Get device name complete event */
    "ESP_BT_GAP_EVT_MAX",
};


// add by nishi end

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    ESP_LOGI(BT_HF_TAG, "%s passed #1",__func__);

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    // add by nishi 2025.9.10
    if(event < ESP_BT_GAP_EVT_MAX)
        ESP_LOGI(BT_HF_TAG, "%s #1: %s",__func__,c_gap_evt_str[event]);
    else
        ESP_LOGI(BT_HF_TAG, "%s #1 event: %d",__func__,event);

    // 16: ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT  --> これが来た時なにか対応が必要では?

    // 16:ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT,              /*!< ACL connection complete status event */
    // 17:ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT,           /*!< ACL disconnection complete status event */


    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        for (int i = 0; i < param->disc_res.num_prop; i++){
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR
                && get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len)){
                if (strcmp(peer_bdname, remote_device_name) == 0) {
                    memcpy(peer_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                    ESP_LOGI(BT_HF_TAG, "Found a target device address:");
                    ESP_LOG_BUFFER_HEX(BT_HF_TAG, peer_addr, ESP_BD_ADDR_LEN);
                    ESP_LOGI(BT_HF_TAG, "Found a target device name: %s", peer_bdname);
                    printf("Connect.\n");
                    esp_hf_client_connect(peer_addr);
                    esp_bt_gap_cancel_discovery();
                }
            }
        }
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(BT_HF_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
    case ESP_BT_GAP_RMT_SRVCS_EVT:
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_HF_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            ESP_LOG_BUFFER_HEX(BT_HF_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(BT_HF_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        ESP_LOGI(BT_HF_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(BT_HF_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(BT_HF_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(BT_HF_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %06"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(BT_HF_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%06"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(BT_HF_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(BT_HF_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;

    default: {
        ESP_LOGI(BT_HF_TAG, "%s event: %d", __func__,event);
        break;
    }
    }
    return;
}

/* event for handler "bt_av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/* handler for bluetooth stack enabled events */
static void bt_hf_client_hdl_stack_evt(uint16_t event, void *p_param);

void app_main(void)
{
    char bda_str[18] = {0};
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(BT_HF_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    // ここまでは、~/Documents/esp-idf/Bluedroid_Connection/main/main.c と同じ

    // 下記が、~/Documents/esp-idf/Bluedroid_Connection/main/main.c と違う
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(BT_HF_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    // 下記が、Bluedroid_Connection と違う
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
#if (CONFIG_EXAMPLE_SSP_ENABLED == false)
    bluedroid_cfg.ssp_en = false;
#endif
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(BT_HF_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(BT_HF_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(BT_HF_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
    /* create application task */
    bt_app_task_start_up();


    /* Bluetooth device name, connection mode and profile set up */
    // xx_register_callback() は、下記の bt_hf_client_hdl_stack_evt で行っている。 by nishi 2025.9.12
    bt_app_work_dispatch(bt_hf_client_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);

#if CONFIG_BT_HFP_AUDIO_DATA_PATH_PCM
    /* configure the PCM interface and PINs used */
    app_gpio_pcm_io_cfg();
#endif

    /* configure external chip for acoustic echo cancellation */
#if ACOUSTIC_ECHO_CANCELLATION_ENABLE
    app_gpio_aec_io_cfg();
#endif /* ACOUSTIC_ECHO_CANCELLATION_ENABLE */
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    repl_config.prompt = "hfp_hf>";

    // init console REPL environment
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    /* Register commands */
    register_hfp_hf();
    printf("\n ==================================================\n");
    printf(" |       Steps to test hfp_hf                     |\n");
    printf(" |                                                |\n");
    printf(" |  1. Print 'help' to gain overview of commands  |\n");
    printf(" |  2. Setup a service level connection           |\n");
    printf(" |  3. Run hfp_hf to test                         |\n");
    printf(" |                                                |\n");
    printf(" =================================================\n\n");

    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

//------------
// xx_register_callback() 等の初期化処理 for hf_clinet
//------------
static void bt_hf_client_hdl_stack_evt(uint16_t event, void *p_param)
{
    //ESP_LOGD(BT_HF_TAG, "%s evt %d", __func__, event);
    ESP_LOGI(BT_HF_TAG, "%s evt %d", __func__, event);
    switch (event) {
    case BT_APP_EVT_STACK_UP: {
        /* set up device name */
        char *dev_name = "ESP_HFP_HF";
        esp_bt_gap_set_device_name(dev_name);

        /* register GAP callback function */
        esp_bt_gap_register_callback(esp_bt_gap_cb);
        
        // コールバック登録と初期化が、セットみたい。
        // hf_clinet 用の設定
        esp_hf_client_register_callback(bt_app_hf_client_cb);
        // 下のコールで、 hf_client としての動作になるみたい。
        esp_hf_client_init();

        // pbac 用の設定
        esp_pbac_register_callback(bt_app_pbac_cb);
        esp_pbac_init();

        // SSP: Secure Simple Pairing
#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

        esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
        esp_bt_pin_code_t pin_code;
        pin_code[0] = '0';
        pin_code[1] = '0';
        pin_code[2] = '0';
        pin_code[3] = '0';
        esp_bt_gap_set_pin(pin_type, 4, pin_code);

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

        /* start device discovery */
        ESP_LOGI(BT_HF_TAG, "Starting device discovery...");
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
        break;
    }
    default:
        ESP_LOGE(BT_HF_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}
