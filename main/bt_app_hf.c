/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"

#include "bt_app_core.h"
#include "bt_app_hf.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
// add by nishi
//#include "esp_hf_ag_api.h"

#include "esp_pbac_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "time.h"
#include "sys/time.h"
#include "sdkconfig.h"

// add by nishi 2025.9.11
#include "esp_timer.h"
#include "esp_err.h"
#include "osi/allocator.h"
#include "i2s_acc_std.h"


const char *c_hf_evt_str[] = {
    "CONNECTION_STATE_EVT",              /*!< connection state changed event */
    "AUDIO_STATE_EVT",                   /*!< audio connection state change event */
    "VR_STATE_CHANGE_EVT",                /*!< voice recognition state changed */
    "CALL_IND_EVT",                      /*!< call indication event */
    "CALL_SETUP_IND_EVT",                /*!< call setup indication event */
    "CALL_HELD_IND_EVT",                 /*!< call held indicator event */
    "NETWORK_STATE_EVT",                 /*!< network state change event */
    "SIGNAL_STRENGTH_IND_EVT",           /*!< signal strength indication event */
    "ROAMING_STATUS_IND_EVT",            /*!< roaming status indication event */
    "BATTERY_LEVEL_IND_EVT",             /*!< battery level indication event */
    "CURRENT_OPERATOR_EVT",              /*!< current operator name event */
    "RESP_AND_HOLD_EVT",                 /*!< response and hold event */
    "CLIP_EVT",                          /*!< Calling Line Identification notification event */
    "CALL_WAITING_EVT",                  /*!< call waiting notification */
    "CLCC_EVT",                          /*!< listing current calls event */
    "VOLUME_CONTROL_EVT",                /*!< audio volume control event */
    "AT_RESPONSE",                       /*!< audio volume control event */
    "SUBSCRIBER_INFO_EVT",               /*!< subscriber information event */
    "INBAND_RING_TONE_EVT",              /*!< in-band ring tone settings */
    "LAST_VOICE_TAG_NUMBER_EVT",         /*!< requested number from AG event */
    "RING_IND_EVT",                      /*!< ring indication event */
    "PKT_STAT_EVT",                      /*!< requested number of packet status event */
    "PROF_STATE_EVT",                    /*!< Indicate HF CLIENT init or deinit complete */
};

// esp_hf_client_connection_state_t
const char *c_connection_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "slc_connected",
    "disconnecting",
};

// esp_hf_client_audio_state_t
const char *c_audio_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "connected_msbc",
};

/// esp_hf_vr_state_t
const char *c_vr_state_str[] = {
    "disabled",
    "enabled",
};

// esp_hf_service_availability_status_t
const char *c_service_availability_status_str[] = {
    "unavailable",
    "available",
};

// esp_hf_roaming_status_t
const char *c_roaming_status_str[] = {
    "inactive",
    "active",
};

// esp_hf_client_call_state_t
const char *c_call_str[] = {
    "NO call in progress",
    "call in progress",
};

// esp_hf_client_callsetup_t
const char *c_call_setup_str[] = {
    "NONE",
    "INCOMING",
    "OUTGOING_DIALING",
    "OUTGOING_ALERTING"
};

// esp_hf_client_callheld_t
const char *c_call_held_str[] = {
    "NONE held",
    "Held and Active",
    "Held",
};

// esp_hf_response_and_hold_status_t
const char *c_resp_and_hold_str[] = {
    "HELD",
    "HELD ACCEPTED",
    "HELD REJECTED",
};

// esp_hf_client_call_direction_t
const char *c_call_dir_str[] = {
    "outgoing",
    "incoming",
};

// esp_hf_client_call_state_t
const char *c_call_state_str[] = {
    "active",
    "held",
    "dialing",
    "alerting",
    "incoming",
    "waiting",
    "held_by_resp_hold",
};

// esp_hf_current_call_mpty_type_t
const char *c_call_mpty_type_str[] = {
    "single",
    "multi",
};

// esp_hf_volume_control_target_t
const char *c_volume_control_target_str[] = {
    "SPEAKER",
    "MICROPHONE"
};

// esp_hf_at_response_code_t
const char *c_at_response_code_str[] = {
    "OK",
    "ERROR"
    "ERR_NO_CARRIER",
    "ERR_BUSY",
    "ERR_NO_ANSWER",
    "ERR_DELAYED",
    "ERR_BLACKLILSTED",
    "ERR_CME",
};

// esp_hf_subscriber_service_type_t
const char *c_subscriber_service_type_str[] = {
    "unknown",
    "voice",
    "fax",
};

// esp_hf_client_in_band_ring_state_t
const char *c_inband_ring_state_str[] = {
    "NOT provided",
    "Provided",
};

extern esp_bd_addr_t peer_addr;
// If you want to connect a specific device, add it's address here
// esp_bd_addr_t peer_addr = {0xac, 0x67, 0xb2, 0x53, 0x77, 0xbe};

//#if CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI && CONFIG_BT_HFP_USE_EXTERNAL_CODEC
#if defined(CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI)

static esp_hf_sync_conn_hdl_t s_sync_conn_hdl;
static bool s_msbc_air_mode = false;
QueueHandle_t s_audio_buff_queue = NULL;
static int s_audio_buff_cnt = 0;

// start add by nishi from hfp_ag/bt_app_hf.c


#define ESP_HFP_RINGBUF_SIZE 3600

// 7500 microseconds(=12 slots) is aligned to 1 msbc frame duration, and is multiple of common Tesco for eSCO link with EV3 or 2-EV3 packet type
#define PCM_BLOCK_DURATION_US        (7500)

#define WBS_PCM_SAMPLING_RATE_KHZ    (16)
#define PCM_SAMPLING_RATE_KHZ        (8)

#define BYTES_PER_SAMPLE             (2)

// input can refer to Enhanced Setup Synchronous Connection Command in core spec4.2 Vol2, Part E
#define WBS_PCM_INPUT_DATA_SIZE  (WBS_PCM_SAMPLING_RATE_KHZ * PCM_BLOCK_DURATION_US / 1000 * BYTES_PER_SAMPLE) //240
#define PCM_INPUT_DATA_SIZE      (PCM_SAMPLING_RATE_KHZ * PCM_BLOCK_DURATION_US / 1000 * BYTES_PER_SAMPLE)     //120

//#define PCM_GENERATOR_TICK_US        (4000)         // 1000 / 4 = 250[Hz]  --> オリジナル値  i2s の速度に対して、遅い
//#define PCM_GENERATOR_TICK_US        (3773)         // 1000 / 3.773 = 265[Hz]  --> i2s の速度に対して、遅い
//#define PCM_GENERATOR_TICK_US        (3759)         // 1000 / 3.759 = 266[Hz]  --> i2s に対して、少し遅い。
#define PCM_GENERATOR_TICK_US        (3745)         // 1000 / 3.745 = 267[Hz]  -->  i2s が少しだけ遅い。 
//#define PCM_GENERATOR_TICK_US        (3703)         // 1000 / 3.703 = 270[Hz]  --> ちょっと早い!! i2s の速度に対応しているが、送信でエラーがでる。
// 1000 / 4 = 250[Hz]
// 16000*2 [Byte] Per Sec のデータを、250[Hz] で送信するには、 1[cycle] でのデータ量は、 128[Byte]

static RingbufHandle_t s_m_rb = NULL;

static uint64_t s_time_new, s_time_old;
static esp_timer_handle_t s_periodic_timer;
static uint64_t s_last_enter_time, s_now_enter_time;
static uint64_t s_us_duration;

static SemaphoreHandle_t s_send_data_Semaphore = NULL;
static TaskHandle_t s_bt_app_send_data_task_handler = NULL;

static size_t item_size_one;
static bool xRing_busy=false;

static uint32_t bt_app_hf_outgoing_cb(uint8_t *p_buf, uint32_t sz)
{
    size_t item_size = 0;
    uint8_t *data;
    if (!s_m_rb) {
        return 0;
    }
    vRingbufferGetInfo(s_m_rb, NULL, NULL, NULL, NULL, &item_size);
    if (item_size >= sz) {
        data = xRingbufferReceiveUpTo(s_m_rb, &item_size, 0, sz);
        memcpy(p_buf, data, item_size);
        vRingbufferReturnItem(s_m_rb, data);
        return sz;
    } 
    else {
        // data not enough, do not read\n
        return 0;
    }
    return 0;
}

static void bt_app_hf_incoming_cb(const uint8_t *buf, uint32_t sz)
{
    s_time_new = esp_timer_get_time();
    //s_data_num += sz;
    if ((s_time_new - s_time_old) >= 3000000) {
        //print_speed();
    }
}


//#define USE_TEST_SOUND
#if defined(USE_TEST_SOUND)
    #define TABLE_SIZE         100
    #define TABLE_SIZE_BYTE    200
    // Produce a sine audio
    static const int16_t sine_int16[TABLE_SIZE] = {
        0,    2057,    4107,    6140,    8149,   10126,   12062,   13952,   15786,   17557,
    19260,   20886,   22431,   23886,   25247,   26509,   27666,   28714,   29648,   30466,
    31163,   31738,   32187,   32509,   32702,   32767,   32702,   32509,   32187,   31738,
    31163,   30466,   29648,   28714,   27666,   26509,   25247,   23886,   22431,   20886,
    19260,   17557,   15786,   13952,   12062,   10126,    8149,    6140,    4107,    2057,
        0,   -2057,   -4107,   -6140,   -8149,  -10126,  -12062,  -13952,  -15786,  -17557,
    -19260,  -20886,  -22431,  -23886,  -25247,  -26509,  -27666,  -28714,  -29648,  -30466,
    -31163,  -31738,  -32187,  -32509,  -32702,  -32767,  -32702,  -32509,  -32187,  -31738,
    -31163,  -30466,  -29648,  -28714,  -27666,  -26509,  -25247,  -23886,  -22431,  -20886,
    -19260,  -17557,  -15786,  -13952,  -12062,  -10126,   -8149,   -6140,   -4107,   -2057,
    };


    static uint32_t bt_app_hf_create_audio_data(uint8_t *p_buf, uint32_t sz)
    {
        static int index = 0;
        uint8_t *data = (uint8_t *)sine_int16;

        for (uint32_t i = 0; i < sz; i++) {
            p_buf[i] = data[index++];
            if (index >= TABLE_SIZE_BYTE) {
                index -= TABLE_SIZE_BYTE;
            }
        }

        return sz;
    }
#else
    static int t_cnt=0;

#endif


static bool bt_app_send_data_task_f=false;

static void bt_app_send_data_timer_cb(void *arg)
{
    if (!xSemaphoreGive(s_send_data_Semaphore)) {
        if(xRing_busy != true){
            ESP_LOGE(BT_HF_TAG, "%s xSemaphoreGive failed", __func__);
        }
        return;
    }
    return;
}

#define MAX_A_AUDIO_SLENG 57
//#define MAX_A_AUDIO_TIMER_MS 10
//#define MAX_A_AUDIO_TIMER_MS 5

//------------
// Mic の送信処理
// Source 処理  add by nishi
//------------
static void bt_app_send_data_task(void *arg)
{
    uint64_t frame_data_num;
    //size_t item_size = 0;
    size_t remain_item_size = 0;
    uint8_t *buf = NULL;
    size_t buf_busy_cnt=0;

    esp_hf_audio_buff_t *audio_data_to_send;

    xRing_busy=false;

    for (bt_app_send_data_task_f=true;;) {
        if (xSemaphoreTake(s_send_data_Semaphore, (TickType_t)portMAX_DELAY)) {

            s_now_enter_time = esp_timer_get_time();
            s_us_duration = s_now_enter_time - s_last_enter_time;

            //if ( s_us_duration >= pdMS_TO_TICKS(MAX_A_AUDIO_TIMER_MS)) {

                //ESP_LOGI(BT_HF_TAG, "%s #2 wake",__func__);
                #if defined(USE_ORG_SOURCE_1)
                    if (s_msbc_air_mode == true){
                        // time of a frame is 7.5ms, sample is 120, data is 2 (byte/sample), so a frame is 240 byte (HF_SBC_ENC_RAW_DATA_SIZE)
                        frame_data_num = s_us_duration / PCM_BLOCK_DURATION_US * WBS_PCM_INPUT_DATA_SIZE;
                        //s_last_enter_time += frame_data_num / WBS_PCM_INPUT_DATA_SIZE * PCM_BLOCK_DURATION_US;
                    } 
                    else {
                        frame_data_num = s_us_duration / PCM_BLOCK_DURATION_US * PCM_INPUT_DATA_SIZE;
                        //s_last_enter_time += frame_data_num / PCM_INPUT_DATA_SIZE * PCM_BLOCK_DURATION_US;
                    }
                    if (frame_data_num < MAX_A_AUDIO_SLENG) {
                        //ESP_LOGE(BT_HF_TAG, "%s #3 frame_data_num:%d", __FUNCTION__,frame_data_num);
                        continue;
                    }
                    if(frame_data_num > MAX_A_AUDIO_SLENG){
                        frame_data_num=MAX_A_AUDIO_SLENG;
                    }
                #else
                    frame_data_num = (uint64_t)((double)s_us_duration / (double)PCM_BLOCK_DURATION_US * (double)item_size_one);
                    // fetch data が足りない時は、次回にする。
                    //if (frame_data_num < (uint64_t)((double)item_size_one * 0.7)) {
                    if (frame_data_num < (uint64_t)((double)item_size_one * 0.65)) {
                        //ESP_LOGE(BT_HF_TAG, "%s #3 frame_data_num:%d", __FUNCTION__,frame_data_num);
                        continue;
                    }
                    else{
                        //ESP_LOGE(BT_HF_TAG, "%s #3.1 frame_data_num:%d", __FUNCTION__,frame_data_num);
                        frame_data_num = item_size_one;
                    }
                #endif


                //ESP_LOGI(BT_HF_TAG, "%s #3 frame_data_num:%d",__func__,frame_data_num);

                if(bt_app_send_data_task_f !=true)
                    continue;

                //frame_data_num=240;
                frame_data_num = item_size_one;
                buf = osi_malloc(frame_data_num);
                if (!buf) {
                    ESP_LOGE(BT_HF_TAG, "%s #4 no mem", __FUNCTION__);
                    continue;
                }

                #if defined(USE_TEST_SOUND)
                    // Test Audio Data 作成
                    bt_app_hf_create_audio_data(buf, frame_data_num);
                #else
                    int limit_cnt = 5;
                    int conv_type=1;
                    int c_cnt=1;
                    size_t bytes_read =i2s_std_getSample(buf,frame_data_num,conv_type);
                    while(bytes_read==0){
                        //t_cnt++;
                        //if(t_cnt > 50){
                              vTaskDelay(1);
                        //    t_cnt=0;
                        //}
                        if(frame_data_num != bytes_read){
                            ESP_LOGI(BT_HF_TAG, "%s #5 bytes_read=%d", __func__,bytes_read);
                        }
                        bytes_read =i2s_std_getSample(buf,frame_data_num,conv_type);
                        if(bytes_read >0)
                            break;
                        c_cnt++;
                        if(c_cnt >= limit_cnt){
                            break;
                        }
                    }
                    if(bytes_read==0){
                        ESP_LOGI(BT_HF_TAG, "%s #5.1 read failed", __func__);
                        osi_free(buf);
                        continue;
                    }              
                #endif

                //s_last_enter_time = s_now_enter_time + frame_data_num / WBS_PCM_INPUT_DATA_SIZE * PCM_BLOCK_DURATION_US;
                s_last_enter_time = s_now_enter_time;

                vRingbufferGetInfo(s_m_rb, NULL, NULL, NULL, NULL, &remain_item_size);

                if(remain_item_size <= item_size_one*3){
                    BaseType_t done = xRingbufferSend(s_m_rb, buf, frame_data_num, 0);
                    if (!done) {
                        ESP_LOGE(BT_HF_TAG, "%s #6 rb send fail",__func__);
                        xRing_busy=true;
                    }
                    else
                        xRing_busy=false;

                    buf_busy_cnt=0;
                }
                else{
                    buf_busy_cnt++;
                    xRing_busy=true;
                    if(buf_busy_cnt > 10){
                        ESP_LOGE(BT_HF_TAG, "%s #7 send buff busy",__func__);
                        ESP_LOGI(BT_HF_TAG, "%s #8 remain_item_size:%d",__func__,remain_item_size);
                        buf_busy_cnt=0;
                    }
                }

                osi_free(buf);

                //if(item_size > item_size_one){
                //    ESP_LOGI(BT_HF_TAG, "%s #8 item_size:%d",__func__,item_size);
                //}

                // grep -rnH "esp_hf_" ./ | grep "outgoing_data_ready"
                // ./bt/host/bluedroid/api/esp_hf_client_api.c:554:void esp_hf_client_outgoing_data_ready(void)
                if(remain_item_size >= item_size_one) {
                    //ESP_LOGI(BT_HF_TAG, "%s #9 ",__func__);
                    esp_hf_client_outgoing_data_ready();
                }
            //}
        }
        //vTaskDelay(1);
    }
}

void bt_app_send_data(void)
{
    //bt_app_send_data_task_f=true;

    // add by nishi 2025.9.14
    i2s_std_init(item_size_one);

    // add by nishi 2025.9.14
    i2s_std_start();

    s_send_data_Semaphore = xSemaphoreCreateBinary();
    //xTaskCreate(bt_app_send_data_task, "BtAppSendDataTask", 2048, NULL, configMAX_PRIORITIES - 3, &s_bt_app_send_data_task_handler);
    xTaskCreate(bt_app_send_data_task, "BtAppSendDataTask", 4096, NULL, configMAX_PRIORITIES - 3, &s_bt_app_send_data_task_handler);
    s_m_rb = xRingbufferCreate(ESP_HFP_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    const esp_timer_create_args_t c_periodic_timer_args = {
            .callback = &bt_app_send_data_timer_cb,
            .name = "periodic"
    };
    ESP_ERROR_CHECK(esp_timer_create(&c_periodic_timer_args, &s_periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_periodic_timer, PCM_GENERATOR_TICK_US));
    s_last_enter_time = esp_timer_get_time();
    return;
}

void bt_app_send_data_shut_down(void)
{

    ESP_LOGI(BT_HF_TAG, "%s #1 ",__func__);

    bt_app_send_data_task_f=false;

    // add by nishi 2025.9.14
    i2s_std_stop();

    if (s_bt_app_send_data_task_handler) {
        vTaskDelete(s_bt_app_send_data_task_handler);
        s_bt_app_send_data_task_handler = NULL;
    }
    if(s_periodic_timer) {
        ESP_ERROR_CHECK(esp_timer_stop(s_periodic_timer));
        ESP_ERROR_CHECK(esp_timer_delete(s_periodic_timer));
    }
    if (s_send_data_Semaphore) {
        vSemaphoreDelete(s_send_data_Semaphore);
        s_send_data_Semaphore = NULL;
    }
    if (s_m_rb) {
        vRingbufferDelete(s_m_rb);
    }
    return;
}
// end add by nishi from hfp_ag/bt_app_hf.c


//--------------
// ここが Speraker データの受信処理 by nishi 2025.9.10
//--------------
// Rece Data event proc
// Speaker sink 処理
static void bt_app_hf_client_audio_data_cb(esp_hf_sync_conn_hdl_t sync_conn_hdl, esp_hf_audio_buff_t *audio_buf, bool is_bad_frame)
{
    // add by nishi 2025.9.10
    ESP_LOGI(BT_HF_TAG, "%s #1",__func__);

    if (is_bad_frame) {
        esp_hf_client_audio_buff_free(audio_buf);
        return;
    }

    esp_hf_client_audio_buff_free(audio_buf);
    return;

    ESP_LOGI(BT_HF_TAG, "%s #2",__func__);

    // ここで、処理している。データの受信ではないのか?
    if (s_audio_buff_queue && xQueueSend(s_audio_buff_queue, &audio_buf, 0)) {
        s_audio_buff_cnt++;
    }
    else {
        esp_hf_client_audio_buff_free(audio_buf);
    }

    /* cache some data to add latency */
    if (s_audio_buff_cnt < 20) {
        return;
    }

    ESP_LOGI(BT_HF_TAG, "%s #3",__func__);

    esp_hf_audio_buff_t *audio_data_to_send;
    if (!xQueueReceive(s_audio_buff_queue, &audio_data_to_send, 0)) {
        return;
    }

    ESP_LOGI(BT_HF_TAG, "%s #4",__func__);

    s_audio_buff_cnt--;
    if (s_msbc_air_mode && audio_data_to_send->data_len > ESP_HF_MSBC_ENCODED_FRAME_SIZE) {
        /*
         * in mSBC air mode, we may receive a mSBC frame with some padding bytes at the end,
         * but esp_hf_client_audio_data_send API do not allow adding padding bytes at the end,
         * so we need to remove those padding bytes before send back to peer device.
         */
        audio_data_to_send->data_len = ESP_HF_MSBC_ENCODED_FRAME_SIZE;
    }

    ESP_LOGI(BT_HF_TAG, "%s #5",__func__);

    /* send audio data back to AG */
    if (esp_hf_client_audio_data_send(s_sync_conn_hdl, audio_data_to_send) != ESP_OK) {
        esp_hf_client_audio_buff_free(audio_data_to_send);
        ESP_LOGW(BT_HF_TAG, "fail to send audio data");
    }
}

#endif /* #if CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI && CONFIG_BT_HFP_USE_EXTERNAL_CODEC */

//-----------------------
// callback for HF_CLIENT
// hf_client としての処理みたい。
//-----------------------
void bt_app_hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    if (event <= ESP_HF_CLIENT_PROF_STATE_EVT) {
        ESP_LOGI(BT_HF_TAG, "%s #1 APP HFP event: %s",__func__,c_hf_evt_str[event]);
    } 
    else {
        ESP_LOGE(BT_HF_TAG, "%s #1 APP HFP invalid event %d",__func__, event);
    }

    switch (event) {
        case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--connection state %s, peer feats 0x%"PRIx32", chld_feats 0x%"PRIx32,
                    c_connection_state_str[param->conn_stat.state],
                    param->conn_stat.peer_feat,
                    param->conn_stat.chld_feat);
            memcpy(peer_addr,param->conn_stat.remote_bda,ESP_BD_ADDR_LEN);
            if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED) {
                esp_pbac_connect(peer_addr);
            }
            break;
        }

        case ESP_HF_CLIENT_AUDIO_STATE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--audio state %s",c_audio_state_str[param->audio_stat.state]);

            // ここで、Mic アクセスの仕掛けが必要か? by nishi 2025.910
            //#if defined(CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI) && defined(CONFIG_BT_HFP_USE_EXTERNAL_CODEC)
            #if defined(CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI)
                if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
                    s_msbc_air_mode = true;
                    ESP_LOGI(BT_HF_TAG, "--audio air mode: mSBC , preferred_frame_size: %d", param->audio_stat.preferred_frame_size);
                }
                else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED) {
                    s_msbc_air_mode = false;
                    ESP_LOGI(BT_HF_TAG, "--audio air mode: CVSD , preferred_frame_size: %d", param->audio_stat.preferred_frame_size);
                }

                if (s_msbc_air_mode == true){
                    item_size_one = WBS_PCM_INPUT_DATA_SIZE;
                }
                else{
                    item_size_one = PCM_INPUT_DATA_SIZE;
                }

                // audio connected
                if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
                    param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
                    s_sync_conn_hdl = param->audio_stat.sync_conn_handle;
                    s_audio_buff_queue = xQueueCreate(50, sizeof(esp_hf_audio_buff_t*));

                    // incoming data and outgoing data の中継が必要みたい。add by nishi 2025.9.13
                    esp_hf_client_register_data_callback(bt_app_hf_incoming_cb,bt_app_hf_outgoing_cb);

                    // sink 処理  元々あった処理。
                    esp_hf_client_register_audio_data_callback(bt_app_hf_client_audio_data_cb);
                    // source 処理 add by nishi 2025.9.11
                    bt_app_send_data();
                }
                // audio disconnected 
                else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED) {
                    s_sync_conn_hdl = 0;
                    s_msbc_air_mode = false;
                    esp_hf_audio_buff_t *buff_to_free = NULL;
                    while (xQueueReceive(s_audio_buff_queue, &buff_to_free, 0)) {
                        esp_hf_client_audio_buff_free(buff_to_free);
                    }
                    vQueueDelete(s_audio_buff_queue);
                    s_audio_buff_cnt = 0;

                    // add by nishi 2025.9.11
                    bt_app_send_data_shut_down();
                }

            #endif /* #if CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI && CONFIG_BT_HFP_USE_EXTERNAL_CODEC */
            break;
        }

        case ESP_HF_CLIENT_BVRA_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--VR state %s",
                    c_vr_state_str[param->bvra.value]);
            break;
        }

        case ESP_HF_CLIENT_CIND_SERVICE_AVAILABILITY_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--NETWORK STATE %s",
                    c_service_availability_status_str[param->service_availability.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_ROAMING_STATUS_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--ROAMING: %s",
                    c_roaming_status_str[param->roaming.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_SIGNAL_STRENGTH_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "-- signal strength: %d",
                    param->signal_strength.value);
            break;
        }

        case ESP_HF_CLIENT_CIND_BATTERY_LEVEL_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--battery level %d",
                    param->battery_level.value);
            break;
        }

        case ESP_HF_CLIENT_COPS_CURRENT_OPERATOR_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--operator name: %s",
                    param->cops.name);
            break;
        }

        case ESP_HF_CLIENT_CIND_CALL_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Call indicator %s",
                    c_call_str[param->call.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Call setup indicator %s",
                    c_call_setup_str[param->call_setup.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_CALL_HELD_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Call held indicator %s",
                    c_call_held_str[param->call_held.status]);
            break;
        }

        case ESP_HF_CLIENT_BTRH_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--response and hold %s",
                    c_resp_and_hold_str[param->btrh.status]);
            break;
        }

        case ESP_HF_CLIENT_CLIP_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--clip number %s",
                    (param->clip.number == NULL) ? "NULL" : (param->clip.number));
            break;
        }

        case ESP_HF_CLIENT_CCWA_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--call_waiting %s",
                    (param->ccwa.number == NULL) ? "NULL" : (param->ccwa.number));
            break;
        }

        case ESP_HF_CLIENT_CLCC_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Current call: idx %d, dir %s, state %s, mpty %s, number %s",
                    param->clcc.idx,
                    c_call_dir_str[param->clcc.dir],
                    c_call_state_str[param->clcc.status],
                    c_call_mpty_type_str[param->clcc.mpty],
                    (param->clcc.number == NULL) ? "NULL" : (param->clcc.number));
            break;
        }

        case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--volume_target: %s, volume %d",
                    c_volume_control_target_str[param->volume_control.type],
                    param->volume_control.volume);
            break;
        }

        case ESP_HF_CLIENT_AT_RESPONSE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--AT response event, code %d, cme %d",
                    param->at_response.code, param->at_response.cme);
            break;
        }

        case ESP_HF_CLIENT_CNUM_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--subscriber type %s, number %s",
                    c_subscriber_service_type_str[param->cnum.type],
                    (param->cnum.number == NULL) ? "NULL" : param->cnum.number);
            break;
        }

        case ESP_HF_CLIENT_BSIR_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--inband ring state %s",
                    c_inband_ring_state_str[param->bsir.state]);
            break;
        }

        case ESP_HF_CLIENT_BINP_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--last voice tag number: %s",
                    (param->binp.number == NULL) ? "NULL" : param->binp.number);
            break;
        }
        case ESP_HF_CLIENT_PKT_STAT_NUMS_GET_EVT:
        {
            ESP_LOGE(BT_HF_TAG, "ESP_HF_CLIENT_PKT_STAT_NUMS_GET_EVT: %d", event);
            break;
        }
        case ESP_HF_CLIENT_PROF_STATE_EVT:
        {
            if (ESP_HF_INIT_SUCCESS == param->prof_stat.state) {
                ESP_LOGI(BT_HF_TAG, "HF PROF STATE: Init Complete");
            } else if (ESP_HF_DEINIT_SUCCESS == param->prof_stat.state) {
                ESP_LOGI(BT_HF_TAG, "HF PROF STATE: Deinit Complete");
            } else {
                ESP_LOGE(BT_HF_TAG, "HF PROF STATE error: %d", param->prof_stat.state);
            }
            break;
        }
        default:
            ESP_LOGE(BT_HF_TAG, "HF_CLIENT EVT: %d", event);
            break;
    }
}
