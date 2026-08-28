/** =================================================================*
 * @file   usb_link.c
 * @brief  USB CDC通信
 * ================================================================= */
#include "usb_link.h"                                       /* USB CDC通信API */

#include "app_config.h"                                     /* USB送信タイムアウト */
#include "esp_check.h"                                      /* ESP-IDFエラー検査マクロ */
#include "freertos/FreeRTOS.h"                              /* FreeRTOS基本型 */
#include "freertos/queue.h"                                 /* FreeRTOSキューAPI */
#include "freertos/task.h"                                  /* FreeRTOSタスクAPI */
#include "tinyusb.h"                                        /* TinyUSB初期化API */
#include "tinyusb_cdc_acm.h"                                /* TinyUSB CDC ACM API */
#include "tinyusb_default_config.h"                         /* TinyUSB既定設定 */
#include "tusb.h"                                           /* TinyUSB接続状態API */
#include <string.h>                                         /* メモリーコピーAPI */

static bool s_previous_mounted;                             /**< 前回のUSB接続状態 */
static QueueHandle_t s_rx_queue;                            /**< CPU0からの受信キュー */
static uint32_t s_rx_drop_count;                            /**< 受信キュー破棄数 */

typedef struct {
    size_t length;                                          /**< 受信データ長[byte] */
    uint8_t data[CONFIG_TINYUSB_CDC_RX_BUFSIZE];            /**< 受信データ */
} usb_link_rx_message_t;                                    /**< USB受信キュー要素 */

/** =================================================================*
 * @brief  TinyUSB CDC受信処理
 * @param[in] interface CDCインターフェース番号
 * @param[in] event TinyUSBイベント情報
 * ================================================================= */
static void usb_link_rx_callback(int interface, cdcacm_event_t * event) {
    (void) event;

    usb_link_rx_message_t message = {0};
    esp_err_t const err = tinyusb_cdcacm_read(interface, message.data, sizeof(message.data), &message.length);
    if ((err != ESP_OK) || (message.length == 0U)) {
        return;
    }

    if (xQueueSend(s_rx_queue, &message, 0U) != pdPASS) {
        s_rx_drop_count++;
    }
}

/** =================================================================*
 * @brief  USB CDC初期化
 * @return TinyUSB初期化結果
 * ================================================================= */
esp_err_t usb_link_init(void) {
    s_rx_queue = xQueueCreate(4U, sizeof(usb_link_rx_message_t));
    if (s_rx_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    tinyusb_config_t const usb_config = TINYUSB_DEFAULT_CONFIG();
    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&usb_config), "usb_link", "TinyUSB initialization failed");

    tinyusb_config_cdcacm_t const cdc_config = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = usb_link_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    return tinyusb_cdcacm_init(&cdc_config);
}

/** =================================================================*
 * @brief  USBホスト接続状態取得
 * @return USBホスト接続済みならtrue
 * ================================================================= */
bool usb_link_is_mounted(void) {
    return tud_mounted();
}

/** =================================================================*
 * @brief  新規USB接続検出
 * @return 前回未接続から接続済みへ遷移した場合true
 * ================================================================= */
bool usb_link_take_new_session(void) {
    bool const mounted = usb_link_is_mounted();
    bool const new_session = mounted && !s_previous_mounted;
    s_previous_mounted = mounted;
    return new_session;
}

/** =================================================================*
 * @brief  USB CDC送信
 * @param[in] data 送信データ
 * @param[in] length 送信長[byte]
 * @return USB CDC送信結果
 * ================================================================= */
esp_err_t usb_link_send(uint8_t const * data, size_t length) {
    if ((data == NULL) || (length == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!usb_link_is_mounted()) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t offset = 0U;
    TickType_t const deadline = xTaskGetTickCount() + pdMS_TO_TICKS(APP_USB_TX_TIMEOUT_MS);

    while (offset < length) {
        size_t const queued = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, &data[offset], length - offset);
        offset += queued;
        (void) tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0U);

        if (offset == length) {
            return ESP_OK;
        }
        if ((int32_t) (xTaskGetTickCount() - deadline) >= 0) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1U);
    }

    return ESP_OK;
}

/** =================================================================*
 * @brief  USB CDC受信
 * @param[out] data 受信データ格納先
 * @param[in] capacity 格納先容量[byte]
 * @param[out] received_length 実際の受信長[byte]
 * @param[in] timeout_ms 受信待ち時間[ms]
 * @return USB CDC受信結果
 * ================================================================= */
esp_err_t usb_link_receive(uint8_t * data, size_t capacity, size_t * received_length, uint32_t timeout_ms) {
    if ((data == NULL) || (received_length == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_rx_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    usb_link_rx_message_t message;
    if (xQueueReceive(s_rx_queue, &message, pdMS_TO_TICKS(timeout_ms)) != pdPASS) {
        *received_length = 0U;
        return ESP_ERR_TIMEOUT;
    }
    if (capacity < message.length) {
        *received_length = 0U;
        s_rx_drop_count++;
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(data, message.data, message.length);
    *received_length = message.length;
    return ESP_OK;
}

/** =================================================================*
 * @brief  受信キュー破棄数取得
 * @return キュー満杯または容量不足で破棄した回数
 * ================================================================= */
uint32_t usb_link_rx_drop_count(void) {
    return s_rx_drop_count;
}
