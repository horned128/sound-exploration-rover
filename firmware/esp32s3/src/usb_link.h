/** =================================================================*
 * @file   usb_link.h
 * @brief  USB CDC通信API
 * ================================================================= */
#ifndef RESPEAKER_USB_LINK_H
#define RESPEAKER_USB_LINK_H

#include "esp_err.h"                                        /* ESP-IDFエラー型 */
#include <stdbool.h>                                        /* 真偽値 */
#include <stddef.h>                                         /* size_t */
#include <stdint.h>                                         /* 固定幅整数型 */

esp_err_t usb_link_init(void);                              /* USB CDC初期化 */
bool usb_link_is_mounted(void);                             /* USBホスト接続状態取得 */
bool usb_link_take_new_session(void);                       /* 新規USB接続検出 */
esp_err_t usb_link_send(uint8_t const * data, size_t length); /* USB CDC送信 */
/* USB CDC受信 */
esp_err_t usb_link_receive(uint8_t * data, size_t capacity, size_t * received_length, uint32_t timeout_ms);
uint32_t usb_link_rx_drop_count(void);                      /* 受信キュー破棄数取得 */

#endif /* RESPEAKER_USB_LINK_H */
