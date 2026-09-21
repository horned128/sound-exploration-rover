/** =================================================================*
 * @file   acoustic_ai_lab_link.h
 * @brief  J11直結の音響AI学習・推論診断リンク
 * ================================================================= */
#ifndef SEROV_CPU0_ACOUSTIC_AI_LAB_LINK_H
#define SEROV_CPU0_ACOUSTIC_AI_LAB_LINK_H

#include "hal_data.h"                                     /* USBイベント型 */
#include <tk/tkernel.h>                                    /* μT-Kernel基本型 */

/* USB event queueは既存のtask_acoustic_linkだけが消費する。 */
EXPORT void acoustic_ai_lab_link_init(void);                /* J11 CDC初期化 */
EXPORT void acoustic_ai_lab_link_deinit(void);              /* J11 CDC終了 */
EXPORT void acoustic_ai_lab_link_event(const usb_event_info_t * p_event_info, usb_status_t event,
                                       UW now_ms);           /* 共有USBイベント配信 */
EXPORT void acoustic_ai_lab_link_poll(UW now_ms);           /* snapshot/chunk送信進行 */

IMPORT volatile BOOL g_acoustic_ai_lab_usb_open;            /**< J11 USB driver open済み */
IMPORT volatile BOOL g_acoustic_ai_lab_usb_configured;      /**< PC CDC列挙済み */
IMPORT volatile UW g_acoustic_ai_lab_snapshot_count;        /**< snapshot送信完了数 */
IMPORT volatile UW g_acoustic_ai_lab_command_count;         /**< PC操作受信数 */
IMPORT volatile UW g_acoustic_ai_lab_error_count;           /**< USB/protocol異常数 */
IMPORT volatile fsp_err_t g_acoustic_ai_lab_last_error;     /**< 直近USBエラー */

#endif /* SEROV_CPU0_ACOUSTIC_AI_LAB_LINK_H */
