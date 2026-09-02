/** =================================================================*
 * @file   tk_audio.h
 * @brief  CPU0音響USB受信タスクAPI
 * ================================================================= */
#ifndef SEROV_CPU0_TK_AUDIO_H
#define SEROV_CPU0_TK_AUDIO_H

#include "../../../../../common/acoustic_protocol.h"        /* 音響通信の型、定数 */
#include "hal_data.h"                                       /* FSPエラー型 */
#include "task_common.h"                                    /* CPU0タスク共通異常型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

typedef enum e_cpu0_audio_usb_state {
    CPU0_AUDIO_USB_STATE_CLOSED = 0,
    CPU0_AUDIO_USB_STATE_WAIT_DEVICE,
    CPU0_AUDIO_USB_STATE_SET_LINE_CODING,
    CPU0_AUDIO_USB_STATE_SET_CONTROL_LINE_STATE,
    CPU0_AUDIO_USB_STATE_GET_LINE_CODING,
    CPU0_AUDIO_USB_STATE_READY,
} cpu0_audio_usb_state_t;

typedef struct st_cpu0_audio_snapshot {
    BOOL usb_configured;
    BOOL hello_received;
    BOOL observation_received;
    UW link_age_ms;
    UW observation_age_ms;
    UW observation_sequence;
    acoustic_observation_t observation;
    acoustic_hello_t hello;
    acoustic_health_t health;
} cpu0_audio_snapshot_t;

EXPORT cpu0_fault_t cpu0_audio_task_create(void);                  /* 音響タスクと共有資源生成 */
EXPORT cpu0_fault_t cpu0_audio_task_start(void);                   /* 音響タスク開始 */
EXPORT void cpu0_audio_task_delete(void);                          /* 音響タスクと共有資源解放 */
EXPORT ER cpu0_audio_snapshot_get(cpu0_audio_snapshot_t * p_snapshot); /* 最新音響状態取得 */

IMPORT volatile BOOL g_cpu0_audio_usb_configured;           /**< USB列挙状態（Live Watch用） */
/**< CDC初期化段階（Live Watch用） */
IMPORT volatile cpu0_audio_usb_state_t zg_cpu0_audio_usb_state;
IMPORT volatile usb_status_t g_cpu0_audio_last_event;       /**< 最終USBイベント（Live Watch用） */
IMPORT volatile UB g_cpu0_audio_device_address;        /**< CDCアドレス（Live Watch用） */
IMPORT volatile UW g_cpu0_audio_event_count;          /**< USBイベント数（Live Watch用） */
IMPORT volatile UW g_cpu0_audio_transfer_busy_count;  /**< USB転送BUSY回数（Live Watch用） */
IMPORT volatile BOOL g_cpu0_audio_hello_received;           /**< HELLO受信状態（Live Watch用） */
IMPORT volatile UW g_cpu0_audio_frame_count;          /**< 正常フレーム数（Live Watch用） */
IMPORT volatile UW g_cpu0_audio_crc_error_count;      /**< CRC異常数（Live Watch用） */
IMPORT volatile UW g_cpu0_audio_format_error_count;   /**< 形式異常数（Live Watch用） */
IMPORT volatile UW g_cpu0_audio_sequence_drop_count;  /**< 逆行sequence数（Live Watch用） */
IMPORT volatile UW g_cpu0_audio_observation_age_ms;   /**< 観測経過時間（Live Watch用） */
IMPORT volatile UW g_cpu0_audio_telemetry_send_count; /**< 診断送信完了数（Live Watch用） */
IMPORT volatile UW g_cpu0_audio_telemetry_busy_count; /**< 診断送信BUSY数（Live Watch用） */
/**< 最新音響観測（Live Watch用） */
IMPORT volatile acoustic_observation_t g_cpu0_audio_observation;
IMPORT volatile fsp_err_t g_cpu0_audio_last_error;          /**< 最終USBエラー（Live Watch用） */

#endif /* SEROV_CPU0_TK_AUDIO_H */
