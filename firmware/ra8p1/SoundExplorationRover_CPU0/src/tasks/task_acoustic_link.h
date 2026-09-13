/** =================================================================*
 * @file   task_acoustic_link.h
 * @brief  CPU0音響フロントエンドUSB CDCリンクタスクAPI
 * ================================================================= */
#ifndef SEROV_CPU0_TASK_ACOUSTIC_LINK_H
#define SEROV_CPU0_TASK_ACOUSTIC_LINK_H

#include "../../../common/acoustic_protocol.h"              /* 音響通信の型、定数 */
#include "services/acoustic_feature_assembler.h"           /* 再組立済み特徴量パッチ型 */
#include "hal_data.h"                                       /* FSPエラー型 */
#include "task_common.h"                                    /* CPU0タスク共通異常型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

typedef enum e_task_acoustic_link_usb_state {
    CPU0_AUDIO_USB_STATE_CLOSED = 0,
    CPU0_AUDIO_USB_STATE_WAIT_DEVICE,
    CPU0_AUDIO_USB_STATE_SET_LINE_CODING,
    CPU0_AUDIO_USB_STATE_SET_CONTROL_LINE_STATE,
    CPU0_AUDIO_USB_STATE_GET_LINE_CODING,
    CPU0_AUDIO_USB_STATE_READY,
} task_acoustic_link_usb_state_t;

typedef struct st_task_acoustic_link_snapshot {
    BOOL usb_configured;
    BOOL hello_received;
    BOOL observation_received;
    UW link_age_ms;
    UW observation_age_ms;
    UW observation_sequence;
    acoustic_observation_t observation;
    acoustic_hello_t hello;
    acoustic_health_t health;
} task_acoustic_link_snapshot_t;

EXPORT app_fault_t task_acoustic_link_create(void);                  /* 音響リンクタスク生成 */
EXPORT app_fault_t task_acoustic_link_start(void);                   /* 音響リンクタスク開始 */
EXPORT void task_acoustic_link_delete(void);                          /* 音響リンクタスク解放 */
EXPORT ER task_acoustic_link_snapshot_get(task_acoustic_link_snapshot_t * p_snapshot); /* 最新音響状態取得 */
/* 最新完成特徴量パッチ取得 */
EXPORT ER task_acoustic_link_feature_get(acoustic_feature_patch_t * p_patch, UW * p_generation);

IMPORT volatile BOOL g_task_acoustic_link_usb_configured;           /**< USB列挙状態（Live Watch用） */
/**< CDC初期化段階（Live Watch用） */
IMPORT volatile task_acoustic_link_usb_state_t g_task_acoustic_link_usb_state;
IMPORT volatile usb_status_t g_task_acoustic_link_last_event;       /**< 最終USBイベント（Live Watch用） */
IMPORT volatile UB g_task_acoustic_link_device_address;        /**< CDCアドレス（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_event_count;          /**< USBイベント数（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_transfer_busy_count;  /**< USB転送BUSY回数（Live Watch用） */
IMPORT volatile BOOL g_task_acoustic_link_hello_received;           /**< HELLO受信状態（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_frame_count;          /**< 正常フレーム数（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_crc_error_count;      /**< CRC異常数（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_format_error_count;   /**< 形式異常数（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_sequence_drop_count;  /**< 逆行sequence数（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_feature_complete_count; /**< 特徴量イベント完成数（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_feature_drop_count;  /**< 特徴量イベント破棄数（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_feature_generation;  /**< 最新特徴量世代（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_observation_age_ms;   /**< 観測経過時間（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_telemetry_send_count; /**< 診断送信完了数（Live Watch用） */
IMPORT volatile UW g_task_acoustic_link_telemetry_busy_count; /**< 診断送信BUSY数（Live Watch用） */
IMPORT volatile acoustic_observation_t g_task_acoustic_link_observation; /**< 最新音響観測（Live Watch用） */
IMPORT volatile fsp_err_t g_task_acoustic_link_last_error;          /**< 最終USBエラー（Live Watch用） */

#endif /* SEROV_CPU0_TASK_ACOUSTIC_LINK_H */
