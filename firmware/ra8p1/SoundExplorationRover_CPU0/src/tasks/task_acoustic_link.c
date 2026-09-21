/** =================================================================*
 * @file   task_acoustic_link.c
 * @brief  ReSpeaker音響フロントエンドとのUSB CDCリンク
 * ================================================================= */
#include "task_acoustic_link.h"                             /* CPU0音響リンクタスクAPI */
#include "config/control_config.h"                          /* 自律走行モード */
#include "config/task_config.h"                             /* USB受信周期、優先度、バッファ長 */
#include "config/sensor_config.h"                           /* センサー安全判定値 */
#include "ipc/actuator_ipc_client.h"                        /* CPU1実出力状態取得API */
#include "services/acoustic_feature_assembler.h"            /* 特徴量イベント再組立 */
#include "services/acoustic_ai_lab_link.h"                  /* J11 PC直結の学習・推論診断 */
#include "services/odometry.h"                              /* オドメトリ位置姿勢 */
#include "task_command.h"                                   /* 最新指令状態取得API */
#include "task_infer.h"                                     /* 完成特徴量を推論タスクへ通知 */
#include "task_sensor.h"                                    /* 最新I2Cセンサー状態取得API */
#include "task_think.h"                                     /* 思考タスクへの異常通知 */
#include <string.h>                                         /* memset */

#define CPU0_AUDIO_SET_LINE_CODING         /**< USBホストから受けるLine Coding要求値 */ \
    (USB_CDC_SET_LINE_CODING | USB_HOST_TO_DEV | USB_CLASS | USB_INTERFACE)
#define CPU0_AUDIO_SET_CONTROL_LINE_STATE                   /**< USB制御線状態要求の判定値 */ \
    (USB_CDC_SET_CONTROL_LINE_STATE | USB_HOST_TO_DEV | USB_CLASS | USB_INTERFACE)
#define CPU0_AUDIO_GET_LINE_CODING         /**< USBデバイスから返すLine Coding要求値 */ \
    (USB_CDC_GET_LINE_CODING | USB_DEV_TO_HOST | USB_CLASS | USB_INTERFACE)
#define CPU0_AUDIO_LINE_CODING_LENGTH      (7U)             /**< USB Line Codingデータ長[byte] */

LOCAL void task_acoustic_link_entry(INT stacd, void * exinf); /* 音響リンクタスク本体 */
LOCAL UW task_acoustic_link_monotonic_ms(void);             /* カーネル単調時刻取得 */
LOCAL void task_acoustic_link_link_reset(void);             /* 音響リンク状態初期化 */
LOCAL void task_acoustic_link_observation_reset(void);      /* 旧音響観測無効化 */
LOCAL void task_acoustic_link_feature_reset(void);          /* 旧特徴量イベント無効化 */
LOCAL void task_acoustic_link_bus_recovery(void);           /* USBバスリセット・再列挙リカバリ */
LOCAL fsp_err_t task_acoustic_link_control_start(void);     /* CDC class request開始 */
LOCAL void task_acoustic_link_control_complete(const usb_event_info_t * p_event_info); /* CDC class request完了 */
LOCAL fsp_err_t task_acoustic_link_read_start(void);        /* USB Bulk IN開始 */
LOCAL fsp_err_t task_acoustic_link_actuator_telemetry_start(void); /* CPU1診断Bulk OUT開始 */
LOCAL fsp_err_t task_acoustic_link_pose_telemetry_start(void); /* オドメトリ診断Bulk OUT開始 */
LOCAL fsp_err_t task_acoustic_link_telemetry_start(void);   /* USB Bulk OUT開始 */
LOCAL void task_acoustic_link_receive(UW length);           /* USB受信データ処理 */
LOCAL void task_acoustic_link_frame_handle(const acoustic_frame_t * p_frame); /* 正常フレーム反映 */
LOCAL BOOL task_acoustic_link_sequence_accept(UW sequence); /* sequence新旧判定 */

/**< 音響リンク状態を保護するμT-Kernel mutex設定 */
LOCAL T_CMTX const acoustic_link_mutex_config = {
    .mtxatr = TA_INHERIT,
    .ceilpri = 0,
};

/**< 音響リンクタスク設定 */
LOCAL T_CTSK const acoustic_link_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) task_acoustic_link_entry,
    .itskpri = CPU0_AUDIO_TASK_PRIORITY,
    .stksz = CPU0_AUDIO_TASK_STACK_SIZE,
    .bufptr = NULL,
};

LOCAL ID acoustic_link_task_id;                             /**< 音響リンクタスクID */
LOCAL ID acoustic_link_mutex_id;                            /**< 音響リンク状態保護mutex ID */
LOCAL BOOL acoustic_link_task_started;                      /**< 音響リンクタスク開始状態 */
LOCAL BOOL audio_usb_open;                                  /**< USBドライバopen状態 */
LOCAL BOOL audio_read_pending;                              /**< Bulk IN要求実行中 */
LOCAL BOOL audio_write_pending;                             /**< Bulk OUT要求実行中 */
LOCAL UW audio_write_started_at_ms;                         /**< Bulk OUT要求開始時刻 */
LOCAL BOOL audio_actuator_telemetry_pending;                /**< CPU1診断送信待ち */
LOCAL BOOL audio_pose_telemetry_pending;                    /**< オドメトリ診断送信待ち */
LOCAL BOOL audio_control_pending;                           /**< CDC class request実行中 */
LOCAL BOOL audio_sequence_valid;                            /**< sequence初回受信済み */
LOCAL BOOL audio_boot_id_valid;                             /**< boot ID初回受信済み */
LOCAL UB audio_device_address;                              /**< CDCデバイスアドレス */
LOCAL UW audio_now_ms;                                      /**< 音響リンクタスク単調時刻 */
LOCAL UW audio_last_recovery_ms;                            /**< 最終リカバリ実行時刻 */
LOCAL UW audio_continuous_error_count;                      /**< 連続USBエラー回数 */
LOCAL UW audio_configured_at_ms;                            /**< USB列挙時刻 */
LOCAL UW audio_observation_at_ms;                           /**< 最終観測受信時刻 */
LOCAL UW audio_last_sequence;                               /**< 最終受信sequence */
LOCAL UW audio_observation_sequence;                        /**< 最終観測sequence */
LOCAL UW audio_telemetry_sequence;                          /**< 診断送信sequence */
LOCAL UW audio_actuator_telemetry_sequence;                 /**< CPU1診断送信sequence */
LOCAL UW audio_pose_telemetry_sequence;                     /**< オドメトリ診断送信sequence */
LOCAL UW audio_last_telemetry_ms;                           /**< 最終診断送信要求時刻 */
LOCAL UW audio_actuator_status_at_ms;                       /**< CPU1状態最終更新時刻 */
LOCAL UW audio_actuator_status_sequence;                    /**< CPU1状態最終sequence */
LOCAL UW audio_boot_id;                                     /**< ESP32 boot ID */
LOCAL BOOL audio_actuator_status_sequence_valid;            /**< CPU1状態sequence受信済み */
LOCAL acoustic_protocol_parser_t audio_parser;              /**< CDCストリームパーサー */
LOCAL acoustic_hello_t audio_hello;                         /**< 最新HELLO */
LOCAL acoustic_health_t audio_health;                       /**< 最新HEALTH */
LOCAL acoustic_feature_assembler_t audio_feature_assembler; /**< 特徴量再組立状態 */
LOCAL acoustic_feature_patch_t audio_feature_patch;         /**< 最新完成特徴量パッチ */
LOCAL BOOL audio_feature_ready;                             /**< 完成特徴量保持状態 */
/**< CDC仮想UART設定 (115200bps, 1 stop bit, no parity, 8 data bits - 7 bytes packed) */
LOCAL UB audio_cdc_line_coding[CPU0_AUDIO_LINE_CODING_LENGTH] = {
    0x00, 0xC2, 0x01, 0x00, /* 115200 bps (little-endian) */
    0x00,                   /* 1 stop bit */
    0x00,                   /* no parity */
    0x08                    /* 8 data bits */
};
LOCAL UB audio_control_dummy;                               /**< data無しcontrol転送用 */
LOCAL UB audio_rx_buffer[CPU0_AUDIO_USB_RX_SIZE];           /**< USB Bulk INバッファ */
/**< USB Bulk OUTバッファ */
LOCAL UB audio_tx_buffer[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE]; /**< USB送信用フレームバッファ */

EXPORT volatile BOOL g_task_acoustic_link_usb_configured;   /**< USB列挙状態 */
/**< CDC初期化段階 */
EXPORT volatile task_acoustic_link_usb_state_t g_task_acoustic_link_usb_state;
/**< 最終USBイベント */
EXPORT volatile usb_status_t g_task_acoustic_link_last_event;
EXPORT volatile UB g_task_acoustic_link_device_address;     /**< CDCアドレス */
EXPORT volatile UW g_task_acoustic_link_event_count;        /**< USBイベント数 */
EXPORT volatile UW g_task_acoustic_link_transfer_busy_count;/**< USB転送BUSY回数 */
EXPORT volatile BOOL g_task_acoustic_link_hello_received;   /**< HELLO受信状態 */
EXPORT volatile UW g_task_acoustic_link_frame_count;        /**< 正常フレーム数 */
EXPORT volatile UW g_task_acoustic_link_crc_error_count;    /**< CRC異常数 */
EXPORT volatile UW g_task_acoustic_link_format_error_count; /**< 形式異常数 */
EXPORT volatile UW g_task_acoustic_link_sequence_drop_count;/**< 逆行sequence数 */
/**< 特徴量イベント完成数 */
EXPORT volatile UW g_task_acoustic_link_feature_complete_count;
EXPORT volatile UW g_task_acoustic_link_feature_drop_count; /**< 特徴量イベント破棄数 */
EXPORT volatile UW g_task_acoustic_link_feature_generation; /**< 最新特徴量世代 */
EXPORT volatile UW g_task_acoustic_link_observation_age_ms; /**< 観測経過時間 */
/**< 診断送信完了数 */
EXPORT volatile UW g_task_acoustic_link_telemetry_send_count;
/**< 診断送信BUSY数 */
EXPORT volatile UW g_task_acoustic_link_telemetry_busy_count;
/**< 最新音響観測 */
EXPORT volatile acoustic_observation_t g_task_acoustic_link_observation;
EXPORT volatile fsp_err_t g_task_acoustic_link_last_error;  /**< 最終USBエラー */

/** =================================================================*
 * @brief  カーネルの単調時刻を32bitミリ秒で取得
 * @details 32bit wrap後も符号なし差分で経過時間を計算できる。
 * @return システム起動後の単調時刻
 * ================================================================= */
LOCAL UW task_acoustic_link_monotonic_ms(void) {
    SYSTIM system_time = {0};
    if (E_OK == tk_get_otm(&system_time)) {
        return system_time.lo;
    }

    return audio_now_ms + CPU0_AUDIO_USB_POLL_MS;
}

/** =================================================================*
 * @brief  音響リンクタスクと共有資源生成
 * @return CPU0異常コード
 * ================================================================= */
EXPORT app_fault_t task_acoustic_link_create(void) {
    acoustic_link_task_id = 0;
    acoustic_link_mutex_id = 0;
    acoustic_link_task_started = FALSE;
    audio_usb_open = FALSE;
    audio_read_pending = FALSE;
    audio_write_pending = FALSE;
    audio_write_started_at_ms = 0U;
    audio_actuator_telemetry_pending = FALSE;
    audio_control_pending = FALSE;
    audio_now_ms = 0U;
    audio_last_recovery_ms = 0U;
    audio_continuous_error_count = 0U;
    g_task_acoustic_link_usb_state = CPU0_AUDIO_USB_STATE_CLOSED;
    g_task_acoustic_link_last_event = (usb_status_t) 0U;
    g_task_acoustic_link_device_address = 0U;
    g_task_acoustic_link_event_count = 0U;
    g_task_acoustic_link_transfer_busy_count = 0U;
    g_task_acoustic_link_frame_count = 0U;
    g_task_acoustic_link_crc_error_count = 0U;
    g_task_acoustic_link_format_error_count = 0U;
    g_task_acoustic_link_sequence_drop_count = 0U;
    g_task_acoustic_link_feature_complete_count = 0U;
    g_task_acoustic_link_feature_drop_count = 0U;
    g_task_acoustic_link_feature_generation = 0U;
    g_task_acoustic_link_telemetry_send_count = 0U;
    g_task_acoustic_link_telemetry_busy_count = 0U;
    g_task_acoustic_link_last_error = FSP_SUCCESS;
    task_acoustic_link_link_reset();

    acoustic_link_mutex_id = tk_cre_mtx(&acoustic_link_mutex_config);
    if (acoustic_link_mutex_id <= 0) {
        acoustic_link_mutex_id = 0;
        return APP_FAULT_TASK_CREATE;
    }

    acoustic_link_task_id = tk_cre_tsk(&acoustic_link_task_config);
    if (acoustic_link_task_id <= 0) {
        acoustic_link_task_id = 0;
        task_acoustic_link_delete();
        return APP_FAULT_TASK_CREATE;
    }

    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  音響リンクタスク開始
 * @return CPU0異常コード
 * ================================================================= */
EXPORT app_fault_t task_acoustic_link_start(void) {
    if (acoustic_link_task_id <= 0) {
        return APP_FAULT_TASK_CREATE;
    }

    ER const err = tk_sta_tsk(acoustic_link_task_id, 0);
    if (E_OK != err) {
        return APP_FAULT_TASK_START;
    }
    acoustic_link_task_started = TRUE;
    return APP_FAULT_NONE;
}

/** =================================================================*
 * @brief  音響リンクタスクと共有資源解放
 * ================================================================= */
EXPORT void task_acoustic_link_delete(void) {
    if (acoustic_link_task_id > 0) {
        if (acoustic_link_task_started) {
            (void) tk_ter_tsk(acoustic_link_task_id);
        }
        (void) tk_del_tsk(acoustic_link_task_id);
        acoustic_link_task_id = 0;
        acoustic_link_task_started = FALSE;
    }

    if (audio_usb_open) {
        (void) g_usb_on_usb.close(&g_basic0_ctrl);
        audio_usb_open = FALSE;
    }
    acoustic_ai_lab_link_deinit();

    if (acoustic_link_mutex_id > 0) {
        (void) tk_del_mtx(acoustic_link_mutex_id);
        acoustic_link_mutex_id = 0;
    }
}

/** =================================================================*
 * @brief  音響リンク状態初期化
 * @details USB切断後の古い観測が走行判断へ残らないよう無効化する。
 * ================================================================= */
LOCAL void task_acoustic_link_link_reset(void) {
    g_task_acoustic_link_usb_configured = FALSE;
    g_task_acoustic_link_usb_state = audio_usb_open ? CPU0_AUDIO_USB_STATE_WAIT_DEVICE : CPU0_AUDIO_USB_STATE_CLOSED;
    g_task_acoustic_link_device_address = 0U;
    g_task_acoustic_link_hello_received = FALSE;
    task_acoustic_link_observation_reset();
    task_acoustic_link_feature_reset();
    memset(&audio_hello, 0, sizeof(audio_hello));
    audio_sequence_valid = FALSE;
    audio_boot_id_valid = FALSE;
    audio_read_pending = FALSE;
    audio_write_pending = FALSE;
    audio_write_started_at_ms = 0U;
    audio_actuator_telemetry_pending = FALSE;
    audio_pose_telemetry_pending = FALSE;
    audio_control_pending = FALSE;
    audio_device_address = 0U;
    audio_configured_at_ms = audio_now_ms;
    audio_last_sequence = 0U;
    audio_telemetry_sequence = 0U;
    audio_actuator_telemetry_sequence = 0U;
    audio_pose_telemetry_sequence = 0U;
    audio_last_telemetry_ms = audio_now_ms;
    audio_actuator_status_at_ms = audio_now_ms;
    audio_actuator_status_sequence = 0U;
    audio_actuator_status_sequence_valid = FALSE;
    audio_boot_id = 0U;
    audio_continuous_error_count = 0U;
    acoustic_protocol_parser_init(&audio_parser);
}

/** =================================================================*
 * @brief  USBバスリセットおよび再列挙リカバリ
 * @details デバイス側の再起動や通信途絶時にUSBポートを再初期化して復旧する。
 * ================================================================= */
LOCAL void task_acoustic_link_bus_recovery(void) {
    ER const lock_err = tk_loc_mtx(acoustic_link_mutex_id, TMO_FEVR);
    if (E_OK == lock_err) {
        task_acoustic_link_link_reset();
        (void) tk_unl_mtx(acoustic_link_mutex_id);
    }

    if (audio_usb_open) {
        (void) g_usb_on_usb.close(&g_basic0_ctrl);
        audio_usb_open = FALSE;
    }

    (void) tk_dly_tsk(100);

    g_task_acoustic_link_last_error = g_usb_on_usb.open(&g_basic0_ctrl, &g_basic0_cfg);
    if (FSP_SUCCESS == g_task_acoustic_link_last_error) {
        audio_usb_open = TRUE;
        g_task_acoustic_link_usb_state = CPU0_AUDIO_USB_STATE_WAIT_DEVICE;
    }
}

/** =================================================================*
 * @brief  CDC class request開始
 * @details FSP HCDCの推奨順序で仮想UARTを初期化する。
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t task_acoustic_link_control_start(void) {
    if (!g_task_acoustic_link_usb_configured || (0U == audio_device_address) || audio_control_pending) {
        return FSP_SUCCESS;
    }

    usb_setup_t setup = {0};
    UB * p_data = &audio_control_dummy;

    if (CPU0_AUDIO_USB_STATE_SET_LINE_CODING == g_task_acoustic_link_usb_state) {
        setup.request_type = CPU0_AUDIO_SET_LINE_CODING;
        setup.request_length = CPU0_AUDIO_LINE_CODING_LENGTH;
        p_data = audio_cdc_line_coding;
    } else if (CPU0_AUDIO_USB_STATE_SET_CONTROL_LINE_STATE == g_task_acoustic_link_usb_state) {
        setup.request_type = CPU0_AUDIO_SET_CONTROL_LINE_STATE;
        /* CDCホストへDTR/RTSの両方を有効として通知する。 */
        setup.request_value = 0x0003U;
    } else if (CPU0_AUDIO_USB_STATE_GET_LINE_CODING == g_task_acoustic_link_usb_state) {
        setup.request_type = CPU0_AUDIO_GET_LINE_CODING;
        setup.request_length = CPU0_AUDIO_LINE_CODING_LENGTH;
        p_data = audio_cdc_line_coding;
    } else {
        return FSP_SUCCESS;
    }

    fsp_err_t const err = g_usb_on_usb.hostControlTransfer(&g_basic0_ctrl, &setup, p_data, audio_device_address);
    if (FSP_SUCCESS == err) {
        audio_control_pending = TRUE;
    } else if (FSP_ERR_USB_BUSY == err) {
        g_task_acoustic_link_transfer_busy_count++;
    }
    return err;
}

/** =================================================================*
 * @brief  CDC class request完了
 * @param[in] p_event_info USB request完了情報
 * ================================================================= */
LOCAL void task_acoustic_link_control_complete(const usb_event_info_t * p_event_info) {
    if ((NULL == p_event_info) || !audio_control_pending) {
        return;
    }

    UH const request = p_event_info->setup.request_type & USB_BREQUEST;
    audio_control_pending = FALSE;
    if (USB_SETUP_STATUS_ACK != p_event_info->status) {
        g_task_acoustic_link_last_error = FSP_ERR_USB_FAILED;
        /* フォールバック: control requestがACK以外でもREADYへ進めてBulk通信を試みる */
        g_task_acoustic_link_usb_state = CPU0_AUDIO_USB_STATE_READY;
        return;
    }

    if ((USB_CDC_SET_LINE_CODING == request) &&
        (CPU0_AUDIO_USB_STATE_SET_LINE_CODING == g_task_acoustic_link_usb_state)) {
        g_task_acoustic_link_usb_state = CPU0_AUDIO_USB_STATE_SET_CONTROL_LINE_STATE;
    } else if ((USB_CDC_SET_CONTROL_LINE_STATE == request) &&
               (CPU0_AUDIO_USB_STATE_SET_CONTROL_LINE_STATE == g_task_acoustic_link_usb_state)) {
        g_task_acoustic_link_usb_state = CPU0_AUDIO_USB_STATE_GET_LINE_CODING;
    } else if ((USB_CDC_GET_LINE_CODING == request) &&
               (CPU0_AUDIO_USB_STATE_GET_LINE_CODING == g_task_acoustic_link_usb_state)) {
        g_task_acoustic_link_usb_state = CPU0_AUDIO_USB_STATE_READY;
    }
}

/** =================================================================*
 * @brief  旧音響観測無効化
 * @details frontend再起動後に再起動前のDoAを走行判断へ渡さない。
 * ================================================================= */
LOCAL void task_acoustic_link_observation_reset(void) {
    g_task_acoustic_link_observation_age_ms = UINT32_MAX;
    memset((void *) &g_task_acoustic_link_observation, 0, sizeof(g_task_acoustic_link_observation));
    g_task_acoustic_link_observation.doa_deg = ACOUSTIC_PROTOCOL_DOA_INVALID;
    memset(&audio_health, 0, sizeof(audio_health));
    audio_observation_at_ms = audio_now_ms;
    audio_observation_sequence = 0U;
}

/** =================================================================*
 * @brief  最新音響状態取得
 * @param[out] p_snapshot 音響状態スナップショット
 * @return μT-Kernelエラーコード
 * ================================================================= */
EXPORT ER task_acoustic_link_snapshot_get(task_acoustic_link_snapshot_t * p_snapshot) {
    if (NULL == p_snapshot) {
        return E_PAR;
    }
    if (acoustic_link_mutex_id <= 0) {
        return E_NOEXS;
    }

    ER err = tk_loc_mtx(acoustic_link_mutex_id, TMO_POL);
    if (E_OK != err) {
        return err;
    }

    p_snapshot->usb_configured = g_task_acoustic_link_usb_configured;
    p_snapshot->hello_received = g_task_acoustic_link_hello_received;
    p_snapshot->observation_received = ACOUSTIC_PROTOCOL_DOA_INVALID != g_task_acoustic_link_observation.doa_deg;
    p_snapshot->link_age_ms = audio_now_ms - audio_configured_at_ms;
    p_snapshot->observation_age_ms =
        p_snapshot->observation_received ? (audio_now_ms - audio_observation_at_ms) : UINT32_MAX;
    p_snapshot->observation_sequence = audio_observation_sequence;
    p_snapshot->observation = g_task_acoustic_link_observation;
    p_snapshot->hello = audio_hello;
    p_snapshot->health = audio_health;
    g_task_acoustic_link_observation_age_ms = p_snapshot->observation_age_ms;

    err = tk_unl_mtx(acoustic_link_mutex_id);
    return err;
}

/** =================================================================*
 * @brief  旧特徴量イベント無効化
 * @details USB切断またはfrontend再起動をまたぐ不完全・完成パッチを推論へ渡さない。
 * ================================================================= */
LOCAL void task_acoustic_link_feature_reset(void) {
    audio_feature_ready = FALSE;
    memset(&audio_feature_patch, 0, sizeof(audio_feature_patch));
    acoustic_feature_assembler_init(&audio_feature_assembler);
}

/** =================================================================*
 * @brief  最新完成特徴量パッチ取得
 * @details mutex保持中はコピーだけを行い、推論処理は呼出側でmutex解放後に実行する。
 * @param[out] p_patch 80フレーム×32 binの特徴量パッチ
 * @param[out] p_generation 最新完成世代
 * @return μT-Kernelエラーコード。完成パッチが無い場合はE_NOEXS
 * ================================================================= */
EXPORT ER task_acoustic_link_feature_get(acoustic_feature_patch_t * p_patch, UW * p_generation) {
    if ((NULL == p_patch) || (NULL == p_generation)) {
        return E_PAR;
    }
    if (acoustic_link_mutex_id <= 0) {
        return E_NOEXS;
    }

    ER err = tk_loc_mtx(acoustic_link_mutex_id, TMO_POL);
    if (E_OK != err) {
        return err;
    }
    if (!audio_feature_ready) {
        (void) tk_unl_mtx(acoustic_link_mutex_id);
        return E_NOEXS;
    }

    memcpy(p_patch, &audio_feature_patch, sizeof(*p_patch));
    *p_generation = g_task_acoustic_link_feature_generation;
    err = tk_unl_mtx(acoustic_link_mutex_id);
    return err;
}

/** =================================================================*
 * @brief  USB Bulk IN開始
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t task_acoustic_link_read_start(void) {
    if (!g_task_acoustic_link_usb_configured || (CPU0_AUDIO_USB_STATE_READY != g_task_acoustic_link_usb_state) ||
        (0U == audio_device_address)) {
        return FSP_ERR_NOT_OPEN;
    }

    if (audio_read_pending) {
        return FSP_SUCCESS;
    }

    fsp_err_t const err =
        g_usb_on_usb.read(&g_basic0_ctrl, audio_rx_buffer, sizeof(audio_rx_buffer), audio_device_address);
    if (FSP_SUCCESS == err) {
        audio_read_pending = TRUE;
        return FSP_SUCCESS;
    }
    if (FSP_ERR_USB_BUSY == err) {
        g_task_acoustic_link_transfer_busy_count++;
    }
    return err;
}

/** =================================================================*
 * @brief  Bulk OUT送信中判定およびタイムアウト保護
 * @return 送信中ならTRUE、完了またはタイムアウト解除済みならFALSE
 * ================================================================= */
LOCAL BOOL task_acoustic_link_write_in_progress(void) {
    if (audio_write_pending) {
        if ((audio_now_ms - audio_write_started_at_ms) > 1000U) {
            audio_write_pending = FALSE;
            audio_actuator_telemetry_pending = FALSE;
            audio_pose_telemetry_pending = FALSE;
            g_task_acoustic_link_last_error = FSP_ERR_TIMEOUT;
            return FALSE;
        }
        return TRUE;
    }
    return FALSE;
}

/** =================================================================*
 * @brief  CPU1アクチュエータ診断情報のUSB Bulk OUT開始
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t task_acoustic_link_actuator_telemetry_start(void) {
    if (task_acoustic_link_write_in_progress()) {
        return FSP_SUCCESS;
    }

    actuator_status_t actuator_status = {0};
    BOOL const status_valid = actuator_ipc_client_status_get(&actuator_status);
    if (status_valid &&
        (!audio_actuator_status_sequence_valid ||
         (audio_actuator_status_sequence != actuator_status.sequence_number))) {
        audio_actuator_status_sequence = actuator_status.sequence_number;
        audio_actuator_status_at_ms = audio_now_ms;
        audio_actuator_status_sequence_valid = TRUE;
    }

    acoustic_actuator_telemetry_t const telemetry = {
        .schema_version = 1U,
        .status_valid = status_valid ? 1U : 0U,
        .fault_flags = actuator_status.fault_flags,
        .actuator_status_age_ms = status_valid ? (audio_now_ms - audio_actuator_status_at_ms) : UINT32_MAX,
        .actuator_status_sequence = actuator_status.sequence_number,
        .actuator_applied_command_sequence = actuator_status.applied_command_sequence,
        .actuator_left_duty_permille = actuator_status.left_duty_permille,
        .actuator_right_duty_permille = actuator_status.right_duty_permille,
        .actuator_left_encoder_rpm_x10 = actuator_status.left_encoder_rpm_x10,
        .actuator_right_encoder_rpm_x10 = actuator_status.right_encoder_rpm_x10,
    };
    size_t const length = acoustic_protocol_encode_actuator_telemetry(audio_actuator_telemetry_sequence, audio_now_ms,
                                                                      &telemetry, audio_tx_buffer,
                                                                      sizeof(audio_tx_buffer));
    if (0U == length) {
        return FSP_ERR_INVALID_SIZE;
    }

    fsp_err_t const err = g_usb_on_usb.write(&g_basic0_ctrl, audio_tx_buffer, (UW) length, audio_device_address);
    if (FSP_SUCCESS == err) {
        audio_write_pending = TRUE;
        audio_write_started_at_ms = audio_now_ms;
        audio_actuator_telemetry_pending = FALSE;
        audio_pose_telemetry_pending = TRUE;
        audio_actuator_telemetry_sequence++;
    } else if (FSP_ERR_USB_BUSY == err) {
        g_task_acoustic_link_telemetry_busy_count++;
    }
    return err;
}

/** =================================================================*
 * @brief  オドメトリ診断情報のUSB Bulk OUT開始
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t task_acoustic_link_pose_telemetry_start(void) {
    if (!g_task_acoustic_link_usb_configured || (CPU0_AUDIO_USB_STATE_READY != g_task_acoustic_link_usb_state) ||
        (0U == audio_device_address)) {
        return FSP_ERR_NOT_OPEN;
    }
    if (task_acoustic_link_write_in_progress()) {
        return FSP_SUCCESS;
    }

    odometry_pose_t pose;
    memset(&pose, 0, sizeof(pose));
    (void) odometry_service_get_pose(&pose);

    acoustic_pose_telemetry_t telemetry = {
        .x_mm = (int32_t) pose.x_mm,
        .y_mm = (int32_t) pose.y_mm,
        .theta_mrad = (int32_t) pose.theta_mrad,
        .v_mm_s = (int32_t) pose.linear_speed_mm_s,
        .omega_mrad_s = (int32_t) pose.angular_speed_mrad_s,
        .left_encoder_count = (int32_t) pose.left_encoder_total,
        .right_encoder_count = (int32_t) pose.right_encoder_total,
        .gyro_bias_dps_x10 = 0,
        .flags = pose.valid ? (uint8_t) ACOUSTIC_POSE_FLAG_CALIBRATED : 0U,
        .reserved = 0U,
        .uptime_ms = (uint32_t) pose.timestamp_ms,
    };

    size_t const length = acoustic_protocol_encode_pose_telemetry(audio_pose_telemetry_sequence, audio_now_ms,
                                                                  &telemetry, audio_tx_buffer,
                                                                  sizeof(audio_tx_buffer));
    if (0U == length) {
        return FSP_ERR_INVALID_SIZE;
    }

    fsp_err_t const err = g_usb_on_usb.write(&g_basic0_ctrl, audio_tx_buffer, (UW) length, audio_device_address);
    if (FSP_SUCCESS == err) {
        audio_write_pending = TRUE;
        audio_write_started_at_ms = audio_now_ms;
        audio_pose_telemetry_pending = FALSE;
        audio_pose_telemetry_sequence++;
    } else if (FSP_ERR_USB_BUSY == err) {
        g_task_acoustic_link_telemetry_busy_count++;
    }
    return err;
}

/** =================================================================*
 * @brief  CPU0診断情報のUSB Bulk OUT開始
 * @details 音響受信を止めず、最新判断とIPC指令をESP32へ返送する。
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t task_acoustic_link_telemetry_start(void) {
    if (!g_task_acoustic_link_usb_configured || (CPU0_AUDIO_USB_STATE_READY != g_task_acoustic_link_usb_state) ||
        (0U == audio_device_address)) {
        return FSP_ERR_NOT_OPEN;
    }
    if (task_acoustic_link_write_in_progress()) {
        return FSP_SUCCESS;
    }
    if (audio_actuator_telemetry_pending) {
        return task_acoustic_link_actuator_telemetry_start();
    }
    if (audio_pose_telemetry_pending) {
        return task_acoustic_link_pose_telemetry_start();
    }
    if ((audio_now_ms - audio_last_telemetry_ms) < CPU0_AUDIO_TELEMETRY_PERIOD_MS) {
        return FSP_SUCCESS;
    }

    /* USBテレメトリの送信中も再利用する最新CPU0指令スナップショット。 */
    LOCAL task_command_snapshot_t s_command_snapshot;
    ER const snapshot_err = task_command_snapshot_get(&s_command_snapshot);
    if (E_OK != snapshot_err) {
        g_task_acoustic_link_telemetry_busy_count++;
        return FSP_ERR_USB_BUSY;
    }

    /* センサー取得失敗時も診断値を組み立てられる再利用バッファ。 */
    LOCAL sensor_snapshot_t s_sensor_snapshot;
    memset(&s_sensor_snapshot, 0, sizeof(s_sensor_snapshot));
    s_sensor_snapshot.age_ms = UINT32_MAX;
    s_sensor_snapshot.last_error = (W) FSP_ERR_NOT_OPEN;
    s_sensor_snapshot.error_flags = CPU0_SENSOR_ERROR_I2C_INIT;
#if (CPU0_SENSOR_I2C_ENABLED != 0U)
    if (E_OK != task_sensor_snapshot_get(&s_sensor_snapshot)) {
        s_sensor_snapshot.valid_flags = 0U;
    }
#endif

    UB flags = 0U;
    if (g_task_acoustic_link_usb_configured) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_USB_CONFIGURED;
    }
    if (g_task_acoustic_link_hello_received) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_HELLO_RECEIVED;
    }
    if (ACOUSTIC_PROTOCOL_DOA_INVALID != g_task_acoustic_link_observation.doa_deg) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_OBSERVATION;
    }
    if (g_task_think_link_ready) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_LINK_READY;
    }
    if (g_task_think_new_observation) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_NEW_OBSERVATION;
    }
    if (s_command_snapshot.last_sent_target.actuator_enable) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_ACTUATOR_ENABLE;
    }
    if (s_command_snapshot.last_sent_target.emergency_stop) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_EMERGENCY_STOP;
    }
    if (s_command_snapshot.target_stale) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_COMMAND_STALE;
    }

    /* 推論結果をUSBテレメトリへ変換する再利用バッファ。 */
    LOCAL task_infer_result_t s_infer_result;
    memset(&s_infer_result, 0, sizeof(s_infer_result));
    BOOL const infer_ready = (E_OK == task_infer_result_get(&s_infer_result));

    task_infer_prototype_telemetry_t proto_telem = {0};
    BOOL const proto_ready = (E_OK == task_infer_prototype_telemetry_get(&proto_telem));

    UB current_peak_bin = 255U;
    if (infer_ready && s_infer_result.summary_valid && (s_infer_result.active_frame_count >= 10U)) {
        current_peak_bin = acoustic_identifier_find_peak_bin(s_infer_result.summary, 1U);
    }

    UB const target_peak_bin = proto_ready ? proto_telem.target_peak_bin : 255U;

    UH cosine_dist_x1000 = 0xFFFFU;
    UH similarity_permille = 0U;
    if (infer_ready && (s_infer_result.identifier.minimum_cosine_distance >= 0.0F)) {
        float const dist = s_infer_result.identifier.minimum_cosine_distance;
        cosine_dist_x1000 = (UH) ((dist > 2.0F) ? 2000U : (UW) (dist * 1000.0F + 0.5F));
        float sim = (1.0F - dist) * 1000.0F;
        if (sim < 0.0F) {
            sim = 0.0F;
        } else if (sim > 1000.0F) {
            sim = 1000.0F;
        }
        similarity_permille = (UH) (sim + 0.5F);
    }

    UH threshold_x1000 = 0U;
    if (infer_ready && (s_infer_result.identifier.threshold >= 0.0F)) {
        float const th = s_infer_result.identifier.threshold;
        threshold_x1000 = (UH) ((th > 2.0F) ? 2000U : (th < 0.0F) ? 0U : (UW) (th * 1000.0F + 0.5F));
    } else if (proto_ready && proto_telem.storage_valid) {
        float const th = proto_telem.identifier_threshold;
        threshold_x1000 = (UH) ((th > 2.0F) ? 2000U : (th < 0.0F) ? 0U : (UW) (th * 1000.0F + 0.5F));
    }

    /* CPU0状態・センサー・推論結果を一つのwire形式へまとめる作業領域。 */
    LOCAL acoustic_rover_telemetry_t s_telemetry;
    memset(&s_telemetry, 0, sizeof(s_telemetry));
    s_telemetry.schema_version = 2U;
    s_telemetry.think_state = (UB) g_task_think_state;
    s_telemetry.usb_state = (UB) g_task_acoustic_link_usb_state;
    s_telemetry.flags = flags;
    s_telemetry.doa_deg = g_task_acoustic_link_observation.doa_deg;
    s_telemetry.level_dbfs_x100 = g_task_acoustic_link_observation.level_dbfs_x100;
    s_telemetry.peak_dbfs_x100 = g_task_acoustic_link_observation.peak_dbfs_x100;
    s_telemetry.vad = g_task_acoustic_link_observation.vad;
    s_telemetry.xvf_status = g_task_acoustic_link_observation.xvf_status;
    s_telemetry.audio_flags = g_task_acoustic_link_observation.audio_flags;
    s_telemetry.xvf_raw_status = g_task_acoustic_link_observation.xvf_raw_status;
    s_telemetry.observation_sequence = audio_observation_sequence;
    s_telemetry.observation_age_ms = (ACOUSTIC_PROTOCOL_DOA_INVALID != g_task_acoustic_link_observation.doa_deg)
                                  ? (audio_now_ms - audio_observation_at_ms)
                                  : UINT32_MAX;
    s_telemetry.audio_frame_count = g_task_acoustic_link_observation.audio_frame_count;
    s_telemetry.audio_crc_error_count = g_task_acoustic_link_crc_error_count;
    s_telemetry.fault_flags = g_task_think_fault_flags;
    s_telemetry.steering_deg = g_task_think_steering_deg;
    s_telemetry.left_target_rpm = s_command_snapshot.last_sent_target.left_target_rpm;
    s_telemetry.right_target_rpm = s_command_snapshot.last_sent_target.right_target_rpm;
    s_telemetry.infer_status = infer_ready ? (uint8_t) s_infer_result.identifier.status : 0U;
    s_telemetry.infer_sample_count = proto_ready ? proto_telem.sample_count : 0U;
    s_telemetry.infer_active_frames = infer_ready ? s_infer_result.active_frame_count : 0U;
    s_telemetry.infer_nearest_sample = infer_ready ? (uint8_t) s_infer_result.identifier.sample_index : 255U;
    s_telemetry.infer_cosine_dist_x1000 = cosine_dist_x1000;
    s_telemetry.infer_threshold_x1000 = threshold_x1000;
    s_telemetry.command_sequence = g_task_command_sequence;
    s_telemetry.command_last_error = (W) g_task_command_last_error;
    s_telemetry.command_target_age_ms = s_command_snapshot.target_age_ms;
    s_telemetry.infer_target_peak_bin = target_peak_bin;
    s_telemetry.infer_current_peak_bin = current_peak_bin;
    s_telemetry.infer_similarity_permille = similarity_permille;
    /* 音響リンクが合成した自律走行状態として通知する。 */
    s_telemetry.autonomy_mode = (UB) 2;
    s_telemetry.sensor_rule = (UB) g_task_think_sensor_rule;
    s_telemetry.sensor_valid_flags = s_sensor_snapshot.valid_flags;
    s_telemetry.sensor_reserved =
        ((UB) g_task_think_storage_result & ACOUSTIC_TELEMETRY_STORAGE_RESULT_MASK) |
        (g_task_think_storage_valid ? ACOUSTIC_TELEMETRY_STORAGE_VALID : 0U) |
        (g_task_think_learning_mode ? ACOUSTIC_TELEMETRY_LEARNING_MODE : 0U);
    s_telemetry.tof_distance_mm[0] = s_sensor_snapshot.tof_distance_mm[0];
    s_telemetry.tof_distance_mm[1] = s_sensor_snapshot.tof_distance_mm[1];
    s_telemetry.tof_distance_mm[2] = s_sensor_snapshot.tof_distance_mm[2];
    s_telemetry.accel_mg[0] = s_sensor_snapshot.accel_mg[0];
    s_telemetry.accel_mg[1] = s_sensor_snapshot.accel_mg[1];
    s_telemetry.accel_mg[2] = s_sensor_snapshot.accel_mg[2];
    s_telemetry.gyro_dps_x10[0] = s_sensor_snapshot.gyro_dps_x10[0];
    s_telemetry.gyro_dps_x10[1] = s_sensor_snapshot.gyro_dps_x10[1];
    s_telemetry.gyro_dps_x10[2] = s_sensor_snapshot.gyro_dps_x10[2];
    s_telemetry.sensor_error_flags = (UH) s_sensor_snapshot.error_flags;
    s_telemetry.sensor_last_error = (W) s_sensor_snapshot.last_error;
    s_telemetry.sensor_age_ms = s_sensor_snapshot.age_ms;

    size_t const length = acoustic_protocol_encode_rover_telemetry(audio_telemetry_sequence, audio_now_ms, &s_telemetry,
                                                                   audio_tx_buffer, sizeof(audio_tx_buffer));
    if (0U == length) {
        return FSP_ERR_INVALID_SIZE;
    }

    fsp_err_t const err = g_usb_on_usb.write(&g_basic0_ctrl, audio_tx_buffer, (UW) length, audio_device_address);
    if (FSP_SUCCESS == err) {
        audio_write_pending = TRUE;
        audio_write_started_at_ms = audio_now_ms;
        audio_actuator_telemetry_pending = TRUE;
        audio_last_telemetry_ms = audio_now_ms;
        audio_telemetry_sequence++;
    } else if (FSP_ERR_USB_BUSY == err) {
        g_task_acoustic_link_telemetry_busy_count++;
    }
    return err;
}

/** =================================================================*
 * @brief  sequence新旧判定
 * @details UW wrapを許容し、同値または逆行フレームを破棄する。
 * @param[in] sequence 受信sequence
 * @return 受理可能ならtrue
 * ================================================================= */
LOCAL BOOL task_acoustic_link_sequence_accept(UW sequence) {
    if (!audio_sequence_valid) {
        audio_sequence_valid = TRUE;
        audio_last_sequence = sequence;
        return TRUE;
    }

    if ((W) (sequence - audio_last_sequence) <= 0) {
        g_task_acoustic_link_sequence_drop_count++;
        return FALSE;
    }

    audio_last_sequence = sequence;
    return TRUE;
}

/** =================================================================*
 * @brief  正常フレーム反映
 * @param[in] p_frame CRC検証済みフレーム
 * ================================================================= */
LOCAL void task_acoustic_link_frame_handle(const acoustic_frame_t * p_frame) {
    acoustic_hello_t hello;

    if ((ACOUSTIC_MESSAGE_HELLO == p_frame->type) && acoustic_protocol_decode_hello(p_frame, &hello)) {
        if (audio_boot_id_valid && (audio_boot_id != hello.boot_id)) {
            audio_sequence_valid = FALSE;
            g_task_acoustic_link_hello_received = FALSE;
            task_acoustic_link_observation_reset();
            task_acoustic_link_feature_reset();
        }
        if (!task_acoustic_link_sequence_accept(p_frame->sequence)) {
            return;
        }

        UW const required = ACOUSTIC_CAPABILITY_DOA | ACOUSTIC_CAPABILITY_VAD | ACOUSTIC_CAPABILITY_LEVEL;
        audio_hello = hello;
        audio_boot_id = hello.boot_id;
        audio_boot_id_valid = TRUE;
        g_task_acoustic_link_hello_received = required == (hello.capabilities & required);
        return;
    }

    if (!task_acoustic_link_sequence_accept(p_frame->sequence)) {
        return;
    }

    if (ACOUSTIC_MESSAGE_OBSERVATION == p_frame->type) {
        acoustic_observation_t observation;
        if (acoustic_protocol_decode_observation(p_frame, &observation) && (observation.doa_deg < 360U)) {
            g_task_acoustic_link_observation = observation;
            audio_observation_sequence = p_frame->sequence;
            audio_observation_at_ms = audio_now_ms;
        }
    } else if (ACOUSTIC_MESSAGE_HEALTH == p_frame->type) {
        (void) acoustic_protocol_decode_health(p_frame, &audio_health);
    } else if (ACOUSTIC_MESSAGE_FEATURE == p_frame->type) {
        acoustic_feature_t feature;
        if (!acoustic_protocol_decode_feature(p_frame, &feature)) {
            g_task_acoustic_link_feature_drop_count++;
            g_task_acoustic_link_format_error_count++;
            return;
        }

        acoustic_feature_assembler_result_t const result =
            acoustic_feature_assembler_push(&audio_feature_assembler, &feature);
        if (CPU0_ACOUSTIC_FEATURE_COMPLETE == result) {
            memcpy(&audio_feature_patch, &audio_feature_assembler.patch, sizeof(audio_feature_patch));
            audio_feature_ready = TRUE;
            g_task_acoustic_link_feature_generation++;
            g_task_acoustic_link_feature_complete_count++;
            (void) task_infer_notify_feature_ready();
        } else if (CPU0_ACOUSTIC_FEATURE_RESTARTED == result) {
            g_task_acoustic_link_feature_drop_count++;
        } else if ((CPU0_ACOUSTIC_FEATURE_FORMAT_ERROR == result) ||
                   (CPU0_ACOUSTIC_FEATURE_SEQUENCE_ERROR == result)) {
            g_task_acoustic_link_feature_drop_count++;
        }
    }
}

/** =================================================================*
 * @brief  USB受信データ処理
 * @param[in] length 有効受信長
 * ================================================================= */
LOCAL void task_acoustic_link_receive(UW length) {
    if (length > sizeof(audio_rx_buffer)) {
        length = sizeof(audio_rx_buffer);
        g_task_acoustic_link_format_error_count++;
    }

    for (UW index = 0U; index < length; index++) {
        acoustic_frame_t frame;
        acoustic_parse_result_t const result =
            acoustic_protocol_parser_push(&audio_parser, audio_rx_buffer[index], &frame);
        if (ACOUSTIC_PARSE_FRAME_READY == result) {
            g_task_acoustic_link_frame_count++;
            task_acoustic_link_frame_handle(&frame);
        } else if (ACOUSTIC_PARSE_CRC_ERROR == result) {
            g_task_acoustic_link_crc_error_count++;
        } else if ((ACOUSTIC_PARSE_FORMAT_ERROR == result) || (ACOUSTIC_PARSE_UNSUPPORTED_VERSION == result)) {
            g_task_acoustic_link_format_error_count++;
        }
    }
}

/** =================================================================*
 * @brief  音響リンクタスク本体
 * @details Bare Metal USBイベントをμT-Kernelタスクからポーリングする。
 * ================================================================= */
LOCAL void task_acoustic_link_entry(INT stacd, void * exinf) {
    (void) stacd;
    (void) exinf;

    /*
     * J11のAI LabはJ7のReSpeaker linkとは独立に開始する。J7のUSB host
     * 初期化が失敗しても、PCから学習・推論の診断を続けられるようにする。
     */
    acoustic_ai_lab_link_init();
    g_task_acoustic_link_last_error = g_usb_on_usb.open(&g_basic0_ctrl, &g_basic0_cfg);
    if (FSP_SUCCESS != g_task_acoustic_link_last_error) {
        (void) task_think_report_fault(APP_FAULT_USB_INIT);
        audio_usb_open = FALSE;
    } else {
        audio_usb_open = TRUE;
        g_task_acoustic_link_usb_state = CPU0_AUDIO_USB_STATE_WAIT_DEVICE;
    }

    while (1) {
        audio_now_ms = task_acoustic_link_monotonic_ms();

        usb_event_info_t event_info = {0};
        usb_status_t event = (usb_status_t) 0U;
        fsp_err_t const event_err = g_usb_on_usb.eventGet(&event_info, &event);

        if (FSP_SUCCESS == event_err) {
            if (((usb_status_t) 0U != event) && (USB_STATUS_NONE != event)) {
                g_task_acoustic_link_last_event = event;
                g_task_acoustic_link_event_count++;
            }
            if (0U == event_info.module_number) {
                /* USB event queueは全IPで共通。J11イベントをJ7音響linkへ混ぜない。 */
                acoustic_ai_lab_link_event(&event_info, event, audio_now_ms);
            } else if (audio_usb_open && (USB_STATUS_CONFIGURED == event)) {
                ER const lock_err = tk_loc_mtx(acoustic_link_mutex_id, TMO_FEVR);
                if (E_OK == lock_err) {
                    task_acoustic_link_link_reset();
                    g_task_acoustic_link_usb_configured = TRUE;
                    audio_device_address = event_info.device_address;
                    g_task_acoustic_link_device_address = audio_device_address;
                    g_task_acoustic_link_usb_state = CPU0_AUDIO_USB_STATE_SET_LINE_CODING;
                    audio_configured_at_ms = audio_now_ms;
                    (void) tk_unl_mtx(acoustic_link_mutex_id);
                }
            } else if (audio_usb_open && (USB_STATUS_READ_COMPLETE == event)) {
                audio_read_pending = FALSE;
                if ((USB_CLASS_HCDC == event_info.type) &&
                    ((FSP_SUCCESS == event_info.status) || (FSP_ERR_USB_SIZE_SHORT == event_info.status))) {
                    ER const lock_err = tk_loc_mtx(acoustic_link_mutex_id, TMO_FEVR);
                    if (E_OK == lock_err) {
                        task_acoustic_link_receive(event_info.data_size);
                        (void) tk_unl_mtx(acoustic_link_mutex_id);
                    }
                    audio_continuous_error_count = 0U;
                } else if (USB_CLASS_HCDC == event_info.type) {
                    g_task_acoustic_link_last_error = event_info.status;
                    audio_continuous_error_count++;
                }
            } else if (audio_usb_open && (USB_STATUS_WRITE_COMPLETE == event)) {
                audio_write_pending = FALSE;
                if ((USB_CLASS_HCDC == event_info.type) && (FSP_SUCCESS == event_info.status)) {
                    g_task_acoustic_link_telemetry_send_count++;
                    audio_continuous_error_count = 0U;
                } else if (USB_CLASS_HCDC == event_info.type) {
                    g_task_acoustic_link_last_error = event_info.status;
                    audio_continuous_error_count++;
                }
            } else if (audio_usb_open && (USB_STATUS_REQUEST_COMPLETE == event)) {
                task_acoustic_link_control_complete(&event_info);
            } else if (audio_usb_open && (USB_STATUS_DETACH == event)) {
                ER const lock_err = tk_loc_mtx(acoustic_link_mutex_id, TMO_FEVR);
                if (E_OK == lock_err) {
                    task_acoustic_link_link_reset();
                    (void) tk_unl_mtx(acoustic_link_mutex_id);
                }
            }
        } else {
            g_task_acoustic_link_last_error = event_err;
        }

        /* 通信途絶または連続転送失敗を検知した場合は、USBバスを */
        /* リセットして再列挙する。 */
        if (g_task_acoustic_link_usb_configured && ((audio_now_ms - audio_last_recovery_ms) >= 3000U)) {
            BOOL trigger_recovery = FALSE;
            if (audio_continuous_error_count >= 5U) {
                trigger_recovery = TRUE;
            } else if (g_task_acoustic_link_hello_received && ((audio_now_ms - audio_observation_at_ms) > 2000U)) {
                trigger_recovery = TRUE;
            } else if (!g_task_acoustic_link_hello_received && ((audio_now_ms - audio_configured_at_ms) > 3000U)) {
                trigger_recovery = TRUE;
            }

            if (trigger_recovery) {
                audio_last_recovery_ms = audio_now_ms;
                task_acoustic_link_bus_recovery();
                (void) tk_dly_tsk(CPU0_AUDIO_USB_POLL_MS);
                continue;
            }
        }

        if (g_task_acoustic_link_usb_configured && (CPU0_AUDIO_USB_STATE_READY != g_task_acoustic_link_usb_state)) {
            fsp_err_t const control_err = task_acoustic_link_control_start();
            if ((FSP_SUCCESS != control_err) && (FSP_ERR_USB_BUSY != control_err)) {
                g_task_acoustic_link_last_error = control_err;
                audio_continuous_error_count++;
            }
        } else if (g_task_acoustic_link_usb_configured && !audio_read_pending) {
            fsp_err_t const read_err = task_acoustic_link_read_start();
            if ((FSP_SUCCESS != read_err) && (FSP_ERR_USB_BUSY != read_err)) {
                g_task_acoustic_link_last_error = read_err;
                audio_continuous_error_count++;
            }
        }

        if (g_task_acoustic_link_usb_configured && (CPU0_AUDIO_USB_STATE_READY == g_task_acoustic_link_usb_state)) {
            fsp_err_t const telemetry_err = task_acoustic_link_telemetry_start();
            if ((FSP_SUCCESS != telemetry_err) && (FSP_ERR_USB_BUSY != telemetry_err)) {
                g_task_acoustic_link_last_error = telemetry_err;
                audio_continuous_error_count++;
            }
        }

        /* snapshot、現在特徴量、保存見本の送信はJ7制御を妨げない時だけ進める。 */
        acoustic_ai_lab_link_poll(audio_now_ms);

        (void) tk_dly_tsk(CPU0_AUDIO_USB_POLL_MS);
    }
}
