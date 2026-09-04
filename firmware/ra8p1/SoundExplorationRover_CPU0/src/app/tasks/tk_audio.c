/** =================================================================*
 * @file   tk_audio.c
 * @brief  ReSpeaker USB CDC音響観測受信
 * ================================================================= */
#include "tk_audio.h"                                       /* CPU0音響タスクAPI */
#include "../../cpu0_config.h"                              /* USB受信周期、優先度、バッファ長 */
#include "../../ipc/actuator_ipc_client.h"                 /* CPU1実出力状態取得API */
#include "tk_command.h"                                     /* 最新指令状態取得API */
#include "tk_sensor.h"                                      /* 最新I2Cセンサー状態取得API */
#include "tk_think.h"                                       /* 思考タスクへの異常通知 */
#include <string.h>                                         /* memset */

#define CPU0_AUDIO_SET_LINE_CODING         (USB_CDC_SET_LINE_CODING | USB_HOST_TO_DEV | USB_CLASS | USB_INTERFACE)
#define CPU0_AUDIO_SET_CONTROL_LINE_STATE \
    (USB_CDC_SET_CONTROL_LINE_STATE | USB_HOST_TO_DEV | USB_CLASS | USB_INTERFACE)
#define CPU0_AUDIO_GET_LINE_CODING         (USB_CDC_GET_LINE_CODING | USB_DEV_TO_HOST | USB_CLASS | USB_INTERFACE)
#define CPU0_AUDIO_LINE_CODING_LENGTH      (7U)

LOCAL void cpu0_audio_task(INT stacd, void * exinf);       /* 音響タスク本体 */
LOCAL UW cpu0_audio_monotonic_ms(void);              /* カーネル単調時刻取得 */
LOCAL void cpu0_audio_link_reset(void);                    /* 音響リンク状態初期化 */
LOCAL void cpu0_audio_observation_reset(void);             /* 旧音響観測無効化 */
LOCAL fsp_err_t cpu0_audio_control_start(void);            /* CDC class request開始 */
LOCAL void cpu0_audio_control_complete(const usb_event_info_t * p_event_info); /* CDC class request完了 */
LOCAL fsp_err_t cpu0_audio_read_start(void);               /* USB Bulk IN開始 */
LOCAL fsp_err_t cpu0_audio_actuator_telemetry_start(void); /* CPU1診断Bulk OUT開始 */
LOCAL fsp_err_t cpu0_audio_telemetry_start(void);          /* USB Bulk OUT開始 */
LOCAL void cpu0_audio_receive(UW length);            /* USB受信データ処理 */
LOCAL void cpu0_audio_frame_handle(const acoustic_frame_t * p_frame); /* 正常フレーム反映 */
LOCAL BOOL cpu0_audio_sequence_accept(UW sequence);  /* sequence新旧判定 */

/**< 音響状態を保護するμT-Kernel mutex設定 */
LOCAL T_CMTX const audio_mutex_config = {
    .mtxatr = TA_INHERIT,
    .ceilpri = 0,
};

/**< 音響受信タスク設定 */
LOCAL T_CTSK const audio_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) cpu0_audio_task,
    .itskpri = CPU0_AUDIO_TASK_PRIORITY,
    .stksz = CPU0_AUDIO_TASK_STACK_SIZE,
    .bufptr = NULL,
};

LOCAL ID audio_task_id;                                    /**< 音響タスクID */
LOCAL ID audio_mutex_id;                                   /**< 音響状態保護mutex ID */
LOCAL BOOL audio_task_started;                             /**< 音響タスク開始状態 */
LOCAL BOOL audio_usb_open;                                 /**< USBドライバopen状態 */
LOCAL BOOL audio_read_pending;                             /**< Bulk IN要求実行中 */
LOCAL BOOL audio_write_pending;                            /**< Bulk OUT要求実行中 */
LOCAL BOOL audio_actuator_telemetry_pending;               /**< CPU1診断送信待ち */
LOCAL BOOL audio_control_pending;                          /**< CDC class request実行中 */
LOCAL BOOL audio_sequence_valid;                           /**< sequence初回受信済み */
LOCAL BOOL audio_boot_id_valid;                            /**< boot ID初回受信済み */
LOCAL UB audio_device_address;                        /**< CDCデバイスアドレス */
LOCAL UW audio_now_ms;                               /**< 音響タスク単調時刻 */
LOCAL UW audio_configured_at_ms;                     /**< USB列挙時刻 */
LOCAL UW audio_observation_at_ms;                    /**< 最終観測受信時刻 */
LOCAL UW audio_last_sequence;                        /**< 最終受信sequence */
LOCAL UW audio_observation_sequence;                 /**< 最終観測sequence */
LOCAL UW audio_telemetry_sequence;                   /**< 診断送信sequence */
LOCAL UW audio_actuator_telemetry_sequence;          /**< CPU1診断送信sequence */
LOCAL UW audio_last_telemetry_ms;                    /**< 最終診断送信要求時刻 */
LOCAL UW audio_actuator_status_at_ms;                /**< CPU1状態最終更新時刻 */
LOCAL UW audio_actuator_status_sequence;             /**< CPU1状態最終sequence */
LOCAL UW audio_boot_id;                              /**< ESP32 boot ID */
LOCAL BOOL audio_actuator_status_sequence_valid;           /**< CPU1状態sequence受信済み */
LOCAL acoustic_protocol_parser_t audio_parser;             /**< CDCストリームパーサー */
LOCAL acoustic_hello_t audio_hello;                        /**< 最新HELLO */
LOCAL acoustic_health_t audio_health;                      /**< 最新HEALTH */
/**< CDC仮想UART設定 */
LOCAL usb_hcdc_linecoding_t audio_line_coding = {
    .dwdte_rate = USB_HCDC_SPEED_115200,
    .bchar_format = USB_HCDC_STOP_BIT_1,
    .bparity_type = USB_HCDC_PARITY_BIT_NONE,
    .bdata_bits = USB_HCDC_DATA_BIT_8,
    .rsv = 0U,
};
LOCAL UB audio_control_dummy;                         /**< data無しcontrol転送用 */
LOCAL UB audio_rx_buffer[CPU0_AUDIO_USB_RX_SIZE];     /**< USB Bulk INバッファ */
/**< USB Bulk OUTバッファ */
LOCAL UB audio_tx_buffer[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE];

EXPORT volatile BOOL g_cpu0_audio_usb_configured;                  /**< USB列挙状態 */
EXPORT volatile cpu0_audio_usb_state_t g_cpu0_audio_usb_state;     /**< CDC初期化段階 */
EXPORT volatile usb_status_t g_cpu0_audio_last_event;              /**< 最終USBイベント */
EXPORT volatile UB g_cpu0_audio_device_address;               /**< CDCアドレス */
EXPORT volatile UW g_cpu0_audio_event_count;                 /**< USBイベント数 */
EXPORT volatile UW g_cpu0_audio_transfer_busy_count;         /**< USB転送BUSY回数 */
EXPORT volatile BOOL g_cpu0_audio_hello_received;                  /**< HELLO受信状態 */
EXPORT volatile UW g_cpu0_audio_frame_count;                 /**< 正常フレーム数 */
EXPORT volatile UW g_cpu0_audio_crc_error_count;             /**< CRC異常数 */
EXPORT volatile UW g_cpu0_audio_format_error_count;          /**< 形式異常数 */
EXPORT volatile UW g_cpu0_audio_sequence_drop_count;         /**< 逆行sequence数 */
EXPORT volatile UW g_cpu0_audio_observation_age_ms;          /**< 観測経過時間 */
EXPORT volatile UW g_cpu0_audio_telemetry_send_count;        /**< 診断送信完了数 */
EXPORT volatile UW g_cpu0_audio_telemetry_busy_count;        /**< 診断送信BUSY数 */
EXPORT volatile acoustic_observation_t g_cpu0_audio_observation;   /**< 最新音響観測 */
EXPORT volatile fsp_err_t g_cpu0_audio_last_error;                 /**< 最終USBエラー */

/** =================================================================*
 * @brief  カーネルの単調時刻を32bitミリ秒で取得
 * @details 32bit wrap後も符号なし差分で経過時間を計算できる。
 * @return システム起動後の単調時刻
 * ================================================================= */
LOCAL UW cpu0_audio_monotonic_ms(void) {
    SYSTIM system_time = {0};
    if (E_OK == tk_get_otm(&system_time)) {
        return system_time.lo;
    }

    return audio_now_ms + CPU0_AUDIO_USB_POLL_MS;
}

/** =================================================================*
 * @brief  音響タスクと共有資源生成
 * @return CPU0異常コード
 * ================================================================= */
EXPORT cpu0_fault_t cpu0_audio_task_create(void) {
    audio_task_id = 0;
    audio_mutex_id = 0;
    audio_task_started = FALSE;
    audio_usb_open = FALSE;
    audio_read_pending = FALSE;
    audio_write_pending = FALSE;
    audio_actuator_telemetry_pending = FALSE;
    audio_control_pending = FALSE;
    audio_now_ms = 0U;
    g_cpu0_audio_usb_state = CPU0_AUDIO_USB_STATE_CLOSED;
    g_cpu0_audio_last_event = (usb_status_t) 0U;
    g_cpu0_audio_device_address = 0U;
    g_cpu0_audio_event_count = 0U;
    g_cpu0_audio_transfer_busy_count = 0U;
    g_cpu0_audio_frame_count = 0U;
    g_cpu0_audio_crc_error_count = 0U;
    g_cpu0_audio_format_error_count = 0U;
    g_cpu0_audio_sequence_drop_count = 0U;
    g_cpu0_audio_telemetry_send_count = 0U;
    g_cpu0_audio_telemetry_busy_count = 0U;
    g_cpu0_audio_last_error = FSP_SUCCESS;
    cpu0_audio_link_reset();

    audio_mutex_id = tk_cre_mtx(&audio_mutex_config);
    if (audio_mutex_id <= 0) {
        audio_mutex_id = 0;
        return CPU0_FAULT_TASK_CREATE;
    }

    audio_task_id = tk_cre_tsk(&audio_task_config);
    if (audio_task_id <= 0) {
        audio_task_id = 0;
        cpu0_audio_task_delete();
        return CPU0_FAULT_TASK_CREATE;
    }

    return CPU0_FAULT_NONE;
}

/** =================================================================*
 * @brief  音響タスク開始
 * @return CPU0異常コード
 * ================================================================= */
EXPORT cpu0_fault_t cpu0_audio_task_start(void) {
    if (audio_task_id <= 0) {
        return CPU0_FAULT_TASK_CREATE;
    }

    ER const err = tk_sta_tsk(audio_task_id, 0);
    if (E_OK != err) {
        return CPU0_FAULT_TASK_START;
    }
    audio_task_started = TRUE;
    return CPU0_FAULT_NONE;
}

/** =================================================================*
 * @brief  音響タスクと共有資源解放
 * ================================================================= */
EXPORT void cpu0_audio_task_delete(void) {
    if (audio_task_id > 0) {
        if (audio_task_started) {
            (void) tk_ter_tsk(audio_task_id);
        }
        (void) tk_del_tsk(audio_task_id);
        audio_task_id = 0;
        audio_task_started = FALSE;
    }

    if (audio_usb_open) {
        (void) g_usb_on_usb.close(&g_basic0_ctrl);
        audio_usb_open = FALSE;
    }

    if (audio_mutex_id > 0) {
        (void) tk_del_mtx(audio_mutex_id);
        audio_mutex_id = 0;
    }
}

/** =================================================================*
 * @brief  音響リンク状態初期化
 * @details USB切断後の古い観測が走行判断へ残らないよう無効化する。
 * ================================================================= */
LOCAL void cpu0_audio_link_reset(void) {
    g_cpu0_audio_usb_configured = FALSE;
    g_cpu0_audio_usb_state = audio_usb_open ? CPU0_AUDIO_USB_STATE_WAIT_DEVICE : CPU0_AUDIO_USB_STATE_CLOSED;
    g_cpu0_audio_device_address = 0U;
    g_cpu0_audio_hello_received = FALSE;
    cpu0_audio_observation_reset();
    memset(&audio_hello, 0, sizeof(audio_hello));
    audio_sequence_valid = FALSE;
    audio_boot_id_valid = FALSE;
    audio_read_pending = FALSE;
    audio_write_pending = FALSE;
    audio_actuator_telemetry_pending = FALSE;
    audio_control_pending = FALSE;
    audio_device_address = 0U;
    audio_configured_at_ms = audio_now_ms;
    audio_last_sequence = 0U;
    audio_telemetry_sequence = 0U;
    audio_actuator_telemetry_sequence = 0U;
    audio_last_telemetry_ms = audio_now_ms;
    audio_actuator_status_at_ms = audio_now_ms;
    audio_actuator_status_sequence = 0U;
    audio_actuator_status_sequence_valid = FALSE;
    audio_boot_id = 0U;
    acoustic_protocol_parser_init(&audio_parser);
}

/** =================================================================*
 * @brief  CDC class request開始
 * @details FSP HCDCの推奨順序で仮想UARTを初期化する。
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t cpu0_audio_control_start(void) {
    if (!g_cpu0_audio_usb_configured || (0U == audio_device_address) || audio_control_pending) {
        return FSP_SUCCESS;
    }

    usb_setup_t setup = {0};
    UB * p_data = &audio_control_dummy;

    if (CPU0_AUDIO_USB_STATE_SET_LINE_CODING == g_cpu0_audio_usb_state) {
        setup.request_type = CPU0_AUDIO_SET_LINE_CODING;
        setup.request_length = CPU0_AUDIO_LINE_CODING_LENGTH;
        p_data = (UB *) &audio_line_coding;
    } else if (CPU0_AUDIO_USB_STATE_SET_CONTROL_LINE_STATE == g_cpu0_audio_usb_state) {
        setup.request_type = CPU0_AUDIO_SET_CONTROL_LINE_STATE;
    } else if (CPU0_AUDIO_USB_STATE_GET_LINE_CODING == g_cpu0_audio_usb_state) {
        setup.request_type = CPU0_AUDIO_GET_LINE_CODING;
        setup.request_length = CPU0_AUDIO_LINE_CODING_LENGTH;
        p_data = (UB *) &audio_line_coding;
    } else {
        return FSP_SUCCESS;
    }

    fsp_err_t const err = g_usb_on_usb.hostControlTransfer(&g_basic0_ctrl, &setup, p_data, audio_device_address);
    if (FSP_SUCCESS == err) {
        audio_control_pending = TRUE;
    } else if (FSP_ERR_USB_BUSY == err) {
        g_cpu0_audio_transfer_busy_count++;
    }
    return err;
}

/** =================================================================*
 * @brief  CDC class request完了
 * @param[in] p_event_info USB request完了情報
 * ================================================================= */
LOCAL void cpu0_audio_control_complete(const usb_event_info_t * p_event_info) {
    if ((NULL == p_event_info) || !audio_control_pending) {
        return;
    }

    UH const request = p_event_info->setup.request_type & USB_BREQUEST;
    audio_control_pending = FALSE;
    if (USB_SETUP_STATUS_ACK != p_event_info->status) {
        g_cpu0_audio_last_error = FSP_ERR_USB_FAILED;
        return;
    }

    if ((USB_CDC_SET_LINE_CODING == request) && (CPU0_AUDIO_USB_STATE_SET_LINE_CODING == g_cpu0_audio_usb_state)) {
        g_cpu0_audio_usb_state = CPU0_AUDIO_USB_STATE_SET_CONTROL_LINE_STATE;
    } else if ((USB_CDC_SET_CONTROL_LINE_STATE == request) &&
               (CPU0_AUDIO_USB_STATE_SET_CONTROL_LINE_STATE == g_cpu0_audio_usb_state)) {
        g_cpu0_audio_usb_state = CPU0_AUDIO_USB_STATE_GET_LINE_CODING;
    } else if ((USB_CDC_GET_LINE_CODING == request) &&
               (CPU0_AUDIO_USB_STATE_GET_LINE_CODING == g_cpu0_audio_usb_state)) {
        g_cpu0_audio_usb_state = CPU0_AUDIO_USB_STATE_READY;
    }
}

/** =================================================================*
 * @brief  旧音響観測無効化
 * @details frontend再起動後に再起動前のDoAを走行判断へ渡さない。
 * ================================================================= */
LOCAL void cpu0_audio_observation_reset(void) {
    g_cpu0_audio_observation_age_ms = UINT32_MAX;
    memset((void *) &g_cpu0_audio_observation, 0, sizeof(g_cpu0_audio_observation));
    g_cpu0_audio_observation.doa_deg = ACOUSTIC_PROTOCOL_DOA_INVALID;
    memset(&audio_health, 0, sizeof(audio_health));
    audio_observation_at_ms = audio_now_ms;
    audio_observation_sequence = 0U;
}

/** =================================================================*
 * @brief  最新音響状態取得
 * @param[out] p_snapshot 音響状態スナップショット
 * @return μT-Kernelエラーコード
 * ================================================================= */
EXPORT ER cpu0_audio_snapshot_get(cpu0_audio_snapshot_t * p_snapshot) {
    if (NULL == p_snapshot) {
        return E_PAR;
    }
    if (audio_mutex_id <= 0) {
        return E_NOEXS;
    }

    ER err = tk_loc_mtx(audio_mutex_id, TMO_POL);
    if (E_OK != err) {
        return err;
    }

    p_snapshot->usb_configured = g_cpu0_audio_usb_configured;
    p_snapshot->hello_received = g_cpu0_audio_hello_received;
    p_snapshot->observation_received = ACOUSTIC_PROTOCOL_DOA_INVALID != g_cpu0_audio_observation.doa_deg;
    p_snapshot->link_age_ms = audio_now_ms - audio_configured_at_ms;
    p_snapshot->observation_age_ms =
        p_snapshot->observation_received ? (audio_now_ms - audio_observation_at_ms) : UINT32_MAX;
    p_snapshot->observation_sequence = audio_observation_sequence;
    p_snapshot->observation = g_cpu0_audio_observation;
    p_snapshot->hello = audio_hello;
    p_snapshot->health = audio_health;
    g_cpu0_audio_observation_age_ms = p_snapshot->observation_age_ms;

    err = tk_unl_mtx(audio_mutex_id);
    return err;
}

/** =================================================================*
 * @brief  USB Bulk IN開始
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t cpu0_audio_read_start(void) {
    if (!g_cpu0_audio_usb_configured || (CPU0_AUDIO_USB_STATE_READY != g_cpu0_audio_usb_state) ||
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
        g_cpu0_audio_transfer_busy_count++;
    }
    return err;
}

/** =================================================================*
 * @brief  CPU1アクチュエータ診断情報のUSB Bulk OUT開始
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t cpu0_audio_actuator_telemetry_start(void) {
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
        audio_actuator_telemetry_pending = FALSE;
        audio_actuator_telemetry_sequence++;
    } else if (FSP_ERR_USB_BUSY == err) {
        g_cpu0_audio_telemetry_busy_count++;
    }
    return err;
}

/** =================================================================*
 * @brief  CPU0診断情報のUSB Bulk OUT開始
 * @details 音響受信を止めず、最新判断とIPC指令をESP32へ返送する。
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t cpu0_audio_telemetry_start(void) {
    if (!g_cpu0_audio_usb_configured || (CPU0_AUDIO_USB_STATE_READY != g_cpu0_audio_usb_state) ||
        (0U == audio_device_address)) {
        return FSP_ERR_NOT_OPEN;
    }
    if (audio_write_pending) {
        return FSP_SUCCESS;
    }
    if (audio_actuator_telemetry_pending) {
        return cpu0_audio_actuator_telemetry_start();
    }
    if ((audio_now_ms - audio_last_telemetry_ms) < CPU0_AUDIO_TELEMETRY_PERIOD_MS) {
        return FSP_SUCCESS;
    }

    cpu0_command_snapshot_t command_snapshot;
    ER const snapshot_err = cpu0_command_snapshot_get(&command_snapshot);
    if (E_OK != snapshot_err) {
        g_cpu0_audio_telemetry_busy_count++;
        return FSP_ERR_USB_BUSY;
    }

    cpu0_sensor_snapshot_t sensor_snapshot = {
        .age_ms = UINT32_MAX,
        .last_error = (W) FSP_ERR_NOT_OPEN,
        .error_flags = CPU0_SENSOR_ERROR_I2C_INIT,
    };
#if (CPU0_SENSOR_I2C_ENABLED != 0U)
    if (E_OK != cpu0_sensor_snapshot_get(&sensor_snapshot)) {
        sensor_snapshot.valid_flags = 0U;
    }
#endif

    UB flags = 0U;
    if (g_cpu0_audio_usb_configured) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_USB_CONFIGURED;
    }
    if (g_cpu0_audio_hello_received) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_HELLO_RECEIVED;
    }
    if (ACOUSTIC_PROTOCOL_DOA_INVALID != g_cpu0_audio_observation.doa_deg) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_OBSERVATION;
    }
    if (g_cpu0_think_link_ready) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_LINK_READY;
    }
    if (g_cpu0_think_new_observation) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_NEW_OBSERVATION;
    }
    if (command_snapshot.last_sent_target.actuator_enable) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_ACTUATOR_ENABLE;
    }
    if (command_snapshot.last_sent_target.emergency_stop) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_EMERGENCY_STOP;
    }
    if (command_snapshot.target_stale) {
        flags |= ACOUSTIC_TELEMETRY_FLAG_COMMAND_STALE;
    }

    acoustic_rover_telemetry_t telemetry = {
        .schema_version = 2U,
        .think_state = (UB) g_cpu0_think_state,
        .usb_state = (UB) g_cpu0_audio_usb_state,
        .flags = flags,
        .doa_deg = g_cpu0_audio_observation.doa_deg,
        .level_dbfs_x100 = g_cpu0_audio_observation.level_dbfs_x100,
        .peak_dbfs_x100 = g_cpu0_audio_observation.peak_dbfs_x100,
        .vad = g_cpu0_audio_observation.vad,
        .xvf_status = g_cpu0_audio_observation.xvf_status,
        .audio_flags = g_cpu0_audio_observation.audio_flags,
        .xvf_raw_status = g_cpu0_audio_observation.xvf_raw_status,
        .observation_sequence = audio_observation_sequence,
        .observation_age_ms = (ACOUSTIC_PROTOCOL_DOA_INVALID != g_cpu0_audio_observation.doa_deg)
                                  ? (audio_now_ms - audio_observation_at_ms)
                                  : UINT32_MAX,
        .audio_frame_count = g_cpu0_audio_observation.audio_frame_count,
        .audio_crc_error_count = g_cpu0_audio_crc_error_count,
        .fault_flags = g_cpu0_fault_flags,
        .steering_deg = g_cpu0_think_steering_deg,
        .left_target_rpm = command_snapshot.last_sent_target.left_target_rpm,
        .right_target_rpm = command_snapshot.last_sent_target.right_target_rpm,
        .command_sequence = g_cpu0_command_sequence,
        .command_last_error = (W) g_cpu0_command_last_error,
        .command_target_age_ms = command_snapshot.target_age_ms,
        .command_send_count = g_cpu0_command_send_count,
        .autonomy_mode = (UB) CPU0_AUTONOMY_MODE,
        .sensor_rule = (UB) g_cpu0_sensor_rule,
        .sensor_valid_flags = sensor_snapshot.valid_flags,
        .tof_distance_mm = {
            sensor_snapshot.tof_distance_mm[0],
            sensor_snapshot.tof_distance_mm[1],
            sensor_snapshot.tof_distance_mm[2],
        },
        .accel_mg = {
            sensor_snapshot.accel_mg[0],
            sensor_snapshot.accel_mg[1],
            sensor_snapshot.accel_mg[2],
        },
        .gyro_dps_x10 = {
            sensor_snapshot.gyro_dps_x10[0],
            sensor_snapshot.gyro_dps_x10[1],
            sensor_snapshot.gyro_dps_x10[2],
        },
        .sensor_error_flags = (UH) sensor_snapshot.error_flags,
        .sensor_last_error = (W) sensor_snapshot.last_error,
        .sensor_age_ms = sensor_snapshot.age_ms,
    };
    for (UW index = 0U; index < ACTUATOR_SERVO_COUNT; index++) {
        telemetry.servo_target_deg[index] = command_snapshot.last_sent_target.servo_target_deg[index];
    }

    size_t const length = acoustic_protocol_encode_rover_telemetry(audio_telemetry_sequence, audio_now_ms, &telemetry,
                                                                   audio_tx_buffer, sizeof(audio_tx_buffer));
    if (0U == length) {
        return FSP_ERR_INVALID_SIZE;
    }

    fsp_err_t const err = g_usb_on_usb.write(&g_basic0_ctrl, audio_tx_buffer, (UW) length, audio_device_address);
    if (FSP_SUCCESS == err) {
        audio_write_pending = TRUE;
        audio_actuator_telemetry_pending = TRUE;
        audio_last_telemetry_ms = audio_now_ms;
        audio_telemetry_sequence++;
    } else if (FSP_ERR_USB_BUSY == err) {
        g_cpu0_audio_telemetry_busy_count++;
    }
    return err;
}

/** =================================================================*
 * @brief  sequence新旧判定
 * @details UW wrapを許容し、同値または逆行フレームを破棄する。
 * @param[in] sequence 受信sequence
 * @return 受理可能ならtrue
 * ================================================================= */
LOCAL BOOL cpu0_audio_sequence_accept(UW sequence) {
    if (!audio_sequence_valid) {
        audio_sequence_valid = TRUE;
        audio_last_sequence = sequence;
        return TRUE;
    }

    if ((W) (sequence - audio_last_sequence) <= 0) {
        g_cpu0_audio_sequence_drop_count++;
        return FALSE;
    }

    audio_last_sequence = sequence;
    return TRUE;
}

/** =================================================================*
 * @brief  正常フレーム反映
 * @param[in] p_frame CRC検証済みフレーム
 * ================================================================= */
LOCAL void cpu0_audio_frame_handle(const acoustic_frame_t * p_frame) {
    acoustic_hello_t hello;

    if ((ACOUSTIC_MESSAGE_HELLO == p_frame->type) && acoustic_protocol_decode_hello(p_frame, &hello)) {
        if (audio_boot_id_valid && (audio_boot_id != hello.boot_id)) {
            audio_sequence_valid = FALSE;
            g_cpu0_audio_hello_received = FALSE;
            cpu0_audio_observation_reset();
        }
        if (!cpu0_audio_sequence_accept(p_frame->sequence)) {
            return;
        }

        UW const required = ACOUSTIC_CAPABILITY_DOA | ACOUSTIC_CAPABILITY_VAD | ACOUSTIC_CAPABILITY_LEVEL;
        audio_hello = hello;
        audio_boot_id = hello.boot_id;
        audio_boot_id_valid = TRUE;
        g_cpu0_audio_hello_received = required == (hello.capabilities & required);
        return;
    }

    if (!cpu0_audio_sequence_accept(p_frame->sequence)) {
        return;
    }

    if (ACOUSTIC_MESSAGE_OBSERVATION == p_frame->type) {
        acoustic_observation_t observation;
        if (acoustic_protocol_decode_observation(p_frame, &observation) && (observation.doa_deg < 360U)) {
            g_cpu0_audio_observation = observation;
            audio_observation_sequence = p_frame->sequence;
            audio_observation_at_ms = audio_now_ms;
        }
    } else if (ACOUSTIC_MESSAGE_HEALTH == p_frame->type) {
        (void) acoustic_protocol_decode_health(p_frame, &audio_health);
    }
}

/** =================================================================*
 * @brief  USB受信データ処理
 * @param[in] length 有効受信長
 * ================================================================= */
LOCAL void cpu0_audio_receive(UW length) {
    if (length > sizeof(audio_rx_buffer)) {
        length = sizeof(audio_rx_buffer);
        g_cpu0_audio_format_error_count++;
    }

    for (UW index = 0U; index < length; index++) {
        acoustic_frame_t frame;
        acoustic_parse_result_t const result =
            acoustic_protocol_parser_push(&audio_parser, audio_rx_buffer[index], &frame);
        if (ACOUSTIC_PARSE_FRAME_READY == result) {
            g_cpu0_audio_frame_count++;
            cpu0_audio_frame_handle(&frame);
        } else if (ACOUSTIC_PARSE_CRC_ERROR == result) {
            g_cpu0_audio_crc_error_count++;
        } else if ((ACOUSTIC_PARSE_FORMAT_ERROR == result) || (ACOUSTIC_PARSE_UNSUPPORTED_VERSION == result)) {
            g_cpu0_audio_format_error_count++;
        }
    }
}

/** =================================================================*
 * @brief  音響タスク本体
 * @details Bare Metal USBイベントをμT-Kernelタスクからポーリングする。
 * ================================================================= */
LOCAL void cpu0_audio_task(INT stacd, void * exinf) {
    (void) stacd;
    (void) exinf;

    g_cpu0_audio_last_error = g_usb_on_usb.open(&g_basic0_ctrl, &g_basic0_cfg);
    if (FSP_SUCCESS != g_cpu0_audio_last_error) {
        (void) cpu0_think_report_fault(CPU0_FAULT_USB_INIT);
        while (1) {
            (void) tk_dly_tsk(CPU0_AUDIO_USB_POLL_MS);
        }
    }
    audio_usb_open = TRUE;
    g_cpu0_audio_usb_state = CPU0_AUDIO_USB_STATE_WAIT_DEVICE;

    while (1) {
        audio_now_ms = cpu0_audio_monotonic_ms();

        usb_event_info_t event_info = {0};
        usb_status_t event = (usb_status_t) 0U;
        fsp_err_t const event_err = g_usb_on_usb.eventGet(&event_info, &event);

        if (FSP_SUCCESS == event_err) {
            if ((usb_status_t) 0U != event) {
                g_cpu0_audio_last_event = event;
                g_cpu0_audio_event_count++;
            }
            if (USB_STATUS_CONFIGURED == event) {
                ER const lock_err = tk_loc_mtx(audio_mutex_id, TMO_FEVR);
                if (E_OK == lock_err) {
                    cpu0_audio_link_reset();
                    g_cpu0_audio_usb_configured = TRUE;
                    audio_device_address = event_info.device_address;
                    g_cpu0_audio_device_address = audio_device_address;
                    g_cpu0_audio_usb_state = CPU0_AUDIO_USB_STATE_SET_LINE_CODING;
                    audio_configured_at_ms = audio_now_ms;
                    (void) tk_unl_mtx(audio_mutex_id);
                }
            } else if (USB_STATUS_READ_COMPLETE == event) {
                audio_read_pending = FALSE;
                if ((USB_CLASS_HCDC == event_info.type) &&
                    ((FSP_SUCCESS == event_info.status) || (FSP_ERR_USB_SIZE_SHORT == event_info.status))) {
                    ER const lock_err = tk_loc_mtx(audio_mutex_id, TMO_FEVR);
                    if (E_OK == lock_err) {
                        cpu0_audio_receive(event_info.data_size);
                        (void) tk_unl_mtx(audio_mutex_id);
                    }
                } else if (USB_CLASS_HCDC == event_info.type) {
                    g_cpu0_audio_last_error = event_info.status;
                }
            } else if (USB_STATUS_WRITE_COMPLETE == event) {
                audio_write_pending = FALSE;
                if ((USB_CLASS_HCDC == event_info.type) && (FSP_SUCCESS == event_info.status)) {
                    g_cpu0_audio_telemetry_send_count++;
                } else if (USB_CLASS_HCDC == event_info.type) {
                    g_cpu0_audio_last_error = event_info.status;
                }
            } else if (USB_STATUS_REQUEST_COMPLETE == event) {
                cpu0_audio_control_complete(&event_info);
            } else if (USB_STATUS_DETACH == event) {
                ER const lock_err = tk_loc_mtx(audio_mutex_id, TMO_FEVR);
                if (E_OK == lock_err) {
                    cpu0_audio_link_reset();
                    (void) tk_unl_mtx(audio_mutex_id);
                }
            }
        } else {
            g_cpu0_audio_last_error = event_err;
        }

        if (g_cpu0_audio_usb_configured && (CPU0_AUDIO_USB_STATE_READY != g_cpu0_audio_usb_state)) {
            fsp_err_t const control_err = cpu0_audio_control_start();
            if ((FSP_SUCCESS != control_err) && (FSP_ERR_USB_BUSY != control_err)) {
                g_cpu0_audio_last_error = control_err;
            }
        } else if (g_cpu0_audio_usb_configured && !audio_read_pending) {
            fsp_err_t const read_err = cpu0_audio_read_start();
            if ((FSP_SUCCESS != read_err) && (FSP_ERR_USB_BUSY != read_err)) {
                g_cpu0_audio_last_error = read_err;
            }
        }

        if (g_cpu0_audio_usb_configured && (CPU0_AUDIO_USB_STATE_READY == g_cpu0_audio_usb_state)) {
            fsp_err_t const telemetry_err = cpu0_audio_telemetry_start();
            if ((FSP_SUCCESS != telemetry_err) && (FSP_ERR_USB_BUSY != telemetry_err)) {
                g_cpu0_audio_last_error = telemetry_err;
            }
        }

        (void) tk_dly_tsk(CPU0_AUDIO_USB_POLL_MS);
    }
}
