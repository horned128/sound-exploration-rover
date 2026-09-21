/** =================================================================*
 * @file   acoustic_ai_lab_link.c
 * @brief  J11 USB CDC経由の音響AIラボ通信
 * @details J7の音響host linkとUSBイベントqueueを共用するため、このserviceは
 *          task_acoustic_linkのtask contextからだけ呼ばれる。PC未接続や本link
 *          の異常は走行安全状態へ影響させない。
 * ================================================================= */
#include "services/acoustic_ai_lab_link.h"                 /* AIラボ通信API */
#include "platform/acoustic_ai_lab_usb_descriptor.h"       /* J11 USB descriptor */
#include "services/acoustic_identifier.h"                  /* ピークbin算出 */
#include "tasks/task_acoustic_link.h"                      /* 音響観測snapshot */
#include "tasks/task_infer.h"                              /* 推論結果と保存見本 */
#include "tasks/task_think.h"                              /* 現場学習操作要求 */
#include "r_usb_basic.h"                                   /* USB descriptor定数 */
#include <string.h>                                         /* memcpy */

#define AI_LAB_USB_RX_SIZE                 (256U)           /**< PC command受信バッファ長[byte] */
#define AI_LAB_SNAPSHOT_PERIOD_MS          (250U)           /**< snapshot送信周期[ms] */
#define AI_LAB_SUMMARY_CHUNK_COUNT         (3U)             /**< 要約chunk数 */
#define AI_LAB_PROFILE_CHUNK_COUNT         (3U)             /**< 保存見本chunk数 */
#define AI_LAB_USB_VENDOR_ID              (0x0000U)         /**< 固有VIDを主張しない */
#define AI_LAB_USB_PRODUCT_ID             (0x0001U)         /**< 固定PID */
#define AI_LAB_USB_RELEASE                (0x0100U)         /**< USB release番号 */
#define AI_LAB_USB_CONFIGURATION_SIZE     (67U)            /**< configuration descriptor長[byte] */

/* CDC ACM descriptor。J11をmacOS/Windows/Linux標準CDCとして列挙する。 */
/**< J11 CDCデバイスdescriptor */
LOCAL UB ai_lab_device_descriptor[] = {
    18U, USB_DT_DEVICE, 0x00U, 0x02U, USB_IFCLS_CDCC, 0x00U, 0x00U, 64U,
    (UB) (AI_LAB_USB_VENDOR_ID & 0xFFU), (UB) (AI_LAB_USB_VENDOR_ID >> 8U),
    (UB) (AI_LAB_USB_PRODUCT_ID & 0xFFU), (UB) (AI_LAB_USB_PRODUCT_ID >> 8U),
    (UB) (AI_LAB_USB_RELEASE & 0xFFU), (UB) (AI_LAB_USB_RELEASE >> 8U), 1U, 2U, 3U, 1U,
};
/**< J11 CDC configuration descriptor */
LOCAL UB ai_lab_configuration_descriptor[] = {
    9U, USB_DT_CONFIGURATION, AI_LAB_USB_CONFIGURATION_SIZE, 0x00U,
    2U, 1U, 0U, (USB_CF_RESERVED | USB_CF_BUSP), 50U,
    9U, USB_DT_INTERFACE, 0U, 0U, 1U, USB_IFCLS_CDCC, 0x02U, 0x01U, 0U,
    5U, 0x24U, 0x00U, 0x10U, 0x01U, 4U, 0x24U, 0x02U, 0x02U,
    5U, 0x24U, 0x06U, 0U, 1U, 5U, 0x24U, 0x01U, 0x03U, 1U,
    7U, USB_DT_ENDPOINT, (USB_EP_IN | USB_EP3), USB_EP_INT, 16U, 0U, 16U,
    9U, USB_DT_INTERFACE, 1U, 0U, 2U, USB_IFCLS_CDCD, 0U, 0U, 0U,
    7U, USB_DT_ENDPOINT, (USB_EP_IN | USB_EP1), USB_EP_BULK, 64U, 0U, 0U,
    7U, USB_DT_ENDPOINT, (USB_EP_OUT | USB_EP2), USB_EP_BULK, 64U, 0U, 0U,
};
/*
 * FSPはFull-Speed deviceでもother-speed descriptorのtypeを書き換える。
 * 同じ配列をp_config_f/p_config_hへ渡すと通常のconfiguration descriptor
 * までUSB_DT_OTHER_SPEED_CONFになり、hostがCONFIGUREDへ遷移できない。
 */
/**< J11 CDC other-speed configuration descriptor */
LOCAL UB ai_lab_other_speed_configuration_descriptor[] = {
    9U, USB_DT_CONFIGURATION, AI_LAB_USB_CONFIGURATION_SIZE, 0x00U,
    2U, 1U, 0U, (USB_CF_RESERVED | USB_CF_BUSP), 50U,
    9U, USB_DT_INTERFACE, 0U, 0U, 1U, USB_IFCLS_CDCC, 0x02U, 0x01U, 0U,
    5U, 0x24U, 0x00U, 0x10U, 0x01U, 4U, 0x24U, 0x02U, 0x02U,
    5U, 0x24U, 0x06U, 0U, 1U, 5U, 0x24U, 0x01U, 0x03U, 1U,
    7U, USB_DT_ENDPOINT, (USB_EP_IN | USB_EP3), USB_EP_INT, 16U, 0U, 16U,
    9U, USB_DT_INTERFACE, 1U, 0U, 2U, USB_IFCLS_CDCD, 0U, 0U, 0U,
    7U, USB_DT_ENDPOINT, (USB_EP_IN | USB_EP1), USB_EP_BULK, 64U, 0U, 0U,
    7U, USB_DT_ENDPOINT, (USB_EP_OUT | USB_EP2), USB_EP_BULK, 64U, 0U, 0U,
};
LOCAL UB ai_lab_string_language[] = {4U, USB_DT_STRING, 0x09U, 0x04U}; /**< USB言語ID */
LOCAL UB ai_lab_string_manufacturer[] = {
    24U, USB_DT_STRING, 'S', 0U, 'o', 0U, 'u', 0U, 'n', 0U, 'd', 0U, ' ', 0U,
    'R', 0U, 'o', 0U, 'v', 0U, 'e', 0U, 'r', 0U,
}; /**< USBメーカー文字列 */
LOCAL UB ai_lab_string_product[] = {
    32U, USB_DT_STRING, 'A', 0U, 'c', 0U, 'o', 0U, 'u', 0U, 's', 0U, 't', 0U,
    'i', 0U, 'c', 0U, ' ', 0U, 'A', 0U, 'I', 0U, ' ', 0U, 'L', 0U, 'a', 0U, 'b', 0U,
}; /**< USB製品文字列 */
/**< USBシリアル文字列 */
LOCAL UB ai_lab_string_serial[] = {
    10U, USB_DT_STRING, '0', 0U, '0', 0U, '0', 0U, '1', 0U,
};
LOCAL UB * ai_lab_string_table[] = {
    ai_lab_string_language, ai_lab_string_manufacturer, ai_lab_string_product, ai_lab_string_serial,
}; /**< USB文字列descriptor table */
/**< J11 CDC descriptor registration */
EXPORT usb_descriptor_t g_acoustic_ai_lab_usb_descriptor = {
    .p_device = ai_lab_device_descriptor,
    .p_config_f = ai_lab_configuration_descriptor,
    .p_config_h = ai_lab_other_speed_configuration_descriptor,
    .p_qualifier = NULL,
    .p_string = ai_lab_string_table,
    .num_string = (UB) (sizeof(ai_lab_string_table) / sizeof(ai_lab_string_table[0])),
};

LOCAL usb_cfg_t ai_lab_usb_config;                          /**< descriptorを差したJ11設定コピー */
LOCAL UB ai_lab_rx_buffer[AI_LAB_USB_RX_SIZE];               /**< PCDC Bulk OUT受信 */
LOCAL UB ai_lab_tx_buffer[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE]; /**< PCDC Bulk IN送信 */
LOCAL acoustic_protocol_parser_t ai_lab_parser;              /**< PC commandフレーム復元 */
/**< PCDC line coding設定 */
LOCAL usb_pcdc_linecoding_t ai_lab_line_coding = {
    .dw_dte_rate = 115200U, .b_char_format = 0U, .b_parity_type = 0U, .b_data_bits = 8U, .rsv = 0U,
};
LOCAL BOOL ai_lab_read_pending;                             /**< PCDC read実行中 */
LOCAL BOOL ai_lab_write_pending;                            /**< PCDC write実行中 */
LOCAL UW ai_lab_last_snapshot_ms;                           /**< 最終snapshot送信時刻[ms] */
LOCAL UW ai_lab_tx_sequence;                                /**< AIラボ送信sequence */
LOCAL BOOL ai_lab_snapshot_requested;                       /**< 即時snapshot要求 */
LOCAL BOOL ai_lab_command_result_pending;                   /**< 操作結果送信待ち */
LOCAL UB ai_lab_command_result_command;                     /**< 操作結果の要求種別 */
LOCAL UB ai_lab_command_result_status;                      /**< 操作結果コード */
LOCAL B ai_lab_summary[CPU0_ACOUSTIC_SUMMARY_DIMENSION];    /**< 最新特徴量要約 */
LOCAL UW ai_lab_summary_generation;                         /**< 要約の特徴量世代 */
LOCAL UB ai_lab_summary_chunk_index;                        /**< 次に送る要約chunk */
LOCAL BOOL ai_lab_summary_pending;                          /**< 要約chunk送信待ち */
LOCAL prototype_storage_data_t ai_lab_profile_data;          /**< stackを使わない保存見本コピー */
LOCAL BOOL ai_lab_profile_valid;                            /**< 保存見本の有効状態 */
LOCAL UB ai_lab_profile_sample_index;                       /**< 次に送る保存見本番号 */
LOCAL UB ai_lab_profile_chunk_index;                        /**< 次に送る保存見本chunk */
LOCAL BOOL ai_lab_profile_pending;                          /**< 保存見本送信待ち */

EXPORT volatile BOOL g_acoustic_ai_lab_usb_open;             /**< J11 USB driver open状態 */
EXPORT volatile BOOL g_acoustic_ai_lab_usb_configured;       /**< PC CDC列挙状態 */
EXPORT volatile UW g_acoustic_ai_lab_snapshot_count;         /**< snapshot送信完了数 */
EXPORT volatile UW g_acoustic_ai_lab_command_count;          /**< PC操作受信数 */
EXPORT volatile UW g_acoustic_ai_lab_error_count;            /**< USB/protocol異常数 */
EXPORT volatile fsp_err_t g_acoustic_ai_lab_last_error;      /**< 直近USBエラー */

LOCAL void acoustic_ai_lab_read_start(void);                 /* 次のPC command受信を開始 */
LOCAL void acoustic_ai_lab_snapshot_queue(UW now_ms);        /* snapshot送信をキュー */
LOCAL void acoustic_ai_lab_summary_queue(void);              /* 要約chunk送信をキュー */
LOCAL void acoustic_ai_lab_command_handle(const acoustic_frame_t * p_frame); /* PC操作を処理 */
LOCAL void acoustic_ai_lab_command_result_queue(UB command, acoustic_ai_lab_command_result_t result); /* 結果 */
LOCAL BOOL acoustic_ai_lab_write(const UB * p_bytes, UW length); /* PCDC Bulk IN送信開始 */
LOCAL UH acoustic_ai_lab_float_x1000(float value);           /**< floatを診断wire値へ縮約 */

/** =================================================================*
 * @brief 0..65.535のfloatをu16のx1000固定小数点へ変換
 * @param[in] value 変換するfloat値
 * @return 0～65535の固定小数点値
 * ================================================================= */
LOCAL UH acoustic_ai_lab_float_x1000(float value) {
    if (value <= 0.0F) {
        return 0U;
    }
    if (value >= 65.535F) {
        return UINT16_MAX;
    }
    return (UH) (value * 1000.0F + 0.5F);
}

/** =================================================================*
 * @brief J11をPC側USB hostへ接続するためのCDC deviceを初期化
 * ================================================================= */
EXPORT void acoustic_ai_lab_link_init(void) {
    if (g_acoustic_ai_lab_usb_open) {
        return;
    }

    g_acoustic_ai_lab_usb_configured = FALSE;
    g_acoustic_ai_lab_snapshot_count = 0U;
    g_acoustic_ai_lab_command_count = 0U;
    g_acoustic_ai_lab_error_count = 0U;
    g_acoustic_ai_lab_last_error = FSP_SUCCESS;
    ai_lab_read_pending = FALSE;
    ai_lab_write_pending = FALSE;
    ai_lab_last_snapshot_ms = 0U;
    ai_lab_tx_sequence = 0U;
    ai_lab_snapshot_requested = FALSE;
    ai_lab_command_result_pending = FALSE;
    ai_lab_summary_pending = FALSE;
    ai_lab_profile_pending = FALSE;
    acoustic_protocol_parser_init(&ai_lab_parser);

    /*
     * J11のrole/VBUSはFSPの起動時pin設定で確定する。
     * P500はdevice role用Low、P407はUSB VBUS入力。D+/D- (P814/P815)は
     * USB FS周辺回路の専用信号であり、GPIOとして再設定しない。
     */

    ai_lab_usb_config = g_acoustic_ai_lab_usb_cfg;
    ai_lab_usb_config.p_usb_reg = &g_acoustic_ai_lab_usb_descriptor;
    g_acoustic_ai_lab_last_error = g_usb_on_usb.open(&g_acoustic_ai_lab_usb_ctrl, &ai_lab_usb_config);
    if (FSP_SUCCESS == g_acoustic_ai_lab_last_error) {
        g_acoustic_ai_lab_usb_open = TRUE;
    } else {
        g_acoustic_ai_lab_error_count++;
    }
}

/** =================================================================*
 * @brief J11 CDCを終了
 * ================================================================= */
EXPORT void acoustic_ai_lab_link_deinit(void) {
    if (g_acoustic_ai_lab_usb_open) {
        (void) g_usb_on_usb.close(&g_acoustic_ai_lab_usb_ctrl);
    }
    g_acoustic_ai_lab_usb_open = FALSE;
    g_acoustic_ai_lab_usb_configured = FALSE;
    ai_lab_read_pending = FALSE;
    ai_lab_write_pending = FALSE;
}

/** =================================================================*
 * @brief 次のPC command受信を開始
 * ================================================================= */
LOCAL void acoustic_ai_lab_read_start(void) {
    if (!g_acoustic_ai_lab_usb_configured || ai_lab_read_pending) {
        return;
    }
    fsp_err_t err = g_usb_on_usb.read(&g_acoustic_ai_lab_usb_ctrl, ai_lab_rx_buffer,
                                      sizeof(ai_lab_rx_buffer), USB_CLASS_PCDC);
    if (FSP_SUCCESS == err) {
        ai_lab_read_pending = TRUE;
    } else if (FSP_ERR_USB_BUSY != err) {
        g_acoustic_ai_lab_last_error = err;
        g_acoustic_ai_lab_error_count++;
    }
}

/** =================================================================*
 * @brief PCDC Bulk INへ完成済みframeを開始
 * @param[in] p_bytes 送信するframe
 * @param[in] length 送信長[byte]
 * @return 送信開始ならTRUE
 * ================================================================= */
LOCAL BOOL acoustic_ai_lab_write(const UB * p_bytes, UW length) {
    if (!g_acoustic_ai_lab_usb_configured || ai_lab_write_pending || (NULL == p_bytes) ||
        (0U == length) || (length > sizeof(ai_lab_tx_buffer))) {
        return FALSE;
    }
    if (p_bytes != ai_lab_tx_buffer) {
        memcpy(ai_lab_tx_buffer, p_bytes, length);
    }
    fsp_err_t err = g_usb_on_usb.write(&g_acoustic_ai_lab_usb_ctrl, ai_lab_tx_buffer, length,
                                       USB_CLASS_PCDC);
    if (FSP_SUCCESS == err) {
        ai_lab_write_pending = TRUE;
        return TRUE;
    }
    if (FSP_ERR_USB_BUSY != err) {
        g_acoustic_ai_lab_last_error = err;
        g_acoustic_ai_lab_error_count++;
    }
    return FALSE;
}

/** =================================================================*
 * @brief 現在の音響・推論・学習状態をsnapshotとして送信
 * @param[in] now_ms 現在時刻[ms]
 * ================================================================= */
LOCAL void acoustic_ai_lab_snapshot_queue(UW now_ms) {
    task_acoustic_link_snapshot_t audio = {0};
    task_infer_result_t infer = {0};
    task_infer_prototype_telemetry_t prototype = {0};
    BOOL audio_ready = (E_OK == task_acoustic_link_snapshot_get(&audio));
    BOOL infer_ready = (E_OK == task_infer_result_get(&infer));
    BOOL prototype_ready = (E_OK == task_infer_prototype_telemetry_get(&prototype));

    acoustic_ai_lab_snapshot_t snapshot = {0};
    snapshot.schema_version = 1U;
    snapshot.think_state = (UB) g_task_think_state;
    snapshot.doa_deg = audio_ready ? audio.observation.doa_deg : ACOUSTIC_PROTOCOL_DOA_INVALID;
    snapshot.level_dbfs_x100 = audio_ready ? audio.observation.level_dbfs_x100 : 0;
    snapshot.peak_dbfs_x100 = audio_ready ? audio.observation.peak_dbfs_x100 : 0;
    snapshot.vad = audio_ready ? audio.observation.vad : 0U;
    snapshot.xvf_status = audio_ready ? audio.observation.xvf_status : ACOUSTIC_XVF_STATUS_ERROR;
    snapshot.audio_flags = audio_ready ? audio.observation.audio_flags : 0U;
    snapshot.learning_samples = g_task_think_learning_samples;
    snapshot.target_peak_bin = prototype_ready ? prototype.target_peak_bin : 255U;
    snapshot.current_peak_bin = 255U;
    snapshot.nearest_sample = 255U;
    snapshot.cosine_distance_x1000 = UINT16_MAX;
    snapshot.identifier_threshold_x1000 = prototype_ready ?
        acoustic_ai_lab_float_x1000(prototype.identifier_threshold) : 0U;
    snapshot.observation_sequence = audio_ready ? audio.observation_sequence : 0U;
    snapshot.feature_generation = infer_ready ? infer.feature_generation : 0U;
    snapshot.inference_count = g_task_infer_inference_count;
    snapshot.match_count = g_task_infer_match_count;
    snapshot.storage_result = (UB) g_task_think_storage_result;

    if (g_task_think_link_ready) {
        snapshot.flags |= ACOUSTIC_AI_LAB_FLAG_LINK_READY;
    }
    if (g_task_think_learning_mode) {
        snapshot.flags |= ACOUSTIC_AI_LAB_FLAG_LEARNING_ACTIVE;
    }
    if (prototype_ready && prototype.storage_valid) {
        snapshot.flags |= ACOUSTIC_AI_LAB_FLAG_STORAGE_VALID;
    }
    if (infer_ready) {
        snapshot.infer_status = (UB) infer.identifier.status;
        snapshot.active_frame_count = infer.active_frame_count;
        snapshot.background_mse_x1000 = acoustic_ai_lab_float_x1000(infer.event_mse);
        snapshot.background_threshold_x1000 = infer.background_threshold_valid ?
            acoustic_ai_lab_float_x1000(infer.background_threshold) : 0U;
        if (infer.summary_valid) {
            snapshot.flags |= ACOUSTIC_AI_LAB_FLAG_SUMMARY_VALID;
        }
        if (infer.background_anomaly) {
            snapshot.flags |= ACOUSTIC_AI_LAB_FLAG_BACKGROUND_ANOMALY;
        }
        if (infer.identifier.minimum_cosine_distance >= 0.0F) {
            snapshot.cosine_distance_x1000 = acoustic_ai_lab_float_x1000(infer.identifier.minimum_cosine_distance);
            snapshot.similarity_permille = (snapshot.cosine_distance_x1000 > 1000U) ? 0U :
                                         (UH) (1000U - snapshot.cosine_distance_x1000);
        }
        if (infer.identifier.threshold >= 0.0F) {
            snapshot.identifier_threshold_x1000 = acoustic_ai_lab_float_x1000(infer.identifier.threshold);
        }
        if (infer.identifier.sample_index <= 255U) {
            snapshot.nearest_sample = (UB) infer.identifier.sample_index;
        }
        if (infer.summary_valid && (infer.active_frame_count >= CPU0_ACOUSTIC_MIN_ACTIVE_FRAME_COUNT)) {
            snapshot.current_peak_bin = acoustic_identifier_find_peak_bin(infer.summary, 1U);
        }
    }

    UW length = (UW) acoustic_protocol_encode_ai_lab_snapshot(ai_lab_tx_sequence, now_ms, &snapshot,
                                                              ai_lab_tx_buffer, sizeof(ai_lab_tx_buffer));
    if ((0U != length) && acoustic_ai_lab_write(ai_lab_tx_buffer, (UW) length)) {
        ai_lab_tx_sequence++;
        ai_lab_last_snapshot_ms = now_ms;
        ai_lab_snapshot_requested = FALSE;
        g_acoustic_ai_lab_snapshot_count++;
    }
}

/** =================================================================*
 * @brief 新しい192次元要約を3つの64byte chunkへキュー
 * ================================================================= */
LOCAL void acoustic_ai_lab_summary_queue(void) {
    task_infer_result_t infer = {0};
    if ((E_OK != task_infer_result_get(&infer)) || (infer.feature_generation == ai_lab_summary_generation)) {
        return;
    }
    ai_lab_summary_generation = infer.feature_generation;
    memcpy(ai_lab_summary, infer.summary, sizeof(ai_lab_summary));
    ai_lab_summary_chunk_index = 0U;
    ai_lab_summary_pending = TRUE;
}

/** =================================================================*
 * @brief AIラボ操作結果を次のBulk INへキュー
 * @param[in] command 操作種別
 * @param[in] result 受付結果
 * ================================================================= */
LOCAL void acoustic_ai_lab_command_result_queue(UB command, acoustic_ai_lab_command_result_t result) {
    ai_lab_command_result_command = command;
    ai_lab_command_result_status = (UB) result;
    ai_lab_command_result_pending = TRUE;
}

/** =================================================================*
 * @brief PCから受信した1操作を、走行を迂回せず思考taskへ渡す
 * @param[in] p_frame 受信済みcommand frame
 * ================================================================= */
LOCAL void acoustic_ai_lab_command_handle(const acoustic_frame_t * p_frame) {
    if ((NULL == p_frame) || (ACOUSTIC_MESSAGE_AI_LAB_COMMAND != p_frame->type) ||
        (ACOUSTIC_AI_LAB_COMMAND_PAYLOAD_SIZE != p_frame->payload_length)) {
        g_acoustic_ai_lab_error_count++;
        return;
    }
    UB command = p_frame->payload[0];
    g_acoustic_ai_lab_command_count++;
    if (ACOUSTIC_AI_LAB_COMMAND_STATUS == command) {
        ai_lab_snapshot_requested = TRUE;
        acoustic_ai_lab_command_result_queue(command, ACOUSTIC_AI_LAB_COMMAND_ACCEPTED);
    } else if (ACOUSTIC_AI_LAB_COMMAND_PROFILE_READ == command) {
        if (E_OK == task_infer_prototype_get(&ai_lab_profile_data, &ai_lab_profile_valid)) {
            ai_lab_profile_sample_index = 0U;
            ai_lab_profile_chunk_index = 0U;
            ai_lab_profile_pending = ai_lab_profile_valid && (0U != ai_lab_profile_data.sample_count);
            acoustic_ai_lab_command_result_queue(command, ai_lab_profile_pending ?
                                                ACOUSTIC_AI_LAB_COMMAND_ACCEPTED : ACOUSTIC_AI_LAB_COMMAND_UNAVAILABLE);
        } else {
            acoustic_ai_lab_command_result_queue(command, ACOUSTIC_AI_LAB_COMMAND_UNAVAILABLE);
        }
    } else {
        task_think_learning_command_t learning_command = TASK_THINK_LEARNING_COMMAND_NONE;
        if (ACOUSTIC_AI_LAB_COMMAND_LEARNING_START == command) {
            learning_command = TASK_THINK_LEARNING_COMMAND_START;
        } else if (ACOUSTIC_AI_LAB_COMMAND_LEARNING_COMMIT == command) {
            learning_command = TASK_THINK_LEARNING_COMMAND_COMMIT;
        } else if (ACOUSTIC_AI_LAB_COMMAND_LEARNING_CANCEL == command) {
            learning_command = TASK_THINK_LEARNING_COMMAND_CANCEL;
        }
        if (TASK_THINK_LEARNING_COMMAND_NONE == learning_command) {
            acoustic_ai_lab_command_result_queue(command, ACOUSTIC_AI_LAB_COMMAND_REJECTED);
        } else {
            acoustic_ai_lab_command_result_queue(command,
                (E_OK == task_think_learning_request(learning_command)) ?
                    ACOUSTIC_AI_LAB_COMMAND_ACCEPTED : ACOUSTIC_AI_LAB_COMMAND_UNAVAILABLE);
        }
    }
}

/** =================================================================*
 * @brief 共有USB event queueからJ11向けイベントだけを処理
 * @param[in] p_event_info USBイベント情報
 * @param[in] event USBイベント種別
 * @param[in] now_ms 現在時刻[ms]
 * ================================================================= */
EXPORT void acoustic_ai_lab_link_event(const usb_event_info_t * p_event_info, usb_status_t event, UW now_ms) {
    if ((NULL == p_event_info) || (0U != p_event_info->module_number) || !g_acoustic_ai_lab_usb_open) {
        return;
    }
    if (USB_STATUS_CONFIGURED == event) {
        g_acoustic_ai_lab_usb_configured = TRUE;
        ai_lab_snapshot_requested = TRUE;
        acoustic_ai_lab_read_start();
    } else if (USB_STATUS_READ_COMPLETE == event) {
        ai_lab_read_pending = FALSE;
        if ((USB_CLASS_PCDC == p_event_info->type) &&
            ((FSP_SUCCESS == p_event_info->status) || (FSP_ERR_USB_SIZE_SHORT == p_event_info->status))) {
            UW length = (p_event_info->data_size > sizeof(ai_lab_rx_buffer)) ?
                        sizeof(ai_lab_rx_buffer) : p_event_info->data_size;
            for (UW index = 0U; index < length; index++) {
                acoustic_frame_t frame;
                acoustic_parse_result_t parse = acoustic_protocol_parser_push(&ai_lab_parser,
                                                                                ai_lab_rx_buffer[index], &frame);
                if (ACOUSTIC_PARSE_FRAME_READY == parse) {
                    acoustic_ai_lab_command_handle(&frame);
                } else if ((ACOUSTIC_PARSE_CRC_ERROR == parse) || (ACOUSTIC_PARSE_FORMAT_ERROR == parse) ||
                           (ACOUSTIC_PARSE_UNSUPPORTED_VERSION == parse)) {
                    g_acoustic_ai_lab_error_count++;
                }
            }
        } else {
            g_acoustic_ai_lab_last_error = p_event_info->status;
            g_acoustic_ai_lab_error_count++;
        }
        acoustic_ai_lab_read_start();
    } else if (USB_STATUS_WRITE_COMPLETE == event) {
        ai_lab_write_pending = FALSE;
    } else if (USB_STATUS_REQUEST == event) {
        UH request = (UH) (p_event_info->setup.request_type & USB_BREQUEST);
        if (USB_PCDC_SET_LINE_CODING == request) {
            (void) g_usb_on_usb.periControlDataGet((usb_ctrl_t *) p_event_info,
                                                   (UB *) &ai_lab_line_coding, 7U);
        } else if (USB_PCDC_GET_LINE_CODING == request) {
            (void) g_usb_on_usb.periControlDataSet((usb_ctrl_t *) p_event_info,
                                                   (UB *) &ai_lab_line_coding, 7U);
        } else {
            (void) g_usb_on_usb.periControlStatusSet((usb_ctrl_t *) p_event_info, USB_SETUP_STATUS_ACK);
        }
    } else if (USB_STATUS_RESUME == event) {
        /* USB suspend後も選択済みconfigurationを保持し、復帰時に状態を再送する。 */
        if (g_acoustic_ai_lab_usb_configured) {
            ai_lab_snapshot_requested = TRUE;
            acoustic_ai_lab_read_start();
        }
    } else if (USB_STATUS_DETACH == event) {
        g_acoustic_ai_lab_usb_configured = FALSE;
        ai_lab_read_pending = FALSE;
        ai_lab_write_pending = FALSE;
    }
    (void) now_ms;
}

/** =================================================================*
 * @brief 未送信の操作結果・snapshot・特徴量・保存見本を優先順で送出
 * @param[in] now_ms 現在時刻[ms]
 * ================================================================= */
EXPORT void acoustic_ai_lab_link_poll(UW now_ms) {
    if (!g_acoustic_ai_lab_usb_open || !g_acoustic_ai_lab_usb_configured || ai_lab_write_pending) {
        return;
    }
    acoustic_ai_lab_read_start();
    if (ai_lab_command_result_pending) {
        UB payload[ACOUSTIC_AI_LAB_COMMAND_RESULT_PAYLOAD_SIZE] = {
            ai_lab_command_result_command, ai_lab_command_result_status,
            g_task_think_learning_mode ? 1U : 0U, g_task_think_learning_samples,
        };
        UW length = (UW) acoustic_protocol_encode(ACOUSTIC_MESSAGE_AI_LAB_COMMAND_RESULT, ai_lab_tx_sequence,
                                                  now_ms, payload, sizeof(payload), ai_lab_tx_buffer,
                                                  sizeof(ai_lab_tx_buffer));
        if ((0U != length) && acoustic_ai_lab_write(ai_lab_tx_buffer, (UW) length)) {
            ai_lab_tx_sequence++;
            ai_lab_command_result_pending = FALSE;
        }
        return;
    }
    if (ai_lab_snapshot_requested || ((now_ms - ai_lab_last_snapshot_ms) >= AI_LAB_SNAPSHOT_PERIOD_MS)) {
        acoustic_ai_lab_snapshot_queue(now_ms);
        return;
    }

    acoustic_ai_lab_summary_queue();
    if (ai_lab_summary_pending) {
        UB payload[ACOUSTIC_AI_LAB_SUMMARY_CHUNK_PAYLOAD_SIZE] = {0};
        payload[0] = (UB) ai_lab_summary_generation;
        payload[1] = (UB) (ai_lab_summary_generation >> 8U);
        payload[2] = (UB) (ai_lab_summary_generation >> 16U);
        payload[3] = (UB) (ai_lab_summary_generation >> 24U);
        payload[4] = ai_lab_summary_chunk_index;
        payload[5] = AI_LAB_SUMMARY_CHUNK_COUNT;
        payload[6] = 1U;
        payload[7] = 0U;
        memcpy(&payload[8], &ai_lab_summary[ai_lab_summary_chunk_index * ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE],
               ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE);
        UW length = (UW) acoustic_protocol_encode(ACOUSTIC_MESSAGE_AI_LAB_SUMMARY_CHUNK, ai_lab_tx_sequence,
                                                  now_ms, payload, sizeof(payload), ai_lab_tx_buffer,
                                                  sizeof(ai_lab_tx_buffer));
        if ((0U != length) && acoustic_ai_lab_write(ai_lab_tx_buffer, (UW) length)) {
            ai_lab_tx_sequence++;
            ai_lab_summary_chunk_index++;
            if (AI_LAB_SUMMARY_CHUNK_COUNT == ai_lab_summary_chunk_index) {
                ai_lab_summary_pending = FALSE;
            }
        }
        return;
    }

    if (ai_lab_profile_pending) {
        UB payload[ACOUSTIC_AI_LAB_PROFILE_CHUNK_PAYLOAD_SIZE] = {0};
        payload[0] = (UB) ai_lab_profile_data.generation;
        payload[1] = (UB) (ai_lab_profile_data.generation >> 8U);
        payload[2] = (UB) (ai_lab_profile_data.generation >> 16U);
        payload[3] = (UB) (ai_lab_profile_data.generation >> 24U);
        payload[4] = ai_lab_profile_sample_index;
        payload[5] = ai_lab_profile_chunk_index;
        payload[6] = AI_LAB_PROFILE_CHUNK_COUNT;
        payload[7] = ai_lab_profile_data.sample_count;
        memcpy(&payload[8],
               &ai_lab_profile_data.samples[ai_lab_profile_sample_index]
                                           [ai_lab_profile_chunk_index * ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE],
               ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE);
        UW length = (UW) acoustic_protocol_encode(ACOUSTIC_MESSAGE_AI_LAB_PROFILE_CHUNK, ai_lab_tx_sequence,
                                                  now_ms, payload, sizeof(payload), ai_lab_tx_buffer,
                                                  sizeof(ai_lab_tx_buffer));
        if ((0U != length) && acoustic_ai_lab_write(ai_lab_tx_buffer, (UW) length)) {
            ai_lab_tx_sequence++;
            ai_lab_profile_chunk_index++;
            if (AI_LAB_PROFILE_CHUNK_COUNT == ai_lab_profile_chunk_index) {
                ai_lab_profile_chunk_index = 0U;
                ai_lab_profile_sample_index++;
                if (ai_lab_profile_sample_index >= ai_lab_profile_data.sample_count) {
                    ai_lab_profile_pending = FALSE;
                }
            }
        }
    }
}
