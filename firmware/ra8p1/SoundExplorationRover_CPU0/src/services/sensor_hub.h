/** =================================================================*
 * @file   sensor_hub.h
 * @brief  ToF 3台とBMI270をまとめるCPU0センサー層API
 * ================================================================= */
#ifndef SEROV_CPU0_SERVICE_SENSOR_HUB_H
#define SEROV_CPU0_SERVICE_SENSOR_HUB_H

#include "hal_data.h"                                       /* FSPエラー型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

#define CPU0_SENSOR_TOF_COUNT              (3U)             /**< ToFセンサー接続数（左・中央・右） */
#define CPU0_SENSOR_FAILURE_CHANNEL_NONE   (-1)             /**< センサー異常の対象チャネル未特定値 */

/* VL53L1Xドライバを公開せず、サービス層の診断値として保持する。 */
/**< ToFドライバの測距取得結果 */
typedef enum e_sensor_tof_result {
    SENSOR_TOF_RESULT_VALID = 0U,                           /**< 測距値とレンジ状態が有効 */
    SENSOR_TOF_RESULT_RANGE_STATUS_INVALID,                 /**< レンジステータスが異常 */
    SENSOR_TOF_RESULT_DISTANCE_INVALID,                     /**< 距離値が異常 */
    SENSOR_TOF_RESULT_DATA_READY_TIMEOUT,                   /**< データ準備待ちタイムアウト */
    SENSOR_TOF_RESULT_TRANSPORT_ERROR,                      /**< 通信転送エラー */
} sensor_tof_result_t;

#define SENSOR_TOF_RANGE_STATUS_UNAVAILABLE (0xFFU)         /**< ToF測距ステータス未取得値 */

/**< 車体に対するToFセンサー位置 */
typedef enum e_tof_position {
    CPU0_TOF_LEFT = 0U,                                     /**< 車体左側のToF */
    CPU0_TOF_CENTER,                                        /**< 車体中央のToF */
    CPU0_TOF_RIGHT,                                         /**< 車体右側のToF */
} tof_position_t;

#define CPU0_SENSOR_VALID_TOF_LEFT         (1U << 0)        /**< 左ToF測距値の有効ビット */
#define CPU0_SENSOR_VALID_TOF_CENTER       (1U << 1)        /**< 中央ToF測距値の有効ビット */
#define CPU0_SENSOR_VALID_TOF_RIGHT        (1U << 2)        /**< 右ToF測距値の有効ビット */
#define CPU0_SENSOR_VALID_IMU              (1U << 3)        /**< IMUデータの有効ビット */
#define CPU0_SENSOR_VALID_ALL                /**< 3眼ToFとIMUの全有効ビット */ \
    (CPU0_SENSOR_VALID_TOF_LEFT | CPU0_SENSOR_VALID_TOF_CENTER | \
     CPU0_SENSOR_VALID_TOF_RIGHT | CPU0_SENSOR_VALID_IMU)

#define CPU0_SENSOR_ERROR_I2C_INIT         (1U << 0)        /**< センサーI2C初期化異常ビット */
#define CPU0_SENSOR_ERROR_TCA9548A         (1U << 1)        /**< TCA9548A選択異常ビット */
#define CPU0_SENSOR_ERROR_TOF_LEFT         (1U << 2)        /**< 左ToF測距異常ビット */
#define CPU0_SENSOR_ERROR_TOF_CENTER       (1U << 3)        /**< 中央ToF測距異常ビット */
#define CPU0_SENSOR_ERROR_TOF_RIGHT        (1U << 4)        /**< 右ToF測距異常ビット */
#define CPU0_SENSOR_ERROR_BMI270           (1U << 5)        /**< BMI270取得異常ビット */
#define CPU0_SENSOR_ERROR_STALE            (1U << 6)        /**< センサーデータ停滞異常ビット */

/**< センサー異常の分類 */
typedef enum e_sensor_failure_kind {
    CPU0_SENSOR_FAILURE_NONE = 0U,                          /**< センサー異常なし */
    CPU0_SENSOR_FAILURE_MEASUREMENT_INVALID,                /**< 測定値が無効 */
    CPU0_SENSOR_FAILURE_I2C_TRANSFER_TIMEOUT,               /**< I2C転送がタイムアウト */
    CPU0_SENSOR_FAILURE_VL53L1X_DATA_READY_TIMEOUT,         /**< VL53L1Xデータ待ちがタイムアウト */
    CPU0_SENSOR_FAILURE_TRANSPORT,                          /**< 通信転送が異常 */
} sensor_failure_kind_t;

/**< センサー異常の対象デバイス */
typedef enum e_sensor_failure_device {
    CPU0_SENSOR_FAILURE_DEVICE_NONE = 0U,                   /**< 対象デバイスなし */
    CPU0_SENSOR_FAILURE_DEVICE_I2C_BUS,                     /**< I2Cバス */
    CPU0_SENSOR_FAILURE_DEVICE_TCA9548A,                    /**< TCA9548A */
    CPU0_SENSOR_FAILURE_DEVICE_TOF_LEFT,                    /**< 左ToF */
    CPU0_SENSOR_FAILURE_DEVICE_TOF_CENTER,                  /**< 中央ToF */
    CPU0_SENSOR_FAILURE_DEVICE_TOF_RIGHT,                   /**< 右ToF */
    CPU0_SENSOR_FAILURE_DEVICE_BMI270,                      /**< BMI270 */
} sensor_failure_device_t;

/**< センサー処理の異常発生段階 */
typedef enum e_sensor_failure_stage {
    CPU0_SENSOR_FAILURE_STAGE_NONE = 0U,                    /**< 異常段階なし */
    CPU0_SENSOR_FAILURE_STAGE_I2C_OPEN,                     /**< I2Cオープン */
    CPU0_SENSOR_FAILURE_STAGE_TCA_DISABLE_ALL,              /**< TCA全チャネル切離し */
    CPU0_SENSOR_FAILURE_STAGE_TOF_LEFT_TCA_SELECT,          /**< 左ToF用TCAチャネル選択 */
    CPU0_SENSOR_FAILURE_STAGE_TOF_LEFT_INIT,                /**< 左ToF初期化 */
    CPU0_SENSOR_FAILURE_STAGE_TOF_CENTER_TCA_SELECT,        /**< 中央ToF用TCAチャネル選択 */
    CPU0_SENSOR_FAILURE_STAGE_TOF_CENTER_INIT,              /**< 中央ToF初期化 */
    CPU0_SENSOR_FAILURE_STAGE_TOF_RIGHT_TCA_SELECT,         /**< 右ToF用TCAチャネル選択 */
    CPU0_SENSOR_FAILURE_STAGE_TOF_RIGHT_INIT,               /**< 右ToF初期化 */
    CPU0_SENSOR_FAILURE_STAGE_BMI270_TCA_SELECT,            /**< BMI270用TCAチャネル選択 */
    CPU0_SENSOR_FAILURE_STAGE_BMI270_INIT,                  /**< BMI270初期化 */
    CPU0_SENSOR_FAILURE_STAGE_TOF_LEFT_READ,                /**< 左ToF読出し */
    CPU0_SENSOR_FAILURE_STAGE_TOF_CENTER_READ,              /**< 中央ToF読出し */
    CPU0_SENSOR_FAILURE_STAGE_TOF_RIGHT_READ,               /**< 右ToF読出し */
    CPU0_SENSOR_FAILURE_STAGE_BMI270_READ,                  /**< BMI270読出し */
} sensor_failure_stage_t;

/**< センサー取得結果と異常発生箇所をまとめた診断情報 */
typedef struct st_sensor_diagnostics {
    UB tof_range_status[CPU0_SENSOR_TOF_COUNT];             /**< 各ToFのレンジステータス */
    UB tof_result[CPU0_SENSOR_TOF_COUNT];                   /**< 各ToFの取得結果 */
    sensor_failure_kind_t failure_kind;                     /**< センサー異常の種別 */
    sensor_failure_device_t failure_device;                 /**< センサー異常の対象デバイス */
    sensor_failure_stage_t failure_stage;                   /**< センサー異常の発生段階 */
    B failure_channel;                                      /**< センサー異常の対象チャネル */
} sensor_diagnostics_t;

/**< 1周期分のToF・IMU値とセンサーサービス状態 */
typedef struct st_sensor_snapshot {
    UH tof_distance_mm[CPU0_SENSOR_TOF_COUNT];              /**< 各ToFの距離[mm] */
    H accel_mg[3];                                          /**< 3軸加速度[mg] */
    H gyro_dps_x10[3];                                      /**< 3軸角速度[0.1dps] */
    UW age_ms;                                              /**< 最新更新からの経過時間[ms] */
    UW update_count;                                        /**< 正常更新回数 */
    UW error_flags;                                         /**< センサー異常ビットマスク */
    W last_error;                                           /**< 最後に発生したエラーコード */
    UB valid_flags;                                         /**< センサー値の有効ビットマスク */
    BOOL initialized;                                       /**< センサー初期化完了状態 */
    sensor_diagnostics_t diagnostics;                       /**< 最新取得の詳細診断情報 */
} sensor_snapshot_t;

EXPORT fsp_err_t sensor_hub_init(void);                     /* I2C、TCA、ToF 3台、BMI270を初期化 */
EXPORT void sensor_hub_deinit(void);                        /* センサーとI2Cを停止 */
EXPORT void sensor_hub_transport_fault_deinit(void);        /* 通信障害後にTCA書込みなしでI2Cを停止 */
EXPORT void sensor_hub_diagnostics_get(sensor_diagnostics_t * p_diagnostics); /* 最終診断を取得 */
EXPORT fsp_err_t sensor_hub_poll(sensor_snapshot_t * p_snapshot); /* 1周期の全センサー状態を更新 */

#endif /* SEROV_CPU0_SERVICE_SENSOR_HUB_H */
