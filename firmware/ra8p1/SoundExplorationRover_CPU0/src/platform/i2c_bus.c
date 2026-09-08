/** =================================================================*
 * @file   i2c_bus.c
 * @brief  CPU0センサー用I2C1バス実装
 * ================================================================= */
#include "platform/i2c_bus.h"                              /* I2CバスAPI */
#include "config/sensor_config.h"                          /* I2C有効化、転送timeout設定 */

#if (CPU0_SENSOR_I2C_ENABLED != 0U)
#include "r_iic_master.h"                                   /* FSP IIC Master直接API */

LOCAL volatile BOOL sensor_i2c_transfer_done;               /**< ISRから通知される転送完了 */
LOCAL volatile i2c_master_event_t sensor_i2c_transfer_event; /**< 最終I2Cイベント */
LOCAL BOOL i2c_bus_open;                             /**< I2C1 open状態 */

LOCAL fsp_err_t i2c_bus_wait(i2c_master_event_t expected_event); /* 転送完了待ち */
LOCAL fsp_err_t i2c_bus_address_set(UB address);    /* 7bitスレーブアドレス設定 */

/** =================================================================*
 * @brief  FSP I2C転送完了コールバック
 * @param[in] p_args FSPから通知されるイベント
 * ================================================================= */
EXPORT void i2c_bus_callback(i2c_master_callback_args_t * p_args) {
    if (NULL != p_args) {
        sensor_i2c_transfer_event = p_args->event;
        if ((I2C_MASTER_EVENT_TX_COMPLETE == p_args->event) || (I2C_MASTER_EVENT_RX_COMPLETE == p_args->event) ||
            (I2C_MASTER_EVENT_ABORTED == p_args->event)) {
            sensor_i2c_transfer_done = TRUE;
        }
    }
}

/** =================================================================*
 * @brief  I2C転送の完了または異常を待機
 * @param[in] expected_event 正常完了として期待するイベント
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t i2c_bus_wait(i2c_master_event_t expected_event) {
    for (UW elapsed_ms = 0U; elapsed_ms < CPU0_SENSOR_I2C_TIMEOUT_MS; elapsed_ms++) {
        if (sensor_i2c_transfer_done) {
            return (expected_event == sensor_i2c_transfer_event) ? FSP_SUCCESS : FSP_ERR_ABORTED;
        }
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
    }

    (void) R_IIC_MASTER_Abort(&g_i2c_sensor_ctrl);
    return FSP_ERR_TIMEOUT;
}

/** =================================================================*
 * @brief  I2C1のスレーブアドレスを設定
 * @param[in] address 7bit I2Cアドレス
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t i2c_bus_address_set(UB address) {
    if (!i2c_bus_open) {
        return FSP_ERR_NOT_OPEN;
    }

    return R_IIC_MASTER_SlaveAddressSet(&g_i2c_sensor_ctrl, address, I2C_MASTER_ADDR_MODE_7BIT);
}

/** =================================================================*
 * @brief  I2C1をopenし、転送完了コールバックを登録
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t i2c_bus_init(void) {
    if (i2c_bus_open) {
        return FSP_SUCCESS;
    }

    fsp_err_t err = R_IIC_MASTER_Open(&g_i2c_sensor_ctrl, &g_i2c_sensor_cfg);
    if (FSP_SUCCESS != err) {
        return err;
    }

    err = R_IIC_MASTER_CallbackSet(&g_i2c_sensor_ctrl, i2c_bus_callback, NULL, NULL);
    if (FSP_SUCCESS != err) {
        (void) R_IIC_MASTER_Close(&g_i2c_sensor_ctrl);
        return err;
    }

    sensor_i2c_transfer_done = FALSE;
    sensor_i2c_transfer_event = I2C_MASTER_EVENT_ABORTED;
    i2c_bus_open = TRUE;
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  I2C1をclose
 * ================================================================= */
EXPORT void i2c_bus_deinit(void) {
    if (i2c_bus_open) {
        (void) R_IIC_MASTER_Close(&g_i2c_sensor_ctrl);
        i2c_bus_open = FALSE;
    }
}

/** =================================================================*
 * @brief  STOP付きI2C書込み
 * @param[in] address 7bit I2Cアドレス
 * @param[in] p_data 書込みデータ
 * @param[in] length 書込み長[byte]
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t i2c_bus_write(UB address, const UB * p_data, UW length) {
    if ((NULL == p_data) || (0U == length)) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    fsp_err_t err = i2c_bus_address_set(address);
    if (FSP_SUCCESS != err) {
        return err;
    }

    sensor_i2c_transfer_done = FALSE;
    sensor_i2c_transfer_event = I2C_MASTER_EVENT_ABORTED;
    err = R_IIC_MASTER_Write(&g_i2c_sensor_ctrl, (UB *) p_data, length, false);
    return (FSP_SUCCESS == err) ? i2c_bus_wait(I2C_MASTER_EVENT_TX_COMPLETE) : err;
}

/** =================================================================*
 * @brief  STOP付きI2C読出し
 * @param[in] address 7bit I2Cアドレス
 * @param[out] p_data 読出し先
 * @param[in] length 読出し長[byte]
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t i2c_bus_read(UB address, UB * p_data, UW length) {
    if ((NULL == p_data) || (0U == length)) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    fsp_err_t err = i2c_bus_address_set(address);
    if (FSP_SUCCESS != err) {
        return err;
    }

    sensor_i2c_transfer_done = FALSE;
    sensor_i2c_transfer_event = I2C_MASTER_EVENT_ABORTED;
    err = R_IIC_MASTER_Read(&g_i2c_sensor_ctrl, p_data, length, false);
    return (FSP_SUCCESS == err) ? i2c_bus_wait(I2C_MASTER_EVENT_RX_COMPLETE) : err;
}

/** =================================================================*
 * @brief  ReSTART付きI2C書込み・読出し
 * @param[in] address 7bit I2Cアドレス
 * @param[in] p_write 書込みデータ
 * @param[in] write_length 書込み長[byte]
 * @param[out] p_read 読出し先
 * @param[in] read_length 読出し長[byte]
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t i2c_bus_write_read(UB address, const UB * p_write, UW write_length, UB * p_read,
                                           UW read_length) {
    if ((NULL == p_write) || (0U == write_length) || (NULL == p_read) || (0U == read_length)) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    fsp_err_t err = i2c_bus_address_set(address);
    if (FSP_SUCCESS != err) {
        return err;
    }

    sensor_i2c_transfer_done = FALSE;
    sensor_i2c_transfer_event = I2C_MASTER_EVENT_ABORTED;
    err = R_IIC_MASTER_Write(&g_i2c_sensor_ctrl, (UB *) p_write, write_length, true);
    if (FSP_SUCCESS != err) {
        return err;
    }
    err = i2c_bus_wait(I2C_MASTER_EVENT_TX_COMPLETE);
    if (FSP_SUCCESS != err) {
        return err;
    }

    sensor_i2c_transfer_done = FALSE;
    sensor_i2c_transfer_event = I2C_MASTER_EVENT_ABORTED;
    err = R_IIC_MASTER_Read(&g_i2c_sensor_ctrl, p_read, read_length, false);
    return (FSP_SUCCESS == err) ? i2c_bus_wait(I2C_MASTER_EVENT_RX_COMPLETE) : err;
}

#else

/** =================================================================*
 * @brief  無効化されたI2Cバス初期化
 * @return FSP_ERR_NOT_OPEN
 * ================================================================= */
EXPORT fsp_err_t i2c_bus_init(void) {
    return FSP_ERR_NOT_OPEN;
}

/** =================================================================*
 * @brief  無効化されたI2Cバス終了
 * ================================================================= */
EXPORT void i2c_bus_deinit(void) {
}

/** =================================================================*
 * @brief  無効化されたI2Cバス書込み
 * @param[in] address 7bit I2Cアドレス
 * @param[in] p_data 書込みデータ
 * @param[in] length 書込み長[byte]
 * @return FSP_ERR_NOT_OPEN
 * ================================================================= */
EXPORT fsp_err_t i2c_bus_write(UB address, const UB * p_data, UW length) {
    (void) address;
    (void) p_data;
    (void) length;
    return FSP_ERR_NOT_OPEN;
}

/** =================================================================*
 * @brief  無効化されたI2Cバス読出し
 * @param[in] address 7bit I2Cアドレス
 * @param[out] p_data 読出し先
 * @param[in] length 読出し長[byte]
 * @return FSP_ERR_NOT_OPEN
 * ================================================================= */
EXPORT fsp_err_t i2c_bus_read(UB address, UB * p_data, UW length) {
    (void) address;
    (void) p_data;
    (void) length;
    return FSP_ERR_NOT_OPEN;
}

/** =================================================================*
 * @brief  無効化されたI2Cバス書込み・読出し
 * @param[in] address 7bit I2Cアドレス
 * @param[in] p_write 書込みデータ
 * @param[in] write_length 書込み長[byte]
 * @param[out] p_read 読出し先
 * @param[in] read_length 読出し長[byte]
 * @return FSP_ERR_NOT_OPEN
 * ================================================================= */
EXPORT fsp_err_t i2c_bus_write_read(UB address, const UB * p_write, UW write_length, UB * p_read,
                                           UW read_length) {
    (void) address;
    (void) p_write;
    (void) write_length;
    (void) p_read;
    (void) read_length;
    return FSP_ERR_NOT_OPEN;
}

#endif /* CPU0_SENSOR_I2C_ENABLED */
