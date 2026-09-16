/** =================================================================*
 * @file   i2c_bus.c
 * @brief  CPU0センサー用I2Cバス実装（高信頼性GPIOドライバ）
 * @details 破損したJ24-10 (P512) を回避し、健全なJ23-8 (P312 / Arduino D7) を
 *          SCLとして使用。SDAはJ24-9 (P511) を継続使用。
 *          疑似オープンドレイン（出力Low / 入力+内部Pull-up High）および
 *          スレーブクロックストレッチングに対応。
 * ================================================================= */
#include "platform/i2c_bus.h"                              /* I2CバスAPI */
#include "config/sensor_config.h"                          /* I2C有効化設定 */

#if (CPU0_SENSOR_I2C_ENABLED != 0U)
#include "r_ioport.h"                                       /* GPIO直接制御API */
#include "common_data.h"                                    /* g_ioport_ctrl */

/* ピン割り当て: J24-10 (P512破損) から J23-8 (P312 / D7) へ移設 */
#define I2C_BUS_SDA_PIN                     BSP_IO_PORT_05_PIN_11  /* Arduino J24-9 */
#define I2C_BUS_SCL_PIN                     BSP_IO_PORT_03_PIN_12  /* Arduino J23-8 (D7) */

/* 疑似オープンドレイン制御用ピン設定 */
#define I2C_PIN_CFG_LOW                     ((uint32_t) IOPORT_CFG_PORT_DIRECTION_OUTPUT | \
                                             (uint32_t) IOPORT_CFG_PORT_OUTPUT_LOW | \
                                             (uint32_t) IOPORT_CFG_DRIVE_MID)

#define I2C_PIN_CFG_HIGH                    ((uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT | \
                                             (uint32_t) IOPORT_CFG_PULLUP_ENABLE)

LOCAL BOOL i2c_bus_open;                             /**< I2Cバスopen状態 */

/** =================================================================*
 * @brief  I2Cクロック半周期ディレイ（約100kHz動作）
 * ================================================================= */
LOCAL inline void i2c_delay(void) {
    R_BSP_SoftwareDelay(5U, BSP_DELAY_UNITS_MICROSECONDS);
}

/** =================================================================*
 * @brief  SDAをLow（出力Low）へドライブ
 * ================================================================= */
LOCAL inline void i2c_sda_low(void) {
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, I2C_BUS_SDA_PIN, I2C_PIN_CFG_LOW);
}

/** =================================================================*
 * @brief  SDAをHigh（入力解放+内部プルアップ）へ解放
 * ================================================================= */
LOCAL inline void i2c_sda_high(void) {
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, I2C_BUS_SDA_PIN, I2C_PIN_CFG_HIGH);
}

/** =================================================================*
 * @brief  SCLをLow（出力Low）へドライブ
 * ================================================================= */
LOCAL inline void i2c_scl_low(void) {
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, I2C_BUS_SCL_PIN, I2C_PIN_CFG_LOW);
}

/** =================================================================*
 * @brief  SCLをHigh（入力解放+内部プルアップ）へ解放し、クロックストレッチを待機
 * @return FSP_SUCCESS または FSP_ERR_TIMEOUT
 * ================================================================= */
LOCAL fsp_err_t i2c_scl_high(void) {
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, I2C_BUS_SCL_PIN, I2C_PIN_CFG_HIGH);

    /* スレーブのクロックストレッチ解除を待機（最大10ms） */
    for (UW timeout = 0U; timeout < 1000U; timeout++) {
        bsp_io_level_t level = BSP_IO_LEVEL_LOW;
        (void) R_IOPORT_PinRead(&g_ioport_ctrl, I2C_BUS_SCL_PIN, &level);
        if (BSP_IO_LEVEL_HIGH == level) {
            return FSP_SUCCESS;
        }
        R_BSP_SoftwareDelay(10U, BSP_DELAY_UNITS_MICROSECONDS);
    }

    return FSP_ERR_TIMEOUT;
}

/** =================================================================*
 * @brief  I2C START条件を生成
 * ================================================================= */
LOCAL fsp_err_t i2c_start(void) {
    i2c_sda_high();
    i2c_delay();
    if (FSP_SUCCESS != i2c_scl_high()) {
        return FSP_ERR_TIMEOUT;
    }
    i2c_delay();
    i2c_sda_low();
    i2c_delay();
    i2c_scl_low();
    i2c_delay();
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  I2C STOP条件を生成
 * ================================================================= */
LOCAL void i2c_stop(void) {
    i2c_sda_low();
    i2c_delay();
    (void) i2c_scl_high();
    i2c_delay();
    i2c_sda_high();
    i2c_delay();
}

/** =================================================================*
 * @brief  1バイト送信し、スレーブのACKを受信
 * @param[in] byte 送信バイト
 * @return FSP_SUCCESS (ACK受信), FSP_ERR_ABORTED (NACK受信), または FSP_ERR_TIMEOUT
 * ================================================================= */
LOCAL fsp_err_t i2c_write_byte(UB byte) {
    for (UW bit = 0U; bit < 8U; bit++) {
        if ((byte & 0x80U) != 0U) {
            i2c_sda_high();
        } else {
            i2c_sda_low();
        }
        byte <<= 1U;
        i2c_delay();
        if (FSP_SUCCESS != i2c_scl_high()) {
            return FSP_ERR_TIMEOUT;
        }
        i2c_delay();
        i2c_scl_low();
    }

    /* 9クロック目: スレーブACK検出 */
    i2c_sda_high(); /* SDA解放 */
    i2c_delay();
    if (FSP_SUCCESS != i2c_scl_high()) {
        return FSP_ERR_TIMEOUT;
    }
    i2c_delay();

    bsp_io_level_t sda_level = BSP_IO_LEVEL_HIGH;
    (void) R_IOPORT_PinRead(&g_ioport_ctrl, I2C_BUS_SDA_PIN, &sda_level);

    i2c_scl_low();
    i2c_delay();

    return (BSP_IO_LEVEL_LOW == sda_level) ? FSP_SUCCESS : FSP_ERR_ABORTED;
}

/** =================================================================*
 * @brief  1バイト受信し、ACKまたはNACKを応答
 * @param[out] p_byte 受信先
 * @param[in]  ack    TRUE: ACK(0)を応答, FALSE: NACK(1)を応答
 * @return FSPエラーコード
 * ================================================================= */
LOCAL fsp_err_t i2c_read_byte(UB * p_byte, BOOL ack) {
    UB val = 0U;
    i2c_sda_high(); /* SDAを入力解放 */

    for (UW bit = 0U; bit < 8U; bit++) {
        i2c_delay();
        if (FSP_SUCCESS != i2c_scl_high()) {
            return FSP_ERR_TIMEOUT;
        }
        i2c_delay();

        bsp_io_level_t sda_level = BSP_IO_LEVEL_LOW;
        (void) R_IOPORT_PinRead(&g_ioport_ctrl, I2C_BUS_SDA_PIN, &sda_level);
        val = (UB) ((val << 1U) | ((BSP_IO_LEVEL_HIGH == sda_level) ? 1U : 0U));

        i2c_scl_low();
    }

    /* 9クロック目: ACK (LOW) または NACK (HIGH) を送出 */
    if (ack) {
        i2c_sda_low();
    } else {
        i2c_sda_high();
    }
    i2c_delay();
    if (FSP_SUCCESS != i2c_scl_high()) {
        return FSP_ERR_TIMEOUT;
    }
    i2c_delay();
    i2c_scl_low();
    i2c_sda_high();
    i2c_delay();

    *p_byte = val;
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  I2Cバスリカバリー（9クロック送出によるスレーブSDAアンロック）
 * ================================================================= */
EXPORT void i2c_bus_clear(void) {
    i2c_sda_high();
    (void) i2c_scl_high();
    i2c_delay();

    bsp_io_level_t sda_level = BSP_IO_LEVEL_HIGH;
    (void) R_IOPORT_PinRead(&g_ioport_ctrl, I2C_BUS_SDA_PIN, &sda_level);

    if (BSP_IO_LEVEL_LOW == sda_level) {
        /* 最大9回のクロックパルスをトグルしてスレーブをアンロック */
        for (UW pulse = 0U; pulse < 9U; pulse++) {
            i2c_scl_low();
            i2c_delay();
            (void) i2c_scl_high();
            i2c_delay();

            (void) R_IOPORT_PinRead(&g_ioport_ctrl, I2C_BUS_SDA_PIN, &sda_level);
            if (BSP_IO_LEVEL_HIGH == sda_level) {
                break;
            }
        }
    }

    i2c_stop();
}

/** =================================================================*
 * @brief  I2C転送完了通知（互換性のためのダミー）
 * ================================================================= */
EXPORT void i2c_bus_callback(i2c_master_callback_args_t * p_args) {
    (void) p_args;
}

/** =================================================================*
 * @brief  I2Cバスを初期化
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t i2c_bus_init(void) {
    if (i2c_bus_open) {
        return FSP_SUCCESS;
    }

    i2c_sda_high();
    (void) i2c_scl_high();
    i2c_bus_clear();

    i2c_bus_open = TRUE;
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  I2Cバスを解放・終了
 * ================================================================= */
EXPORT void i2c_bus_deinit(void) {
    if (i2c_bus_open) {
        i2c_sda_high();
        (void) i2c_scl_high();
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
    if ((NULL == p_data) || (0U == length) || !i2c_bus_open) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    fsp_err_t err = i2c_start();
    if (FSP_SUCCESS != err) {
        i2c_stop();
        return err;
    }

    /* 7bitアドレス + R/W=0 */
    err = i2c_write_byte((UB) (address << 1U));
    if (FSP_SUCCESS == err) {
        for (UW i = 0U; i < length; i++) {
            err = i2c_write_byte(p_data[i]);
            if (FSP_SUCCESS != err) {
                break;
            }
        }
    }

    i2c_stop();
    return err;
}

/** =================================================================*
 * @brief  STOP付きI2C読出し
 * @param[in] address 7bit I2Cアドレス
 * @param[out] p_data 読出し先
 * @param[in] length 読出し長[byte]
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t i2c_bus_read(UB address, UB * p_data, UW length) {
    if ((NULL == p_data) || (0U == length) || !i2c_bus_open) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    fsp_err_t err = i2c_start();
    if (FSP_SUCCESS != err) {
        i2c_stop();
        return err;
    }

    /* 7bitアドレス + R/W=1 */
    err = i2c_write_byte((UB) ((address << 1U) | 1U));
    if (FSP_SUCCESS == err) {
        for (UW i = 0U; i < length; i++) {
            BOOL const send_ack = (i < (length - 1U));
            err = i2c_read_byte(&p_data[i], send_ack);
            if (FSP_SUCCESS != err) {
                break;
            }
        }
    }

    i2c_stop();
    return err;
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
EXPORT fsp_err_t i2c_bus_write_read(UB address, const UB * p_write, UW write_length,
                                    UB * p_read, UW read_length) {
    if ((NULL == p_write) || (0U == write_length) || (NULL == p_read) || (0U == read_length) || !i2c_bus_open) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    fsp_err_t err = i2c_start();
    if (FSP_SUCCESS != err) {
        i2c_stop();
        return err;
    }

    /* 7bitアドレス + R/W=0 */
    err = i2c_write_byte((UB) (address << 1U));
    if (FSP_SUCCESS == err) {
        for (UW i = 0U; i < write_length; i++) {
            err = i2c_write_byte(p_write[i]);
            if (FSP_SUCCESS != err) {
                break;
            }
        }
    }

    if (FSP_SUCCESS != err) {
        i2c_stop();
        return err;
    }

    /* ReSTART 条件 */
    err = i2c_start();
    if (FSP_SUCCESS != err) {
        i2c_stop();
        return err;
    }

    /* 7bitアドレス + R/W=1 */
    err = i2c_write_byte((UB) ((address << 1U) | 1U));
    if (FSP_SUCCESS == err) {
        for (UW i = 0U; i < read_length; i++) {
            BOOL const send_ack = (i < (read_length - 1U));
            err = i2c_read_byte(&p_read[i], send_ack);
            if (FSP_SUCCESS != err) {
                break;
            }
        }
    }

    i2c_stop();
    return err;
}

#else

EXPORT void i2c_bus_clear(void) {
}

EXPORT void i2c_bus_callback(i2c_master_callback_args_t * p_args) {
    (void) p_args;
}

EXPORT fsp_err_t i2c_bus_init(void) {
    return FSP_ERR_NOT_OPEN;
}

EXPORT void i2c_bus_deinit(void) {
}

EXPORT fsp_err_t i2c_bus_write(UB address, const UB * p_data, UW length) {
    (void) address;
    (void) p_data;
    (void) length;
    return FSP_ERR_NOT_OPEN;
}

EXPORT fsp_err_t i2c_bus_read(UB address, UB * p_data, UW length) {
    (void) address;
    (void) p_data;
    (void) length;
    return FSP_ERR_NOT_OPEN;
}

EXPORT fsp_err_t i2c_bus_write_read(UB address, const UB * p_write, UW write_length,
                                    UB * p_read, UW read_length) {
    (void) address;
    (void) p_write;
    (void) write_length;
    (void) p_read;
    (void) read_length;
    return FSP_ERR_NOT_OPEN;
}

#endif /* CPU0_SENSOR_I2C_ENABLED */
