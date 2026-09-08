/** =================================================================*
 * @file   tca9548a.c
 * @brief  TCA9548A I2Cマルチプレクサ実装
 * ================================================================= */
#include "tca9548a.h"                                      /* TCA9548A API */
#include "platform/i2c_bus.h"                              /* I2CバスAPI */
#include "config/sensor_config.h"                          /* TCA9548Aアドレス */

/** =================================================================*
 * @brief  指定チャネルだけをI2Cバスへ排他的に接続
 * @param[in] channel TCA9548Aチャネル番号（0～7）
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t tca9548a_select_channel(UB channel) {
    if (channel >= 8U) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    /* 常にone-hotを書き込み、以前の選択や他チャネルを同時に残さない。 */
    UB const selection = (UB) (1U << channel);
    return i2c_bus_write(CPU0_TCA9548A_ADDRESS, &selection, 1U);
}

/** =================================================================*
 * @brief  全TCA9548AチャネルをI2Cバスから切り離す
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t tca9548a_disable_all(void) {
    UB const selection = 0U;
    return i2c_bus_write(CPU0_TCA9548A_ADDRESS, &selection, 1U);
}
