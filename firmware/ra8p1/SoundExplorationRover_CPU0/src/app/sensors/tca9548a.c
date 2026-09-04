/** =================================================================*
 * @file   tca9548a.c
 * @brief  TCA9548A I2Cマルチプレクサ実装
 * ================================================================= */
#include "tca9548a.h"                                      /* TCA9548A API */
#include "sensor_i2c_bus.h"                                /* I2CバスAPI */
#include "../../cpu0_config.h"                              /* TCA9548Aアドレス */

/** =================================================================*
 * @brief  指定チャネルだけをI2Cバスへ接続
 * @param[in] channel TCA9548Aチャネル番号（0～7）
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t tca9548a_select_channel(UB channel) {
    if (channel >= 8U) {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    UB const selection = (UB) (1U << channel);
    return sensor_i2c_bus_write(CPU0_TCA9548A_ADDRESS, &selection, 1U);
}

/** =================================================================*
 * @brief  全TCA9548AチャネルをI2Cバスから切り離す
 * @return FSPエラーコード
 * ================================================================= */
EXPORT fsp_err_t tca9548a_disable_all(void) {
    UB const selection = 0U;
    return sensor_i2c_bus_write(CPU0_TCA9548A_ADDRESS, &selection, 1U);
}
