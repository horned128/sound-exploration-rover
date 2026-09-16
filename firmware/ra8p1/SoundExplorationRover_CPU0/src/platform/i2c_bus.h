/** =================================================================*
 * @file   i2c_bus.h
 * @brief  CPU0センサー用I2CバスAPI
 * ================================================================= */
#ifndef SEROV_CPU0_PLATFORM_I2C_BUS_H
#define SEROV_CPU0_PLATFORM_I2C_BUS_H

#include "hal_data.h"                                       /* FSPエラー型 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

EXPORT void i2c_bus_callback(i2c_master_callback_args_t * p_args); /* I2C転送完了通知 */
EXPORT fsp_err_t i2c_bus_init(void);                 /* I2C1をopenし転送コールバックを登録 */
EXPORT void i2c_bus_deinit(void);                    /* I2C1をclose */
EXPORT void i2c_bus_clear(void);                     /* I2Cバスクリア（9クロック送出） */
EXPORT fsp_err_t i2c_bus_write(UB address, const UB * p_data, UW length); /* STOP付きI2C書込み */
EXPORT fsp_err_t i2c_bus_read(UB address, UB * p_data, UW length); /* STOP付きI2C読出し */
EXPORT fsp_err_t i2c_bus_write_read(UB address, const UB * p_write, UW write_length, UB * p_read,
                                           UW read_length); /* ReSTART付きI2C書込み・読出し */

#endif /* SEROV_CPU0_PLATFORM_I2C_BUS_H */
