/** =================================================================*
 * @file   tca9548a.h
 * @brief  TCA9548A I2CマルチプレクサAPI
 * ================================================================= */
#ifndef SEROV_CPU0_DRIVER_TCA9548A_H
#define SEROV_CPU0_DRIVER_TCA9548A_H

#include "hal_data.h"                                       /* FSPエラー型 */
#include <tk/tkernel.h>                                     /* μT-Kernel型 */

EXPORT fsp_err_t tca9548a_select_channel(UB channel);       /* 排他選択: 指定チャネルのみ接続 */
EXPORT fsp_err_t tca9548a_disable_all(void);                /* 全チャネル切離し */

#endif /* SEROV_CPU0_DRIVER_TCA9548A_H */
