/** =================================================================*
 * @file   hal_entry.c
 * @brief  CPU0のμT-Kernel起動入口
 * ================================================================= */
#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンス、周辺機器設定、型定義 */
#include <tk/tkernel.h>                                     /* μT-Kernel基本型と公開範囲マクロ */

IMPORT void knl_start_mtkernel(void);                       /* μT-Kernel起動関数 */

/** =================================================================*
 * @brief  μT-Kernel起動
 * ================================================================= */
EXPORT void hal_entry(void) {
    knl_start_mtkernel();
}
