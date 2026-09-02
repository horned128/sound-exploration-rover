/** =================================================================*
 * @file   hal_entry.c
 * @brief  CPU1のμT-Kernel起動入口
 * ================================================================= */
#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンスと型定義 */
#include <tk/tkernel.h>                                      /* μT-Kernelの公開範囲マクロ */

IMPORT void knl_start_mtkernel(void);                       /* μT-Kernel起動関数 */

/** =================================================================*
 * @brief  μT-Kernel起動
 * ================================================================= */
EXPORT void hal_entry(void) {
    knl_start_mtkernel();
}
