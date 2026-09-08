/** =================================================================*
 * @file   main.c
 * @brief  RA8P1 Cortex-M33向けμT-Kernel 3.0アプリケーション入口
 * ================================================================= */
#include "tasks/task_registry.h"                                  /* CPU1独立タスクの初期化API */
#include "tasks/task_status.h"                                /* CPU1起動異常のLED表示API */
#include <tk/tkernel.h>                                      /* μT-Kernelのタスク休止API、型定義 */

EXPORT INT usermain(void);                                  /* CPU1アプリケーション起動 */

/** =================================================================*
 * @brief  CPU1アプリケーション起動
 * @details μT-Kernel初期タスク上でCPU1タスク群を生成・開始して永久休止する。
 * @return μT-Kernelへ返す終了コード（通常は到達しない）。
 * ================================================================= */
EXPORT INT usermain(void) {
    app_fault_t const fault = task_registry_init();
    if (APP_FAULT_NONE != fault) {
        task_status_halt(fault);
    }

    while (1) {
        (void) tk_slp_tsk(TMO_FEVR);
    }

    return 0;
}
