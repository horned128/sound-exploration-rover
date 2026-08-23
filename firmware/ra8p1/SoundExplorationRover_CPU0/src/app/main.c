/** =================================================================*
 * @file   main.c
 * @brief  RA8P1 Cortex-M85向けμT-Kernel 3.0アプリケーション入口
 * @author hino.a
 * @date   2026-08
 * ================================================================= */
#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンス、周辺機器設定、型定義 */
#include <tk/tkernel.h>                                     /* μT-Kernelのタスク休止API、型定義、共通定義 */
#include "tasks/tk_init.h"                                  /* CPU0独立タスクの初期化API */
#include "tasks/tk_think.h"                                 /* CPU0起動異常のLED表示API */

EXPORT INT usermain(void);                                  /* CPU0アプリケーション起動 */

/** =================================================================*
 * @brief  CPU0アプリケーション起動
 * @details CPU1を起動してCPU0タスク群を開始し、初期タスクを永久休止させる。
 * @return μT-Kernelへ返す終了コード（通常は到達しない）。
 * ================================================================= */
EXPORT INT usermain(void) {
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD
    /* CPU1（セカンダリコア）を起動する。 */
    R_BSP_SecondaryCoreStart();
#endif

    cpu0_fault_t const fault = cpu0_tasks_init();           /* CPU0独立タスクの起動結果 */
    if (CPU0_FAULT_NONE != fault) {
        cpu0_think_halt(fault);
    }

    while (1) {
        (void) tk_slp_tsk(TMO_FEVR);
    }

    return 0;
}
