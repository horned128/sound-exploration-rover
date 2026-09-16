/** =================================================================*
 * @file   main.c
 * @brief  RA8P1 Cortex-M85向けμT-Kernel 3.0アプリケーション入口
 * @author hino.a
 * @date   2026-08
 * ================================================================= */
#include "hal_data.h"                                       /* FSP生成のHAL/BSPインスタンス、周辺機器設定、型定義 */
#include "tasks/task_infer.h"                               /* 非致命の音響推論タスク起動API */
#include "tasks/task_registry.h"                            /* CPU0独立タスクの初期化API */
#include "tasks/task_think.h"                               /* CPU0起動異常のLED表示API */
#include <tk/tkernel.h>                                     /* μT-Kernelのタスク休止API、型定義、共通定義 */

EXPORT INT usermain(void);                                  /* CPU0アプリケーション起動 */

/** =================================================================*
 * @brief  CPU0アプリケーション起動
 * @details CPU1を起動し、μT-Kernel初期タスク上でCPU0タスク群を生成・開始して永久休止する。
 * @return μT-Kernelへ返す終了コード（通常は到達しない）。
 * ================================================================= */
EXPORT INT usermain(void) {
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD
    /* CPU1（セカンダリコア）を起動する。 */
    R_BSP_SecondaryCoreStart();
#endif

    /* 推論資源の不足で、既存の安全・走行タスク群を止めない。 */
    task_infer_start_optional();

    /* CPU0独立タスクの起動結果 */
    app_fault_t const fault = task_registry_init();
    if (APP_FAULT_NONE != fault) {
        task_infer_stop();
        task_think_halt(fault);
    }

    while (1) {
        (void) tk_slp_tsk(TMO_FEVR);
    }

    return 0;
}
