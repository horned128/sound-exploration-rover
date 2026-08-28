/** =================================================================*
 * @file   tk_init.c
 * @brief  CPU0独立タスクの初期化
 * ================================================================= */
#include "tk_init.h"                                      /* CPU0タスク初期化API */
#include "tk_audio.h"                                     /* 音響タスク生成・開始API */
#include "tk_command.h"                                   /* 指令タスク生成・開始API */
#include "tk_think.h"                                     /* 思考タスク生成・開始API */
#include "../../cpu0_config.h"                            /* 初期化タスク優先度、スタック */

static void cpu0_init_task(INT stacd, void * exinf);       /* 初期化タスク本体 */

static T_CTSK const init_task_config = {
    .exinf = NULL,
    .tskatr = TA_HLNG | TA_RNG3,
    .task = (FP) cpu0_init_task,
    .itskpri = CPU0_INIT_TASK_PRIORITY,
    .stksz = CPU0_INIT_TASK_STACK_SIZE,
    .bufptr = NULL,
};

static ID init_task_id;                                    /**< 初期化タスクID */

/** =================================================================*
 * @brief  CPU0初期化タスク生成・開始
 * @details tk_init自身を独立タスクとして生成し、子タスク初期化を委譲する。
 * @return CPU0異常コード
 * ================================================================= */
cpu0_fault_t cpu0_tasks_init(void) {
    init_task_id = tk_cre_tsk(&init_task_config);
    if (init_task_id <= 0) {
        init_task_id = 0;
        return CPU0_FAULT_TASK_CREATE;
    }

    ER const err = tk_sta_tsk(init_task_id, 0);
    if (E_OK != err) {
        (void) tk_del_tsk(init_task_id);
        init_task_id = 0;
        return CPU0_FAULT_TASK_START;
    }

    return CPU0_FAULT_NONE;
}

/** =================================================================*
 * @brief  CPU0初期化タスク本体
 * @details 全共有資源と子タスクを生成後、指令、音響、思考の順で開始し自己削除する。
 * ================================================================= */
static void cpu0_init_task(INT stacd, void * exinf) {
    (void) stacd;
    (void) exinf;

    cpu0_fault_t fault = cpu0_think_task_create();
    if (CPU0_FAULT_NONE != fault) {
        cpu0_think_halt(fault);
    }

    fault = cpu0_command_task_create();
    if (CPU0_FAULT_NONE != fault) {
        cpu0_think_task_delete();
        cpu0_think_halt(fault);
    }

    fault = cpu0_audio_task_create();
    if (CPU0_FAULT_NONE != fault) {
        cpu0_command_task_delete();
        cpu0_think_task_delete();
        cpu0_think_halt(fault);
    }

    fault = cpu0_command_task_start();
    if (CPU0_FAULT_NONE != fault) {
        cpu0_audio_task_delete();
        cpu0_command_task_delete();
        cpu0_think_task_delete();
        cpu0_think_halt(fault);
    }

    fault = cpu0_audio_task_start();
    if (CPU0_FAULT_NONE != fault) {
        cpu0_audio_task_delete();
        cpu0_command_task_delete();
        cpu0_think_task_delete();
        cpu0_think_halt(fault);
    }

    fault = cpu0_think_task_start();
    if (CPU0_FAULT_NONE != fault) {
        cpu0_audio_task_delete();
        cpu0_command_task_delete();
        cpu0_think_task_delete();
        cpu0_think_halt(fault);
    }

    init_task_id = 0;
    tk_exd_tsk();
}
