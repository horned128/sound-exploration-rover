/* IPC送信断から安全に自動復帰する実際の指令タスクを模擬実行する。 */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "tasks/task_command.h"
#include "tasks/task_think.h"
#include "ipc/actuator_ipc_client.h"
#include "config/task_config.h"

static void (*task_entry)(INT, void *);
static jmp_buf test_done;
static actuator_status_t peer_status;
static actuator_command_t sent_commands[8];
static actuator_command_t last_successful_command;
static UW send_calls;
static UW successful_sends;
static UW delay_cycles;
static UW fault_reports;
static UW fault_clears;

ID tk_cre_mtx(const T_CMTX * config) {
    assert(config->mtxatr == TA_INHERIT);
    return 1;
}

ID tk_cre_tsk(const T_CTSK * config) {
    task_entry = config->task;
    return 2;
}

ER tk_sta_tsk(ID id, INT code) {
    assert((id == 2) && (code == 0));
    return E_OK;
}

ER tk_ter_tsk(ID id) { assert(id == 2); return E_OK; }
ER tk_del_tsk(ID id) { assert(id == 2); return E_OK; }
ER tk_del_mtx(ID id) { assert(id == 1); return E_OK; }
ER tk_loc_mtx(ID id, INT timeout) { assert((id == 1) && (timeout == TMO_FEVR)); return E_OK; }
ER tk_unl_mtx(ID id) { assert(id == 1); return E_OK; }

ER tk_dly_tsk(INT delay) {
    if (CPU0_ACTUATOR_STARTUP_DELAY_MS == (UW) delay) {
        return E_OK;
    }
    assert(CPU0_COMMAND_PERIOD_MS == (UW) delay);
    delay_cycles++;

    if (successful_sends > 0U) {
        peer_status.sequence_number = (peer_status.sequence_number + 1U) & ACTUATOR_IPC_SEQUENCE_MASK;
        peer_status.applied_command_sequence = last_successful_command.sequence_number;
        peer_status.fault_flags = last_successful_command.emergency_stop
            ? ACTUATOR_FAULT_EMERGENCY_STOP_ACTIVE : ACTUATOR_FAULT_NONE;
    } else {
        peer_status.sequence_number++;
        peer_status.fault_flags = ACTUATOR_FAULT_COMMAND_TIMEOUT;
    }

    if (delay_cycles >= 5U) {
        longjmp(test_done, 1);
    }
    return E_OK;
}

fsp_err_t actuator_ipc_client_init(void) { return FSP_SUCCESS; }
fsp_err_t actuator_ipc_client_deinit(void) { return FSP_SUCCESS; }

BOOL actuator_ipc_client_status_get(actuator_status_t * status) {
    *status = peer_status;
    return TRUE;
}

fsp_err_t actuator_ipc_client_send(const actuator_command_t * command) {
    assert(send_calls < (sizeof(sent_commands) / sizeof(sent_commands[0])));
    sent_commands[send_calls] = *command;
    send_calls++;
    if (1U == send_calls) {
        return 12; /* FSP_ERR_OVERFLOW */
    }
    last_successful_command = *command;
    successful_sends++;
    return FSP_SUCCESS;
}

fsp_err_t actuator_ipc_client_emergency_stop(UW sequence_number) {
    (void) sequence_number;
    return 12; /* FIFO満杯中は緊急停止ワードも送信できない条件 */
}

ER task_think_report_fault(app_fault_t fault) {
    assert(APP_FAULT_IPC_SEND == fault);
    fault_reports++;
    return E_OK;
}

ER task_think_clear_fault(app_fault_t fault) {
    assert(APP_FAULT_IPC_SEND == fault);
    fault_clears++;
    return E_OK;
}

int main(void) {
    memset(&peer_status, 0, sizeof(peer_status));
    assert(APP_FAULT_NONE == task_command_create());
    assert(APP_FAULT_NONE == task_command_start());

    rover_motion_target_t target = {
        .left_target_rpm = 55,
        .right_target_rpm = 55,
        .servo_target_deg = {4, -4, -4, 4},
        .actuator_enable = TRUE,
        .emergency_stop = FALSE,
    };
    assert(E_OK == task_command_set_target(&target));

    if (0 == setjmp(test_done)) {
        task_entry(0, NULL);
    }

    assert(1U == fault_reports);
    assert(1U == fault_clears);
    assert(5U == send_calls);

    /* 送信失敗後は、CPU1が応答しても安全確認完了まで非常停止を送る。 */
    for (UW index = 1U; index <= 2U; index++) {
        assert(0 == sent_commands[index].left_target_rpm);
        assert(0 == sent_commands[index].right_target_rpm);
        assert(0U == sent_commands[index].actuator_enable);
        assert(1U == sent_commands[index].emergency_stop);
        for (UW servo = 0U; servo < ACTUATOR_SERVO_COUNT; servo++) {
            assert(0 == sent_commands[index].servo_target_deg[servo]);
        }
    }

    /* 2状態フレーム確認後、無効指令でCPU1の非常停止ラッチを解除する。 */
    assert(0U == sent_commands[3].actuator_enable);
    assert(0U == sent_commands[3].emergency_stop);

    /* ラッチ解除フレームの次から、最新の通常目標へ復帰する。 */
    assert(1U == sent_commands[4].actuator_enable);
    assert(0U == sent_commands[4].emergency_stop);
    assert(55 == sent_commands[4].left_target_rpm);
    assert(55 == sent_commands[4].right_target_rpm);

    task_command_delete();
    puts("CPU0 command: IPC fault, safe acknowledgements, latch clear and recovery passed");
    return 0;
}
