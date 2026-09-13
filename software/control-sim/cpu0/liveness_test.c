#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "tasks/task_think.h"
#include "tasks/task_sensor.h"
#include "tasks/task_command.h"
#include "tasks/task_acoustic_link.h"
#include "control/sensor_liveness.h"
#include "config/sensor_config.h"
#include "config/control_config.h"

static void (*entry)(INT,void *);
static jmp_buf done;
static UD now;
static UW count, loops, stage, publications, freeze_publications;
static int mode, failures, recovery_loops;
static rover_motion_target_t target;
bsp_leds_t g_bsp_leds={0,NULL};
void R_BSP_PinAccessEnable(void) {}
void R_BSP_PinAccessDisable(void) {}
void R_BSP_PinWrite(bsp_io_port_pin_t pin,bsp_io_level_t level) { (void)pin;(void)level; }
ID tk_cre_tsk(const T_CTSK *c) { entry=c->task;return 1; }
ID tk_cre_flg(const T_CFLG *c) { (void)c;return 2; }
ER tk_sta_tsk(ID id,INT code) { (void)id;(void)code;return 0; }
ER tk_ter_tsk(ID id) { (void)id;return 0; }
ER tk_del_tsk(ID id) { (void)id;return 0; }
ER tk_del_flg(ID id) { (void)id;return 0; }
ER tk_set_flg(ID id,UINT bits) { (void)id;(void)bits;return 0; }
ER tk_wai_flg(ID id,UINT bits,UINT mode_,UINT *pattern,INT timeout) {
    (void)id;(void)bits;(void)mode_;(void)pattern;assert(timeout==TMO_POL);return E_TMOUT;
}
ER tk_get_otm(SYSTIM *out) {
    if(mode==1 && stage==1) { failures++;return -5; }
    out->hi=(W)(now>>32);out->lo=(UW)now;return 0;
}
ER task_sensor_snapshot_get(sensor_snapshot_t *out) {
    if(stage!=1)count++;
    *out=(sensor_snapshot_t){.initialized=TRUE,.valid_flags=CPU0_SENSOR_VALID_ALL,.update_count=count,.age_ms=0};
    for(unsigned i=0;i<CPU0_SENSOR_TOF_COUNT;i++)out->tof_distance_mm[i]=2000;
    if(mode==2 && stage==1) { failures++;return E_TMOUT; }
    return 0;
}
ER task_acoustic_link_snapshot_get(task_acoustic_link_snapshot_t *out) {
    *out=(task_acoustic_link_snapshot_t){
        .usb_configured=TRUE,.hello_received=TRUE,.observation_received=TRUE,.observation_sequence=loops+1,
        .observation={.xvf_status=ACOUSTIC_XVF_STATUS_READY,.level_dbfs_x100=-2000,.vad=1},
    };
    if (stage==2 && ++recovery_loops<=10) {
        out->observation.vad=0;
        out->observation.level_dbfs_x100=-8000;
    }
    return 0;
}
ER task_command_set_target(const rover_motion_target_t *in) { target=*in;publications++;return 0; }
ER tk_dly_tsk(INT delay) {
    assert(delay==100);loops++;assert(loops<300);
    if(stage==0 && target.left_target_rpm!=0) {
        stage=1;freeze_publications=publications;
    } else if(stage==1 && !g_task_think_sensor_fresh) {
        assert(target.left_target_rpm==0 && target.right_target_rpm==0 && !target.emergency_stop);
        assert(publications>freeze_publications && g_task_think_fault_flags==0);
        if(mode==0)assert(g_task_think_sensor_watchdog_ms==200);
        else assert(failures>0);
        stage=2;
    } else if(stage==2 && g_task_think_sensor_fresh && target.left_target_rpm!=0) {
        longjmp(done,1);
    }
    now+=100;return 0;
}
static void pure_boundaries(void) {
    sensor_liveness_t s={0};
    assert(!sensor_liveness_update(&s,7,0,TRUE,200));
    assert(!sensor_liveness_update(&s,7,1000,TRUE,200));
    assert(sensor_liveness_update(&s,8,1001,TRUE,200));
    assert(sensor_liveness_update(&s,8,1200,TRUE,200));
    assert(!sensor_liveness_update(&s,8,1201,TRUE,200));
    assert(!sensor_liveness_update(&s,8,(UD)UINT32_MAX+2000,TRUE,200));
    assert(s.age_ms==UINT32_MAX);
    assert(sensor_liveness_update(&s,UINT32_MAX,(UD)UINT32_MAX+2001,TRUE,200));
    assert(sensor_liveness_update(&s,0,(UD)UINT32_MAX+2002,TRUE,200));
    assert(!sensor_liveness_update(&s,1,0,TRUE,200));
    assert(!sensor_liveness_update(&s,1,1,TRUE,200));
    assert(sensor_liveness_update(&s,2,2,TRUE,200));
    assert(!sensor_liveness_update(&s,3,3,FALSE,200));
    assert(!sensor_liveness_update(&s,3,4,TRUE,200));
    assert(sensor_liveness_update(&s,4,5,TRUE,200));
}
int main(void) {
    pure_boundaries();
    for(mode=0;mode<3;mode++) {
        now=(UD)UINT32_MAX-500;count=loops=stage=publications=failures=recovery_loops=0;
        assert(task_think_create()==0 && task_think_start()==0);
        if(setjmp(done)==0)entry(0,NULL);
        assert(stage==2);task_think_delete();
    }
    puts("sensor liveness: frozen age=0, deadline, clock/read failure, wrap, stop and recovery passed");
}
