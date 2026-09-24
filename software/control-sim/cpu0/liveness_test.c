#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "tasks/task_infer.h"
#include "tasks/task_think.h"
#include "tasks/task_sensor.h"
#include "tasks/task_command.h"
#include "ipc/actuator_ipc_client.h"
#include "tasks/task_acoustic_link.h"
#include "control/sensor_liveness.h"
#include "config/sensor_config.h"
#include "config/control_config.h"

static void (*entry)(INT,void *);
static jmp_buf done;
static UD now;
static UW count, loops, stage, publications, freeze_publications;
static int mode, failures, recovery_loops;
static int clear_calls, save_calls, infer_reset_calls, green_level;
static rover_motion_target_t target;
static const bsp_io_port_pin_t led_pins[]={10,11};
bsp_leds_t g_bsp_leds={2,led_pins};
volatile UW g_task_acoustic_link_feature_generation;
volatile UW g_task_infer_feature_generation;
BOOL actuator_ipc_client_status_get(actuator_status_t * status) {
    (void) status;
    return FALSE;
}
void odometry_service_get_pose(odometry_pose_t *pose) {
    *pose=(odometry_pose_t){.valid=TRUE};
}
void sound_source_localizer_init(void) {}
void sound_source_localizer_step(const sound_source_localizer_input_t *input,
                                 sound_source_localizer_output_t *output) {
    (void)input;*output=(sound_source_localizer_output_t){0};
}
static fsp_err_t pin_read(void *ctrl,bsp_io_port_pin_t pin,bsp_io_level_t *level) {
    (void)ctrl;(void)pin;*level=BSP_IO_LEVEL_HIGH;return FSP_SUCCESS;
}
static const ioport_api_t io_api={.pinRead=pin_read};
const ioport_instance_t g_ioport={NULL,&io_api};
void R_BSP_PinAccessEnable(void) {}
void R_BSP_PinAccessDisable(void) {}
void R_BSP_PinWrite(bsp_io_port_pin_t pin,bsp_io_level_t level) {
    if(pin==11)green_level=level;
}
prototype_storage_result_t prototype_storage_init(void) {
    return mode>=3 ? CPU0_PROTOTYPE_STORAGE_OK : CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED;
}
prototype_storage_result_t prototype_storage_load(prototype_storage_data_t *data) {
    if(mode>=3) {
        *data=(prototype_storage_data_t){.sample_count=5,.generation=18};
        return CPU0_PROTOTYPE_STORAGE_OK;
    }
    return CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED;
}
prototype_storage_result_t prototype_storage_clear(void) {
    clear_calls++;
    return mode==4 ? CPU0_PROTOTYPE_STORAGE_BLANK_ERROR : CPU0_PROTOTYPE_STORAGE_OK;
}
prototype_storage_result_t prototype_storage_save(prototype_storage_data_t *data) {
    assert(data->sample_count==5);
    save_calls++;
    return CPU0_PROTOTYPE_STORAGE_OK;
}
ER task_acoustic_link_feature_get(acoustic_feature_patch_t *patch,UW *generation) {
    (void)patch;(void)generation;return E_NOEXS;
}
ER task_infer_result_get(task_infer_result_t *result) {
    if(mode==3 && loops>=12) {
        *result=(task_infer_result_t){.feature_generation=loops+1,.summary_valid=TRUE};
        return E_OK;
    }
    return E_NOEXS;
}
ER task_infer_prototype_set(const prototype_storage_data_t *data,BOOL storage_valid) {
    (void)data;
    if(!storage_valid)infer_reset_calls++;
    return E_OK;
}
ER task_infer_background_export(prototype_storage_data_t *data) {
    (void)data;return mode==3 ? E_OK : E_NOEXS;
}

UB acoustic_identifier_find_peak_bin(const B *samples, UW sample_count) {
    (void)samples;
    (void)sample_count;
    return 0U;
}

UB acoustic_identifier_consensus_peak_bin(const B *samples, UW sample_count) {
    return acoustic_identifier_find_peak_bin(samples, sample_count);
}

BOOL acoustic_identifier_isolated_sample_find(const B *samples, UW sample_count, UW *index) {
    (void)samples;
    (void)sample_count;
    (void)index;
    return FALSE;
}

void acoustic_identifier_build_weights(UB peak_bin, float *weights) {
    (void)peak_bin;
    if (weights != NULL) {
        for (int i = 0; i < 32; i++) {
            weights[i] = 1.0F;
        }
    }
}

BOOL acoustic_identifier_leave_one_out_threshold(const B *samples,
                                                 UW sample_count,
                                                 const float *bin_weights,
                                                 float *threshold) {
    (void)samples;
    (void)sample_count;
    (void)bin_weights;
    if (threshold != NULL) {
        *threshold=0.0F;
    }
    return mode==3 ? TRUE : FALSE;
}

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
        .observation={.doa_deg=0,.raw_doa_deg=0,.level_dbfs_x100=-2000,.vad=1,
                      .doa_confidence=90,.xvf_status=ACOUSTIC_XVF_STATUS_READY},
    };
    if (stage==2 && ++recovery_loops<=20) {
        out->observation.vad=0;
        out->observation.level_dbfs_x100=-8000;
    }
    return 0;
}
ER task_command_set_target(const rover_motion_target_t *in) { target=*in;publications++;return 0; }
ER tk_dly_tsk(INT delay) {
    if(delay==100 && mode>=3) {
        assert(target.left_target_rpm==0 && target.right_target_rpm==0 && target.emergency_stop);
        now+=delay;return E_OK;
    }
    assert(delay==50);loops++;assert(loops<600);
    if(mode>=3) {
        if(loops==3) {
            assert(g_task_think_storage_valid && green_level==BSP_IO_LEVEL_HIGH);
            assert(task_think_learning_request(TASK_THINK_LEARNING_COMMAND_START)==E_OK);
        }
        if(loops==4) {
            assert(clear_calls==1 && infer_reset_calls>=1);
            assert(!g_task_think_storage_valid);
            assert(g_task_think_learning_mode==(mode==3));
            if(mode==4)assert(g_task_think_storage_result==CPU0_PROTOTYPE_STORAGE_BLANK_ERROR);
        }
        if(mode==3 && loops==11) {
            assert(g_task_think_learning_samples==0 && green_level==BSP_IO_LEVEL_LOW);
        }
        if(mode==3 && loops==19) {
            assert(g_task_think_learning_samples==5 && green_level==BSP_IO_LEVEL_LOW);
        }
        if(mode==3 && loops==21) {
            assert(g_task_think_learning_samples==5 && green_level==BSP_IO_LEVEL_HIGH);
        }
        if(mode==3 && loops==25) {
            assert(g_task_think_learning_samples==5);
            assert(task_think_learning_request(TASK_THINK_LEARNING_COMMAND_COMMIT)==E_OK);
        }
        if(mode==3 && loops==26) {
            assert(save_calls==1 && g_task_think_storage_valid && !g_task_think_learning_mode);
            assert(green_level==BSP_IO_LEVEL_HIGH);
            longjmp(done,1);
        }
        if(mode==4 && loops==20) {
            assert(green_level==BSP_IO_LEVEL_LOW);
            longjmp(done,1);
        }
        now+=delay;return 0;
    }
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
    now+=delay;return 0;
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
    for(mode=3;mode<5;mode++) {
        now=count=loops=stage=publications=clear_calls=save_calls=infer_reset_calls=0;
        green_level=BSP_IO_LEVEL_LOW;
        assert(task_think_create()==0 && task_think_start()==0);
        if(setjmp(done)==0)entry(0,NULL);
        task_think_delete();
    }
    puts("sensor liveness and learning UI: stop/recovery, MRAM reset, LED, save and failure passed");
}
