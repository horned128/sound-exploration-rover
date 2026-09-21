/** =================================================================*
 * @file   tflm_runtime_shim.c
 * @brief  Host test shim for TFLM runtime API
 * ================================================================= */
#include "ai/tflm_runtime.h"
#include "control/control_mlp_model.h"
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef INT (*fn_tflm_init)(UB const * model_data, UW model_bytes);
typedef INT (*fn_tflm_invoke)(B const * input, UW input_bytes, B * output, UW output_bytes);
typedef INT (*fn_tflm_get_info)(tflm_runtime_info_t * info);
typedef void (*fn_tflm_reset)(void);

static void * s_lib_handle = NULL;
static fn_tflm_init s_fn_init = NULL;
static fn_tflm_invoke s_fn_invoke = NULL;
static fn_tflm_get_info s_fn_get_info = NULL;
static fn_tflm_reset s_fn_reset = NULL;
static BOOL s_mock_initialized = FALSE;

static void ensure_loaded(void) {
    if (s_lib_handle != NULL) {
        return;
    }
    char const * env_path = getenv("TFLM_RUNTIME_LIB_PATH");
    if (env_path != NULL) {
        s_lib_handle = dlopen(env_path, RTLD_NOW | RTLD_LOCAL);
    }
    if (s_lib_handle == NULL) {
        char const * paths[] = {
            "../../software/acoustic-trainer/build/libtflm_runtime.dylib",
            "../software/acoustic-trainer/build/libtflm_runtime.dylib",
            "software/acoustic-trainer/build/libtflm_runtime.dylib",
            "../../software/acoustic-trainer/build/libtflm_runtime.so",
            "../software/acoustic-trainer/build/libtflm_runtime.so",
            "software/acoustic-trainer/build/libtflm_runtime.so",
            "libtflm_runtime.dylib",
            "libtflm_runtime.so",
            NULL
        };
        for (int i = 0; paths[i] != NULL; ++i) {
            s_lib_handle = dlopen(paths[i], RTLD_NOW | RTLD_LOCAL);
            if (s_lib_handle != NULL) {
                break;
            }
        }
    }
    if (s_lib_handle != NULL) {
        s_fn_init = (fn_tflm_init)dlsym(s_lib_handle, "tflm_runtime_init");
        s_fn_invoke = (fn_tflm_invoke)dlsym(s_lib_handle, "tflm_runtime_invoke");
        s_fn_get_info = (fn_tflm_get_info)dlsym(s_lib_handle, "tflm_runtime_get_info");
        s_fn_reset = (fn_tflm_reset)dlsym(s_lib_handle, "tflm_runtime_reset");
    }
}

INT tflm_runtime_init(UB const * model_data, UW model_bytes) {
    if (model_data == NULL || model_bytes == 0) {
        return TFLM_RUNTIME_ARGUMENT_ERROR;
    }
    ensure_loaded();
    if (s_fn_init != NULL) {
        return s_fn_init(model_data, model_bytes);
    }
    s_mock_initialized = TRUE;
    return TFLM_RUNTIME_OK;
}

INT tflm_runtime_get_info(tflm_runtime_info_t * info) {
    if (info == NULL) {
        return TFLM_RUNTIME_ARGUMENT_ERROR;
    }
    ensure_loaded();
    if (s_fn_get_info != NULL) {
        return s_fn_get_info(info);
    }
    if (!s_mock_initialized) {
        return TFLM_RUNTIME_NOT_READY;
    }
    info->input_bytes = CONTROL_MLP_INPUT_DIMENSION;
    info->output_bytes = CONTROL_MLP_OUTPUT_DIMENSION;
    info->arena_used_bytes = 4096;
    info->input_scale = CONTROL_MLP_INPUT_SCALE;
    info->output_scale = CONTROL_MLP_OUTPUT_SCALE;
    info->input_zero_point = CONTROL_MLP_INPUT_ZERO_POINT;
    info->output_zero_point = CONTROL_MLP_OUTPUT_ZERO_POINT;
    return TFLM_RUNTIME_OK;
}

INT tflm_runtime_invoke(B const * input, UW input_bytes, B * output, UW output_bytes) {
    if (input == NULL || output == NULL ||
        input_bytes != CONTROL_MLP_INPUT_DIMENSION ||
        output_bytes != CONTROL_MLP_OUTPUT_DIMENSION) {
        return TFLM_RUNTIME_ARGUMENT_ERROR;
    }
    ensure_loaded();
    if (s_fn_invoke != NULL) {
        return s_fn_invoke(input, input_bytes, output, output_bytes);
    }
    if (!s_mock_initialized) {
        return TFLM_RUNTIME_NOT_READY;
    }
    output[0] = 0;
    output[1] = 125;
    return TFLM_RUNTIME_OK;
}

void tflm_runtime_reset(void) {
    ensure_loaded();
    if (s_fn_reset != NULL) {
        s_fn_reset();
    }
    s_mock_initialized = FALSE;
}
