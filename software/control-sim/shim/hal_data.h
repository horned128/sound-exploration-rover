#ifndef CONTROL_SIM_HAL_DATA_H
#define CONTROL_SIM_HAL_DATA_H

#include <stdint.h>

typedef int32_t fsp_err_t;

#define FSP_SUCCESS ((fsp_err_t) 0)

#ifndef FSP_CRITICAL_SECTION_DEFINE
#define FSP_CRITICAL_SECTION_DEFINE
#define FSP_CRITICAL_SECTION_ENTER
#define FSP_CRITICAL_SECTION_EXIT
#endif

#endif /* CONTROL_SIM_HAL_DATA_H */
