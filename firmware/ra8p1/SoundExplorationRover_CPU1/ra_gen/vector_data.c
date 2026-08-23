/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = ipc_isr, /* IPC IRQ0 (CPU Mutual Interrupt 0) */
            [1] = r_icu_isr, /* ICU IRQ11 (External pin interrupt 11) */
            [2] = r_icu_isr, /* ICU IRQ18 (External pin interrupt 18) */
            [3] = r_icu_isr, /* ICU IRQ16 (External pin interrupt 16) */
            [4] = r_icu_isr, /* ICU IRQ20 (External pin interrupt 20) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_IPC_IRQ0,GROUP0), /* IPC IRQ0 (CPU Mutual Interrupt 0) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ11,GROUP1), /* ICU IRQ11 (External pin interrupt 11) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ18,GROUP2), /* ICU IRQ18 (External pin interrupt 18) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ16,GROUP3), /* ICU IRQ16 (External pin interrupt 16) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ20,GROUP4), /* ICU IRQ20 (External pin interrupt 20) */
        };
        #endif
        #endif
