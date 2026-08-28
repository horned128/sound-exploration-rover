/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = usbhs_interrupt_handler, /* USBHS USB INT RESUME (USBHS interrupt) */
            [1] = usbhs_d0fifo_handler, /* USBHS FIFO 0 (DMA transfer request 0) */
            [2] = usbhs_d1fifo_handler, /* USBHS FIFO 1 (DMA transfer request 1) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_USBHS_USB_INT_RESUME,GROUP0), /* USBHS USB INT RESUME (USBHS interrupt) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_USBHS_FIFO_0,GROUP1), /* USBHS FIFO 0 (DMA transfer request 0) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_USBHS_FIFO_1,GROUP2), /* USBHS FIFO 1 (DMA transfer request 1) */
        };
        #endif
        #endif
