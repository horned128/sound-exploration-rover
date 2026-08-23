/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (5)
#endif
/* ISR prototypes */
void ipc_isr(void);
void r_icu_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_IPC_IRQ0 ((IRQn_Type) 0) /* IPC IRQ0 (CPU Mutual Interrupt 0) */
#define IPC_IRQ0_IRQn          ((IRQn_Type) 0) /* IPC IRQ0 (CPU Mutual Interrupt 0) */
#define VECTOR_NUMBER_ICU_IRQ11 ((IRQn_Type) 1) /* ICU IRQ11 (External pin interrupt 11) */
#define ICU_IRQ11_IRQn          ((IRQn_Type) 1) /* ICU IRQ11 (External pin interrupt 11) */
#define VECTOR_NUMBER_ICU_IRQ18 ((IRQn_Type) 2) /* ICU IRQ18 (External pin interrupt 18) */
#define ICU_IRQ18_IRQn          ((IRQn_Type) 2) /* ICU IRQ18 (External pin interrupt 18) */
#define VECTOR_NUMBER_ICU_IRQ16 ((IRQn_Type) 3) /* ICU IRQ16 (External pin interrupt 16) */
#define ICU_IRQ16_IRQn          ((IRQn_Type) 3) /* ICU IRQ16 (External pin interrupt 16) */
#define VECTOR_NUMBER_ICU_IRQ20 ((IRQn_Type) 4) /* ICU IRQ20 (External pin interrupt 20) */
#define ICU_IRQ20_IRQn          ((IRQn_Type) 4) /* ICU IRQ20 (External pin interrupt 20) */
/* The number of entries required for the ICU vector table. */
#define BSP_ICU_VECTOR_NUM_ENTRIES (5)

#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */
