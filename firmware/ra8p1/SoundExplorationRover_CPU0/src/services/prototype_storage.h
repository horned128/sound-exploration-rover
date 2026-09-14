/** =================================================================*
 * @file   prototype_storage.h
 * @brief  音響プロトタイプのCode MRAM永続化
 * ================================================================= */
#ifndef SEROV_CPU0_SERVICE_PROTOTYPE_STORAGE_H
#define SEROV_CPU0_SERVICE_PROTOTYPE_STORAGE_H

#include <tk/tkernel.h>                                     /* μT-Kernel基本型とリンケージ定義 */

#define CPU0_PROTOTYPE_STORAGE_BYTES       (64U)

typedef enum e_prototype_storage_result {
    CPU0_PROTOTYPE_STORAGE_OK = 0U,
    CPU0_PROTOTYPE_STORAGE_EMPTY,
    CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED,
    CPU0_PROTOTYPE_STORAGE_ARGUMENT_ERROR,
    CPU0_PROTOTYPE_STORAGE_LAYOUT_ERROR,
    CPU0_PROTOTYPE_STORAGE_OPEN_ERROR,
    CPU0_PROTOTYPE_STORAGE_CORE_STALL_ERROR,
    CPU0_PROTOTYPE_STORAGE_BLANK_ERROR,
    CPU0_PROTOTYPE_STORAGE_WRITE_ERROR,
    CPU0_PROTOTYPE_STORAGE_VERIFY_ERROR,
} prototype_storage_result_t;

typedef struct st_prototype_storage_data {
    UW generation;
    UB prototype_count;
    B prototype[CPU0_PROTOTYPE_STORAGE_BYTES];
} prototype_storage_data_t;

EXPORT prototype_storage_result_t prototype_storage_init(void); /* MRAMドライバと保存領域検証 */
EXPORT prototype_storage_result_t prototype_storage_load(prototype_storage_data_t * p_data); /* 最新有効値読込 */
EXPORT prototype_storage_result_t prototype_storage_save(prototype_storage_data_t * p_data); /* A/Bスロット保存 */

#endif /* SEROV_CPU0_SERVICE_PROTOTYPE_STORAGE_H */
