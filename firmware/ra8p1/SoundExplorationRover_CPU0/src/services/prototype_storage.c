/** =================================================================*
 * @file   prototype_storage.c
 * @brief  音響プロトタイプのCode MRAM永続化
 * ================================================================= */
#include "services/prototype_storage.h"                     /* 永続化データ型と結果コード */
#include "hal_data.h"                                       /* FSP MRAMインスタンスとCPU制御レジスタ */
#include <stddef.h>                                         /* レコードCRC範囲のoffsetof */
#include <string.h>                                         /* レコード初期化とコピー */

#define CPU0_PROTOTYPE_STORAGE_MAGIC       (0x53525650U)
#define CPU0_PROTOTYPE_STORAGE_COMMIT      (0x434F4D54U)
#define CPU0_PROTOTYPE_STORAGE_VERSION     (1U)
#define CPU0_PROTOTYPE_STORAGE_SLOT_BYTES  (96U)
#define CPU0_PROTOTYPE_STORAGE_SLOT_COUNT  (2U)
#define CPU0_PROTOTYPE_STORAGE_REGION_BYTES (4096U)
#define CPU0_PROTOTYPE_STORAGE_PROGRAM_BYTES (32U)

typedef struct st_prototype_storage_record {
    UW magic;
    UH format_version;
    UH payload_bytes;
    UW generation;
    B prototype[CPU0_PROTOTYPE_STORAGE_BYTES];
    UB prototype_count;
    UB reserved[11];
    UW crc32;
    UW commit;
} prototype_storage_record_t;

_Static_assert(sizeof(prototype_storage_record_t) == CPU0_PROTOTYPE_STORAGE_SLOT_BYTES,
               "prototype storage record must occupy three MRAM programming units");
_Static_assert((CPU0_PROTOTYPE_STORAGE_SLOT_BYTES % CPU0_PROTOTYPE_STORAGE_PROGRAM_BYTES) == 0U,
               "prototype storage slot must be MRAM programming aligned");
_Static_assert(offsetof(prototype_storage_record_t, commit) ==
                   (CPU0_PROTOTYPE_STORAGE_SLOT_BYTES - sizeof(UW)),
               "prototype storage commit marker must be written last");

IMPORT UB __prototype_storage_start[];                      /**< リンカが予約したCPU0 Code MRAM先頭 */
IMPORT UB __prototype_storage_end[];                        /**< リンカが予約したCPU0 Code MRAM終端 */

LOCAL BOOL storage_initialized;                             /**< MRAMドライバ初期化済み */
/**< 読込検証バッファ */
LOCAL prototype_storage_record_t storage_records[CPU0_PROTOTYPE_STORAGE_SLOT_COUNT];
/**< 書込み元RAMバッファ */
LOCAL prototype_storage_record_t storage_write_record __attribute__((aligned(32)));
/**< 消去相当値 */
LOCAL UB storage_blank_record[CPU0_PROTOTYPE_STORAGE_SLOT_BYTES] __attribute__((aligned(32)));
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD
LOCAL UB storage_secondary_wait_state;                      /**< 保存前のCPU1WAITCR状態 */
#endif

LOCAL UW prototype_storage_address(void);                   /* リンカ予約領域先頭アドレス */
LOCAL UW prototype_storage_crc32(const UB * p_data, UW length); /* CRC-32/ISO-HDLC算出 */
LOCAL BOOL prototype_storage_record_valid(const prototype_storage_record_t * p_record); /* レコード検証 */
LOCAL void prototype_storage_records_read(void);            /* A/BスロットをRAMへ読込 */
LOCAL UW prototype_storage_latest_slot(BOOL * p_found);     /* 最新有効スロット選択 */
LOCAL BOOL prototype_storage_secondary_stall(void);         /* CPU1 Code MRAMアクセス停止 */
LOCAL void prototype_storage_secondary_resume(void);        /* CPU1実行再開 */

/** =================================================================*
 * @brief  リンカ予約領域先頭アドレス取得
 * @return CPU0 Code MRAM内の保存領域先頭
 * ================================================================= */
LOCAL UW prototype_storage_address(void) {
    return (UW) (uintptr_t) __prototype_storage_start;
}

/** =================================================================*
 * @brief  CRC-32/ISO-HDLC算出
 * @param[in] p_data 入力バイト列
 * @param[in] length 入力長[byte]
 * @return CRC-32値
 * ================================================================= */
LOCAL UW prototype_storage_crc32(const UB * p_data, UW length) {
    UW crc = 0xFFFFFFFFU;
    for (UW index = 0U; index < length; index++) {
        crc ^= p_data[index];
        for (UW bit = 0U; bit < 8U; bit++) {
            crc = (0U != (crc & 1U)) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/** =================================================================*
 * @brief  保存レコード検証
 * @param[in] p_record 検証対象
 * @return 形式、commit、CRCがすべて正常ならTRUE
 * ================================================================= */
LOCAL BOOL prototype_storage_record_valid(const prototype_storage_record_t * p_record) {
    if ((NULL == p_record) || (CPU0_PROTOTYPE_STORAGE_MAGIC != p_record->magic) ||
        (CPU0_PROTOTYPE_STORAGE_VERSION != p_record->format_version) ||
        (CPU0_PROTOTYPE_STORAGE_BYTES != p_record->payload_bytes) ||
        (p_record->prototype_count > 1U) || (CPU0_PROTOTYPE_STORAGE_COMMIT != p_record->commit)) {
        return FALSE;
    }

    UW const crc = prototype_storage_crc32((const UB *) p_record,
                                           (UW) offsetof(prototype_storage_record_t, crc32));
    return crc == p_record->crc32;
}

/** =================================================================*
 * @brief  A/Bスロット読込
 * ================================================================= */
LOCAL void prototype_storage_records_read(void) {
    UW const region_start = prototype_storage_address();
    for (UW slot = 0U; slot < CPU0_PROTOTYPE_STORAGE_SLOT_COUNT; slot++) {
        memcpy(&storage_records[slot],
               (const void *) (uintptr_t) (region_start + (slot * CPU0_PROTOTYPE_STORAGE_SLOT_BYTES)),
               sizeof(storage_records[slot]));
    }
}

/** =================================================================*
 * @brief  最新有効スロット選択
 * @param[out] p_found 有効スロットが存在すればTRUE
 * @return 最新スロット番号。有効データなしの場合は0
 * ================================================================= */
LOCAL UW prototype_storage_latest_slot(BOOL * p_found) {
    BOOL const slot0_valid = prototype_storage_record_valid(&storage_records[0]);
    BOOL const slot1_valid = prototype_storage_record_valid(&storage_records[1]);
    *p_found = slot0_valid || slot1_valid;

    if (!slot0_valid) {
        return slot1_valid ? 1U : 0U;
    }
    if (!slot1_valid) {
        return 0U;
    }
    return ((W) (storage_records[1].generation - storage_records[0].generation) > 0) ? 1U : 0U;
}

/** =================================================================*
 * @brief  CPU1 Code MRAMアクセス停止
 * @details MRAM P/E中にCPU1が命令フェッチしないよう、
 *          セカンダリコアをquiescent状態へ遷移させる。
 * @return CPU1WAITCRへ休止設定を反映できればTRUE
 * ================================================================= */
LOCAL BOOL prototype_storage_secondary_stall(void) {
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD
    storage_secondary_wait_state = (UB) (R_CPU_CTRL->CPU1WAITCR & R_CPU_CTRL_CPU1WAITCR_CPUWAIT_Msk);
    R_CPU_CTRL->CPU1WAITCR = R_CPU_CTRL_CPU1WAITCR_CPUWAIT_Msk;
    __DSB();
    __ISB();
    return 0U != (R_CPU_CTRL->CPU1WAITCR & R_CPU_CTRL_CPU1WAITCR_CPUWAIT_Msk);
#else
    return TRUE;
#endif
}

/** =================================================================*
 * @brief  CPU1実行再開
 * ================================================================= */
LOCAL void prototype_storage_secondary_resume(void) {
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD
    R_CPU_CTRL->CPU1WAITCR = storage_secondary_wait_state;
    __DSB();
    __ISB();
#endif
}

/** =================================================================*
 * @brief  MRAMドライバと保存領域検証
 * @return 初期化結果
 * ================================================================= */
EXPORT prototype_storage_result_t prototype_storage_init(void) {
    UW const region_start = prototype_storage_address();
    UW const region_end = (UW) (uintptr_t) __prototype_storage_end;
    if (((region_start % CPU0_PROTOTYPE_STORAGE_PROGRAM_BYTES) != 0U) ||
        ((region_end - region_start) != CPU0_PROTOTYPE_STORAGE_REGION_BYTES) ||
        ((CPU0_PROTOTYPE_STORAGE_SLOT_BYTES * CPU0_PROTOTYPE_STORAGE_SLOT_COUNT) > (region_end - region_start))) {
        return CPU0_PROTOTYPE_STORAGE_LAYOUT_ERROR;
    }

    fsp_err_t const err = R_MRAM_Open(&g_mram0_ctrl, &g_mram0_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err)) {
        return CPU0_PROTOTYPE_STORAGE_OPEN_ERROR;
    }
    storage_initialized = TRUE;
    return CPU0_PROTOTYPE_STORAGE_OK;
}

/** =================================================================*
 * @brief  最新有効値読込
 * @param[out] p_data 読込先
 * @return 読込結果。有効記録なしはCPU0_PROTOTYPE_STORAGE_EMPTY
 * ================================================================= */
EXPORT prototype_storage_result_t prototype_storage_load(prototype_storage_data_t * p_data) {
    if (NULL == p_data) {
        return CPU0_PROTOTYPE_STORAGE_ARGUMENT_ERROR;
    }
    if (!storage_initialized) {
        return CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED;
    }

    prototype_storage_records_read();

    BOOL found = FALSE;
    UW const latest_slot = prototype_storage_latest_slot(&found);
    if (!found) {
        memset(p_data, 0, sizeof(*p_data));
        return CPU0_PROTOTYPE_STORAGE_EMPTY;
    }

    p_data->generation = storage_records[latest_slot].generation;
    p_data->prototype_count = storage_records[latest_slot].prototype_count;
    memcpy(p_data->prototype, storage_records[latest_slot].prototype, sizeof(p_data->prototype));
    return CPU0_PROTOTYPE_STORAGE_OK;
}

/** =================================================================*
 * @brief  A/Bスロット保存
 * @details 古い側だけを32 byte単位で0xFFへ上書き後に更新し、
 *          書込み中の電源断でも直前スロットを保持する。
 *          FSP 6.4の消去経路がCode MRAM上のライブラリ関数を呼ぶ可能性を避けるため、
 *          RAM配置されたR_MRAM_Writeだけで消去相当処理を行う。
 *          操作中はCPU0割込みとCPU1実行を止める。
 * @param[in,out] p_data 保存値。成功時にgenerationを更新する
 * @return 保存および読戻し検証結果
 * ================================================================= */
EXPORT prototype_storage_result_t prototype_storage_save(prototype_storage_data_t * p_data) {
    if ((NULL == p_data) || (p_data->prototype_count > 1U)) {
        return CPU0_PROTOTYPE_STORAGE_ARGUMENT_ERROR;
    }
    if (!storage_initialized) {
        return CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED;
    }

    prototype_storage_records_read();

    BOOL latest_found = FALSE;
    UW const latest_slot = prototype_storage_latest_slot(&latest_found);
    UW const target_slot = latest_found ? (latest_slot ^ 1U) : 0U;
    UW const next_generation = latest_found ? (storage_records[latest_slot].generation + 1U) : 1U;
    UW const target_address = prototype_storage_address() + (target_slot * CPU0_PROTOTYPE_STORAGE_SLOT_BYTES);

    memset(&storage_write_record, 0, sizeof(storage_write_record));
    storage_write_record.magic = CPU0_PROTOTYPE_STORAGE_MAGIC;
    storage_write_record.format_version = CPU0_PROTOTYPE_STORAGE_VERSION;
    storage_write_record.payload_bytes = CPU0_PROTOTYPE_STORAGE_BYTES;
    storage_write_record.generation = next_generation;
    memcpy(storage_write_record.prototype, p_data->prototype, sizeof(storage_write_record.prototype));
    storage_write_record.prototype_count = p_data->prototype_count;
    storage_write_record.crc32 = prototype_storage_crc32(
        (const UB *) &storage_write_record, (UW) offsetof(prototype_storage_record_t, crc32));
    storage_write_record.commit = CPU0_PROTOTYPE_STORAGE_COMMIT;
    memset(storage_blank_record, 0xFF, sizeof(storage_blank_record));

    UW const interrupt_state = (UW) __get_PRIMASK();
    __disable_irq();
    if (!prototype_storage_secondary_stall()) {
        if (0U == interrupt_state) {
            __enable_irq();
        }
        return CPU0_PROTOTYPE_STORAGE_CORE_STALL_ERROR;
    }

    fsp_err_t const blank_err =
        R_MRAM_Write(&g_mram0_ctrl, (uint32_t) (uintptr_t) storage_blank_record,
                     (uint32_t) target_address, (uint32_t) sizeof(storage_blank_record));
    fsp_err_t write_err = FSP_SUCCESS;
    if (FSP_SUCCESS == blank_err) {
        write_err = R_MRAM_Write(&g_mram0_ctrl, (uint32_t) (uintptr_t) &storage_write_record,
                                 (uint32_t) target_address, (uint32_t) sizeof(storage_write_record));
    }

    prototype_storage_secondary_resume();
    if (0U == interrupt_state) {
        __enable_irq();
    }

    if (FSP_SUCCESS != blank_err) {
        return CPU0_PROTOTYPE_STORAGE_BLANK_ERROR;
    }
    if (FSP_SUCCESS != write_err) {
        return CPU0_PROTOTYPE_STORAGE_WRITE_ERROR;
    }

    memcpy(&storage_records[target_slot], (const void *) (uintptr_t) target_address,
           sizeof(storage_records[target_slot]));
    if (!prototype_storage_record_valid(&storage_records[target_slot]) ||
        (0 != memcmp(&storage_write_record, &storage_records[target_slot], sizeof(storage_write_record)))) {
        return CPU0_PROTOTYPE_STORAGE_VERIFY_ERROR;
    }

    p_data->generation = next_generation;
    return CPU0_PROTOTYPE_STORAGE_OK;
}
