/** =================================================================*
 * @file   prototype_storage.c
 * @brief  背景モデルと現場見本のCode MRAM永続化
 * ================================================================= */
#include "services/prototype_storage.h"                     /* 永続化データ型と結果コード */
#include "hal_data.h"                                       /* FSP MRAMとCPU制御レジスタ */
#include <stddef.h>                                         /* offsetof: CRC範囲 */
#include <string.h>                                         /* record初期化とコピー */

#define CPU0_PROTOTYPE_STORAGE_MAGIC       (0x53525650U)    /**< 見本保存の識別値 */
#define CPU0_PROTOTYPE_STORAGE_COMMIT      (0x434F4D54U)    /**< 見本保存の確定 */
/* v6ではDSP見本に加えて音響埋め込みCNNの5見本(64D)を保存する。 */
#define CPU0_PROTOTYPE_STORAGE_VERSION     (6U)             /**< 見本保存の版 */
#define CPU0_PROTOTYPE_STORAGE_SLOT_BYTES  (16384U)         /**< 見本保存のスロット[byte] */
#define CPU0_PROTOTYPE_STORAGE_SLOT_COUNT  (2U)             /**< 見本保存スロットの個数 */
#define CPU0_PROTOTYPE_STORAGE_REGION_BYTES (32768U)        /**< 見本保存の領域[byte] */
#define CPU0_PROTOTYPE_STORAGE_PROGRAM_BYTES (32U)          /**< 見本保存の書込み[byte] */
#define CPU0_PROTOTYPE_STORAGE_RECORD_HEADER_BYTES (8U)     /**< 見本保存レコードのヘッダ[byte] */
#define CPU0_PROTOTYPE_STORAGE_RECORD_FOOTER_BYTES (8U)     /**< 見本保存レコードのフッタ[byte] */
#define CPU0_PROTOTYPE_STORAGE_RECORD_RESERVED_BYTES        /**< MRAMレコードの予約領域サイズ */ \
    (CPU0_PROTOTYPE_STORAGE_SLOT_BYTES - CPU0_PROTOTYPE_STORAGE_RECORD_HEADER_BYTES - \
     sizeof(prototype_storage_data_t) - CPU0_PROTOTYPE_STORAGE_RECORD_FOOTER_BYTES)

/**< MRAMの1スロットに格納する検証付き保存レコード */
typedef struct st_prototype_storage_record {
    UW magic;                                               /**< 保存レコードの識別値 */
    UH format_version;                                      /**< 保存データ形式の版 */
    UH payload_bytes;                                       /**< payloadのバイト数 */
    prototype_storage_data_t data;                          /**< 背景モデルと見本のpayload */
    /**< スロット末尾までを埋める予約領域 */
    UB reserved[CPU0_PROTOTYPE_STORAGE_RECORD_RESERVED_BYTES];
    UW crc32;                                               /**< magicからpayload末尾までのCRC32 */
    UW commit;                                              /**< 書込み完了を示す確定マーカー */
} prototype_storage_record_t;

_Static_assert(CPU0_PROTOTYPE_STORAGE_RECORD_RESERVED_BYTES > 0U,
               "prototype storage payload must fit a single A/B slot");
_Static_assert(sizeof(prototype_storage_record_t) == CPU0_PROTOTYPE_STORAGE_SLOT_BYTES,
               "prototype storage record must occupy exactly one MRAM slot");
_Static_assert((CPU0_PROTOTYPE_STORAGE_SLOT_BYTES % CPU0_PROTOTYPE_STORAGE_PROGRAM_BYTES) == 0U,
               "prototype storage slot must be MRAM programming aligned");
_Static_assert(offsetof(prototype_storage_record_t, commit) ==
                   (CPU0_PROTOTYPE_STORAGE_SLOT_BYTES - sizeof(UW)),
               "prototype storage commit marker must be written last");

IMPORT UB __prototype_storage_start[];                      /**< MRAM見本保存領域の開始アドレス */
IMPORT UB __prototype_storage_end[];                        /**< MRAM見本保存領域の終了アドレス */

LOCAL BOOL storage_initialized;                             /**< MRAM保存サービス初期化状態 */
/**< MRAM読出しレコード */
LOCAL prototype_storage_record_t storage_records[CPU0_PROTOTYPE_STORAGE_SLOT_COUNT];
LOCAL prototype_storage_record_t storage_write_record __attribute__((aligned(32))); /**< MRAM書込みレコード */
LOCAL UB storage_blank_record[CPU0_PROTOTYPE_STORAGE_SLOT_BYTES] __attribute__((aligned(32))); /**< 空レコード */
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD
LOCAL UB storage_secondary_wait_state;                      /**< CPU1停止状態の保存値 */
#endif

LOCAL UW prototype_storage_address(void);                   /* MRAM保存領域アドレス取得 */
LOCAL UW prototype_storage_crc32(const UB * p_data, UW length); /* MRAMレコードCRC32算出 */
LOCAL BOOL prototype_storage_record_valid(const prototype_storage_record_t * p_record); /* record判定 */
LOCAL void prototype_storage_records_read(void);            /* MRAMレコード読出し */
LOCAL UW prototype_storage_latest_slot(BOOL * p_found);     /* 最新MRAM保存スロット取得 */
LOCAL BOOL prototype_storage_secondary_stall(void);         /* CPU1停止要求 */
LOCAL void prototype_storage_secondary_resume(void);        /* CPU1停止解除 */
LOCAL fsp_err_t prototype_storage_write_units(const void * p_source, UW destination,
                                              UW byte_count); /* MRAM単位書込み */

/** =================================================================*
 * @brief  リンカ予約領域の先頭アドレス
 * ================================================================= */
LOCAL UW prototype_storage_address(void) {
    return (UW) (uintptr_t) __prototype_storage_start;
}

/** =================================================================*
 * @brief  CRC-32/ISO-HDLC
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
 * @brief  recordの形式、範囲、CRC、commitを検証
 * ================================================================= */
LOCAL BOOL prototype_storage_record_valid(const prototype_storage_record_t * p_record) {
    if ((NULL == p_record) || (CPU0_PROTOTYPE_STORAGE_MAGIC != p_record->magic) ||
        (CPU0_PROTOTYPE_STORAGE_VERSION != p_record->format_version) ||
        ((UH) sizeof(prototype_storage_data_t) != p_record->payload_bytes) ||
        (p_record->data.sample_count > CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT) ||
        (CPU0_PROTOTYPE_STORAGE_COMMIT != p_record->commit)) {
        return FALSE;
    }
    return prototype_storage_crc32((const UB *) p_record,
                                   (UW) offsetof(prototype_storage_record_t, crc32)) == p_record->crc32;
}

/** =================================================================*
 * @brief  A/BスロットをRAMへ読み込む
 * ================================================================= */
LOCAL void prototype_storage_records_read(void) {
    UW const start = prototype_storage_address();
    for (UW slot = 0U; slot < CPU0_PROTOTYPE_STORAGE_SLOT_COUNT; slot++) {
        memcpy(&storage_records[slot],
               (const void *) (uintptr_t) (start + (slot * CPU0_PROTOTYPE_STORAGE_SLOT_BYTES)),
               sizeof(storage_records[slot]));
    }
}

/** =================================================================*
 * @brief  最新世代の有効A/Bスロットを選ぶ
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
    return ((W) (storage_records[1].data.generation - storage_records[0].data.generation) > 0) ? 1U : 0U;
}

/** =================================================================*
 * @brief  Code MRAM書込み中のCPU1命令フェッチを停止
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
 * @brief  CPU1実行を保存前の状態へ戻す
 * ================================================================= */
LOCAL void prototype_storage_secondary_resume(void) {
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD
    R_CPU_CTRL->CPU1WAITCR = storage_secondary_wait_state;
    __DSB();
    __ISB();
#endif
}

/** =================================================================*
 * @brief  32byte単位のMRAM書込み
 * @details 長いA/BスロットもFSPの最小プログラム単位で確実に書き込む。
 * ================================================================= */
LOCAL fsp_err_t prototype_storage_write_units(const void * p_source, UW destination, UW byte_count) {
    const UB * p_bytes = (const UB *) p_source;
    for (UW offset = 0U; offset < byte_count; offset += CPU0_PROTOTYPE_STORAGE_PROGRAM_BYTES) {
        fsp_err_t const err = R_MRAM_Write(&g_mram0_ctrl,
                                            (uint32_t) (uintptr_t) &p_bytes[offset],
                                            (uint32_t) (destination + offset),
                                            CPU0_PROTOTYPE_STORAGE_PROGRAM_BYTES);
        if (FSP_SUCCESS != err) {
            return err;
        }
    }
    return FSP_SUCCESS;
}

/** =================================================================*
 * @brief  MRAMドライバと32KiB予約領域を初期化・検証
 * ================================================================= */
EXPORT prototype_storage_result_t prototype_storage_init(void) {
    UW const start = prototype_storage_address();
    UW const end = (UW) (uintptr_t) __prototype_storage_end;
    if (((start % CPU0_PROTOTYPE_STORAGE_PROGRAM_BYTES) != 0U) ||
        ((end - start) != CPU0_PROTOTYPE_STORAGE_REGION_BYTES) ||
        ((CPU0_PROTOTYPE_STORAGE_SLOT_BYTES * CPU0_PROTOTYPE_STORAGE_SLOT_COUNT) > (end - start))) {
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
 * @brief  最新の有効な背景モデル・見本を読み込む
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
    UW const slot = prototype_storage_latest_slot(&found);
    if (!found) {
        memset(p_data, 0, sizeof(*p_data));
        return CPU0_PROTOTYPE_STORAGE_EMPTY;
    }
    *p_data = storage_records[slot].data;
    return CPU0_PROTOTYPE_STORAGE_OK;
}

/** =================================================================*
 * @brief 学習開始時にA/B両スロットを0xFFで初期化し読戻し検証する
 * @details 片方だけ初期化すると再起動時に旧世代が復活するため、両方消す。
 * ================================================================= */
EXPORT prototype_storage_result_t prototype_storage_clear(void) {
    if (!storage_initialized) {
        return CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED;
    }
    memset(storage_blank_record, 0xFF, sizeof(storage_blank_record));

    UW const interrupt_state = (UW) __get_PRIMASK();
    __disable_irq();
    if (!prototype_storage_secondary_stall()) {
        if (0U == interrupt_state) {
            __enable_irq();
        }
        return CPU0_PROTOTYPE_STORAGE_CORE_STALL_ERROR;
    }
    BOOL write_failed = FALSE;
    for (UW slot = 0U; slot < CPU0_PROTOTYPE_STORAGE_SLOT_COUNT; slot++) {
        UW const target = prototype_storage_address() + (slot * CPU0_PROTOTYPE_STORAGE_SLOT_BYTES);
        if (FSP_SUCCESS != prototype_storage_write_units(storage_blank_record, target,
                                                           CPU0_PROTOTYPE_STORAGE_SLOT_BYTES)) {
            write_failed = TRUE;
        }
    }
    prototype_storage_secondary_resume();
    if (0U == interrupt_state) {
        __enable_irq();
    }
    if (write_failed) {
        return CPU0_PROTOTYPE_STORAGE_BLANK_ERROR;
    }

    prototype_storage_records_read();
    for (UW slot = 0U; slot < CPU0_PROTOTYPE_STORAGE_SLOT_COUNT; slot++) {
        if (0 != memcmp(&storage_records[slot], storage_blank_record,
                        CPU0_PROTOTYPE_STORAGE_SLOT_BYTES)) {
            return CPU0_PROTOTYPE_STORAGE_VERIFY_ERROR;
        }
    }
    return CPU0_PROTOTYPE_STORAGE_OK;
}

/** =================================================================*
 * @brief  背景モデル・見本をA/Bスロットへ保存し読戻し検証する
 * @details 旧4KiB領域ではなく新32KiB領域だけを読むため、旧保存値を誤用しない。
 * ================================================================= */
EXPORT prototype_storage_result_t prototype_storage_save(prototype_storage_data_t * p_data) {
    if ((NULL == p_data) || (p_data->sample_count > CPU0_PROTOTYPE_STORAGE_SAMPLE_COUNT)) {
        return CPU0_PROTOTYPE_STORAGE_ARGUMENT_ERROR;
    }
    if (!storage_initialized) {
        return CPU0_PROTOTYPE_STORAGE_NOT_INITIALIZED;
    }
    prototype_storage_records_read();
    BOOL latest_found = FALSE;
    UW const latest_slot = prototype_storage_latest_slot(&latest_found);
    UW const target_slot = latest_found ? (latest_slot ^ 1U) : 0U;
    UW const target = prototype_storage_address() + (target_slot * CPU0_PROTOTYPE_STORAGE_SLOT_BYTES);

    memset(&storage_write_record, 0, sizeof(storage_write_record));
    storage_write_record.magic = CPU0_PROTOTYPE_STORAGE_MAGIC;
    storage_write_record.format_version = CPU0_PROTOTYPE_STORAGE_VERSION;
    storage_write_record.payload_bytes = (UH) sizeof(prototype_storage_data_t);
    p_data->generation = latest_found ? (storage_records[latest_slot].data.generation + 1U) : 1U;
    storage_write_record.data = *p_data;
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
    fsp_err_t const blank_err = prototype_storage_write_units(storage_blank_record, target,
                                                                CPU0_PROTOTYPE_STORAGE_SLOT_BYTES);
    fsp_err_t write_err = FSP_SUCCESS;
    if (FSP_SUCCESS == blank_err) {
        write_err = prototype_storage_write_units(&storage_write_record, target,
                                                  CPU0_PROTOTYPE_STORAGE_SLOT_BYTES);
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

    memcpy(&storage_records[target_slot], (const void *) (uintptr_t) target,
           sizeof(storage_records[target_slot]));
    if (!prototype_storage_record_valid(&storage_records[target_slot]) ||
        (0 != memcmp(&storage_write_record, &storage_records[target_slot], sizeof(storage_write_record)))) {
        return CPU0_PROTOTYPE_STORAGE_VERIFY_ERROR;
    }
    return CPU0_PROTOTYPE_STORAGE_OK;
}
