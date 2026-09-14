/*
                  Linker File for Renesas FSP
*/

INCLUDE memory_regions.ld

/* CPU0 Code MRAM末尾4 KiBを学習プロトタイプ保存専用にする。 */
PROVIDE(__prototype_storage_start = ORIGIN(FLASH) + LENGTH(FLASH) - 0x1000);
PROVIDE(__prototype_storage_end = ORIGIN(FLASH) + LENGTH(FLASH));

INCLUDE fsp_gen.ld

/* ファームウェアが保存領域へ成長した場合はリンク時に停止し、実機上の上書きを防ぐ。 */
ASSERT(__ddsc_FLASH_NSC <= __prototype_storage_start,
       "CPU0 Code MRAM image overlaps the reserved prototype storage area")
