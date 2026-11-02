/*
 * Copyright (c) 2026 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 *
 * Place the DWC3 USB host DMA buffers in SRAM0 for the UVC host camera, which the USB DMA can reach
 * (the M55 TCM is not).
 */

SECTION_DATA_PROLOGUE(.sram0.bss.uhc_dma, (NOLOAD),)
{
	. = ALIGN(64);
	*(.sram0.bss.uhc_dma)
	*(.sram0.bss.uhc_dma.*)
	. = ALIGN(64);
} GROUP_DATA_LINK_IN(SRAM0, SRAM0)
