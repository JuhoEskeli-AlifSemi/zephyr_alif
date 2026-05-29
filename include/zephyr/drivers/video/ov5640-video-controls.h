/**
 * @file
 *
 * @brief Vendor-specific control IDs for the OV5640 camera sensor
 */

/*
 * Copyright (c) 2026 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_OV5640_VIDEO_CONTROLS_H_
#define __ZEPHYR_INCLUDE_DRIVERS_OV5640_VIDEO_CONTROLS_H_

#include <zephyr/drivers/video-controls.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Raw value of OV5640 register 0x302A (Chip Revision).
 *   Bit[7:4] — Process: 0xA = FSI, 0xB = BSI
 *   Bit[3:0] — Chip revision
 *
 * value points to a uint8_t.
 */
#define VIDEO_OV5640_CID_CHIP_REVISION		(VIDEO_CID_PRIVATE_BASE + 0x20)

#ifdef __cplusplus
}
#endif

#endif /* __ZEPHYR_INCLUDE_DRIVERS_OV5640_VIDEO_CONTROLS_H_ */
