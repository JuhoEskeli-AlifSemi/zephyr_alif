/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Host Mass Storage (Bulk-Only Transport) class driver
 *
 * Host-side USB Mass Storage class driver for devices using the Bulk-Only
 * Transport (BOT) protocol with the SCSI transparent command set (interface
 * class 0x08, subclass 0x06, protocol 0x50) - i.e. ordinary USB flash drives.
 *
 * The driver enumerates the device, runs the SCSI discovery sequence
 * (TEST UNIT READY / INQUIRY / READ CAPACITY) and registers a disk-access
 * device so a block filesystem such as FAT can be mounted on the medium.
 */

#ifndef ZEPHYR_INCLUDE_USB_HOST_MSC_H_
#define ZEPHYR_INCLUDE_USB_HOST_MSC_H_

#include <zephyr/device.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief USB host Mass Storage instance
 *
 * All fields are managed by the driver. Consumers may read the identity and
 * geometry fields after a successful @ref usbh_msc_probe.
 */
struct usbh_msc_dev {
	/** USB host controller this device is attached to */
	const struct device *uhc;
	/** Disk-access registration backing the logical unit */
	struct disk_info disk;
	/** Logical block size in bytes (e.g. 512) */
	uint32_t block_size;
	/** Number of logical blocks on the medium */
	uint32_t block_count;
	/** USB Vendor ID from the device descriptor */
	uint16_t vid;
	/** USB Product ID from the device descriptor */
	uint16_t pid;
	/** Mass Storage interface number */
	uint8_t iface;
	/** Highest logical unit number (0 => a single LUN) */
	uint8_t max_lun;
	/** Logical unit this instance talks to */
	uint8_t lun;
	/** SCSI INQUIRY vendor identification (NUL-terminated) */
	char vendor[9];
	/** SCSI INQUIRY product identification (NUL-terminated) */
	char product[17];
	/** SCSI INQUIRY product revision (NUL-terminated) */
	char revision[5];
	/** Monotonic Command Block Wrapper tag */
	uint32_t tag;
	/** Disk registered with the disk-access subsystem */
	bool registered;
};

/**
 * @brief Enumerate and set up a connected USB mass storage device
 *
 * Enumerates the device on @p uhc, locates its Bulk-Only / SCSI mass storage
 * interface, runs the SCSI discovery sequence and, on success, registers a
 * disk-access device named @kconfig{CONFIG_USBH_MSC_DISK_NAME}. After this
 * returns 0 a filesystem can be mounted on that disk.
 *
 * @param uhc USB host controller device
 * @param msc Instance to populate (zero-initialised by the driver)
 *
 * @retval 0 on success
 * @retval -EINVAL invalid arguments
 * @retval -ENOTSUP no Bulk-Only SCSI mass storage interface present
 * @retval -EIO the medium did not become ready or a SCSI command failed
 * @retval negative errno on other failures
 */
int usbh_msc_probe(const struct device *uhc, struct usbh_msc_dev *msc);

/**
 * @brief Tear down a USB mass storage device
 *
 * Unregisters the disk-access device. Call after the medium is removed.
 *
 * @param msc Instance previously set up with @ref usbh_msc_probe
 * @return 0 on success, negative errno on failure
 */
int usbh_msc_remove(struct usbh_msc_dev *msc);

/**
 * @brief Read logical blocks from the medium
 *
 * @param msc   Instance set up with @ref usbh_msc_probe
 * @param lba   First logical block address
 * @param count Number of blocks to read
 * @param buf   Destination buffer (count * block_size bytes)
 * @return number of bytes read on success, negative errno on failure
 */
int usbh_msc_read(struct usbh_msc_dev *msc, uint32_t lba, uint16_t count,
		  uint8_t *buf);

/**
 * @brief Write logical blocks to the medium
 *
 * @param msc   Instance set up with @ref usbh_msc_probe
 * @param lba   First logical block address
 * @param count Number of blocks to write
 * @param buf   Source buffer (count * block_size bytes)
 * @return number of bytes written on success, negative errno on failure
 */
int usbh_msc_write(struct usbh_msc_dev *msc, uint32_t lba, uint16_t count,
		   const uint8_t *buf);

/**
 * @brief Get the disk-access name backing the mass storage device
 *
 * Use this to build a filesystem mount point, e.g. "/<name>:".
 *
 * @param msc Instance set up with @ref usbh_msc_probe
 * @return disk name string
 */
static inline const char *usbh_msc_disk_name(const struct usbh_msc_dev *msc)
{
	return msc->disk.name;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USB_HOST_MSC_H_ */
