/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Host Mass Storage (Bulk-Only Transport) class driver
 *
 * Implements the host side of the USB Mass Storage class for devices using the
 * Bulk-Only Transport (BOT) protocol with the SCSI transparent command set
 * (interface class 0x08, subclass 0x06, protocol 0x50) - ordinary USB flash
 * drives.
 *
 * Every SCSI command is framed by BOT: the host sends a 31-byte Command Block
 * Wrapper (CBW) on the bulk OUT endpoint, an optional data phase runs, then the
 * host reads a 13-byte Command Status Wrapper (CSW) on the bulk IN endpoint.
 * Each logical sector access is mapped to one SCSI READ(10)/WRITE(10) so that
 * a block filesystem (e.g. FAT) can be mounted through the disk-access
 * subsystem.
 *
 * Note: transfers currently run through the DWC3 host controller's synchronous
 * enumeration/transfer helpers (hence the Kconfig dependency on UHC_DWC3).
 * These are exported by the controller driver but not via a public header, so
 * they are declared extern below.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/host/msc.h>
#include <zephyr/drivers/usb/uhc_dwc3.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbh_msc, CONFIG_USBH_LOG_LEVEL);

/* ---- USB Mass Storage class (BOT + SCSI transparent) ---- */
#define MSC_IF_CLASS		0x08
#define MSC_IF_SUBCLASS_SCSI	0x06
#define MSC_IF_PROTO_BOT	0x50

/* Class-specific request: GET MAX LUN (device-to-host, interface recipient) */
#define MSC_REQ_GET_MAX_LUN	0xFE

/* ---- Bulk-Only Transport wrappers ---- */
#define CBW_SIGNATURE		0x43425355  /* "USBC" (LE) */
#define CSW_SIGNATURE		0x53425355  /* "USBS" (LE) */
#define CBW_FLAG_DATA_IN	0x80        /* bmCBWFlags: device-to-host */

#define CSW_STATUS_PASSED	0x00

struct msc_cbw {
	uint32_t dCBWSignature;
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t  bmCBWFlags;
	uint8_t  bCBWLUN;        /* bits [3:0] */
	uint8_t  bCBWCBLength;   /* bits [4:0], 1..16 */
	uint8_t  CBWCB[16];
} __packed;

struct msc_csw {
	uint32_t dCSWSignature;
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t  bCSWStatus;
} __packed;

BUILD_ASSERT(sizeof(struct msc_cbw) == 31, "CBW must be 31 bytes");
BUILD_ASSERT(sizeof(struct msc_csw) == 13, "CSW must be 13 bytes");

/* ---- SCSI opcodes ---- */
#define SCSI_TEST_UNIT_READY	0x00
#define SCSI_REQUEST_SENSE	0x03
#define SCSI_INQUIRY		0x12
#define SCSI_READ_CAPACITY_10	0x25
#define SCSI_READ_10		0x28
#define SCSI_WRITE_10		0x2A

/* TEST UNIT READY polling: the medium may need a moment to become ready */
#define MSC_TUR_RETRIES		20
#define MSC_TUR_INTERVAL_MS	200

/*
 * Configuration descriptor scratch buffer used only during probe to find the
 * mass storage interface. The driver is currently single-instance (one medium
 * at a time), matching the polled sample model, so a shared buffer is safe.
 */
static uint8_t cfg_desc_buf[256];

/*
 * Run one SCSI command over Bulk-Only Transport:
 *   CBW (out) -> optional data phase (in/out) -> CSW (in).
 *
 * The caller's data buffer need not be DMA-accessible: the controller's bulk
 * helpers bounce it through their own SRAM buffers.
 *
 * Returns the number of data bytes transferred on success, or a negative
 * errno. A non-zero SCSI status in the CSW is reported as -EIO.
 */
static int scsi_transfer(struct usbh_msc_dev *msc,
			 const uint8_t *cb, uint8_t cb_len,
			 bool data_in, uint8_t *data, uint32_t data_len)
{
	struct msc_cbw cbw = {0};
	struct msc_csw csw = {0};
	uint32_t tag = ++msc->tag;
	int ret;

	if (cb_len == 0 || cb_len > sizeof(cbw.CBWCB)) {
		return -EINVAL;
	}

	cbw.dCBWSignature = sys_cpu_to_le32(CBW_SIGNATURE);
	cbw.dCBWTag = sys_cpu_to_le32(tag);
	cbw.dCBWDataTransferLength = sys_cpu_to_le32(data_len);
	cbw.bmCBWFlags = (data_len && data_in) ? CBW_FLAG_DATA_IN : 0;
	cbw.bCBWLUN = msc->lun & 0x0F;
	cbw.bCBWCBLength = cb_len & 0x1F;
	memcpy(cbw.CBWCB, cb, cb_len);

	/* Command phase: send the 31-byte CBW */
	ret = uhc_dwc3_bulk_out(msc->uhc, (const uint8_t *)&cbw, sizeof(cbw));
	if (ret < 0) {
		LOG_ERR("CBW send failed (cmd 0x%02x): %d", cb[0], ret);
		return ret;
	}

	/* Data phase (optional) */
	if (data_len > 0 && data != NULL) {
		if (data_in) {
			ret = uhc_dwc3_bulk_in(msc->uhc, data, data_len);
		} else {
			ret = uhc_dwc3_bulk_out(msc->uhc, data, data_len);
		}
		if (ret < 0) {
			LOG_ERR("Data phase failed (cmd 0x%02x): %d",
				cb[0], ret);
			return ret;
		}
	}

	/* Status phase: read the 13-byte CSW */
	ret = uhc_dwc3_bulk_in(msc->uhc, (uint8_t *)&csw, sizeof(csw));
	if (ret < 0) {
		LOG_ERR("CSW read failed (cmd 0x%02x): %d", cb[0], ret);
		return ret;
	}

	if (sys_le32_to_cpu(csw.dCSWSignature) != CSW_SIGNATURE) {
		LOG_ERR("Bad CSW signature 0x%08x (cmd 0x%02x)",
			sys_le32_to_cpu(csw.dCSWSignature), cb[0]);
		return -EPROTO;
	}
	if (sys_le32_to_cpu(csw.dCSWTag) != tag) {
		LOG_ERR("CSW tag mismatch: got %u expected %u",
			sys_le32_to_cpu(csw.dCSWTag), tag);
		return -EPROTO;
	}
	if (csw.bCSWStatus != CSW_STATUS_PASSED) {
		LOG_WRN("SCSI cmd 0x%02x status=%u residue=%u",
			cb[0], csw.bCSWStatus,
			sys_le32_to_cpu(csw.dCSWDataResidue));
		return -EIO;
	}

	return (int)(data_len - sys_le32_to_cpu(csw.dCSWDataResidue));
}

static int scsi_test_unit_ready(struct usbh_msc_dev *msc)
{
	uint8_t cb[6] = { SCSI_TEST_UNIT_READY, 0, 0, 0, 0, 0 };

	return scsi_transfer(msc, cb, sizeof(cb), false, NULL, 0);
}

static int scsi_request_sense(struct usbh_msc_dev *msc, uint8_t sense[18])
{
	uint8_t cb[6] = { SCSI_REQUEST_SENSE, 0, 0, 0, 18, 0 };

	return scsi_transfer(msc, cb, sizeof(cb), true, sense, 18);
}

static int scsi_inquiry(struct usbh_msc_dev *msc, uint8_t inq[36])
{
	uint8_t cb[6] = { SCSI_INQUIRY, 0, 0, 0, 36, 0 };

	return scsi_transfer(msc, cb, sizeof(cb), true, inq, 36);
}

static int scsi_read_capacity(struct usbh_msc_dev *msc,
			      uint32_t *last_lba, uint32_t *block_size)
{
	uint8_t cb[10] = { SCSI_READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t data[8];
	int ret;

	ret = scsi_transfer(msc, cb, sizeof(cb), true, data, sizeof(data));
	if (ret < 0) {
		return ret;
	}
	if (ret < 8) {
		return -EIO;
	}

	/* READ CAPACITY(10) returns big-endian: last LBA then block length */
	*last_lba = sys_get_be32(&data[0]);
	*block_size = sys_get_be32(&data[4]);
	return 0;
}

static int scsi_read10(struct usbh_msc_dev *msc, uint32_t lba, uint8_t *data)
{
	uint8_t cb[10] = {0};

	cb[0] = SCSI_READ_10;
	sys_put_be32(lba, &cb[2]);
	sys_put_be16(1, &cb[7]);

	return scsi_transfer(msc, cb, sizeof(cb), true, data, msc->block_size);
}

static int scsi_write10(struct usbh_msc_dev *msc, uint32_t lba,
			const uint8_t *data)
{
	uint8_t cb[10] = {0};

	cb[0] = SCSI_WRITE_10;
	sys_put_be32(lba, &cb[2]);
	sys_put_be16(1, &cb[7]);

	/* Data-out phase: scsi_transfer only reads from the buffer here. */
	return scsi_transfer(msc, cb, sizeof(cb), false, (uint8_t *)data,
			     msc->block_size);
}

/* ---- Public block access ---- */

int usbh_msc_read(struct usbh_msc_dev *msc, uint32_t lba, uint16_t count,
		  uint8_t *buf)
{
	if (msc == NULL || buf == NULL || msc->block_size == 0) {
		return -EINVAL;
	}

	for (uint16_t i = 0; i < count; i++) {
		int ret = scsi_read10(msc, lba + i, buf + i * msc->block_size);

		if (ret < 0) {
			LOG_ERR("read LBA %u failed: %d", lba + i, ret);
			return ret;
		}
	}

	return (int)((uint32_t)count * msc->block_size);
}

int usbh_msc_write(struct usbh_msc_dev *msc, uint32_t lba, uint16_t count,
		   const uint8_t *buf)
{
	if (msc == NULL || buf == NULL || msc->block_size == 0) {
		return -EINVAL;
	}

	for (uint16_t i = 0; i < count; i++) {
		int ret = scsi_write10(msc, lba + i, buf + i * msc->block_size);

		if (ret < 0) {
			LOG_ERR("write LBA %u failed: %d", lba + i, ret);
			return ret;
		}
	}

	return (int)((uint32_t)count * msc->block_size);
}

/* ---- Disk-access driver backing the filesystem ----
 *
 * FatFs (and other block filesystems) talk to storage through the disk-access
 * subsystem in units of logical sectors. The ops recover the owning instance
 * from the embedded disk_info via CONTAINER_OF.
 */
static inline struct usbh_msc_dev *disk_to_msc(struct disk_info *disk)
{
	return CONTAINER_OF(disk, struct usbh_msc_dev, disk);
}

static int usbh_msc_disk_status(struct disk_info *disk)
{
	struct usbh_msc_dev *msc = disk_to_msc(disk);

	return (msc->uhc && msc->block_size) ? DISK_STATUS_OK
					     : DISK_STATUS_UNINIT;
}

static int usbh_msc_disk_read(struct disk_info *disk, uint8_t *buff,
			      uint32_t sector, uint32_t count)
{
	struct usbh_msc_dev *msc = disk_to_msc(disk);
	int ret = usbh_msc_read(msc, sector, count, buff);

	return (ret < 0) ? ret : 0;
}

static int usbh_msc_disk_write(struct disk_info *disk, const uint8_t *buff,
			       uint32_t sector, uint32_t count)
{
	struct usbh_msc_dev *msc = disk_to_msc(disk);
	int ret = usbh_msc_write(msc, sector, count, buff);

	return (ret < 0) ? ret : 0;
}

static int usbh_msc_disk_ioctl(struct disk_info *disk, uint8_t cmd, void *buff)
{
	struct usbh_msc_dev *msc = disk_to_msc(disk);

	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(uint32_t *)buff = msc->block_count;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(uint32_t *)buff = msc->block_size;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		*(uint32_t *)buff = 1U;
		break;
	case DISK_IOCTL_CTRL_SYNC:
	case DISK_IOCTL_CTRL_INIT:
	case DISK_IOCTL_CTRL_DEINIT:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int usbh_msc_disk_init(struct disk_info *disk)
{
	ARG_UNUSED(disk);
	return 0;
}

static const struct disk_operations usbh_msc_disk_ops = {
	.init = usbh_msc_disk_init,
	.status = usbh_msc_disk_status,
	.read = usbh_msc_disk_read,
	.write = usbh_msc_disk_write,
	.ioctl = usbh_msc_disk_ioctl,
};

/* ---- Discovery helpers ---- */

/* Class request: GET MAX LUN. Devices that do not support it STALL, which the
 * BOT spec says should be treated as a single-LUN device.
 */
static int msc_get_max_lun(struct usbh_msc_dev *msc)
{
	uint8_t val = 0;
	int ret;

	ret = uhc_dwc3_control_transfer(msc->uhc, 0xA1, MSC_REQ_GET_MAX_LUN,
					0, msc->iface, 1, &val);
	if (ret < 0) {
		return ret;
	}

	msc->max_lun = val;
	return 0;
}

/*
 * Read the configuration descriptor and locate the Bulk-Only / SCSI mass
 * storage interface, storing its number in msc->iface.
 */
static int msc_find_interface(struct usbh_msc_dev *msc)
{
	uint8_t hdr[9];
	uint16_t total;
	int ret;
	int off;

	/* 9-byte configuration descriptor header first (wTotalLength) */
	ret = uhc_dwc3_control_transfer(msc->uhc, 0x80, USB_SREQ_GET_DESCRIPTOR,
					(USB_DESC_CONFIGURATION << 8), 0,
					sizeof(hdr), hdr);
	if (ret < 0) {
		return ret;
	}

	total = sys_get_le16(&hdr[2]);
	if (total > sizeof(cfg_desc_buf)) {
		total = sizeof(cfg_desc_buf);
	}

	ret = uhc_dwc3_control_transfer(msc->uhc, 0x80, USB_SREQ_GET_DESCRIPTOR,
					(USB_DESC_CONFIGURATION << 8), 0,
					total, cfg_desc_buf);
	if (ret < 0) {
		return ret;
	}

	off = 0;
	while (off + 2 <= ret) {
		uint8_t len = cfg_desc_buf[off];
		uint8_t type = cfg_desc_buf[off + 1];

		if (len == 0) {
			break;
		}

		if (type == USB_DESC_INTERFACE && (off + 9) <= ret) {
			uint8_t if_num = cfg_desc_buf[off + 2];
			uint8_t if_class = cfg_desc_buf[off + 5];
			uint8_t if_sub = cfg_desc_buf[off + 6];
			uint8_t if_proto = cfg_desc_buf[off + 7];

			if (if_class == MSC_IF_CLASS &&
			    if_sub == MSC_IF_SUBCLASS_SCSI &&
			    if_proto == MSC_IF_PROTO_BOT) {
				msc->iface = if_num;
				return 0;
			}
		}

		off += len;
	}

	return -ENOTSUP;
}

/* ---- Public API ---- */

int usbh_msc_probe(const struct device *uhc, struct usbh_msc_dev *msc)
{
	struct usb_device_descriptor desc;
	uint8_t bulk_in_ep, bulk_out_ep;
	uint8_t inq[36];
	uint32_t last_lba = 0;
	uint32_t block_size = 0;
	bool ready = false;
	int ret;

	if (uhc == NULL || msc == NULL) {
		return -EINVAL;
	}

	memset(msc, 0, sizeof(*msc));
	msc->uhc = uhc;

	/* Enumerate: reset, address, read descriptors, set config, configure
	 * the bulk IN/OUT endpoints (all handled by the controller driver).
	 */
	ret = uhc_dwc3_setup_device(uhc, &desc, &bulk_in_ep, &bulk_out_ep);
	if (ret) {
		LOG_ERR("device setup failed: %d", ret);
		return ret;
	}

	msc->vid = sys_le16_to_cpu(desc.idVendor);
	msc->pid = sys_le16_to_cpu(desc.idProduct);
	LOG_DBG("enumerated VID=0x%04x PID=0x%04x bulk_in=0x%02x bulk_out=0x%02x",
		msc->vid, msc->pid, bulk_in_ep, bulk_out_ep);

	ret = msc_find_interface(msc);
	if (ret) {
		LOG_ERR("no Bulk-Only SCSI mass storage interface: %d", ret);
		return ret;
	}
	LOG_DBG("mass storage interface %u", msc->iface);

	if (msc_get_max_lun(msc) != 0) {
		LOG_DBG("GET MAX LUN unsupported, assuming a single LUN");
		msc->max_lun = 0;
	}
	msc->lun = 0;

	/* TEST UNIT READY: wait for the medium to spin up / settle */
	for (int i = 0; i < MSC_TUR_RETRIES; i++) {
		if (scsi_test_unit_ready(msc) == 0) {
			ready = true;
			break;
		}

		uint8_t sense[18];

		if (scsi_request_sense(msc, sense) == 0) {
			LOG_DBG("not ready (key=0x%01x ASC=0x%02x ASCQ=0x%02x), retry %d/%d",
				sense[2] & 0x0F, sense[12], sense[13],
				i + 1, MSC_TUR_RETRIES);
		}
		k_sleep(K_MSEC(MSC_TUR_INTERVAL_MS));
	}
	if (!ready) {
		LOG_ERR("unit did not become ready");
		return -EIO;
	}

	/* INQUIRY: vendor / product / revision identification */
	ret = scsi_inquiry(msc, inq);
	if (ret < 0) {
		LOG_ERR("INQUIRY failed: %d", ret);
		return ret;
	}
	memcpy(msc->vendor, &inq[8], 8);
	msc->vendor[8] = '\0';
	memcpy(msc->product, &inq[16], 16);
	msc->product[16] = '\0';
	memcpy(msc->revision, &inq[32], 4);
	msc->revision[4] = '\0';

	/* READ CAPACITY(10): block count and block size */
	ret = scsi_read_capacity(msc, &last_lba, &block_size);
	if (ret < 0) {
		LOG_ERR("READ CAPACITY failed: %d", ret);
		return ret;
	}
	msc->block_size = block_size;
	msc->block_count = last_lba + 1;

	/* Register the medium with the disk-access subsystem */
	msc->disk.name = CONFIG_USBH_MSC_DISK_NAME;
	msc->disk.ops = &usbh_msc_disk_ops;
	ret = disk_access_register(&msc->disk);
	if (ret) {
		LOG_ERR("disk_access_register failed: %d", ret);
		return ret;
	}
	msc->registered = true;

	LOG_INF("MSC ready: '%s' '%s' rev '%s', %u blocks x %u B (disk '%s')",
		msc->vendor, msc->product, msc->revision,
		msc->block_count, msc->block_size, msc->disk.name);

	return 0;
}

int usbh_msc_remove(struct usbh_msc_dev *msc)
{
	int ret = 0;

	if (msc == NULL) {
		return -EINVAL;
	}

	if (msc->registered) {
		ret = disk_access_unregister(&msc->disk);
		msc->registered = false;
	}

	msc->uhc = NULL;
	msc->block_size = 0;
	msc->block_count = 0;

	return ret;
}
