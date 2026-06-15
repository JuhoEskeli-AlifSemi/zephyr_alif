/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Host CDC-ACM class driver implementation
 *
 * Implements the host-side CDC-ACM class driver using the USBH framework.
 * When a device with a CDC-ACM interface is connected, this driver
 * sets up bulk IN/OUT endpoints for data transfer and provides APIs
 * for serial communication.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/host/cdc_acm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbh_cdc_acm, CONFIG_USBH_LOG_LEVEL);

/* CDC descriptor subtypes */
#define CDC_DESC_TYPE_CS_INTERFACE	0x24
#define CDC_DESC_SUBTYPE_HEADER		0x00
#define CDC_DESC_SUBTYPE_CALL_MGMT	0x01
#define CDC_DESC_SUBTYPE_ACM		0x02
#define CDC_DESC_SUBTYPE_UNION		0x06

/* USB request type fields */
#define USB_REQTYPE_DIR_H2D		0x00
#define USB_REQTYPE_DIR_D2H		0x80
#define USB_REQTYPE_TYPE_CLASS		0x20
#define USB_REQTYPE_RCPT_INTERFACE	0x01

int usbh_cdc_acm_init(struct usbh_cdc_acm_data *data,
		       struct usbh_contex *ctx)
{
	if (data == NULL || ctx == NULL) {
		return -EINVAL;
	}

	memset(data, 0, sizeof(*data));
	data->ctx = ctx;

	/* Default line coding: 115200 8N1 */
	data->line_coding.dwDTERate = 115200;
	data->line_coding.bCharFormat = 0;  /* 1 stop bit */
	data->line_coding.bParityType = 0;  /* No parity */
	data->line_coding.bDataBits = 8;

	return 0;
}

/**
 * Scan the configuration descriptor for CDC-ACM interfaces and extract
 * the bulk IN, bulk OUT, and interrupt IN endpoint addresses.
 */
static int cdc_acm_parse_config(struct usbh_cdc_acm_data *data,
				struct usb_device *udev)
{
	struct usb_cfg_descriptor *cfg = udev->cfg_desc;
	struct usb_desc_header *dhp;
	struct usb_if_descriptor *if_desc = NULL;
	struct usb_ep_descriptor *ep_desc;
	void *desc_end;
	bool found_comm = false;
	bool found_data = false;

	if (cfg == NULL) {
		LOG_ERR("No configuration descriptor");
		return -ENODATA;
	}

	dhp = (void *)((uint8_t *)cfg + cfg->bLength);
	desc_end = (void *)((uint8_t *)cfg + cfg->wTotalLength);

	while ((void *)dhp < desc_end && dhp->bLength > 0) {
		if (dhp->bDescriptorType == USB_DESC_INTERFACE) {
			if_desc = (struct usb_if_descriptor *)dhp;

			if (if_desc->bInterfaceClass == USB_BCC_CDC_CONTROL &&
			    if_desc->bInterfaceSubClass == CDC_ACM_SUBCLASS) {
				data->comm_iface = if_desc->bInterfaceNumber;
				found_comm = true;
				LOG_DBG("Found CDC Communication interface %u",
					data->comm_iface);
			} else if (if_desc->bInterfaceClass == USB_BCC_CDC_DATA) {
				data->data_iface = if_desc->bInterfaceNumber;
				found_data = true;
				LOG_DBG("Found CDC Data interface %u",
					data->data_iface);
			}
		}

		if (dhp->bDescriptorType == USB_DESC_ENDPOINT && if_desc != NULL) {
			ep_desc = (struct usb_ep_descriptor *)dhp;

			if (if_desc->bInterfaceClass == USB_BCC_CDC_CONTROL) {
				/* Interrupt IN endpoint for notifications */
				if (USB_EP_DIR_IS_IN(ep_desc->bEndpointAddress) &&
				    (ep_desc->bmAttributes & 0x03) == 0x03) {
					data->int_in_ep = ep_desc->bEndpointAddress;
					LOG_DBG("CDC interrupt IN ep: 0x%02x",
						data->int_in_ep);
				}
			} else if (if_desc->bInterfaceClass == USB_BCC_CDC_DATA) {
				/* Bulk endpoints for data */
				if (USB_EP_DIR_IS_IN(ep_desc->bEndpointAddress) &&
				    (ep_desc->bmAttributes & 0x03) == 0x02) {
					data->bulk_in_ep = ep_desc->bEndpointAddress;
					LOG_DBG("CDC bulk IN ep: 0x%02x",
						data->bulk_in_ep);
				} else if (!USB_EP_DIR_IS_IN(ep_desc->bEndpointAddress) &&
					   (ep_desc->bmAttributes & 0x03) == 0x02) {
					data->bulk_out_ep = ep_desc->bEndpointAddress;
					LOG_DBG("CDC bulk OUT ep: 0x%02x",
						data->bulk_out_ep);
				}
			}
		}

		dhp = (void *)((uint8_t *)dhp + dhp->bLength);
	}

	if (!found_comm || !found_data) {
		LOG_WRN("CDC-ACM interface not found (comm=%d data=%d)",
			found_comm, found_data);
		return -ENOTSUP;
	}

	if (data->bulk_in_ep == 0 || data->bulk_out_ep == 0) {
		LOG_ERR("CDC-ACM bulk endpoints not found");
		return -ENOTSUP;
	}

	return 0;
}

static int xfer_sync_cb(struct usb_device *const udev,
			struct uhc_transfer *const xfer)
{
	struct k_sem *sync_sem = xfer->priv;

	if (sync_sem) {
		k_sem_give(sync_sem);
	}

	return 0;
}

int usbh_cdc_acm_probe(struct usbh_cdc_acm_data *data,
			struct usb_device *udev)
{
	int err;

	if (data == NULL || udev == NULL) {
		return -EINVAL;
	}

	data->udev = udev;

	/* First, make sure a configuration is set */
	if (udev->state != USB_STATE_CONFIGURED) {
		err = usbh_device_set_configuration(udev, 1);
		if (err) {
			LOG_ERR("Failed to set configuration: %d", err);
			return err;
		}
	}

	/* Parse the configuration descriptor for CDC-ACM interfaces */
	err = cdc_acm_parse_config(data, udev);
	if (err) {
		return err;
	}

	/* Set default line coding */
	err = usbh_cdc_acm_set_line_coding(data, &data->line_coding);
	if (err) {
		LOG_WRN("Set line coding failed: %d (non-fatal)", err);
	}

	/* Assert DTR and RTS */
	err = usbh_cdc_acm_set_control_line_state(data, true, true);
	if (err) {
		LOG_WRN("Set control line state failed: %d (non-fatal)", err);
	}

	data->ready = true;
	LOG_INF("CDC-ACM device probed: bulk_in=0x%02x bulk_out=0x%02x",
		data->bulk_in_ep, data->bulk_out_ep);

	return 0;
}

int usbh_cdc_acm_set_line_coding(struct usbh_cdc_acm_data *data,
				  const struct cdc_acm_line_coding *coding)
{
	struct usbh_contex *ctx = data->ctx;
	const struct device *uhc_dev = ctx->dev;
	struct uhc_transfer *xfer;
	struct k_sem sync_sem;
	int err;

	k_sem_init(&sync_sem, 0, 1);

	xfer = uhc_xfer_alloc_with_buf(uhc_dev, USB_CONTROL_EP_OUT,
				       data->udev, xfer_sync_cb,
				       &sync_sem,
				       sizeof(struct cdc_acm_line_coding));
	if (xfer == NULL) {
		return -ENOMEM;
	}

	/* Build setup packet for SET_LINE_CODING */
	struct usb_setup_packet *setup = (struct usb_setup_packet *)xfer->setup_pkt;

	setup->bmRequestType = USB_REQTYPE_DIR_H2D | USB_REQTYPE_TYPE_CLASS |
			       USB_REQTYPE_RCPT_INTERFACE;
	setup->bRequest = CDC_REQ_SET_LINE_CODING;
	setup->wValue = 0;
	setup->wIndex = sys_cpu_to_le16(data->comm_iface);
	setup->wLength = sys_cpu_to_le16(sizeof(struct cdc_acm_line_coding));

	/* Copy line coding data into transfer buffer */
	net_buf_add_mem(xfer->buf, coding, sizeof(struct cdc_acm_line_coding));

	err = uhc_ep_enqueue(uhc_dev, xfer);
	if (err) {
		uhc_xfer_free(uhc_dev, xfer);
		return err;
	}

	/* Wait for completion */
	k_sem_take(&sync_sem, K_MSEC(5000));

	err = xfer->err;
	if (err == 0) {
		memcpy(&data->line_coding, coding, sizeof(data->line_coding));
	}

	uhc_xfer_free(uhc_dev, xfer);

	return err;
}

int usbh_cdc_acm_set_control_line_state(struct usbh_cdc_acm_data *data,
					  bool dtr, bool rts)
{
	struct usbh_contex *ctx = data->ctx;
	const struct device *uhc_dev = ctx->dev;
	struct uhc_transfer *xfer;
	struct k_sem sync_sem;
	uint16_t line_state = 0;
	int err;

	if (dtr) {
		line_state |= BIT(0);
	}
	if (rts) {
		line_state |= BIT(1);
	}

	k_sem_init(&sync_sem, 0, 1);

	xfer = uhc_xfer_alloc(uhc_dev, USB_CONTROL_EP_OUT,
			      data->udev, xfer_sync_cb, &sync_sem);
	if (xfer == NULL) {
		return -ENOMEM;
	}

	struct usb_setup_packet *setup = (struct usb_setup_packet *)xfer->setup_pkt;

	setup->bmRequestType = USB_REQTYPE_DIR_H2D | USB_REQTYPE_TYPE_CLASS |
			       USB_REQTYPE_RCPT_INTERFACE;
	setup->bRequest = CDC_REQ_SET_CONTROL_LINE_STATE;
	setup->wValue = sys_cpu_to_le16(line_state);
	setup->wIndex = sys_cpu_to_le16(data->comm_iface);
	setup->wLength = 0;

	err = uhc_ep_enqueue(uhc_dev, xfer);
	if (err) {
		uhc_xfer_free(uhc_dev, xfer);
		return err;
	}

	k_sem_take(&sync_sem, K_MSEC(5000));
	err = xfer->err;
	uhc_xfer_free(uhc_dev, xfer);

	return err;
}

int usbh_cdc_acm_send(struct usbh_cdc_acm_data *data,
		       const uint8_t *buf, size_t len)
{
	struct usbh_contex *ctx = data->ctx;
	const struct device *uhc_dev = ctx->dev;
	struct uhc_transfer *xfer;
	struct k_sem sync_sem;
	int err;

	if (!data->ready) {
		return -ENODEV;
	}

	if (len == 0) {
		return 0;
	}

	k_sem_init(&sync_sem, 0, 1);

	xfer = uhc_xfer_alloc_with_buf(uhc_dev, data->bulk_out_ep,
				       data->udev, xfer_sync_cb,
				       &sync_sem, len);
	if (xfer == NULL) {
		return -ENOMEM;
	}

	net_buf_add_mem(xfer->buf, buf, len);

	err = uhc_ep_enqueue(uhc_dev, xfer);
	if (err) {
		uhc_xfer_free(uhc_dev, xfer);
		return err;
	}

	k_sem_take(&sync_sem, K_MSEC(5000));
	err = xfer->err;
	uhc_xfer_free(uhc_dev, xfer);

	return err ? err : (int)len;
}

void usbh_cdc_acm_set_rx_callback(struct usbh_cdc_acm_data *data,
				   void (*cb)(const uint8_t *buf, size_t len,
					      void *user_data),
				   void *user_data)
{
	data->rx_cb = cb;
	data->rx_user_data = user_data;
}

/*
 * USBH class driver callbacks - registered via USBH_DEFINE_CLASS
 */

static int cdc_acm_class_request(struct usbh_contex *const uhs_ctx,
				 struct uhc_transfer *const xfer, int err)
{
	LOG_DBG("CDC-ACM class request completed, err=%d", err);
	return 0;
}

static int cdc_acm_class_connected(struct usbh_contex *const uhs_ctx)
{
	LOG_INF("CDC-ACM: device connected");
	return 0;
}

static int cdc_acm_class_removed(struct usbh_contex *const uhs_ctx)
{
	LOG_INF("CDC-ACM: device removed");
	return 0;
}

static int cdc_acm_class_rwup(struct usbh_contex *const uhs_ctx)
{
	return 0;
}

static int cdc_acm_class_suspended(struct usbh_contex *const uhs_ctx)
{
	return 0;
}

static int cdc_acm_class_resumed(struct usbh_contex *const uhs_ctx)
{
	return 0;
}

USBH_DEFINE_CLASS(cdc_acm_class) = {
	.code = {
		.dclass = USB_BCC_CDC_CONTROL,
		.sub = CDC_ACM_SUBCLASS,
		.proto = 0,
	},
	.request = cdc_acm_class_request,
	.connected = cdc_acm_class_connected,
	.removed = cdc_acm_class_removed,
	.rwup = cdc_acm_class_rwup,
	.suspended = cdc_acm_class_suspended,
	.resumed = cdc_acm_class_resumed,
};
