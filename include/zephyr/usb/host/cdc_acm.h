/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Host CDC-ACM class driver
 *
 * This implements a USB host-side CDC-ACM class driver that can communicate
 * with USB devices presenting a CDC-ACM interface (such as USB modems,
 * serial adapters, etc.).
 *
 * The driver registers with the USBH class framework and provides a
 * simple API for sending/receiving data over CDC-ACM bulk endpoints.
 */

#ifndef ZEPHYR_INCLUDE_USB_HOST_CDC_ACM_H_
#define ZEPHYR_INCLUDE_USB_HOST_CDC_ACM_H_

#include <zephyr/usb/usbh.h>
#include <zephyr/drivers/usb/uhc.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CDC class-specific request codes */
#define CDC_REQ_SET_LINE_CODING		0x20
#define CDC_REQ_GET_LINE_CODING		0x21
#define CDC_REQ_SET_CONTROL_LINE_STATE	0x22

/* CDC ACM subclass */
#define CDC_ACM_SUBCLASS		0x02
/* AT command protocol */
#define CDC_AT_COMMAND_PROTOCOL		0x01

/**
 * @brief CDC Line Coding structure
 */
struct cdc_acm_line_coding {
	uint32_t dwDTERate;    /* Data terminal rate, in bits per second */
	uint8_t  bCharFormat;  /* Stop bits: 0=1, 1=1.5, 2=2 */
	uint8_t  bParityType;  /* Parity: 0=None, 1=Odd, 2=Even, 3=Mark, 4=Space */
	uint8_t  bDataBits;    /* Data bits: 5, 6, 7, 8 or 16 */
} __packed;

/**
 * @brief CDC-ACM host instance data
 */
struct usbh_cdc_acm_data {
	/** USBH context this instance belongs to */
	struct usbh_contex *ctx;
	/** USB device this instance is bound to */
	struct usb_device *udev;
	/** Bulk IN endpoint address */
	uint8_t bulk_in_ep;
	/** Bulk OUT endpoint address */
	uint8_t bulk_out_ep;
	/** Interrupt IN endpoint address (for notifications) */
	uint8_t int_in_ep;
	/** CDC communication interface number */
	uint8_t comm_iface;
	/** CDC data interface number */
	uint8_t data_iface;
	/** Current line coding */
	struct cdc_acm_line_coding line_coding;
	/** Device is ready for data transfer */
	bool ready;
	/** Receive callback */
	void (*rx_cb)(const uint8_t *data, size_t len, void *user_data);
	/** Receive callback user data */
	void *rx_user_data;
};

/**
 * @brief Initialize the CDC-ACM host class driver
 *
 * @param data Pointer to CDC-ACM instance data
 * @param ctx  Pointer to USBH context
 * @return 0 on success, negative error code on failure
 */
int usbh_cdc_acm_init(struct usbh_cdc_acm_data *data,
		       struct usbh_contex *ctx);

/**
 * @brief Probe a USB device for CDC-ACM interfaces
 *
 * Scans the device's configuration descriptor for CDC-ACM interfaces
 * and sets up the bulk endpoints for communication.
 *
 * @param data Pointer to CDC-ACM instance data
 * @param udev Pointer to the USB device
 * @return 0 on success, negative error code on failure
 */
int usbh_cdc_acm_probe(struct usbh_cdc_acm_data *data,
			struct usb_device *udev);

/**
 * @brief Set line coding (baud rate, data bits, parity, stop bits)
 *
 * @param data   Pointer to CDC-ACM instance data
 * @param coding Pointer to line coding structure
 * @return 0 on success, negative error code on failure
 */
int usbh_cdc_acm_set_line_coding(struct usbh_cdc_acm_data *data,
				  const struct cdc_acm_line_coding *coding);

/**
 * @brief Set control line state (DTR, RTS)
 *
 * @param data Pointer to CDC-ACM instance data
 * @param dtr  DTR state (true = asserted)
 * @param rts  RTS state (true = asserted)
 * @return 0 on success, negative error code on failure
 */
int usbh_cdc_acm_set_control_line_state(struct usbh_cdc_acm_data *data,
					  bool dtr, bool rts);

/**
 * @brief Send data over CDC-ACM bulk OUT endpoint
 *
 * @param data    Pointer to CDC-ACM instance data
 * @param buf     Data buffer to send
 * @param len     Length of data to send
 * @return number of bytes sent, or negative error code
 */
int usbh_cdc_acm_send(struct usbh_cdc_acm_data *data,
		       const uint8_t *buf, size_t len);

/**
 * @brief Register a receive callback
 *
 * @param data      Pointer to CDC-ACM instance data
 * @param cb        Callback function
 * @param user_data User data passed to callback
 */
void usbh_cdc_acm_set_rx_callback(struct usbh_cdc_acm_data *data,
				   void (*cb)(const uint8_t *data, size_t len,
					      void *user_data),
				   void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USB_HOST_CDC_ACM_H_ */
