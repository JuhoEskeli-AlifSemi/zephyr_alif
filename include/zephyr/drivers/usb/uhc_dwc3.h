/*
 * Copyright (c) 2025 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief DWC3 USB host controller extension helpers
 *
 * These functions are synchronous, polling helpers exported by the DWC3 UHC
 * driver (``zephyr/drivers/usb/uhc/uhc_dwc3.c``) in addition to the standard
 * @ref uhc_api. They drive enumeration and control/bulk/isochronous transfers
 * directly (bypassing the asynchronous ``uhc_ep_enqueue`` path) and are used by
 * the USB host samples and the ``usbh_msc`` class driver.
 *
 * They are specific to the DWC3 controller and require @kconfig{CONFIG_UHC_DWC3}.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_USB_UHC_DWC3_H_
#define ZEPHYR_INCLUDE_DRIVERS_USB_UHC_DWC3_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/usb/usb_ch9.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enumerate a device on the DWC3 xHCI host controller
 *
 * Performs port reset, Enable Slot, Address Device and reads the device
 * descriptor.
 *
 * @param dev  UHC device
 * @param desc Output: device descriptor read from the device
 * @return 0 on success, negative errno on failure
 */
int uhc_dwc3_enumerate_device(const struct device *dev,
			      struct usb_device_descriptor *desc);

/**
 * @brief Enumerate and fully configure a bulk device
 *
 * Enumerates the device, reads the configuration descriptor, issues
 * SET_CONFIGURATION and configures the first bulk IN/OUT endpoint pair in the
 * xHCI controller.
 *
 * @param dev             UHC device
 * @param desc            Output: device descriptor
 * @param out_bulk_in_ep  Output: bulk IN endpoint address
 * @param out_bulk_out_ep Output: bulk OUT endpoint address
 * @return 0 on success, negative errno on failure
 */
int uhc_dwc3_setup_device(const struct device *dev,
			  struct usb_device_descriptor *desc,
			  uint8_t *out_bulk_in_ep,
			  uint8_t *out_bulk_out_ep);

/**
 * @brief Perform an EP0 control transfer
 *
 * @param dev           UHC device
 * @param bmRequestType bmRequestType field
 * @param bRequest      bRequest field
 * @param wValue        wValue field
 * @param wIndex        wIndex field
 * @param wLength       wLength field / data buffer length
 * @param data          Data buffer (may be NULL when wLength is 0)
 * @return number of bytes transferred on success, negative errno on failure
 */
int uhc_dwc3_control_transfer(const struct device *dev,
			      uint8_t bmRequestType,
			      uint8_t bRequest,
			      uint16_t wValue,
			      uint16_t wIndex,
			      uint16_t wLength,
			      void *data);

/**
 * @brief Perform a bulk OUT transfer on the configured bulk OUT endpoint
 *
 * @param dev  UHC device
 * @param data Data to send
 * @param len  Number of bytes to send
 * @return number of bytes sent on success, negative errno on failure
 */
int uhc_dwc3_bulk_out(const struct device *dev,
		      const uint8_t *data, size_t len);

/**
 * @brief Perform a bulk IN transfer on the configured bulk IN endpoint
 *
 * @param dev  UHC device
 * @param data Destination buffer
 * @param len  Maximum number of bytes to receive
 * @return number of bytes received on success, negative errno on failure
 */
int uhc_dwc3_bulk_in(const struct device *dev,
		     uint8_t *data, size_t len);

/**
 * @brief Configure isochronous endpoints for audio streaming
 *
 * @param dev           UHC device
 * @param isoch_out_ep  Isochronous OUT endpoint address (0 to skip)
 * @param isoch_in_ep   Isochronous IN endpoint address (0 to skip)
 * @param out_mps       OUT endpoint max packet size
 * @param in_mps        IN endpoint max packet size
 * @return 0 on success, negative errno on failure
 */
int uhc_dwc3_configure_isoch(const struct device *dev,
			     uint8_t isoch_out_ep,
			     uint8_t isoch_in_ep,
			     uint16_t out_mps,
			     uint16_t in_mps);

/**
 * @brief Start isochronous streaming
 *
 * Resets the double-buffer / loopback streaming state. Call once before the
 * first isochronous transfer.
 *
 * @param dev             UHC device
 * @param out_frame_size  OUT frame size in bytes (currently informational)
 * @param in_frame_size   IN frame size in bytes (currently informational)
 * @return 0 on success, negative errno on failure
 */
int uhc_dwc3_isoch_start(const struct device *dev, size_t out_frame_size,
			 size_t in_frame_size);

/**
 * @brief Send one isochronous OUT (speaker) frame
 *
 * The first call primes the double buffer; subsequent calls pump one frame.
 *
 * @param dev  UHC device
 * @param data Frame data to send
 * @param len  Number of bytes to send
 * @return number of bytes sent on success, negative errno on failure
 */
int uhc_dwc3_isoch_out(const struct device *dev,
		       const uint8_t *data, size_t len);

/**
 * @brief Receive one isochronous IN (microphone) frame
 *
 * @param dev  UHC device
 * @param data Destination buffer
 * @param len  Maximum number of bytes to receive
 * @return number of bytes received on success, negative errno on failure
 */
int uhc_dwc3_isoch_in(const struct device *dev,
		      uint8_t *data, size_t len);

/**
 * @brief Combined isochronous OUT + IN frame transfer
 *
 * @param dev      UHC device
 * @param out_data OUT frame data to send
 * @param out_len  Number of OUT bytes
 * @param in_data  Destination buffer for the IN frame
 * @param in_len   Maximum number of IN bytes
 * @param out_ret  Output: OUT transfer result (may be NULL)
 * @param in_ret   Output: IN transfer result (may be NULL)
 * @return 0 on success, negative errno on a fatal failure
 */
int uhc_dwc3_isoch_stream(const struct device *dev,
			  const uint8_t *out_data, size_t out_len,
			  uint8_t *in_data, size_t in_len,
			  int *out_ret, int *in_ret);

/**
 * @brief Pump one microphone IN -> speaker OUT loopback frame
 *
 * @param dev UHC device
 * @return 0 on success, negative errno on failure
 */
int uhc_dwc3_isoch_loopback(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_USB_UHC_DWC3_H_ */
