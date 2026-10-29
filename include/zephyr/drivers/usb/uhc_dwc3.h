/*
 * Copyright (c) 2025 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief DWC3 USB host controller extension helpers
 *
 * These functions are synchronous, polling helpers exported by the DWC3 UHC
 * driver (``zephyr/drivers/usb/uhc/uhc_dwc3.c``). This xHCI host controller
 * does not implement the asynchronous @ref uhc_api transfer path
 * (``uhc_ep_enqueue`` / ``uhc_ep_dequeue`` return ``-ENOTSUP``); these helpers
 * are the transfer API for the controller instead. They drive enumeration and
 * control/bulk/isochronous transfers directly and are used by the USB host
 * samples and the ``usbh_msc`` class driver.
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
 * @brief Largest data stage uhc_dwc3_control_transfer() can move
 *
 * Big enough for the configuration descriptor of a typical webcam, which
 * lists every format and frame size.
 */
#define UHC_DWC3_CTRL_MAX_LEN 4096

/**
 * @brief Perform an EP0 control transfer
 *
 * @param dev           UHC device
 * @param bmRequestType bmRequestType field
 * @param bRequest      bRequest field
 * @param wValue        wValue field
 * @param wIndex        wIndex field
 * @param wLength       wLength field / data buffer length, at most
 *                      @ref UHC_DWC3_CTRL_MAX_LEN
 * @param data          Data buffer (may be NULL when wLength is 0)
 * @return number of bytes transferred on success, negative errno on failure
 * @retval -EINVAL wLength exceeds the driver's control transfer buffer
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
 * @brief Required alignment of buffers passed to uhc_dwc3_bulk_in_submit()
 *
 * Both the buffer address and its length must be a multiple of this, so
 * the cache maintenance on the buffer covers whole cache lines.
 */
#define UHC_DWC3_BUF_ALIGN 64

/**
 * @brief Configure a lone bulk IN endpoint
 *
 * For devices whose interface has a bulk IN endpoint without a bulk OUT
 * partner, such as a bulk USB Video Class streaming interface. Call after
 * uhc_dwc3_enumerate_device() and SET_CONFIGURATION. The endpoint becomes
 * the target of uhc_dwc3_bulk_in() and uhc_dwc3_bulk_in_submit(); any
 * transfers still queued from an earlier configuration are discarded.
 *
 * @param dev UHC device
 * @param ep  Bulk IN endpoint address (bit 7 set)
 * @param mps Endpoint max packet size (wMaxPacketSize)
 * @return 0 on success, negative errno on failure
 */
int uhc_dwc3_configure_bulk_in(const struct device *dev, uint8_t ep,
			       uint16_t mps);

/**
 * @brief Queue a bulk IN transfer into a caller buffer
 *
 * The controller receives straight into @p buf, without a bounce copy, so
 * several transfers can be kept in flight. Each completes when @p len
 * bytes have arrived or the device ends the transfer with a short packet;
 * collect completions in submission order with uhc_dwc3_bulk_in_reap().
 * Do not mix with uhc_dwc3_bulk_in() while transfers are queued.
 *
 * @p buf must be reachable by the controller's DMA (not in TCM), and must
 * not be accessed until the transfer is reaped.
 *
 * @param dev UHC device
 * @param buf Destination buffer, aligned to @ref UHC_DWC3_BUF_ALIGN, not
 *            crossing a 64 KiB address boundary
 * @param len Bytes to receive: a multiple of @ref UHC_DWC3_BUF_ALIGN and of
 *            the endpoint max packet size, at most 64 KiB
 * @retval 0 Transfer queued
 * @retval -EINVAL Invalid buffer, length or no bulk IN endpoint configured
 * @retval -ENOMEM Too many transfers queued (the limit is 8)
 */
int uhc_dwc3_bulk_in_submit(const struct device *dev, uint8_t *buf,
			    size_t len);

/**
 * @brief Wait for the oldest queued bulk IN transfer to complete
 *
 * On -EIO the endpoint has halted and the transfers still queued will not
 * complete; enumerate and configure the device again to recover.
 *
 * @param dev        UHC device
 * @param buf        Output: buffer of the completed transfer
 * @param timeout_ms Time to wait for the completion
 * @return number of bytes received on success (less than the submitted
 *         length when the device sent a short packet)
 * @retval -ENOENT No transfer queued
 * @retval -EAGAIN Timed out; the transfer stays queued and can be reaped
 *                 again
 * @retval -EIO    Transfer failed (@p buf is still returned)
 */
int uhc_dwc3_bulk_in_reap(const struct device *dev, uint8_t **buf,
			  int timeout_ms);

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
