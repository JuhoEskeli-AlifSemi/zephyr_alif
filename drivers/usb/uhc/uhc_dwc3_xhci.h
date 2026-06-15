/*
 * Copyright (c) 2025 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef UHC_DWC3_XHCI_H
#define UHC_DWC3_XHCI_H

#include <zephyr/usb/usb_ch9.h>

/**
 * @brief Enumerate a device on the DWC3 xHCI host controller.
 *
 * Performs port reset, Enable Slot, Address Device, and
 * reads the device descriptor.
 *
 * @param dev UHC device
 * @param desc Output: device descriptor read from the device
 * @return 0 on success
 */
int uhc_dwc3_enumerate_device(const struct device *dev,
			      struct usb_device_descriptor *desc);

#endif /* UHC_DWC3_XHCI_H */
