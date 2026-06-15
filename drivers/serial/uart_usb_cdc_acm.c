/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief UART driver backed by USB Host CDC-ACM
 *
 * This driver implements the Zephyr UART driver API on top of the
 * USB host CDC-ACM class driver. It allows any driver that uses the
 * UART API (e.g., modem_cellular) to transparently communicate with
 * a USB CDC-ACM device connected to the USB host port.
 *
 * Modelled after uart_bt.c which bridges Bluetooth NUS to UART API.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/host/cdc_acm.h>

#define DT_DRV_COMPAT zephyr_cdc_acm_uart

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_usb_cdc_acm, CONFIG_UART_LOG_LEVEL);

K_THREAD_STACK_DEFINE(cdc_acm_work_queue_stack, CONFIG_UART_USB_CDC_ACM_WORKQUEUE_STACK_SIZE);
static struct k_work_q cdc_acm_work_queue;

#define CDC_ACM_RX_POLL_INTERVAL_MS	10

struct uart_cdc_acm_data {
	struct {
		struct usbh_cdc_acm_data cdc;
		struct usbh_contex *ctx;
		atomic_t connected;
	} usb;
	struct {
		struct ring_buf *rx_ringbuf;
		struct ring_buf *tx_ringbuf;
		struct k_work cb_work;
		struct k_work_delayable tx_work;
		struct k_work_delayable rx_poll_work;
		bool rx_irq_ena;
		bool tx_irq_ena;
		struct {
			const struct device *dev;
			uart_irq_callback_user_data_t cb;
			void *cb_data;
		} callback;
	} uart;
};

/* ---- USB CDC-ACM receive callback ---- */

static void cdc_acm_rx_callback(const uint8_t *data, size_t len, void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct uart_cdc_acm_data *dev_data = dev->data;
	uint32_t put_len;

	put_len = ring_buf_put(dev_data->uart.rx_ringbuf, data, len);
	if (put_len < len) {
		LOG_ERR("RX ring buffer full, dropped %zu bytes", len - put_len);
	}

	k_work_submit_to_queue(&cdc_acm_work_queue, &dev_data->uart.cb_work);
}

/* ---- Work handlers ---- */

static void cb_work_handler(struct k_work *work)
{
	struct uart_cdc_acm_data *dev_data =
		CONTAINER_OF(work, struct uart_cdc_acm_data, uart.cb_work);

	if (dev_data->uart.callback.cb) {
		dev_data->uart.callback.cb(dev_data->uart.callback.dev,
					   dev_data->uart.callback.cb_data);
	}
}

static void tx_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_cdc_acm_data *dev_data =
		CONTAINER_OF(dwork, struct uart_cdc_acm_data, uart.tx_work);
	uint8_t *data;
	uint32_t len;
	int err;

	if (!atomic_get(&dev_data->usb.connected)) {
		return;
	}

	do {
		/* Send in chunks up to 64 bytes (typical bulk MPS) */
		len = ring_buf_get_claim(dev_data->uart.tx_ringbuf, &data, 64);
		if (len > 0) {
			err = usbh_cdc_acm_send(&dev_data->usb.cdc, data, len);
			if (err < 0) {
				LOG_ERR("CDC-ACM send failed: %d", err);
			}
		}
		ring_buf_get_finish(dev_data->uart.tx_ringbuf, len);
	} while (len > 0);

	if (ring_buf_space_get(dev_data->uart.tx_ringbuf) > 0 &&
	    dev_data->uart.tx_irq_ena) {
		k_work_submit_to_queue(&cdc_acm_work_queue,
				       &dev_data->uart.cb_work);
	}
}

static void rx_poll_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_cdc_acm_data *dev_data =
		CONTAINER_OF(dwork, struct uart_cdc_acm_data, uart.rx_poll_work);

	if (!atomic_get(&dev_data->usb.connected)) {
		return;
	}

	/*
	 * TODO: Submit a bulk IN transfer to poll for data from the
	 * CDC-ACM device. When data arrives, put it in the rx_ringbuf
	 * and fire the callback.
	 *
	 * For now, schedule periodic polling. A production implementation
	 * would use continuous bulk IN transfers with completion callbacks.
	 */

	/* Reschedule polling */
	k_work_schedule_for_queue(&cdc_acm_work_queue,
				  &dev_data->uart.rx_poll_work,
				  K_MSEC(CDC_ACM_RX_POLL_INTERVAL_MS));
}

/* ---- UART API implementation ---- */

static int uart_cdc_acm_poll_in(const struct device *dev, unsigned char *c)
{
	struct uart_cdc_acm_data *dev_data = dev->data;

	if (ring_buf_get(dev_data->uart.rx_ringbuf, c, 1) == 1) {
		return 0;
	}

	return -1;
}

static void uart_cdc_acm_poll_out(const struct device *dev, unsigned char c)
{
	struct uart_cdc_acm_data *dev_data = dev->data;

	while (!ring_buf_put(dev_data->uart.tx_ringbuf, &c, 1)) {
		if (k_is_in_isr() || !atomic_get(&dev_data->usb.connected)) {
			LOG_WRN_ONCE("TX ring buffer full, discarding");
			break;
		}
		k_sleep(K_MSEC(1));
	}

	if (atomic_get(&dev_data->usb.connected)) {
		k_work_schedule_for_queue(&cdc_acm_work_queue,
					  &dev_data->uart.tx_work, K_MSEC(1));
	}
}

static int uart_cdc_acm_fifo_fill(const struct device *dev,
				   const uint8_t *tx_data, int len)
{
	struct uart_cdc_acm_data *dev_data = dev->data;
	size_t wrote;

	wrote = ring_buf_put(dev_data->uart.tx_ringbuf, tx_data, len);
	if (wrote < len) {
		LOG_WRN("TX ring buffer full, dropped %d bytes",
			(int)(len - wrote));
	}

	if (atomic_get(&dev_data->usb.connected)) {
		k_work_reschedule_for_queue(&cdc_acm_work_queue,
					    &dev_data->uart.tx_work,
					    K_NO_WAIT);
	}

	return wrote;
}

static int uart_cdc_acm_fifo_read(const struct device *dev,
				   uint8_t *rx_data, const int size)
{
	struct uart_cdc_acm_data *dev_data = dev->data;

	return ring_buf_get(dev_data->uart.rx_ringbuf, rx_data, size);
}

static void uart_cdc_acm_irq_tx_enable(const struct device *dev)
{
	struct uart_cdc_acm_data *dev_data = dev->data;

	dev_data->uart.tx_irq_ena = true;

	if (ring_buf_space_get(dev_data->uart.tx_ringbuf) > 0) {
		k_work_submit_to_queue(&cdc_acm_work_queue,
				       &dev_data->uart.cb_work);
	}
}

static void uart_cdc_acm_irq_tx_disable(const struct device *dev)
{
	struct uart_cdc_acm_data *dev_data = dev->data;

	dev_data->uart.tx_irq_ena = false;
}

static int uart_cdc_acm_irq_tx_ready(const struct device *dev)
{
	struct uart_cdc_acm_data *dev_data = dev->data;

	if (ring_buf_space_get(dev_data->uart.tx_ringbuf) > 0 &&
	    dev_data->uart.tx_irq_ena) {
		return 1;
	}

	return 0;
}

static void uart_cdc_acm_irq_rx_enable(const struct device *dev)
{
	struct uart_cdc_acm_data *dev_data = dev->data;

	dev_data->uart.rx_irq_ena = true;
	k_work_submit_to_queue(&cdc_acm_work_queue, &dev_data->uart.cb_work);
}

static void uart_cdc_acm_irq_rx_disable(const struct device *dev)
{
	struct uart_cdc_acm_data *dev_data = dev->data;

	dev_data->uart.rx_irq_ena = false;
}

static int uart_cdc_acm_irq_rx_ready(const struct device *dev)
{
	struct uart_cdc_acm_data *dev_data = dev->data;

	if (!ring_buf_is_empty(dev_data->uart.rx_ringbuf) &&
	    dev_data->uart.rx_irq_ena) {
		return 1;
	}

	return 0;
}

static int uart_cdc_acm_irq_is_pending(const struct device *dev)
{
	return uart_cdc_acm_irq_rx_ready(dev) ||
	       uart_cdc_acm_irq_tx_ready(dev);
}

static int uart_cdc_acm_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 1;
}

static void uart_cdc_acm_irq_callback_set(const struct device *dev,
					   uart_irq_callback_user_data_t cb,
					   void *cb_data)
{
	struct uart_cdc_acm_data *dev_data = dev->data;

	dev_data->uart.callback.cb = cb;
	dev_data->uart.callback.cb_data = cb_data;
}

static DEVICE_API(uart, uart_cdc_acm_driver_api) = {
	.poll_in = uart_cdc_acm_poll_in,
	.poll_out = uart_cdc_acm_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_cdc_acm_fifo_fill,
	.fifo_read = uart_cdc_acm_fifo_read,
	.irq_tx_enable = uart_cdc_acm_irq_tx_enable,
	.irq_tx_disable = uart_cdc_acm_irq_tx_disable,
	.irq_tx_ready = uart_cdc_acm_irq_tx_ready,
	.irq_rx_enable = uart_cdc_acm_irq_rx_enable,
	.irq_rx_disable = uart_cdc_acm_irq_rx_disable,
	.irq_rx_ready = uart_cdc_acm_irq_rx_ready,
	.irq_is_pending = uart_cdc_acm_irq_is_pending,
	.irq_update = uart_cdc_acm_irq_update,
	.irq_callback_set = uart_cdc_acm_irq_callback_set,
#endif
};

/* ---- Initialization ---- */

static int uart_cdc_acm_workqueue_init(void)
{
	k_work_queue_init(&cdc_acm_work_queue);
	k_work_queue_start(&cdc_acm_work_queue, cdc_acm_work_queue_stack,
			   K_THREAD_STACK_SIZEOF(cdc_acm_work_queue_stack),
			   CONFIG_UART_USB_CDC_ACM_WORKQUEUE_PRIORITY, NULL);

	return 0;
}

SYS_INIT(uart_cdc_acm_workqueue_init, POST_KERNEL, CONFIG_SERIAL_INIT_PRIORITY);

static int uart_cdc_acm_init(const struct device *dev)
{
	struct uart_cdc_acm_data *dev_data = dev->data;

	dev_data->uart.callback.dev = dev;

	k_work_init_delayable(&dev_data->uart.tx_work, tx_work_handler);
	k_work_init(&dev_data->uart.cb_work, cb_work_handler);
	k_work_init_delayable(&dev_data->uart.rx_poll_work, rx_poll_work_handler);

	LOG_DBG("USB CDC-ACM UART bridge initialized");

	return 0;
}

/* ---- DT instantiation ---- */

#define UART_CDC_ACM_RX_FIFO_SIZE(inst) DT_INST_PROP(inst, rx_fifo_size)
#define UART_CDC_ACM_TX_FIFO_SIZE(inst) DT_INST_PROP(inst, tx_fifo_size)

#define UART_CDC_ACM_INIT(n)						\
									\
	RING_BUF_DECLARE(cdc_acm_rx_rb_##n,				\
			 UART_CDC_ACM_RX_FIFO_SIZE(n));			\
	RING_BUF_DECLARE(cdc_acm_tx_rb_##n,				\
			 UART_CDC_ACM_TX_FIFO_SIZE(n));			\
									\
	static struct uart_cdc_acm_data uart_cdc_acm_data_##n = {	\
		.usb = {						\
			.connected = ATOMIC_INIT(0),			\
		},							\
		.uart = {						\
			.rx_ringbuf = &cdc_acm_rx_rb_##n,		\
			.tx_ringbuf = &cdc_acm_tx_rb_##n,		\
		},							\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, uart_cdc_acm_init, NULL,		\
			      &uart_cdc_acm_data_##n, NULL,		\
			      POST_KERNEL,				\
			      CONFIG_SERIAL_INIT_PRIORITY,		\
			      &uart_cdc_acm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_CDC_ACM_INIT)
