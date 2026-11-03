/*
 * Copyright (c) 2026 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB Video Class camera on the DWC3 USB host controller, as a video device.
 *
 * A thread waits for a camera on the host port, enumerates it and reads its
 * formats and frame sizes, which video_get_caps() reports. Once the stream is
 * started, the thread negotiates the format set with video_set_format(),
 * keeps bulk or isochronous transfers queued and reassembles each frame
 * straight into an enqueued video buffer. Bad frames, and frames that arrive
 * while no buffer is enqueued, are dropped.
 *
 * Before each stream the camera is enumerated again, which leaves the
 * controller's endpoints in a known state. Unplugging is handled; the next
 * camera plugged in is used the same way.
 *
 * The DWC3 host helpers drive a single device, so there is one instance.
 */

#define DT_DRV_COMPAT alif_uvc_host

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/drivers/usb/uhc_dwc3.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>

#include <zephyr/drivers/usb/uvc_host.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(video_uvc_host, CONFIG_VIDEO_UVC_HOST_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "Only one UVC host camera is supported");

#define UHC_NODE	DT_INST_PHANDLE(0, usb_controller)

/* xHCI port status register of the DWC3 root port; bit 0 is CCS */
#define PORTSC_OFFSET	0x420

/* Bulk IN transfers kept in flight, each receiving into its own chunk */
#define NUM_CHUNKS	4
#define CHUNK_SIZE	16384

/*
 * Isochronous IN transfers kept in flight, one per service interval (4 ms
 * on High Speed). Each buffer holds the most a High-Speed endpoint carries
 * per microframe (3 x 1024 bytes).
 */
#define NUM_ISO_XFERS	32
#define ISO_BUF_SIZE	4096

/* Wait for a completion before checking that the camera is still there */
#define REAP_TIMEOUT_MS	1000

/* Time a stopping stream may take to wind down */
#define STOP_TIMEOUT	K_MSEC(2 * REAP_TIMEOUT_MS)

#define EVT_PARSED	BIT(0)	/* A camera is enumerated, caps are known */
#define EVT_STREAM_ON	BIT(1)	/* The application started the stream */

/* Aligning each transfer buffer to its size keeps it inside a 64 KiB
 * window, as the controller requires.
 */
static uint8_t chunks[NUM_CHUNKS][CHUNK_SIZE] __aligned(CHUNK_SIZE)
	UHC_DWC3_DMA_SECTION;
static uint8_t iso_bufs[NUM_ISO_XFERS][ISO_BUF_SIZE] __aligned(ISO_BUF_SIZE)
	UHC_DWC3_DMA_SECTION;

static uint8_t cfg_desc[UHC_DWC3_CTRL_MAX_LEN];

static const struct device *const uhc_dev = DEVICE_DT_GET(UHC_NODE);

USBH_CONTROLLER_DEFINE(uvc_host_uhs_ctx, DEVICE_DT_GET(UHC_NODE));

static struct {
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	struct k_event events;
	/* Guards fmt and caps against the streaming thread */
	struct k_mutex lock;
	/* Given when the streaming thread leaves a stream */
	struct k_sem stopped;
	struct k_poll_signal *signal;

	struct video_format fmt;
	/* One entry per frame size of a supported format, zero terminated */
	struct video_format_cap caps[UVC_MAX_FRAMES + 1];

	/* Streaming thread only */
	struct uvc_device uvc;
	struct uvc_stream stream;
	struct video_buffer *vbuf;	/* Being filled, or NULL: dropping */
	size_t expected_size;		/* Uncompressed frames, else 0 */
	bool mjpeg;
	bool streaming;
	uint32_t frames;
	uint32_t bad_frames;
	uint32_t dropped;		/* No buffer enqueued */
	uint32_t lost;			/* Isochronous transfers without data */
} data;

K_THREAD_STACK_DEFINE(uvc_host_stack, CONFIG_VIDEO_UVC_HOST_THREAD_STACK_SIZE);
static struct k_thread uvc_host_thread_data;

static bool port_connected(void)
{
	return (sys_read32(DT_REG_ADDR(UHC_NODE) + PORTSC_OFFSET) & BIT(0)) != 0;
}

static bool stream_on(void)
{
	return k_event_test(&data.events, EVT_STREAM_ON) != 0;
}

/* Pixel format a UVC format is reported as, 0 if it is not supported */
static uint32_t uvc_fourcc(const struct uvc_format *fmt)
{
	if (fmt->subtype == UVC_VS_FORMAT_MJPEG) {
		return VIDEO_PIX_FMT_JPEG;
	} else if (memcmp(fmt->guid, "YUY2", 4) == 0) {
		return VIDEO_PIX_FMT_YUYV;
	} else if (memcmp(fmt->guid, "RGBP", 4) == 0) {
		return VIDEO_PIX_FMT_RGB565;
	}

	return 0;
}

static void build_caps(void)
{
	int n = 0;

	memset(data.caps, 0, sizeof(data.caps));

	for (int i = 0; i < data.uvc.num_frames; i++) {
		const struct uvc_frame *frm = &data.uvc.frames[i];
		const struct uvc_format *fmt =
			uvc_find_format(&data.uvc, frm->format_index);
		uint32_t fourcc = fmt ? uvc_fourcc(fmt) : 0;

		if (fourcc == 0) {
			continue;
		}

		data.caps[n].pixelformat = fourcc;
		data.caps[n].width_min = frm->width;
		data.caps[n].width_max = frm->width;
		data.caps[n].height_min = frm->height;
		data.caps[n].height_max = frm->height;
		n++;
	}
}

/*
 * The frame matching a video format, or with a zero pixel format the
 * default frame of the first supported format, which is then filled in.
 */
static const struct uvc_frame *find_frame(struct video_format *vfmt,
					  const struct uvc_format **fmt_out)
{
	for (int i = 0; i < data.uvc.num_formats; i++) {
		const struct uvc_format *fmt = &data.uvc.formats[i];
		uint32_t fourcc = uvc_fourcc(fmt);
		const struct uvc_frame *frm;

		if (fourcc == 0) {
			continue;
		}

		if (vfmt->pixelformat == 0) {
			frm = uvc_find_frame(&data.uvc, fmt->index,
					     fmt->default_frame);
			if (frm == NULL) {
				continue;
			}
			vfmt->pixelformat = fourcc;
			vfmt->width = frm->width;
			vfmt->height = frm->height;
			vfmt->pitch = frm->width * fmt->bpp / 8;
			*fmt_out = fmt;
			return frm;
		}

		if (fourcc != vfmt->pixelformat) {
			continue;
		}

		for (int j = 0; j < data.uvc.num_frames; j++) {
			frm = &data.uvc.frames[j];
			if (frm->format_index == fmt->index &&
			    frm->width == vfmt->width &&
			    frm->height == vfmt->height) {
				*fmt_out = fmt;
				return frm;
			}
		}
	}

	return NULL;
}

static bool jpeg_ok(const uint8_t *frame, size_t len)
{
	size_t end = len;

	if (len < 4 || frame[0] != 0xFF || frame[1] != 0xD8) {
		return false;
	}

	while (end > 2 && len - end < 16 && frame[end - 1] == 0x00) {
		end--;
	}

	return frame[end - 2] == 0xFF && frame[end - 1] == 0xD9;
}

/* End of a frame: hand a good one over and pick the next buffer */
static void frame_done(const uint8_t *frame, size_t len, bool err)
{
	struct video_buffer *vbuf = data.vbuf;
	bool bad = err;

	if (vbuf == NULL) {
		data.dropped++;
	} else {
		if (!bad && data.mjpeg) {
			bad = !jpeg_ok(frame, len);
		} else if (!bad && data.expected_size != 0) {
			bad = (len != data.expected_size);
		}

		data.frames++;
		if (bad) {
			data.bad_frames++;
			LOG_DBG("Bad frame: %zu bytes", len);
		} else {
			vbuf->bytesused = len;
			vbuf->line_offset = 0;
			vbuf->timestamp = k_uptime_get_32();
			k_fifo_put(&data.fifo_out, vbuf);
			if (IS_ENABLED(CONFIG_POLL) && data.signal != NULL) {
				k_poll_signal_raise(data.signal,
						    VIDEO_BUF_DONE);
			}
			data.vbuf = NULL;
		}
	}

	/* A bad frame's buffer is reused for the next one */
	if (data.vbuf == NULL) {
		data.vbuf = k_fifo_get(&data.fifo_in, K_NO_WAIT);
	}

	uvc_stream_set_buffer(&data.stream,
			      data.vbuf ? data.vbuf->buffer : NULL,
			      data.vbuf ? data.vbuf->size : 0);
}

static int read_config_descriptor(size_t *len)
{
	uint8_t hdr[9];
	uint16_t total;
	int ret;

	ret = uhc_dwc3_control_transfer(uhc_dev, 0x80, USB_SREQ_GET_DESCRIPTOR,
					USB_DESC_CONFIGURATION << 8, 0,
					sizeof(hdr), hdr);
	if (ret < (int)sizeof(hdr)) {
		return ret < 0 ? ret : -EIO;
	}

	total = MIN(sys_get_le16(&hdr[2]), sizeof(cfg_desc));

	ret = uhc_dwc3_control_transfer(uhc_dev, 0x80, USB_SREQ_GET_DESCRIPTOR,
					USB_DESC_CONFIGURATION << 8, 0,
					total, cfg_desc);
	if (ret < 0) {
		return ret;
	}

	*len = ret;

	return 0;
}

/* Enumerate and configure the camera, and update the capabilities */
static int attach(void)
{
	struct usb_device_descriptor desc;
	size_t cfg_len;
	int ret;

	ret = uhc_dwc3_enumerate_device(uhc_dev, &desc);
	if (ret) {
		LOG_ERR("Enumeration failed: %d", ret);
		return ret;
	}

	ret = read_config_descriptor(&cfg_len);
	if (ret) {
		LOG_ERR("Reading the configuration descriptor failed: %d", ret);
		return ret;
	}

	k_mutex_lock(&data.lock, K_FOREVER);
	ret = uvc_parse_config(cfg_desc, cfg_len, &data.uvc);
	if (ret == 0) {
		build_caps();
	}
	k_mutex_unlock(&data.lock);

	if (ret) {
		LOG_ERR("Not a usable USB video device: %d", ret);
		return ret;
	}

	if (data.uvc.ep_type != USB_EP_TYPE_BULK &&
	    data.uvc.ep_type != USB_EP_TYPE_ISO) {
		LOG_ERR("Streaming endpoint type %u is not supported",
			data.uvc.ep_type);
		return -ENOTSUP;
	}

	if (data.caps[0].pixelformat == 0) {
		LOG_ERR("The camera offers no supported format");
		return -ENOTSUP;
	}

	ret = uhc_dwc3_control_transfer(uhc_dev, 0x00,
					USB_SREQ_SET_CONFIGURATION,
					data.uvc.cfg_val, 0, 0, NULL);
	if (ret < 0) {
		LOG_ERR("SET_CONFIGURATION failed: %d", ret);
		return ret;
	}

	LOG_INF("Camera %04x:%04x, %s streaming",
		sys_le16_to_cpu(desc.idVendor), sys_le16_to_cpu(desc.idProduct),
		data.uvc.ep_type == USB_EP_TYPE_BULK ? "bulk" : "isochronous");

	return 0;
}

/*
 * Size of each bulk IN transfer: whole packets, at most one chunk. A payload
 * that fills dwMaxPayloadTransferSize ends without a short packet, so the
 * transfers must then split it exactly.
 */
static size_t bulk_xfer_size(uint32_t max_payload)
{
	size_t step = ROUND_UP(data.uvc.ep_mps, UHC_DWC3_BUF_ALIGN);
	size_t len = MIN(CHUNK_SIZE, ROUND_UP(max_payload, step));

	if (max_payload % data.uvc.ep_mps == 0) {
		while (len > step && max_payload % len != 0) {
			len -= step;
		}
	}

	return len;
}

/* Select the isochronous alternate setting; returns the transfer size */
static int start_isoch(uint32_t max_payload)
{
	const struct uvc_alt *alt = uvc_select_alt(&data.uvc, max_payload);
	uint32_t len;
	int ret;

	if (alt == NULL) {
		LOG_ERR("No alternate setting carries %u byte payloads",
			max_payload);
		return -ENOTSUP;
	}

	len = uvc_alt_payload(alt);
	if (len > ISO_BUF_SIZE) {
		return -ENOTSUP;
	}

	ret = uhc_dwc3_configure_isoch_in(uhc_dev, data.uvc.ep, alt->mps,
					  alt->interval);
	if (ret) {
		LOG_ERR("Configuring EP 0x%02x failed: %d", data.uvc.ep, ret);
		return ret;
	}

	ret = uhc_dwc3_control_transfer(uhc_dev, 0x01, USB_SREQ_SET_INTERFACE,
					alt->alt, data.uvc.vs_iface, 0, NULL);
	if (ret < 0) {
		LOG_ERR("SET_INTERFACE %u alt %u failed: %d",
			data.uvc.vs_iface, alt->alt, ret);
		return ret;
	}

	LOG_DBG("Alternate setting %u: %u bytes per interval", alt->alt, len);

	return len;
}

static int run_isoch(size_t xfer_len)
{
	uint8_t *buf;
	int ret;

	for (int i = 0; i < NUM_ISO_XFERS; i++) {
		ret = uhc_dwc3_isoch_in_submit(uhc_dev, iso_bufs[i], xfer_len);
		if (ret) {
			return ret;
		}
	}

	while (stream_on()) {
		ret = uhc_dwc3_isoch_in_reap(uhc_dev, &buf, REAP_TIMEOUT_MS);
		if (ret == -EAGAIN) {
			/* A streaming endpoint completes a transfer every
			 * interval, also without data.
			 */
			return port_connected() ? -EIO : -ENODEV;
		}

		if (ret == -ENODATA || ret == -EIO) {
			data.lost++;
			uvc_stream_lost(&data.stream);
		} else if (ret >= 0) {
			uvc_stream_feed(&data.stream, buf, ret, true);
		} else {
			return ret;
		}

		ret = uhc_dwc3_isoch_in_submit(uhc_dev, buf, xfer_len);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

static int run_bulk(size_t xfer_len)
{
	uint8_t *buf;
	int ret;

	for (int i = 0; i < NUM_CHUNKS; i++) {
		ret = uhc_dwc3_bulk_in_submit(uhc_dev, chunks[i], xfer_len);
		if (ret) {
			return ret;
		}
	}

	while (stream_on()) {
		ret = uhc_dwc3_bulk_in_reap(uhc_dev, &buf, REAP_TIMEOUT_MS);
		if (ret == -EAGAIN) {
			if (!port_connected()) {
				return -ENODEV;
			}
			continue;
		} else if (ret < 0) {
			return ret;
		}

		uvc_stream_feed(&data.stream, buf, ret, (size_t)ret < xfer_len);

		ret = uhc_dwc3_bulk_in_submit(uhc_dev, buf, xfer_len);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

/* Negotiate the format set and receive until the stream is stopped */
static int run_stream(void)
{
	const struct uvc_format *fmt = NULL;
	const struct uvc_frame *frame;
	struct uvc_probe probe;
	uint32_t max_payload;
	int xfer_len;
	int ret;

	k_mutex_lock(&data.lock, K_FOREVER);
	frame = find_frame(&data.fmt, &fmt);
	k_mutex_unlock(&data.lock);

	if (frame == NULL) {
		LOG_ERR("The camera does not offer the format set");
		return -ENOTSUP;
	}

	if (data.uvc.ep_type == USB_EP_TYPE_BULK) {
		ret = uhc_dwc3_configure_bulk_in(uhc_dev, data.uvc.ep,
						 data.uvc.ep_mps);
		if (ret) {
			return ret;
		}
	}

	ret = uvc_negotiate(uhc_dev, &data.uvc, frame, &probe);
	if (ret) {
		return ret;
	}

	if (probe.bFormatIndex != fmt->index ||
	    probe.bFrameIndex != frame->index) {
		LOG_ERR("The camera chose format %u frame %u instead",
			probe.bFormatIndex, probe.bFrameIndex);
		return -ENOTSUP;
	}

	max_payload = sys_le32_to_cpu(probe.dwMaxPayloadTransferSize);
	if (max_payload == 0) {
		return -EINVAL;
	}

	data.mjpeg = (fmt->subtype == UVC_VS_FORMAT_MJPEG);
	data.expected_size = (fmt->subtype == UVC_VS_FORMAT_UNCOMPRESSED) ?
		(size_t)frame->width * frame->height * fmt->bpp / 8 : 0;

	data.vbuf = k_fifo_get(&data.fifo_in, K_NO_WAIT);
	uvc_stream_init(&data.stream, max_payload,
			data.vbuf ? data.vbuf->buffer : NULL,
			data.vbuf ? data.vbuf->size : 0, frame_done);

	if (data.uvc.ep_type == USB_EP_TYPE_ISO) {
		xfer_len = start_isoch(max_payload);
	} else {
		xfer_len = bulk_xfer_size(max_payload);
	}

	if (xfer_len < 0) {
		ret = xfer_len;
		goto out;
	}

	LOG_INF("Streaming %ux%u, %u00 ns per frame", frame->width,
		frame->height, sys_le32_to_cpu(probe.dwFrameInterval));

	if (data.uvc.ep_type == USB_EP_TYPE_ISO) {
		ret = run_isoch(xfer_len);
		/* Back to the zero-bandwidth alternate setting */
		(void)uhc_dwc3_control_transfer(uhc_dev, 0x01,
						USB_SREQ_SET_INTERFACE, 0,
						data.uvc.vs_iface, 0, NULL);
	} else {
		ret = run_bulk(xfer_len);
	}

	LOG_INF("Stream ended (%d): %u frames, %u bad, %u dropped, %u lost",
		ret, data.frames, data.bad_frames, data.dropped, data.lost);

out:
	if (data.vbuf != NULL) {
		k_fifo_put(&data.fifo_in, data.vbuf);
		data.vbuf = NULL;
	}

	return ret;
}

static int uhc_event_cb(const struct device *dev,
			const struct uhc_event *const event)
{
	LOG_DBG("UHC event %d", event->type);

	return 0;
}

static void uvc_host_thread(void *p1, void *p2, void *p3)
{
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	ret = uhc_init(uhc_dev, uhc_event_cb, &uvc_host_uhs_ctx);
	if (ret == 0) {
		ret = uhc_enable(uhc_dev);
	}

	if (ret) {
		LOG_ERR("Starting the USB host controller failed: %d", ret);
		return;
	}

	LOG_INF("USB host controller running, waiting for a camera");

	while (true) {
		if (!port_connected()) {
			k_event_clear(&data.events, EVT_PARSED);
			k_sleep(K_MSEC(200));
			continue;
		}

		/* Let the camera settle */
		k_sleep(K_MSEC(500));

		if (attach() != 0) {
			k_sleep(K_SECONDS(5));
			continue;
		}

		k_event_post(&data.events, EVT_PARSED);

		while (port_connected() &&
		       k_event_wait(&data.events, EVT_STREAM_ON, false,
				    K_MSEC(REAP_TIMEOUT_MS)) == 0) {
		}

		if (!port_connected()) {
			LOG_INF("Camera disconnected");
			continue;
		}

		data.frames = 0;
		data.bad_frames = 0;
		data.dropped = 0;
		data.lost = 0;
		data.streaming = true;

		ret = run_stream();

		data.streaming = false;
		k_sem_give(&data.stopped);

		/* A failing stream is retried; the camera is enumerated again
		 * before the next stream either way.
		 */
		if (ret && ret != -ENODEV) {
			k_sleep(K_SECONDS(1));
		}
	}
}

static int uvc_host_set_format(const struct device *dev,
			       enum video_endpoint_id ep,
			       struct video_format *fmt)
{
	const struct uvc_format *ufmt;
	struct video_format req = *fmt;
	int ret = 0;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	if (stream_on()) {
		return -EBUSY;
	}

	k_mutex_lock(&data.lock, K_FOREVER);

	/* Without a camera the format is checked when the stream starts */
	if (k_event_test(&data.events, EVT_PARSED) &&
	    find_frame(&req, &ufmt) == NULL) {
		ret = -ENOTSUP;
	} else {
		data.fmt = *fmt;
	}

	k_mutex_unlock(&data.lock);

	return ret;
}

static int uvc_host_get_format(const struct device *dev,
			       enum video_endpoint_id ep,
			       struct video_format *fmt)
{
	const struct uvc_format *ufmt;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	k_mutex_lock(&data.lock, K_FOREVER);

	/* Report the camera's default until a format is set */
	if (data.fmt.pixelformat == 0 &&
	    k_event_test(&data.events, EVT_PARSED)) {
		(void)find_frame(&data.fmt, &ufmt);
	}

	*fmt = data.fmt;

	k_mutex_unlock(&data.lock);

	return 0;
}

static int uvc_host_get_caps(const struct device *dev,
			     enum video_endpoint_id ep,
			     struct video_caps *caps)
{
	k_timeout_t timeout = CONFIG_VIDEO_UVC_HOST_CAPS_TIMEOUT_MS ?
		K_MSEC(CONFIG_VIDEO_UVC_HOST_CAPS_TIMEOUT_MS) : K_FOREVER;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	if (!k_event_test(&data.events, EVT_PARSED)) {
		LOG_INF("Waiting for a USB camera");
	}

	if (k_event_wait(&data.events, EVT_PARSED, false, timeout) == 0) {
		return -EAGAIN;
	}

	caps->format_caps = data.caps;
	caps->min_vbuf_count = 1;
	caps->min_line_count = LINE_COUNT_HEIGHT;
	caps->max_line_count = LINE_COUNT_HEIGHT;

	return 0;
}

static int uvc_host_set_stream(const struct device *dev, bool enable)
{
	if (enable) {
		k_event_post(&data.events, EVT_STREAM_ON);
		return 0;
	}

	if (!stream_on()) {
		return 0;
	}

	k_sem_reset(&data.stopped);
	k_event_clear(&data.events, EVT_STREAM_ON);

	if (data.streaming &&
	    k_sem_take(&data.stopped, STOP_TIMEOUT) != 0) {
		LOG_WRN("The stream did not stop in time");
	}

	return 0;
}

static int uvc_host_enqueue(const struct device *dev,
			    enum video_endpoint_id ep,
			    struct video_buffer *buf)
{
	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	k_fifo_put(&data.fifo_in, buf);

	return 0;
}

static int uvc_host_dequeue(const struct device *dev,
			    enum video_endpoint_id ep,
			    struct video_buffer **buf, k_timeout_t timeout)
{
	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	*buf = k_fifo_get(&data.fifo_out, timeout);

	return *buf == NULL ? -EAGAIN : 0;
}

static int uvc_host_flush(const struct device *dev,
			  enum video_endpoint_id ep, bool cancel)
{
	struct video_buffer *buf;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	if (!cancel) {
		/* Wait until every enqueued buffer has been filled */
		while (!k_fifo_is_empty(&data.fifo_in)) {
			k_sleep(K_MSEC(1));
		}
		return 0;
	}

	/* Hand back the buffers not being filled, unused */
	while ((buf = k_fifo_get(&data.fifo_in, K_NO_WAIT)) != NULL) {
		buf->bytesused = 0;
		k_fifo_put(&data.fifo_out, buf);
		if (IS_ENABLED(CONFIG_POLL) && data.signal != NULL) {
			k_poll_signal_raise(data.signal, VIDEO_BUF_ABORTED);
		}
	}

	return 0;
}

#ifdef CONFIG_POLL
static int uvc_host_set_signal(const struct device *dev,
			       enum video_endpoint_id ep,
			       struct k_poll_signal *signal)
{
	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	data.signal = signal;

	return 0;
}
#endif

static DEVICE_API(video, uvc_host_api) = {
	.set_format = uvc_host_set_format,
	.get_format = uvc_host_get_format,
	.set_stream = uvc_host_set_stream,
	.get_caps = uvc_host_get_caps,
	.enqueue = uvc_host_enqueue,
	.dequeue = uvc_host_dequeue,
	.flush = uvc_host_flush,
#ifdef CONFIG_POLL
	.set_signal = uvc_host_set_signal,
#endif
};

static int uvc_host_init(const struct device *dev)
{
	k_fifo_init(&data.fifo_in);
	k_fifo_init(&data.fifo_out);
	k_event_init(&data.events);
	k_mutex_init(&data.lock);
	k_sem_init(&data.stopped, 0, 1);

	if (!device_is_ready(uhc_dev)) {
		LOG_ERR("USB host controller not ready");
		return -ENODEV;
	}

	/* Started once the system is up, see uvc_host_start() */
	k_thread_create(&uvc_host_thread_data, uvc_host_stack,
			K_THREAD_STACK_SIZEOF(uvc_host_stack),
			uvc_host_thread, NULL, NULL, NULL,
			CONFIG_VIDEO_UVC_HOST_THREAD_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&uvc_host_thread_data, "uvc_host");

	return 0;
}

/*
 * Bring up the USB host controller only after all devices are initialised,
 * not in the middle of device init at a cooperative priority.
 */
static int uvc_host_start(void)
{
	if (device_is_ready(DEVICE_DT_INST_GET(0))) {
		LOG_INF("Starting the USB host controller");
		k_thread_start(&uvc_host_thread_data);
	}

	return 0;
}

SYS_INIT(uvc_host_start, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

DEVICE_DT_INST_DEFINE(0, uvc_host_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_VIDEO_UVC_HOST_INIT_PRIORITY, &uvc_host_api);
