/*
 * Copyright (c) 2026 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal USB Video Class (UVC) host support for the DWC3 host helpers
 * (CONFIG_UVC_HOST):
 * configuration descriptor parsing, Probe/Commit format negotiation and
 * reassembly of video frames from UVC payloads.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_USB_UVC_HOST_H_
#define ZEPHYR_INCLUDE_DRIVERS_USB_UVC_HOST_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

/* Limits on what the descriptor parser records */
#define UVC_MAX_FORMATS		8
#define UVC_MAX_FRAMES		64
#define UVC_MAX_ALTS		16

/* Payload header bmHeaderInfo bits (UVC 1.5, 2.4.3.3) */
#define UVC_HDR_FID		BIT(0)
#define UVC_HDR_EOF		BIT(1)
#define UVC_HDR_ERR		BIT(6)

/* Video format descriptor subtypes this sample can stream */
#define UVC_VS_FORMAT_UNCOMPRESSED	0x04
#define UVC_VS_FORMAT_MJPEG		0x06
#define UVC_VS_FORMAT_FRAME_BASED	0x10

struct uvc_format {
	uint8_t index;		/* bFormatIndex */
	uint8_t subtype;	/* UVC_VS_FORMAT_* */
	uint8_t guid[16];	/* guidFormat (uncompressed / frame based) */
	uint8_t bpp;		/* bBitsPerPixel (uncompressed / frame based) */
	uint8_t default_frame;	/* bDefaultFrameIndex */
};

struct uvc_frame {
	uint8_t format_index;	/* bFormatIndex of the owning format */
	uint8_t index;		/* bFrameIndex */
	uint16_t width;
	uint16_t height;
	uint32_t default_interval; /* dwDefaultFrameInterval, 100 ns units */
};

/* A VideoStreaming alternate setting with an isochronous endpoint */
struct uvc_alt {
	uint8_t alt;		/* bAlternateSetting */
	uint16_t mps;		/* wMaxPacketSize, additional transactions included */
	uint8_t interval;	/* bInterval */
};

/* Bytes the endpoint of an alternate setting carries per service interval */
static inline uint32_t uvc_alt_payload(const struct uvc_alt *alt)
{
	return (alt->mps & 0x7FF) * (1 + ((alt->mps >> 11) & 0x3));
}

/* What the configuration descriptor says about the video function */
struct uvc_device {
	uint8_t cfg_val;	/* bConfigurationValue */
	uint16_t bcd_uvc;	/* UVC version from the VC header */
	uint8_t vs_iface;	/* VideoStreaming interface number */
	uint8_t ep;		/* Streaming endpoint address */
	uint8_t ep_type;	/* USB_EP_TYPE_BULK / USB_EP_TYPE_ISO */
	uint16_t ep_mps;	/* wMaxPacketSize bits 10:0 */
	uint8_t num_formats;
	uint8_t num_frames;
	uint8_t num_alts;
	struct uvc_format formats[UVC_MAX_FORMATS];
	struct uvc_frame frames[UVC_MAX_FRAMES];
	/* Isochronous endpoints: the alternate settings that carry them */
	struct uvc_alt alts[UVC_MAX_ALTS];
};

/* Video Probe and Commit control (UVC 1.5, 4.3.1.1) */
struct uvc_probe {
	uint16_t bmHint;
	uint8_t bFormatIndex;
	uint8_t bFrameIndex;
	uint32_t dwFrameInterval;
	uint16_t wKeyFrameRate;
	uint16_t wPFrameRate;
	uint16_t wCompQuality;
	uint16_t wCompWindowSize;
	uint16_t wDelay;
	uint32_t dwMaxVideoFrameSize;
	uint32_t dwMaxPayloadTransferSize;
	/* UVC 1.1 */
	uint32_t dwClockFrequency;
	uint8_t bmFramingInfo;
	uint8_t bPreferedVersion;
	uint8_t bMinVersion;
	uint8_t bMaxVersion;
	/* UVC 1.5 */
	uint8_t bUsage;
	uint8_t bBitDepthLuma;
	uint8_t bmSettings;
	uint8_t bMaxNumberOfRefFramesPlus1;
	uint16_t bmRateControlModes;
	uint64_t bmLayoutPerStream;
} __packed;

/* Reassembles frames from the payloads of one stream */
struct uvc_stream {
	uint32_t max_payload;	/* Negotiated dwMaxPayloadTransferSize */
	uint8_t *frame_buf;
	size_t frame_buf_size;

	/* Frame being assembled */
	size_t frame_len;
	bool frame_err;

	/* Payload being received */
	bool in_payload;
	bool skip_payload;	/* Bad header: drop until the payload ends */
	uint8_t payload_info;	/* bmHeaderInfo */
	uint32_t payload_len;	/* Bytes received, header included */
	int last_fid;		/* -1 until the first payload */

	/* Counters */
	uint32_t bad_headers;	/* bHeaderLength out of range */
	uint32_t overflows;	/* Frame larger than frame_buf */
	uint32_t err_payloads;	/* Payloads with the ERR bit set */

	void (*frame_done)(const uint8_t *frame, size_t len, bool err);
};

const struct uvc_format *uvc_find_format(const struct uvc_device *uvc,
					 uint8_t index);
const struct uvc_frame *uvc_find_frame(const struct uvc_device *uvc,
				       uint8_t format_index, uint8_t index);

/* Print a format's name, e.g. "MJPEG" or the GUID FourCC "YUY2" */
const char *uvc_format_name(const struct uvc_format *fmt, char buf[5]);

/*
 * The alternate setting with the smallest isochronous endpoint that carries
 * payloads of @p max_payload bytes, or NULL if none does.
 */
const struct uvc_alt *uvc_select_alt(const struct uvc_device *uvc,
				     uint32_t max_payload);

/*
 * Parse a configuration descriptor. Records the first VideoStreaming
 * interface, its streaming endpoint and its formats and frames.
 */
int uvc_parse_config(const uint8_t *desc, size_t len, struct uvc_device *uvc);

/*
 * Negotiate a format and frame with Probe/Commit (UVC 1.5, 4.3.1.1.1).
 * A bulk device starts streaming once the Commit is accepted.
 */
int uvc_negotiate(const struct device *uhc, const struct uvc_device *uvc,
		  const struct uvc_frame *frame, struct uvc_probe *probe);

void uvc_stream_init(struct uvc_stream *s, uint32_t max_payload,
		     uint8_t *frame_buf, size_t frame_buf_size,
		     void (*frame_done)(const uint8_t *frame, size_t len,
					bool err));

/*
 * Feed one completed transfer. @p short_xfer is true when the transfer
 * ended on a short packet, which ends the current payload. Each isochronous
 * transfer is one whole payload, so pass true for those.
 */
void uvc_stream_feed(struct uvc_stream *s, const uint8_t *data, size_t len,
		     bool short_xfer);

/*
 * Report stream data that never arrived, such as a missed isochronous
 * interval. The frame it belonged to is flagged as an error.
 */
void uvc_stream_lost(struct uvc_stream *s);

/*
 * Reassemble the next frames into another buffer, e.g. while the last one
 * is still being shown. Can be called from the frame_done callback. With a
 * NULL buffer the frames are dropped, and frame_done is still called at the
 * end of each, with a NULL frame, so that a buffer can be set again.
 */
void uvc_stream_set_buffer(struct uvc_stream *s, uint8_t *frame_buf,
			   size_t frame_buf_size);

#endif /* ZEPHYR_INCLUDE_DRIVERS_USB_UVC_HOST_H_ */
