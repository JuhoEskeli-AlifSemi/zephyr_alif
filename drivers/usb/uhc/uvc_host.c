/*
 * Copyright (c) 2026 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <errno.h>
#include <string.h>

#include <zephyr/drivers/usb/uhc_dwc3.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_ch9.h>

#include <zephyr/drivers/usb/uvc_host.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uvc_host, CONFIG_UVC_HOST_LOG_LEVEL);

/* Interface class / subclasses (UVC 1.5, A.1 / A.2) */
#define UVC_CC_VIDEO			0x0E
#define UVC_SC_VIDEOCONTROL		0x01
#define UVC_SC_VIDEOSTREAMING		0x02

/* Class-specific descriptor subtypes (UVC 1.5, A.5 / A.6) */
#define UVC_CS_INTERFACE		0x24
#define UVC_VC_HEADER			0x01
#define UVC_VS_INPUT_HEADER		0x01
#define UVC_VS_FRAME_UNCOMPRESSED	0x05
#define UVC_VS_FRAME_MJPEG		0x07
#define UVC_VS_FRAME_FRAME_BASED	0x11

/* Class-specific requests and VideoStreaming controls (UVC 1.5, A.8 / A.9.8) */
#define UVC_SET_CUR			0x01
#define UVC_GET_CUR			0x81
#define UVC_VS_PROBE_CONTROL		0x01
#define UVC_VS_COMMIT_CONTROL		0x02

/* bmRequestType for class requests to an interface */
#define UVC_REQ_TYPE_SET		0x21
#define UVC_REQ_TYPE_GET		0xA1

const struct uvc_format *uvc_find_format(const struct uvc_device *uvc,
					 uint8_t index)
{
	for (int i = 0; i < uvc->num_formats; i++) {
		if (uvc->formats[i].index == index) {
			return &uvc->formats[i];
		}
	}

	return NULL;
}

const struct uvc_frame *uvc_find_frame(const struct uvc_device *uvc,
				       uint8_t format_index, uint8_t index)
{
	for (int i = 0; i < uvc->num_frames; i++) {
		if (uvc->frames[i].format_index == format_index &&
		    uvc->frames[i].index == index) {
			return &uvc->frames[i];
		}
	}

	return NULL;
}

const char *uvc_format_name(const struct uvc_format *fmt, char buf[5])
{
	if (fmt->subtype == UVC_VS_FORMAT_MJPEG) {
		return "MJPEG";
	}

	/* The first four GUID bytes are the FourCC for the common formats */
	for (int i = 0; i < 4; i++) {
		buf[i] = isprint(fmt->guid[i]) ? fmt->guid[i] : '?';
	}
	buf[4] = '\0';

	return buf;
}

const struct uvc_alt *uvc_select_alt(const struct uvc_device *uvc,
				     uint32_t max_payload)
{
	const struct uvc_alt *best = NULL;

	for (int i = 0; i < uvc->num_alts; i++) {
		const struct uvc_alt *alt = &uvc->alts[i];

		if (uvc_alt_payload(alt) >= max_payload &&
		    (best == NULL ||
		     uvc_alt_payload(alt) < uvc_alt_payload(best))) {
			best = alt;
		}
	}

	return best;
}

static void parse_format(const uint8_t *d, uint8_t dlen,
			 struct uvc_device *uvc)
{
	struct uvc_format *fmt;
	uint8_t subtype = d[2];

	if (uvc->num_formats >= UVC_MAX_FORMATS) {
		LOG_WRN("Too many formats, ignoring format %u", d[3]);
		return;
	}

	fmt = &uvc->formats[uvc->num_formats];
	memset(fmt, 0, sizeof(*fmt));
	fmt->index = d[3];
	fmt->subtype = subtype;

	if (subtype == UVC_VS_FORMAT_MJPEG && dlen >= 11) {
		fmt->default_frame = d[6];
	} else if (subtype != UVC_VS_FORMAT_MJPEG && dlen >= 27) {
		memcpy(fmt->guid, &d[5], sizeof(fmt->guid));
		fmt->bpp = d[21];
		fmt->default_frame = d[22];
	} else {
		LOG_WRN("Short format descriptor (%u bytes)", dlen);
		return;
	}

	uvc->num_formats++;
}

static void parse_frame(const uint8_t *d, uint8_t dlen,
			struct uvc_device *uvc)
{
	struct uvc_frame *frm;
	uint8_t subtype = d[2];
	/* dwDefaultFrameInterval moved up in the frame based descriptor */
	uint8_t interval_off = (subtype == UVC_VS_FRAME_FRAME_BASED) ? 17 : 21;

	if (uvc->num_formats == 0 || dlen < interval_off + 4) {
		return;
	}

	if (uvc->num_frames >= UVC_MAX_FRAMES) {
		LOG_WRN("Too many frames, ignoring frame %u", d[3]);
		return;
	}

	frm = &uvc->frames[uvc->num_frames++];
	frm->format_index = uvc->formats[uvc->num_formats - 1].index;
	frm->index = d[3];
	frm->width = sys_get_le16(&d[5]);
	frm->height = sys_get_le16(&d[7]);
	frm->default_interval = sys_get_le32(&d[interval_off]);
}

static void parse_alt(const uint8_t *d, uint8_t alt, struct uvc_device *uvc)
{
	if (uvc->num_alts >= UVC_MAX_ALTS) {
		LOG_WRN("Too many alternate settings, ignoring alt %u", alt);
		return;
	}

	uvc->alts[uvc->num_alts].alt = alt;
	uvc->alts[uvc->num_alts].mps = sys_get_le16(&d[4]);
	uvc->alts[uvc->num_alts].interval = d[6];
	uvc->num_alts++;
}

int uvc_parse_config(const uint8_t *desc, size_t len, struct uvc_device *uvc)
{
	bool in_vc = false;
	bool in_vs = false;
	bool have_vs = false;
	uint8_t vs_alt = 0;
	uint8_t vs_ep = 0;
	size_t off = 0;

	memset(uvc, 0, sizeof(*uvc));

	while (off + 2 <= len) {
		const uint8_t *d = &desc[off];
		uint8_t dlen = d[0];
		uint8_t dtype = d[1];

		if (dlen < 2 || off + dlen > len) {
			break;
		}

		if (dtype == USB_DESC_CONFIGURATION && dlen >= 9) {
			uvc->cfg_val = d[5];
		} else if (dtype == USB_DESC_INTERFACE && dlen >= 9) {
			bool video = (d[5] == UVC_CC_VIDEO);

			in_vc = video && d[6] == UVC_SC_VIDEOCONTROL;
			in_vs = false;

			/* Only the first VideoStreaming interface is used */
			if (video && d[6] == UVC_SC_VIDEOSTREAMING &&
			    (!have_vs || d[2] == uvc->vs_iface)) {
				have_vs = true;
				in_vs = true;
				uvc->vs_iface = d[2];
				vs_alt = d[3];
			}
		} else if (dtype == UVC_CS_INTERFACE && dlen >= 3) {
			uint8_t subtype = d[2];

			if (in_vc && subtype == UVC_VC_HEADER && dlen >= 5) {
				uvc->bcd_uvc = sys_get_le16(&d[3]);
			} else if (in_vs && subtype == UVC_VS_INPUT_HEADER &&
				   dlen >= 7) {
				vs_ep = d[6];
			} else if (in_vs &&
				   (subtype == UVC_VS_FORMAT_UNCOMPRESSED ||
				    subtype == UVC_VS_FORMAT_MJPEG ||
				    subtype == UVC_VS_FORMAT_FRAME_BASED)) {
				parse_format(d, dlen, uvc);
			} else if (in_vs &&
				   (subtype == UVC_VS_FRAME_UNCOMPRESSED ||
				    subtype == UVC_VS_FRAME_MJPEG ||
				    subtype == UVC_VS_FRAME_FRAME_BASED)) {
				parse_frame(d, dlen, uvc);
			}
		} else if (dtype == USB_DESC_ENDPOINT && dlen >= 7 && in_vs &&
			   d[2] == vs_ep) {
			/* A bulk endpoint sits in alt 0; isochronous ones in
			 * the alternate settings, one per bandwidth.
			 */
			if (uvc->ep == 0) {
				uvc->ep = d[2];
				uvc->ep_type = d[3] & USB_EP_TRANSFER_TYPE_MASK;
				uvc->ep_mps = sys_get_le16(&d[4]) & 0x7FF;
				LOG_DBG("Streaming EP 0x%02x in alt %u", d[2],
					vs_alt);
			}

			if ((d[3] & USB_EP_TRANSFER_TYPE_MASK) ==
			    USB_EP_TYPE_ISO && vs_alt != 0) {
				parse_alt(d, vs_alt, uvc);
			}
		}

		off += dlen;
	}

	if (!have_vs) {
		return -ENODEV;
	}

	if (uvc->ep == 0 || uvc->ep_mps == 0 || uvc->num_formats == 0 ||
	    uvc->num_frames == 0) {
		return -EINVAL;
	}

	if (uvc->ep_type == USB_EP_TYPE_ISO && uvc->num_alts == 0) {
		return -EINVAL;
	}

	return 0;
}

/* Probe/Commit length for the device's UVC version */
static uint16_t probe_len(const struct uvc_device *uvc)
{
	if (uvc->bcd_uvc >= 0x0150) {
		return 48;
	} else if (uvc->bcd_uvc >= 0x0110) {
		return 34;
	}

	return 26;
}

static int vs_ctrl(const struct device *uhc, const struct uvc_device *uvc,
		   uint8_t req, uint8_t ctrl, struct uvc_probe *probe)
{
	uint8_t type = (req == UVC_SET_CUR) ? UVC_REQ_TYPE_SET : UVC_REQ_TYPE_GET;

	return uhc_dwc3_control_transfer(uhc, type, req, ctrl << 8,
					 uvc->vs_iface, probe_len(uvc), probe);
}

int uvc_negotiate(const struct device *uhc, const struct uvc_device *uvc,
		  const struct uvc_frame *frame, struct uvc_probe *probe)
{
	int ret;

	memset(probe, 0, sizeof(*probe));
	probe->bmHint = sys_cpu_to_le16(BIT(0)); /* Keep dwFrameInterval */
	probe->bFormatIndex = frame->format_index;
	probe->bFrameIndex = frame->index;
	probe->dwFrameInterval = sys_cpu_to_le32(frame->default_interval);

	ret = vs_ctrl(uhc, uvc, UVC_SET_CUR, UVC_VS_PROBE_CONTROL, probe);
	if (ret < 0) {
		LOG_ERR("SET_CUR(PROBE) failed: %d", ret);
		return ret;
	}

	/* Read back what the device settled on */
	ret = vs_ctrl(uhc, uvc, UVC_GET_CUR, UVC_VS_PROBE_CONTROL, probe);
	if (ret < 0) {
		LOG_ERR("GET_CUR(PROBE) failed: %d", ret);
		return ret;
	}

	if (probe->bFormatIndex != frame->format_index ||
	    probe->bFrameIndex != frame->index) {
		LOG_WRN("Device chose format %u frame %u instead of %u/%u",
			probe->bFormatIndex, probe->bFrameIndex,
			frame->format_index, frame->index);
	}

	ret = vs_ctrl(uhc, uvc, UVC_SET_CUR, UVC_VS_COMMIT_CONTROL, probe);
	if (ret < 0) {
		LOG_ERR("SET_CUR(COMMIT) failed: %d", ret);
		return ret;
	}

	return 0;
}

void uvc_stream_init(struct uvc_stream *s, uint32_t max_payload,
		     uint8_t *frame_buf, size_t frame_buf_size,
		     void (*frame_done)(const uint8_t *frame, size_t len,
					bool err))
{
	memset(s, 0, sizeof(*s));
	s->max_payload = max_payload;
	s->frame_buf = frame_buf;
	s->frame_buf_size = frame_buf_size;
	s->frame_done = frame_done;
	s->last_fid = -1;
}

void uvc_stream_set_buffer(struct uvc_stream *s, uint8_t *frame_buf,
			   size_t frame_buf_size)
{
	s->frame_buf = frame_buf;
	s->frame_buf_size = frame_buf_size;
}

void uvc_stream_lost(struct uvc_stream *s)
{
	s->frame_err = true;
	s->in_payload = false;
}

static void frame_complete(struct uvc_stream *s)
{
	if (s->frame_len > 0 || s->frame_err || s->frame_buf == NULL) {
		s->frame_done(s->frame_buf, s->frame_len, s->frame_err);
	}

	s->frame_len = 0;
	s->frame_err = false;
}

static void frame_append(struct uvc_stream *s, const uint8_t *data,
			 size_t len)
{
	if (len == 0 || s->frame_buf == NULL) {
		return;
	}

	if (s->frame_len + len > s->frame_buf_size) {
		if (!s->frame_err) {
			s->overflows++;
		}
		s->frame_err = true;
		return;
	}

	memcpy(&s->frame_buf[s->frame_len], data, len);
	s->frame_len += len;
}

/*
 * A bulk payload is one header plus data, ended by a short packet or by
 * reaching dwMaxPayloadTransferSize (UVC 1.5, 2.4.3.2). Each submitted
 * transfer is a multiple of the packet size, so a payload always starts at
 * the beginning of a transfer. An isochronous payload is what arrives in one
 * service interval, so it is always one whole transfer.
 */
void uvc_stream_feed(struct uvc_stream *s, const uint8_t *data, size_t len,
		     bool short_xfer)
{
	size_t hlen = 0;

	if (!s->in_payload) {
		uint8_t info;
		int fid;

		if (len == 0) {
			/* Zero-length packet between payloads */
			return;
		}

		s->in_payload = true;
		s->payload_len = 0;
		s->skip_payload = false;

		hlen = data[0];
		if (hlen < 2 || hlen > len) {
			s->bad_headers++;
			s->skip_payload = true;
			s->frame_err = true;
			s->payload_info = 0;
		} else {
			info = data[1];
			fid = info & UVC_HDR_FID;

			/* A toggled FID starts a new frame, also when the
			 * device does not mark the previous one with EOF. If
			 * nothing of the previous frame is pending, an error
			 * flagged since (lost data) belongs to the new one.
			 */
			if (s->last_fid >= 0 && fid != s->last_fid &&
			    (s->frame_len > 0 || s->frame_buf == NULL)) {
				frame_complete(s);
			}
			s->last_fid = fid;

			if (info & UVC_HDR_ERR) {
				s->err_payloads++;
				s->frame_err = true;
			}
			s->payload_info = info;
		}
	}

	if (!s->skip_payload) {
		frame_append(s, data + hlen, len - hlen);
	}
	s->payload_len += len;

	if (short_xfer || s->payload_len >= s->max_payload) {
		s->in_payload = false;
		if (s->payload_info & UVC_HDR_EOF) {
			frame_complete(s);
		}
	}
}
