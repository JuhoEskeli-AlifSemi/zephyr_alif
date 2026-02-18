/*
 * Copyright (c) 2019, Linaro Limited
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aptina_mt9m114

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/drivers/i2c.h>
#ifndef CONFIG_MT9M114_PARALLEL_INIT
#include <zephyr/drivers/gpio.h>
#endif

LOG_MODULE_REGISTER(video_mt9m114, LOG_LEVEL_DBG);

#define MT9M114_CHIP_ID_VAL 0x2481

/* Sysctl registers */
#define MT9M114_CHIP_ID                    0x0000
#define MT9M114_COMMAND_REGISTER           0x0080
#define MT9M114_COMMAND_REGISTER_SET_STATE (1 << 1)
#define MT9M114_COMMAND_REGISTER_OK        (1 << 15)
#define MT9M114_RST_AND_MISC_CONTROL       0x001A

/* Camera Control registers */
#define MT9M114_CAM_SENSOR_CFG_Y_ADDR_START     0xC800
#define MT9M114_CAM_SENSOR_CFG_X_ADDR_START     0xC802
#define MT9M114_CAM_SENSOR_CFG_Y_ADDR_END       0xC804
#define MT9M114_CAM_SENSOR_CFG_X_ADDR_END       0xC806
#define MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW   0xC818
#define MT9M114_CAM_SENSOR_CTRL_READ_MODE       0xC834
#define MT9M114_CAM_CROP_WINDOW_WIDTH           0xC858
#define MT9M114_CAM_CROP_WINDOW_HEIGHT          0xC85A
#define MT9M114_CAM_OUTPUT_WIDTH                0xC868
#define MT9M114_CAM_OUTPUT_HEIGHT               0xC86A
#define MT9M114_CAM_OUTPUT_FORMAT               0xC86C
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND   0xC918
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND   0xC91A
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND 0xC920
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND 0xC922

/* System Manager registers */
#define MT9M114_SYSMGR_NEXT_STATE 0xDC00

/* System States */
#define MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE 0x28
#define MT9M114_SYS_STATE_START_STREAMING     0x34
#define MT9M114_SYS_STATE_ENTER_SUSPEND       0x40

/* Camera output format */
#define MT9M114_CAM_OUTPUT_FORMAT_FORMAT_YUV (0 << 8)
#define MT9M114_CAM_OUTPUT_FORMAT_FORMAT_RGB (1 << 8)
#define MT9M114_CAM_OUTPUT_FORMAT_BAYER       (2 << 8)
#define MT9M114_CAM_OUTPUT_FORMAT_FORMAT_Y10P (0 << 10)
#define MT9M114_CAM_OUTPUT_FORMAT_FORMAT_GREY (3 << 10)


/* Camera control masks */
#define MT9M114_CAM_SENSOR_CTRL_HORZ_FLIP_EN BIT(0)
#define MT9M114_CAM_SENSOR_CTRL_VERT_FLIP_EN BIT(1)

/**
 * enum CPI_PIX_CLKSEL
 * CPI pixel clock source selection
 */
typedef enum _CPI_PIX_CLKSEL {
    CPI_PIX_CLKSEL_400MZ, /**< Select 400 MHz clock source (PLL_CLK1/2) */
    CPI_PIX_CLKSEL_480MZ  /**< Select 480 MHz clock source (PLL_CLK3) */
} CPI_PIX_CLKSEL;

#define RTE_MT9M114_CAMERA_SENSOR_MIPI_CSI_CLK_SCR_DIV         20

struct mt9m114_config {
	#ifndef CONFIG_MT9M114_PARALLEL_INIT
	const struct gpio_dt_spec reset_gpio;
	const struct gpio_dt_spec power_gpio;
	#endif
	struct i2c_dt_spec i2c;
};

struct mt9m114_data {
	struct video_format fmt;
};

struct mt9m114_reg {
	uint16_t addr;
	uint16_t value_size;
	uint32_t value;
};

struct mt9m114_resolution_config {
	uint16_t width;
	uint16_t height;
	struct mt9m114_reg *params;
};

#if defined(CONFIG_MT9M114_PARALLEL_INIT)

static struct mt9m114_reg mt9m114_init_config[] = {
	{0x098E, 2, 0x1000},    /* LOGICAL_ADDRESS_ACCESS */
	{0xC97E, 2, 0x01},      /* CAM_SYSCTL_PLL_ENABLE */
	{0xC980, 2, 0x0120},    /* CAM_SYSCTL_PLL_DIVIDER_M_N = 288 */
	{0xC982, 2, 0x0700},    /* CAM_SYSCTL_PLL_DIVIDER_P = 1792 */
	{0xC984, 2, 0x8000},    /* CAM_PORT_OUTPUT_CONTROL = 32768 (No pixel clock slow down) */
	{0xC808, 4, 0x2DC6C00}, /* CAM_SENSOR_CFG_PIXCLK = 48000000 */
	{0x316A, 2, 0x8270},    /* Auto txlo_row for hot pixel and linear full well optimization */
	{0x316C, 2, 0x8270},    /* Auto txlo for hot pixel and linear full well optimization */
	{0x3ED0, 2, 0x2305},    /* Eclipse setting, ecl range=1, ecl value=2, ivln=3 */
	{0x3ED2, 2, 0x77CF},    /* TX_hi = 12 */
	{0x316E, 2, 0x8202},    /* Auto ecl, threshold 2x */
	{0x3180, 2, 0x87FF},    /* Enable delta dark */
	{0x30D4, 2, 0x6080},    /* Disable column correction due to AE oscillation problem */
	{0xA802, 2, 0x0008},    /* RESERVED_AE_TRACK_02 */
	{0x3E14, 2, 0xFF39},    /* Enabling pixout clamping to VAA to solve column band issue */
	{0xC80C, 2, 0x0001},    /* CAM_SENSOR_CFG_ROW_SPEED */
	{0xC80E, 2, 0x01C3},    /* CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN = 451 */
	{0xC810, 2, 0x28F8},    /* CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX = 10488 */
	{0xC812, 2, 0x036C},    /* CAM_SENSOR_CFG_FRAME_LENGTH_LINES = 876 */
	{0xC814, 2, 0x29E3},    /* CAM_SENSOR_CFG_LINE_LENGTH_PCK = 10723 */
	{0xC816, 2, 0x00E0},    /* CAM_SENSOR_CFG_FINE_CORRECTION = 224 */
	{0xC826, 2, 0x0020},    /* CAM_SENSOR_CFG_REG_0_DATA = 32 */
	{0xC834, 2, 0x0333},    /* CAM_SENSOR_CONTROL_READ_MODE = 819, H and V flip */
	{0xC854, 2, 0x0000},    /* CAM_CROP_WINDOW_XOFFSET = 0 */
	{0xC856, 2, 0x0000},    /* CAM_CROP_WINDOW_YOFFSET = 0 */
	{0xC85C, 1, 0x03},      /* CAM_CROP_CROPMODE = 3 */
	{0xC878, 1, 0x00},      /* CAM_AET_AEMODE = 0 */
	{0xC88C, 2, 0x051C},    /* CAM_AET_MAX_FRAME_RATE = 1308, (5 fps) */
	{0xC88E, 2, 0x051C},    /* CAM_AET_MIN_FRAME_RATE = 1308, (5 fps) */
	{0xC914, 2, 0x0000},    /* CAM_STAT_AWB_CLIP_WINDOW_XSTART = 0 */
	{0xC916, 2, 0x0000},    /* CAM_STAT_AWB_CLIP_WINDOW_YSTART = 0 */
	{0xC91C, 2, 0x0000},    /* CAM_STAT_AE_INITIAL_WINDOW_XSTART = 0 */
	{0xC91E, 2, 0x0000},    /* CAM_STAT_AE_INITIAL_WINDOW_YSTART = 0 */
	{/* NULL terminated */}};

#else

static struct mt9m114_reg mt9m114_init_config[] = {
	{0x098E, 2, 0x1000},    /* LOGICAL_ADDRESS_ACCESS */
	{0xC97E, 1, 0x01},      /* CAM_SYSCTL_PLL_ENABLE */
	{0xC980, 2, 0x0120},    /* CAM_SYSCTL_PLL_DIVIDER_M_N = 288 */
	{0xC982, 2, 0x0700},    /* CAM_SYSCTL_PLL_DIVIDER_P = 1792 */
	{0xC808, 4, 0x2DC6C00}, /* CAM_SENSOR_CFG_PIXCLK = 48 Mhz */
	{0x316A, 2, 0x8270},    /* Auto txlo_row for hot pixel and linear full well optimization */
	{0x316C, 2, 0x8270},    /* Auto txlo for hot pixel and linear full well optimization */
	{0x3ED0, 2, 0x2305},    /* Eclipse setting, ecl range=1, ecl value=2, ivln=3 */
	{0x3ED2, 2, 0x77CF},    /* TX_hi = 12 */
	{0x316E, 2, 0x8202}, /* Auto ecl , threshold 2x, ecl=0 at high gain, ecl=2 for low gain */
	{0x3180, 2, 0x87FF}, /* Enable delta dark */
	{0x30D4, 2, 0x6080}, /* Disable column correction due to AE oscillation problem */
	{0xA802, 2, 0x0008}, /* RESERVED_AE_TRACK_02 */
	{0x3E14, 2, 0xFF39}, /* Enabling pixout clamping to VAA to solve column band issue */
	{0xC80C, 2, 0x0001}, /* CAM_SENSOR_CFG_ROW_SPEED */
	{0xC80E, 2, 0x00DB}, /* CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN = 219 */
	{0xC810, 2, 0x07C2}, /* CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX = 1986 */
	{0xC812, 2, 0x02FE}, /* CAM_SENSOR_CFG_FRAME_LENGTH_LINES = 766 */
	{0xC814, 2, 0x0845}, /* CAM_SENSOR_CFG_LINE_LENGTH_PCK = 2117 */
	{0xC816, 2, 0x0060}, /* CAM_SENSOR_CFG_FINE_CORRECTION = 96 */
	{0xC826, 2, 0x0020}, /* CAM_SENSOR_CFG_REG_0_DATA = 32 */
	{0xC834, 2, 0x0000}, /* CAM_SENSOR_CONTROL_READ_MODE */
	{0xC854, 2, 0x0000}, /* CAM_CROP_WINDOW_XOFFSET */
	{0xC856, 2, 0x0000}, /* CAM_CROP_WINDOW_YOFFSET */
	{0xC85C, 1, 0x03},   /* CAM_CROP_CROPMODE */
	{0xC878, 1, 0x00},   /* CAM_AET_AEMODE */
	{0xC88C, 2, 0x1D9A}, /* CAM_AET_MAX_FRAME_RATE = 7578 */
	{0xC88E, 2, 0x1D9A}, /* CAM_AET_MIN_FRAME_RATE = 7578 */
	{0xC914, 2, 0x0000}, /* CAM_STAT_AWB_CLIP_WINDOW_XSTART */
	{0xC916, 2, 0x0000}, /* CAM_STAT_AWB_CLIP_WINDOW_YSTART */
	{0xC91C, 2, 0x0000}, /* CAM_STAT_AE_INITIAL_WINDOW_XSTART */
	{0xC91E, 2, 0x0000}, /* CAM_STAT_AE_INITIAL_WINDOW_YSTART */
	{0x001E, 2, 0x0777}, /* REG_PAD_SLEW */
	{0xC86E, 2, 0x0038}, /* CAM_OUTPUT_FORMAT_YUV_CLIP for CSI */
	{0xC984, 2, 0x8000}, /* CAM_PORT_OUTPUT_CONTROL, for MIPI CSI-2 interface : 0x8000 */
	{/* NULL terminated */}};

#endif

static struct mt9m114_reg mt9m114_480_272[] = {
	{MT9M114_CAM_SENSOR_CFG_Y_ADDR_START, 2, 0x00D4},     /* 212 */
	{MT9M114_CAM_SENSOR_CFG_X_ADDR_START, 2, 0x00A4},     /* 164 */
	{MT9M114_CAM_SENSOR_CFG_Y_ADDR_END, 2, 0x02FB},       /* 763 */
	{MT9M114_CAM_SENSOR_CFG_X_ADDR_END, 2, 0x046B},       /* 1131 */
	{MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW, 2, 0x0223},   /* 547 */
	{MT9M114_CAM_CROP_WINDOW_WIDTH, 2, 0x03C0},           /* 960 */
	{MT9M114_CAM_CROP_WINDOW_HEIGHT, 2, 0x0220},          /* 544 */
	{MT9M114_CAM_OUTPUT_WIDTH, 2, 0x01E0},                /* 480 */
	{MT9M114_CAM_OUTPUT_HEIGHT, 2, 0x0110},               /* 272 */
	{MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND, 2, 0x01DF},   /* 479 */
	{MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND, 2, 0x010F},   /* 271 */
	{MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND, 2, 0x005F}, /* 95 */
	{MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND, 2, 0x0035}, /* 53 */
	{/* NULL terminated */}};

static struct mt9m114_reg mt9m114_640_480[] = {
	{MT9M114_CAM_SENSOR_CFG_Y_ADDR_START, 2, 0x0000},     /* 0 */
	{MT9M114_CAM_SENSOR_CFG_X_ADDR_START, 2, 0x0000},     /* 0 */
	{MT9M114_CAM_SENSOR_CFG_Y_ADDR_END, 2, 0x03CD},       /* 973 */
	{MT9M114_CAM_SENSOR_CFG_X_ADDR_END, 2, 0x050D},       /* 1293 */
	{MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW, 2, 0x01E3},   /* 483 */
	{MT9M114_CAM_CROP_WINDOW_WIDTH, 2, 0x0280},           /* 640 */
	{MT9M114_CAM_CROP_WINDOW_HEIGHT, 2, 0x01E0},          /* 480 */
	{MT9M114_CAM_OUTPUT_WIDTH, 2, 0x0280},                /* 640 */
	{MT9M114_CAM_OUTPUT_HEIGHT, 2, 0x01E0},               /* 480 */
	{MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND, 2, 0x027F},   /* 639 */
	{MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND, 2, 0x01DF},   /* 479 */
	{MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND, 2, 0x007F}, /* 127 */
	{MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND, 2, 0x005F}, /* 95 */
	{/* NULL terminated */}};

static struct mt9m114_reg mt9m114_1280_720[] = {
	{MT9M114_CAM_SENSOR_CFG_Y_ADDR_START, 2, 0x007C},     /* 124 */
	{MT9M114_CAM_SENSOR_CFG_X_ADDR_START, 2, 0x0004},     /* 4 */
	{MT9M114_CAM_SENSOR_CFG_Y_ADDR_END, 2, 0x0353},       /* 851 */
	{MT9M114_CAM_SENSOR_CFG_X_ADDR_END, 2, 0x050B},       /* 1291 */
	{MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW, 2, 0x02D3},   /* 723 */
	{MT9M114_CAM_CROP_WINDOW_WIDTH, 2, 0x0500},           /* 1280 */
	{MT9M114_CAM_CROP_WINDOW_HEIGHT, 2, 0x02D0},          /* 720 */
	{MT9M114_CAM_OUTPUT_WIDTH, 2, 0x0500},                /* 1280 */
	{MT9M114_CAM_OUTPUT_HEIGHT, 2, 0x02D0},               /* 720 */
	{MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND, 2, 0x04FF},   /* 1279 */
	{MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND, 2, 0x02CF},   /* 719 */
	{MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND, 2, 0x00FF}, /* 255 */
	{MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND, 2, 0x008F}, /* 143 */
	{/* NULL terminated */}};

static struct mt9m114_resolution_config resolutionConfigs[] = {
	{.width = 480, .height = 272, .params = mt9m114_480_272},
	{.width = 640, .height = 480, .params = mt9m114_640_480},
	{.width = 1280, .height = 720, .params = mt9m114_1280_720},
};

#define MT9M114_VIDEO_FORMAT_CAP(width, height, format)                                            \
	{                                                                                          \
		.pixelformat = (format), .width_min = (width), .width_max = (width),               \
		.height_min = (height), .height_max = (height), .width_step = 0, .height_step = 0  \
	}

static const struct video_format_cap fmts[] = {
	MT9M114_VIDEO_FORMAT_CAP(480, 272, VIDEO_PIX_FMT_RGB565),
	MT9M114_VIDEO_FORMAT_CAP(480, 272, VIDEO_PIX_FMT_YUYV),
	MT9M114_VIDEO_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_RGB565),
	MT9M114_VIDEO_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_YUYV),
	MT9M114_VIDEO_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_Y10P),
	MT9M114_VIDEO_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_GREY),
	MT9M114_VIDEO_FORMAT_CAP(1280, 720, VIDEO_PIX_FMT_RGB565),
	MT9M114_VIDEO_FORMAT_CAP(1280, 720, VIDEO_PIX_FMT_YUYV),
	{0}};

static inline int i2c_burst_read16_dt(const struct i2c_dt_spec *spec, uint16_t start_addr,
				      uint8_t *buf, uint32_t num_bytes)
{
	uint8_t addr_buffer[2];

	addr_buffer[1] = start_addr & 0xFF;
	addr_buffer[0] = start_addr >> 8;
	return i2c_write_read_dt(spec, addr_buffer, sizeof(addr_buffer), buf, num_bytes);
}

static inline int i2c_burst_write16_dt(const struct i2c_dt_spec *spec, uint16_t start_addr,
				       const uint8_t *buf, uint32_t num_bytes)
{
	uint8_t addr_buffer[2];
	struct i2c_msg msg[2];

	addr_buffer[1] = start_addr & 0xFF;
	addr_buffer[0] = start_addr >> 8;
	msg[0].buf = addr_buffer;
	msg[0].len = 2U;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer_dt(spec, msg, 2);
}

static int mt9m114_write_reg(const struct device *dev, uint16_t reg_addr, uint8_t reg_size,
			     void *value)
{
	const struct mt9m114_config *cfg = dev->config;

	switch (reg_size) {
	case 2:
		*(uint16_t *)value = sys_cpu_to_be16(*(uint16_t *)value);
		break;
	case 4:
		*(uint32_t *)value = sys_cpu_to_be32(*(uint32_t *)value);
		break;
	case 1:
		break;
	default:
		return -ENOTSUP;
	}

	return i2c_burst_write16_dt(&cfg->i2c, reg_addr, value, reg_size);
}

static int mt9m114_read_reg(const struct device *dev, uint16_t reg_addr, uint8_t reg_size,
			    void *value)
{
	const struct mt9m114_config *cfg = dev->config;
	int err;

	if (reg_size > 4) {
		return -ENOTSUP;
	}

	err = i2c_burst_read16_dt(&cfg->i2c, reg_addr, value, reg_size);
	if (err) {
		return err;
	}

	switch (reg_size) {
	case 2:
		*(uint16_t *)value = sys_be16_to_cpu(*(uint16_t *)value);
		break;
	case 4:
		*(uint32_t *)value = sys_be32_to_cpu(*(uint32_t *)value);
		break;
	case 1:
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int mt9m114_modify_reg(const struct device *dev, const uint16_t addr,
			      uint8_t reg_size, const uint32_t mask, const uint32_t val)
{
	uint32_t oldVal = 0;
	uint32_t newVal = 0;

	int ret = mt9m114_read_reg(dev, addr, reg_size, &oldVal);

	if (ret) {
		return ret;
	}

	newVal = (oldVal & ~mask) | (val & mask);

	return mt9m114_write_reg(dev, addr, reg_size, &newVal);
}

static int mt9m114_write_all(const struct device *dev, struct mt9m114_reg *reg)
{
	int i = 0;

	while (reg[i].value_size) {
		int err;

		err = mt9m114_write_reg(dev, reg[i].addr, reg[i].value_size, &reg[i].value);
		if (err) {
			return err;
		}

		i++;
	}

	return 0;
}

static int mt9m114_software_reset(const struct device *dev)
{
	// int32_t camera_sensor_i2c_write(CAMERA_SENSOR_SLAVE_I2C_CONFIG *i2c, uint32_t reg_addr,
	// uint32_t reg_value, CAMERA_SENSOR_I2C_REG_SIZE reg_size)
	//camera_sensor_i2c_write(i2c_cfg, MT9M114_SYSCTL_REGISTER_RESET_AND_MISC_CONTROL, 0x0001, 2);

	// static int mt9m114_modify_reg(const struct device *dev, const uint16_t addr,
	// uint8_t reg_size, const uint32_t mask, const uint32_t val)
	int ret = mt9m114_modify_reg(dev, MT9M114_RST_AND_MISC_CONTROL, 2, 0x01, 0x01);

	if (ret) {
		return ret;
	}

	k_sleep(K_MSEC(10));

	ret = mt9m114_modify_reg(dev, MT9M114_RST_AND_MISC_CONTROL, 2, 0x01, 0x00);
	if (ret) {
		return ret;
	}

	k_sleep(K_MSEC(100));

	return 0;
}

static int mt9m114_set_state(const struct device *dev, uint8_t state)
{
	uint16_t val;
	int err;

	/* Set next state. */
	mt9m114_write_reg(dev, MT9M114_SYSMGR_NEXT_STATE, 1, &state);

	/* Check that the FW is ready to accept a new command. */
	while (1) {
		err = mt9m114_read_reg(dev, MT9M114_COMMAND_REGISTER, 2, &val);
		if (err) {
			return err;
		}

		if (!(val & MT9M114_COMMAND_REGISTER_SET_STATE)) {
			break;
		}

		k_sleep(K_MSEC(1));
	}

	/* Issue the Set State command. */
	val = MT9M114_COMMAND_REGISTER_SET_STATE | MT9M114_COMMAND_REGISTER_OK;
	mt9m114_write_reg(dev, MT9M114_COMMAND_REGISTER, 2, &val);

	/* Wait for the FW to complete the command. */
	while (1) {
		err = mt9m114_read_reg(dev, MT9M114_COMMAND_REGISTER, 2, &val);
		if (err) {
			return err;
		}

		if (!(val & MT9M114_COMMAND_REGISTER_SET_STATE)) {
			break;
		}

		k_sleep(K_MSEC(1));
	}

	/* Check the 'OK' bit to see if the command was successful. */
	err = mt9m114_read_reg(dev, MT9M114_COMMAND_REGISTER, 2, &val);
	if (err || !(val & MT9M114_COMMAND_REGISTER_OK)) {
		return -EIO;
	}

	return 0;
}

static int mt9m114_set_output_format(const struct device *dev, int pixel_format)
{
	int ret = 0;
	uint16_t output_format;

	if (pixel_format == VIDEO_PIX_FMT_YUYV) {
		output_format = (MT9M114_CAM_OUTPUT_FORMAT_FORMAT_YUV | (1U << 1U));
	} else if (pixel_format == VIDEO_PIX_FMT_RGB565) {
		output_format = (MT9M114_CAM_OUTPUT_FORMAT_FORMAT_RGB | (1U << 1U));
	} else if (pixel_format == VIDEO_PIX_FMT_Y10P) {
		output_format = (MT9M114_CAM_OUTPUT_FORMAT_FORMAT_Y10P |
				MT9M114_CAM_OUTPUT_FORMAT_BAYER);
	} else if (pixel_format == VIDEO_PIX_FMT_GREY) {
		output_format = (MT9M114_CAM_OUTPUT_FORMAT_FORMAT_GREY |
				MT9M114_CAM_OUTPUT_FORMAT_BAYER);
	}

	ret = mt9m114_write_reg(dev, MT9M114_CAM_OUTPUT_FORMAT, sizeof(output_format),
				&output_format);

	return ret;
}

static int mt9m114_set_fmt(const struct device *dev, enum video_endpoint_id ep,
			   struct video_format *fmt)
{
	struct mt9m114_data *drv_data = dev->data;
	int ret;
	int i = 0;

	while (fmts[i].pixelformat) {
		if (fmt->pixelformat == fmts[i].pixelformat && fmt->width >= fmts[i].width_min &&
		    fmt->width <= fmts[i].width_max && fmt->height >= fmts[i].height_min &&
		    fmt->height <= fmts[i].height_max) {
			break;
		}
		i++;
	}

	if (i == (ARRAY_SIZE(fmts) - 1)) {
		LOG_ERR("Unsupported pixel format or resolution");
		return -ENOTSUP;
	}

	if (!memcmp(&drv_data->fmt, fmt, sizeof(drv_data->fmt))) {
		/* nothing to do */
		return 0;
	}

	drv_data->fmt = *fmt;

	/* Set output pixel format */
	ret = mt9m114_set_output_format(dev, fmt->pixelformat);
	if (ret) {
		LOG_ERR("Unable to set pixel format");
		return ret;
	}

	/* Set output resolution */
	for (i = 0; i < ARRAY_SIZE(resolutionConfigs); i++) {
		if (fmt->width == resolutionConfigs[i].width &&
		    fmt->height == resolutionConfigs[i].height) {
			ret = mt9m114_write_all(dev, resolutionConfigs[i].params);
			if (ret) {
				LOG_ERR("Unable to set resolution");
				return ret;
			}

			break;
		}
	}

	/* Apply Config */
	return mt9m114_set_state(dev, MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
}

static int mt9m114_get_fmt(const struct device *dev, enum video_endpoint_id ep,
			   struct video_format *fmt)
{
	struct mt9m114_data *drv_data = dev->data;

	*fmt = drv_data->fmt;

	return 0;
}

static int mt9m114_set_stream(const struct device *dev, bool enable)
{
	return enable ? mt9m114_set_state(dev, MT9M114_SYS_STATE_START_STREAMING)
		      : mt9m114_set_state(dev, MT9M114_SYS_STATE_ENTER_SUSPEND);
}

static int mt9m114_get_caps(const struct device *dev, enum video_endpoint_id ep,
			    struct video_caps *caps)
{
	caps->format_caps = fmts;
	return 0;
}

static int mt9m114_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	int ret = 0;

	switch (cid) {
	case VIDEO_CID_HFLIP:
		ret = mt9m114_modify_reg(dev, MT9M114_CAM_SENSOR_CTRL_READ_MODE, 2,
					MT9M114_CAM_SENSOR_CTRL_HORZ_FLIP_EN,
					(int)value ? MT9M114_CAM_SENSOR_CTRL_HORZ_FLIP_EN : 0);
		break;
	case VIDEO_CID_VFLIP:
		ret = mt9m114_modify_reg(dev, MT9M114_CAM_SENSOR_CTRL_READ_MODE, 2,
					MT9M114_CAM_SENSOR_CTRL_VERT_FLIP_EN,
					(int)value ? MT9M114_CAM_SENSOR_CTRL_VERT_FLIP_EN : 0);
		break;
	default:
		return -ENOTSUP;
	}

	if (ret < 0) {
		return ret;
	}

	/* Apply Config */
	return mt9m114_set_state(dev, MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
}

static DEVICE_API(video, mt9m114_driver_api) = {
	.set_format = mt9m114_set_fmt,
	.get_format = mt9m114_get_fmt,
	.get_caps = mt9m114_get_caps,
	.set_stream = mt9m114_set_stream,
	.set_ctrl = mt9m114_set_ctrl,
};

#define CLKCTL_PER_MST_BASE       0x4903F000UL
typedef struct { /*!< (@ 0x4903F000) CLKCTL_PER_MST Structure                                   */
    __IOM uint32_t CAMERA_PIXCLK_CTRL; /*!< (@ 0x00000000) CPI Pixel Clock Control Register */
    __IOM uint32_t CDC200_PIXCLK_CTRL; /*!< (@ 0x00000004) CDC Pixel Clock Control Register */
    __IOM uint32_t CSI_PIXCLK_CTRL;    /*!< (@ 0x00000008) CSI Pixel Clock Control Register    */
    __IOM uint32_t PERIPH_CLK_ENA;     /*!< (@ 0x0000000C) Peripheral Clock Enable Register     */
    __IOM uint32_t DPHY_PLL_CTRL0;     /*!< (@ 0x00000010) MIPI-DPHY PLL Control Register 0     */
    __IOM uint32_t DPHY_PLL_CTRL1;     /*!< (@ 0x00000014) MIPI-DPHY PLL Control Register 1     */
    __IOM uint32_t DPHY_PLL_CTRL2;     /*!< (@ 0x00000018) MIPI-DPHY PLL Control Register 2     */
    __IM uint32_t  RESERVED;
    __IM uint32_t  DPHY_PLL_STAT0; /*!< (@ 0x00000020) MIPI-DPHY PLL Status Register 0 */
    __IM uint32_t  DPHY_PLL_STAT1; /*!< (@ 0x00000024) MIPI-DPHY PLL Status Register 1 */
    __IM uint32_t  RESERVED1[2];
    __IOM uint32_t TX_DPHY_CTRL0; /*!< (@ 0x00000030) MIPI-DPHY TX Control Register 0 */
    __IOM uint32_t TX_DPHY_CTRL1; /*!< (@ 0x00000034) MIPI-DPHY TX Control Register 1 */
    __IOM uint32_t RX_DPHY_CTRL0; /*!< (@ 0x00000038) MIPI-DPHY RX Control Register 0 */
    __IOM uint32_t RX_DPHY_CTRL1; /*!< (@ 0x0000003C) MIPI-DPHY RX Control Register 1 */
    __IOM uint32_t MIPI_CKEN;     /*!< (@ 0x00000040) MIPI-DPHY Clock Enable Register     */
    __IOM uint32_t DSI_CTRL;      /*!< (@ 0x00000044) DSI Control Register      */
    __IM uint32_t  RESERVED2[10];
    __IOM uint32_t DMA_CTRL;       /*!< (@ 0x00000070) DMA0 Boot Control Register       */
    __IOM uint32_t DMA_IRQ;        /*!< (@ 0x00000074) DMA0 Boot IRQ Non-Secure Register        */
    __IOM uint32_t DMA_PERIPH;     /*!< (@ 0x00000078) DMA0 Boot Peripheral Non-Secure Register     */
    __IOM uint32_t DMA_GLITCH_FLT; /*!< (@ 0x0000007C) DMA0 Glitch Filter Register */
    __IOM uint32_t ETH_CTRL0;      /*!< (@ 0x00000080) ETH Control Register      */
    __IM uint32_t  ETH_STAT0;      /*!< (@ 0x00000084) ETH Status Register      */
    __IM uint32_t  ETH_PTP_TMST0;  /*!< (@ 0x00000088) ETH Timestamp Register 0  */
    __IM uint32_t  ETH_PTP_TMST1;  /*!< (@ 0x0000008C) ETH Timestamp Register 1  */
    __IOM uint32_t SDC_CTRL0;      /*!< (@ 0x00000090) SDMMC Control Register      */
    __IM uint32_t  SDC_STAT0;      /*!< (@ 0x00000094) SDMMC Status Register 0      */
    __IM uint32_t  SDC_STAT1;      /*!< (@ 0x00000098) SDMMC Status Register 1      */
    __IM uint32_t  RESERVED3;
    __IOM uint32_t USB_GPIO0; /*!< (@ 0x000000A0) USB GPIO Register */
    __IM uint32_t  USB_STAT0; /*!< (@ 0x000000A4) USB Status Register */
    __IOM uint32_t USB_CTRL1; /*!< (@ 0x000000A8) USB Control Register 1 */
    __IOM uint32_t USB_CTRL2; /*!< (@ 0x000000AC) USB Control Register 2 */
} CLKCTL_PER_MST_Type;        /*!< Size = 176 (0xb0)        */
#define CLKCTL_PER_MST            ((CLKCTL_PER_MST_Type *) CLKCTL_PER_MST_BASE)


#define PERIPH_CLK_ENA_CPI_CKEN           (1U << 0) /* Enable clock supply for CPI */

/* CLKCTL_PER_MST CAMERA_PIXCLK_CTRL field definitions */
#define CAMERA_PIXCLK_CTRL_CKEN           (1U << 0) /* Camera Pixel clock enables */
#define CAMERA_PIXCLK_CTRL_CLK_SEL        (1U << 4) /* Camera Pixel clock select  */
#define CAMERA_PIXCLK_CTRL_DIVISOR_Pos    16U       /* Camera Pixel clock divisor */
#define CAMERA_PIXCLK_CTRL_DIVISOR_Msk    (0x1FF << CAMERA_PIXCLK_CTRL_DIVISOR_Pos)

/**
  \fn          void set_cpi_pixel_clk(CPI_PIX_CLKSEL clksel, uint32_t div)
  \brief       Set cpi pixel clock.
  param[in]    clksel pixel clock source to select.
  param[in]    div    pixel clock divisor.
  \return      none.
*/
static inline void set_cpi_pixel_clk(CPI_PIX_CLKSEL clksel, uint32_t div)
{

	uint32_t reg = sys_read32((uintptr_t)&CLKCTL_PER_MST->CAMERA_PIXCLK_CTRL);
	reg &= ~CAMERA_PIXCLK_CTRL_DIVISOR_Msk;
	reg |= (div << CAMERA_PIXCLK_CTRL_DIVISOR_Pos);

	switch (clksel) {
	case CPI_PIX_CLKSEL_400MZ:
		reg &= ~CAMERA_PIXCLK_CTRL_CLK_SEL;
		break;
	case CPI_PIX_CLKSEL_480MZ:
		reg |= CAMERA_PIXCLK_CTRL_CLK_SEL;
		break;
	default:
		break;
	}

	reg |= CAMERA_PIXCLK_CTRL_CKEN;
	sys_write32(reg, (uintptr_t)&CLKCTL_PER_MST->CAMERA_PIXCLK_CTRL);
}

#ifndef CONFIG_MT9M114_PARALLEL_INIT
static int mt9m114_power_on(const struct device *dev) {
 	int32_t ret = 0;

	const struct mt9m114_config *cfg = dev->config;
	if(!cfg->power_gpio.port || !cfg->reset_gpio.port) {
		LOG_WRN("No power/reset GPIO defined, skipping power on sequence");
		return -1;
	}
	
	/* Following the GPIO reset sequence as per the datasheet. */
	
	ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure reset GPIO. ret - %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure power GPIO. ret - %d", ret);
		return ret;
	}

	gpio_pin_set_dt(&cfg->power_gpio, 0);

	gpio_pin_set_dt(&cfg->reset_gpio, 1);

	gpio_pin_set_dt(&cfg->power_gpio, 1);
	
    /*Enable camera sensor clock source config*/
    //MT9M114_Sensor_Enable_Clk_Src();
	#if 1
	set_cpi_pixel_clk(CPI_PIX_CLKSEL_400MZ, RTE_MT9M114_CAMERA_SENSOR_MIPI_CSI_CLK_SCR_DIV);
	#endif
	
	gpio_pin_set_dt(&cfg->reset_gpio, 0);	

    k_sleep(K_MSEC(50));

	gpio_pin_set_dt(&cfg->reset_gpio, 1);	

    return 0;
}
#endif

static int mt9m114_init(const struct device *dev)
{
	struct video_format fmt;
	uint16_t val;
	int ret;

#ifndef CONFIG_MT9M114_PARALLEL_INIT
	ret = mt9m114_power_on(dev);
	if (ret) {
		LOG_ERR("Unable to power on the camera");
		return ret;
	}

	k_sleep(K_MSEC(50));
#else
	/* no power control, wait for camera ready */
	k_sleep(K_MSEC(100));
	#endif	

	/* SW reset */
	mt9m114_software_reset(dev);

	ret = mt9m114_read_reg(dev, MT9M114_CHIP_ID, sizeof(val), &val);
	if (ret) {
		LOG_ERR("Unable to read chip ID");
		return -ENODEV;
	}

	if (val != MT9M114_CHIP_ID_VAL) {
		LOG_ERR("Wrong ID: %04x (exp %04x)", val, MT9M114_CHIP_ID_VAL);
		return -ENODEV;
	}

	/* Init registers */
	ret = mt9m114_write_all(dev, mt9m114_init_config);
	if (ret) {
		LOG_ERR("Unable to initialize mt9m114 config");
		return ret;
	}

	/* Set default format to 480x272 RGB565 */
	fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
	fmt.width = 480;
	fmt.height = 272;
	fmt.pitch = fmt.width * 2;

	ret = mt9m114_set_fmt(dev, VIDEO_EP_OUT, &fmt);
	if (ret) {
		LOG_ERR("Unable to configure default format");
		return -EIO;
	}

	/* Suspend any stream */
	mt9m114_set_state(dev, MT9M114_SYS_STATE_ENTER_SUSPEND);

	return 0;
}

#if 0 /* Unique Instance */

static const struct mt9m114_config mt9m114_cfg_0 = {
	.i2c = I2C_DT_SPEC_INST_GET(0),
};

static struct mt9m114_data mt9m114_data_0;

static int mt9m114_init_0(const struct device *dev)
{
	const struct mt9m114_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	return mt9m114_init(dev);
}

EVICE_DT_INST_DEFINE(0, &mt9m114_init_0, NULL, &mt9m114_data_0, &mt9m114_cfg_0, POST_KERNEL,
 		      CONFIG_VIDEO_INIT_PRIORITY, &mt9m114_driver_api);
#endif

#define MT9M114_DEVICE_DEFINE(i)                                            \
    static const struct mt9m114_config mt9m114_cfg_##i = {                  \
        .i2c = I2C_DT_SPEC_INST_GET(i),                                     \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(i, reset_gpios, {}),         \
        .power_gpio = GPIO_DT_SPEC_INST_GET_OR(i, power_gpios, {}),         \
        /* Add any other DT-derived config here, e.g.:                      \
         * .mclk_freq = DT_INST_PROP(i, mclk_frequency),                    \
         * .bus_type  = DT_INST_ENUM_IDX(i, bus_type),                      \
         */                                                                 \
    };                                                                      \
                                                                            \
    static struct mt9m114_data mt9m114_data_##i;                            \
                                                                            \
    DEVICE_DT_INST_DEFINE(i,                                                \
        mt9m114_init,                                                       \
        NULL,                                                               \
        &mt9m114_data_##i,                                                  \
        &mt9m114_cfg_##i,                                                   \
        POST_KERNEL,                                                        \
        CONFIG_VIDEO_INIT_PRIORITY,                                         \
        &mt9m114_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MT9M114_DEVICE_DEFINE)
