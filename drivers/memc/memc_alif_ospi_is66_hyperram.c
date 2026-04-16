/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Driver for ISSI IS66WVH HyperRAM on the Alif Ensemble OSPI (HyperBus) interface.
 *
 * The IS66 requires no special initialisation sequence — it responds to standard
 * HyperBus transactions out of power-on reset.  This driver sets up the OSPI
 * controller in HyperBus XIP mode so the RAM appears as a directly-addressable
 * memory region starting at the address given by the controller's
 * xip-base-address property.
 *
 * IMPORTANT: E7 OSPI HyperBus XIP write path does NOT support hardcoded DFS
 * (SOC_FEAT_AES_OSPI_HAS_XIP_WRITE_HC_DFS=0).  Mapping the XIP window as
 * Normal WB/RA/WA causes D-cache dirty-line evictions that land in IS66 at a
 * wrong offset (observed: 7-byte shift per 32-byte cache line).  The XIP
 * region must be mapped as Normal Non-Cacheable (mpu_regions.c) so all writes
 * bypass the D-cache and reach IS66 directly via the AXI/OSPI XIP path.
 * Single-word 32-bit reads also work correctly without cache bursts.  This
 * matches the Alif CMSIS-DFP demo_hyperram_e7.c reference, which always
 * writes in Device mode and reads in non-cacheable or cacheable mode.
 */

#define DT_DRV_COMPAT alif_is66_hyperram

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>

#include "ospi.h"
#include "ospi_hal.h"

LOG_MODULE_REGISTER(memc_alif_is66_hyperram, CONFIG_MEMC_LOG_LEVEL);

#define DEVICE_NODE     DT_NODELABEL(is66_hyperram)
#define CONTROLLER_NODE DT_PARENT(DEVICE_NODE)

/* IS66 HyperRAM uses 32-bit data frame size in XIP mode */
#define IS66_OSPI_DFS 32

struct alif_ospi_is66_config {
	struct ospi_regs     *regs;
	struct ospi_aes_regs *aes_regs;
	const struct device  *clk_dev;
	clock_control_subsys_t clkid;
	uint32_t bus_speed;
	uint8_t  cs_pin;
	uint8_t  rxds_delay;
	uint8_t  ddr_drive_edge;
	uint8_t  wait_cycles;
	const struct pinctrl_dev_config *pcfg;
#if DT_NODE_HAS_PROP(DEVICE_NODE, reset_gpios)
	struct gpio_dt_spec reset_gpio;
#endif
};

static int is66_hyperram_reset(const struct alif_ospi_is66_config *config)
{
#if DT_NODE_HAS_PROP(DEVICE_NODE, reset_gpios)
	int ret;

	if (!gpio_is_ready_dt(&config->reset_gpio)) {
		LOG_ERR("Reset GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	/* Assert reset (active low — GPIO_OUTPUT_ACTIVE drives it low) */
	ret = gpio_pin_set_dt(&config->reset_gpio, 1);
	if (ret < 0) {
		return ret;
	}

	k_busy_wait(1); /* hold for at least 200 ns */

	/* Deassert reset */
	ret = gpio_pin_set_dt(&config->reset_gpio, 0);
	if (ret < 0) {
		return ret;
	}

	k_busy_wait(150); /* tRPH: 150 µs recovery time after reset */
#else
	ARG_UNUSED(config);
#endif
	return 0;
}

static int memc_alif_ospi_is66_hyperram_init(const struct device *dev)
{
	const struct alif_ospi_is66_config *config = dev->config;
	uint32_t core_clk;
	int ret;

	if (config->bus_speed == 0) {
		LOG_ERR("OSPI bus speed cannot be zero");
		return -EINVAL;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Pinctrl apply failed: %d", ret);
		return ret;
	}

	if (!device_is_ready(config->clk_dev)) {
		LOG_ERR("Clock controller device not ready");
		return -ENODEV;
	}

#if defined(CONFIG_ENSEMBLE_GEN2)
	ret = clock_control_configure(config->clk_dev, config->clkid, NULL);
	if (ret != 0) {
		LOG_ERR("Unable to configure clock: %d", ret);
		return ret;
	}

	ret = clock_control_on(config->clk_dev, config->clkid);
	if (ret != 0) {
		LOG_ERR("Unable to enable clock: %d", ret);
		return ret;
	}
#endif

	ret = clock_control_get_rate(config->clk_dev, config->clkid, &core_clk);
	if (ret != 0) {
		LOG_ERR("Unable to get clock rate: %d", ret);
		return ret;
	}

	/* Optionally pulse nRST before touching the controller */
	ret = is66_hyperram_reset(config);
	if (ret < 0) {
		LOG_ERR("HyperRAM reset failed: %d", ret);
		return ret;
	}

	/* Set DDR drive edge */
	ospi_set_ddr_drive_edge(config->regs, config->ddr_drive_edge);

	/* Mask interrupts and assert master mode before any further config.
	 * The hardware reset value of CTRLR0.IS_MST is implementation-defined;
	 * explicitly enabling master mode ensures the controller drives the bus. */
	ospi_mask_interrupts(config->regs);
	ospi_mode_master(config->regs);

	/* Set OSPI bus speed (inline computes divisor = core_clk / bus_speed) */
	ospi_set_bus_speed(config->regs, config->bus_speed, core_clk);

	/* RxDS delay */
	aes_set_rxds_delay(config->aes_regs, config->rxds_delay);

	/* Data frame size: 32-bit for HyperBus */
	ospi_set_dfs(config->regs, IS66_OSPI_DFS);

	/* Set CTRLR0.SPI_FRF = Octal before calling ospi_hyperbus_xip_init.
	 * ospi_hyperbus_xip_init programs TRANS_TYPE=FRF_DEFINED (=2) in
	 * XIP_CTRL and XIP_WRITE_CTRL, which instructs the XIP controller to
	 * derive bus width from CTRLR0.SPI_FRF.  If SPI_FRF is left at its
	 * reset value of 0 (Standard/single SPI), every HyperBus XIP
	 * transaction is driven as single-bit SPI and the ISSI IS66 never
	 * responds — the controller stalls indefinitely. */
	ospi_disable(config->regs);
	config->regs->OSPI_CTRLR0 = (config->regs->OSPI_CTRLR0
				      & ~SPI_CTRLR0_SPI_FRF_MASK)
				    | SPI_CTRLR0_SPI_FRF_OCTAL;
	ospi_enable(config->regs);

	/* Configure HyperBus XIP (single octal, not dual-octal) */
	ospi_hyperbus_xip_init(config->regs, config->wait_cycles, false);

	/* Reduce TX data overhead by 1 clock cycle (required for correct HyperBus timing) */
	ospi_set_tx_threshold(config->regs, 0);

	/* Enable XIP slave select (OSPI_XIP_SER) — required on Ensemble E7 */
	ospi_control_xip_ss(config->regs, config->cs_pin, SPI_SS_STATE_ENABLE);

	/* Enable AES XIP so the address window becomes accessible */
	aes_enable_xip(config->aes_regs);

	/* Ensure XIP enable write reaches hardware before device is declared ready */
	barrier_dsync_fence_full();
	barrier_isync_fence_full();

	/* Diagnostic: dump key register values so hardware state can be verified.
	 * Expected: CTRLR0 bit31(IS_MST)=1, bits23:22(SPI_FRF)=3(Octal),
	 *           BAUDR = core_clk/bus_speed (e.g. 4 for 400MHz/100MHz),
	 *           XIP_CTRL bit24(HYPERBUS_EN)=1,
	 *           AES_CTRL bit4(XIP_EN)=1, AES_CLK_DIS=0. */
	LOG_ERR("IS66 init: CTRLR0=0x%08x BAUDR=%u",
		config->regs->OSPI_CTRLR0,
		config->regs->OSPI_BAUDR);
	LOG_ERR("IS66 init: XIP_CTRL=0x%08x XIP_WRITE_CTRL=0x%08x",
		config->regs->OSPI_XIP_CTRL,
		config->regs->OSPI_XIP_WRITE_CTRL);
	LOG_ERR("IS66 init: AES_CTRL=0x%08x AES_CLK_DIS=0x%08x",
		config->aes_regs->AES_CTRL,
		config->aes_regs->AES_CLK_DIS);

	LOG_DBG("IS66 HyperRAM XIP ready (bus %u Hz, wait_cycles %u)",
		config->bus_speed, config->wait_cycles);

	return 0;
}

/* PINCTRL is defined on the controller node */
PINCTRL_DT_DEFINE(CONTROLLER_NODE);

static const struct alif_ospi_is66_config is66_config = {
	.pcfg         = PINCTRL_DT_DEV_CONFIG_GET(CONTROLLER_NODE),
	.regs         = (struct ospi_regs *) DT_REG_ADDR(CONTROLLER_NODE),
	.aes_regs     = (struct ospi_aes_regs *) DT_PROP_BY_IDX(CONTROLLER_NODE, aes_reg, 0),
	.bus_speed    = DT_PROP(CONTROLLER_NODE, bus_speed),
	.cs_pin       = DT_PROP(CONTROLLER_NODE, cs_pin),
	.rxds_delay   = DT_PROP(CONTROLLER_NODE, rx_ds_delay),
	.ddr_drive_edge = DT_PROP(CONTROLLER_NODE, ddr_drive_edge),
	.wait_cycles  = DT_PROP(DEVICE_NODE, wait_cycles),
	.clk_dev      = DEVICE_DT_GET(DT_CLOCKS_CTLR(CONTROLLER_NODE)),
	.clkid        = (clock_control_subsys_t) DT_CLOCKS_CELL(CONTROLLER_NODE, clkid),
#if DT_NODE_HAS_PROP(DEVICE_NODE, reset_gpios)
	.reset_gpio   = GPIO_DT_SPEC_GET(DEVICE_NODE, reset_gpios),
#endif
};

DEVICE_DT_DEFINE(DEVICE_NODE,
		 &memc_alif_ospi_is66_hyperram_init,
		 NULL,
		 NULL,
		 &is66_config,
		 POST_KERNEL,
		 CONFIG_MEMC_INIT_PRIORITY,
		 NULL);
