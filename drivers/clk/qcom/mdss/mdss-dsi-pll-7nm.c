/*
 * Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/delay.h>
#include "mdss-dsi-pll.h"
#include "mdss-pll.h"
#include <dt-bindings/clock/mdss-10nm-pll-clk.h>

#define VCO_DELAY_USEC 1

#define MHZ_250		250000000UL
#define MHZ_500		500000000UL
#define MHZ_1000	1000000000UL
#define MHZ_1100	1100000000UL
#define MHZ_1900	1900000000UL
#define MHZ_3000	3000000000UL

/* Register Offsets from PLL base address */
#define PLL_ANALOG_CONTROLS_ONE			0x0000
#define PLL_ANALOG_CONTROLS_TWO			0x0004
#define PLL_INT_LOOP_SETTINGS			0x0008
#define PLL_INT_LOOP_SETTINGS_TWO		0x000C
#define PLL_ANALOG_CONTROLS_THREE		0x0010
#define PLL_ANALOG_CONTROLS_FOUR		0x0014
#define PLL_ANALOG_CONTROLS_FIVE		0x0018
#define PLL_INT_LOOP_CONTROLS			0x001C
#define PLL_DSM_DIVIDER				0x0020
#define PLL_FEEDBACK_DIVIDER			0x0024
#define PLL_SYSTEM_MUXES			0x0028
#define PLL_FREQ_UPDATE_CONTROL_OVERRIDES	0x002C
#define PLL_CMODE				0x0030
#define PLL_PSM_CTRL				0x0034
#define PLL_RSM_CTRL				0x0038
#define PLL_VCO_TUNE_MAP			0x003C
#define PLL_PLL_CNTRL				0x0040
#define PLL_CALIBRATION_SETTINGS		0x0044
#define PLL_BAND_SEL_CAL_TIMER_LOW		0x0048
#define PLL_BAND_SEL_CAL_TIMER_HIGH		0x004C
#define PLL_BAND_SEL_CAL_SETTINGS		0x0050
#define PLL_BAND_SEL_MIN			0x0054
#define PLL_BAND_SEL_MAX			0x0058
#define PLL_BAND_SEL_PFILT			0x005C
#define PLL_BAND_SEL_IFILT			0x0060
#define PLL_BAND_SEL_CAL_SETTINGS_TWO		0x0064
#define PLL_BAND_SEL_CAL_SETTINGS_THREE		0x0068
#define PLL_BAND_SEL_CAL_SETTINGS_FOUR		0x006C
#define PLL_BAND_SEL_ICODE_HIGH			0x0070
#define PLL_BAND_SEL_ICODE_LOW			0x0074
#define PLL_FREQ_DETECT_SETTINGS_ONE		0x0078
#define PLL_FREQ_DETECT_THRESH			0x007C
#define PLL_FREQ_DET_REFCLK_HIGH		0x0080
#define PLL_FREQ_DET_REFCLK_LOW			0x0084
#define PLL_FREQ_DET_PLLCLK_HIGH		0x0088
#define PLL_FREQ_DET_PLLCLK_LOW			0x008C
#define PLL_PFILT				0x0090
#define PLL_IFILT				0x0094
#define PLL_PLL_GAIN				0x0098
#define PLL_ICODE_LOW				0x009C
#define PLL_ICODE_HIGH				0x00A0
#define PLL_LOCKDET				0x00A4
#define PLL_OUTDIV				0x00A8
#define PLL_FASTLOCK_CONTROL			0x00AC
#define PLL_PASS_OUT_OVERRIDE_ONE		0x00B0
#define PLL_PASS_OUT_OVERRIDE_TWO		0x00B4
#define PLL_CORE_OVERRIDE			0x00B8
#define PLL_CORE_INPUT_OVERRIDE			0x00BC
#define PLL_RATE_CHANGE				0x00C0
#define PLL_PLL_DIGITAL_TIMERS			0x00C4
#define PLL_PLL_DIGITAL_TIMERS_TWO		0x00C8
#define PLL_DECIMAL_DIV_START			0x00CC
#define PLL_FRAC_DIV_START_LOW			0x00D0
#define PLL_FRAC_DIV_START_MID			0x00D4
#define PLL_FRAC_DIV_START_HIGH			0x00D8
#define PLL_DEC_FRAC_MUXES			0x00DC
#define PLL_DECIMAL_DIV_START_1			0x00E0
#define PLL_FRAC_DIV_START_LOW_1		0x00E4
#define PLL_FRAC_DIV_START_MID_1		0x00E8
#define PLL_FRAC_DIV_START_HIGH_1		0x00EC
#define PLL_DECIMAL_DIV_START_2			0x00F0
#define PLL_FRAC_DIV_START_LOW_2		0x00F4
#define PLL_FRAC_DIV_START_MID_2		0x00F8
#define PLL_FRAC_DIV_START_HIGH_2		0x00FC
#define PLL_MASH_CONTROL			0x0100
#define PLL_SSC_STEPSIZE_LOW			0x0104
#define PLL_SSC_STEPSIZE_HIGH			0x0108
#define PLL_SSC_DIV_PER_LOW			0x010C
#define PLL_SSC_DIV_PER_HIGH			0x0110
#define PLL_SSC_ADJPER_LOW			0x0114
#define PLL_SSC_ADJPER_HIGH			0x0118
#define PLL_SSC_MUX_CONTROL			0x011C
#define PLL_SSC_STEPSIZE_LOW_1			0x0120
#define PLL_SSC_STEPSIZE_HIGH_1			0x0124
#define PLL_SSC_DIV_PER_LOW_1			0x0128
#define PLL_SSC_DIV_PER_HIGH_1			0x012C
#define PLL_SSC_ADJPER_LOW_1			0x0130
#define PLL_SSC_ADJPER_HIGH_1			0x0134
#define PLL_SSC_STEPSIZE_LOW_2			0x0138
#define PLL_SSC_STEPSIZE_HIGH_2			0x013C
#define PLL_SSC_DIV_PER_LOW_2			0x0140
#define PLL_SSC_DIV_PER_HIGH_2			0x0144
#define PLL_SSC_ADJPER_LOW_2			0x0148
#define PLL_SSC_ADJPER_HIGH_2			0x014C
#define PLL_SSC_CONTROL				0x0150
#define PLL_PLL_OUTDIV_RATE			0x0154
#define PLL_PLL_LOCKDET_RATE_1			0x0158
#define PLL_PLL_LOCKDET_RATE_2			0x015C
#define PLL_PLL_PROP_GAIN_RATE_1		0x0160
#define PLL_PLL_PROP_GAIN_RATE_2		0x0164
#define PLL_PLL_BAND_SEL_RATE_1			0x0168
#define PLL_PLL_BAND_SEL_RATE_2			0x016C
#define PLL_PLL_INT_GAIN_IFILT_BAND_1		0x0170
#define PLL_PLL_INT_GAIN_IFILT_BAND_2		0x0174
#define PLL_PLL_FL_INT_GAIN_PFILT_BAND_1	0x0178
#define PLL_PLL_FL_INT_GAIN_PFILT_BAND_2	0x017C
#define PLL_PLL_FASTLOCK_EN_BAND		0x0180
#define PLL_FREQ_TUNE_ACCUM_INIT_MID		0x0184
#define PLL_FREQ_TUNE_ACCUM_INIT_HIGH		0x0188
#define PLL_FREQ_TUNE_ACCUM_INIT_MUX		0x018C
#define PLL_PLL_LOCK_OVERRIDE			0x0190
#define PLL_PLL_LOCK_DELAY			0x0194
#define PLL_PLL_LOCK_MIN_DELAY			0x0198
#define PLL_CLOCK_INVERTERS			0x019C
#define PLL_SPARE_AND_JPC_OVERRIDES		0x01A0
#define PLL_BIAS_CONTROL_1			0x01A4
#define PLL_BIAS_CONTROL_2			0x01A8
#define PLL_ALOG_OBSV_BUS_CTRL_1		0x01AC
#define PLL_COMMON_STATUS_ONE			0x01B0
#define PLL_COMMON_STATUS_TWO			0x01B4
#define PLL_BAND_SEL_CAL			0x01B8
#define PLL_ICODE_ACCUM_STATUS_LOW		0x01BC
#define PLL_ICODE_ACCUM_STATUS_HIGH		0x01C0
#define PLL_FD_OUT_LOW				0x01C4
#define PLL_FD_OUT_HIGH				0x01C8
#define PLL_ALOG_OBSV_BUS_STATUS_1		0x01CC
#define PLL_PLL_MISC_CONFIG			0x01D0
#define PLL_FLL_CONFIG				0x01D4
#define PLL_FLL_FREQ_ACQ_TIME			0x01D8
#define PLL_FLL_CODE0				0x01DC
#define PLL_FLL_CODE1				0x01E0
#define PLL_FLL_GAIN0				0x01E4
#define PLL_FLL_GAIN1				0x01E8
#define PLL_SW_RESET				0x01EC
#define PLL_FAST_PWRUP				0x01F0
#define PLL_LOCKTIME0				0x01F4
#define PLL_LOCKTIME1				0x01F8
#define PLL_DEBUG_BUS_SEL			0x01FC
#define PLL_DEBUG_BUS0				0x0200
#define PLL_DEBUG_BUS1				0x0204
#define PLL_DEBUG_BUS2				0x0208
#define PLL_DEBUG_BUS3				0x020C
#define PLL_ANALOG_FLL_CONTROL_OVERRIDES	0x0210
#define PLL_VCO_CONFIG				0x0214
#define PLL_VCO_CAL_CODE1_MODE0_STATUS		0x0218
#define PLL_VCO_CAL_CODE1_MODE1_STATUS		0x021C
#define PLL_RESET_SM_STATUS			0x0220
#define PLL_TDC_OFFSET				0x0224
#define PLL_PS3_PWRDOWN_CONTROLS		0x0228
#define PLL_PS4_PWRDOWN_CONTROLS		0x022C
#define PLL_PLL_RST_CONTROLS			0x0230
#define PLL_GEAR_BAND_SELECT_CONTROLS		0x0234
#define PLL_PSM_CLK_CONTROLS			0x0238
#define PLL_SYSTEM_MUXES_2			0x023C
#define PLL_VCO_CONFIG_1			0x0240
#define PLL_VCO_CONFIG_2			0x0244
#define PLL_CLOCK_INVERTERS_1			0x0248
#define PLL_CLOCK_INVERTERS_2			0x024C
#define PLL_CMODE_1				0x0250
#define PLL_CMODE_2				0x0254
#define PLL_ANALOG_CONTROLS_FIVE_1		0x0258
#define PLL_ANALOG_CONTROLS_FIVE_2		0x025C

/* Register Offsets from PHY base address */
#define PHY_CMN_CLK_CFG0	0x010
#define PHY_CMN_CLK_CFG1	0x014
#define PHY_CMN_RBUF_CTRL	0x01C
#define PHY_CMN_CTRL_0		0x024
#define PHY_CMN_CTRL_3		0x030
#define PHY_CMN_PLL_CNTRL	0x03C
#define PHY_CMN_GLBL_DIGTOP_SPARE4 0x128

/* Bit definition of SSC control registers */
#define SSC_CENTER		BIT(0)
#define SSC_EN			BIT(1)
#define SSC_FREQ_UPDATE		BIT(2)
#define SSC_FREQ_UPDATE_MUX	BIT(3)
#define SSC_UPDATE_SSC		BIT(4)
#define SSC_UPDATE_SSC_MUX	BIT(5)
#define SSC_START		BIT(6)
#define SSC_START_MUX		BIT(7)

enum {
	DSI_PLL_0,
	DSI_PLL_1,
	DSI_PLL_MAX
};

struct dsi_pll_regs {
	u32 pll_prop_gain_rate;
	u32 pll_lockdet_rate;
	u32 decimal_div_start;
	u32 frac_div_start_low;
	u32 frac_div_start_mid;
	u32 frac_div_start_high;
	u32 pll_clock_inverters;
	u32 ssc_stepsize_low;
	u32 ssc_stepsize_high;
	u32 ssc_div_per_low;
	u32 ssc_div_per_high;
	u32 ssc_adjper_low;
	u32 ssc_adjper_high;
	u32 ssc_control;
};

struct dsi_pll_config {
	u32 ref_freq;
	bool div_override;
	u32 output_div;
	bool ignore_frac;
	bool disable_prescaler;
	bool enable_ssc;
	bool ssc_center;
	u32 dec_bits;
	u32 frac_bits;
	u32 lock_timer;
	u32 ssc_freq;
	u32 ssc_offset;
	u32 ssc_adj_per;
	u32 thresh_cycles;
	u32 refclk_cycles;
};

struct dsi_pll_7nm {
	struct mdss_pll_resources *rsc;
	struct dsi_pll_config pll_configuration;
	struct dsi_pll_regs reg_setup;
};

static inline bool dsi_pll_7nm_is_hw_revision_v1(
		struct mdss_pll_resources *rsc)
{
	return (rsc->pll_interface_type == MDSS_DSI_PLL_7NM) ? true : false;
}

static inline bool dsi_pll_7nm_is_hw_revision_v2(
		struct mdss_pll_resources *rsc)
{
	return (rsc->pll_interface_type == MDSS_DSI_PLL_7NM_V2) ? true : false;
}

static inline int pll_reg_read(void *context, unsigned int reg,
					unsigned int *val)
{
	int rc = 0;
	u32 data;
	struct mdss_pll_resources *rsc = context;

	rc = mdss_pll_resource_enable(rsc, true);
	if (rc) {
		pr_err("Failed to enable dsi pll resources, rc=%d\n", rc);
		return rc;
	}

	/*
	 * DSI PHY/PLL should be both powered on when reading PLL
	 * registers. Since PHY power has been enabled in DSI PHY
	 * driver, only PLL power is needed to enable here.
	 */
	data = MDSS_PLL_REG_R(rsc->phy_base, PHY_CMN_CTRL_0);
	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_CTRL_0, data | BIT(5));
	ndelay(250);

	*val = MDSS_PLL_REG_R(rsc->pll_base, reg);

	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_CTRL_0, data);

	(void)mdss_pll_resource_enable(rsc, false);

	return rc;
}

static inline int pll_reg_write(void *context, unsigned int reg,
					unsigned int val)
{
	int rc = 0;
	struct mdss_pll_resources *rsc = context;

	rc = mdss_pll_resource_enable(rsc, true);
	if (rc) {
		pr_err("Failed to enable dsi pll resources, rc=%d\n", rc);
		return rc;
	}

	MDSS_PLL_REG_W(rsc->pll_base, reg, val);
	(void)mdss_pll_resource_enable(rsc, false);

	return rc;
}

static inline int phy_reg_read(void *context, unsigned int reg,
					unsigned int *val)
{
	int rc = 0;
	struct mdss_pll_resources *rsc = context;

	rc = mdss_pll_resource_enable(rsc, true);
	if (rc) {
		pr_err("Failed to enable dsi pll resources, rc=%d\n", rc);
		return rc;
	}

	*val = MDSS_PLL_REG_R(rsc->phy_base, reg);
	(void)mdss_pll_resource_enable(rsc, false);

	return rc;
}

static inline int phy_reg_write(void *context, unsigned int reg,
					unsigned int val)
{
	int rc = 0;
	struct mdss_pll_resources *rsc = context;

	rc = mdss_pll_resource_enable(rsc, true);
	if (rc) {
		pr_err("Failed to enable dsi pll resources, rc=%d\n", rc);
		return rc;
	}

	MDSS_PLL_REG_W(rsc->phy_base, reg, val);
	(void)mdss_pll_resource_enable(rsc, false);

	return rc;
}

static inline int phy_reg_update_bits_sub(struct mdss_pll_resources *rsc,
		unsigned int reg, unsigned int mask, unsigned int val)
{
	u32 reg_val;
	int rc = 0;

	reg_val = MDSS_PLL_REG_R(rsc->phy_base, reg);
	reg_val &= ~mask;
	reg_val |= (val & mask);
	MDSS_PLL_REG_W(rsc->phy_base, reg, reg_val);

	return rc;
}

static inline int phy_reg_update_bits(void *context, unsigned int reg,
				unsigned int mask, unsigned int val)
{
	int rc = 0;
	struct mdss_pll_resources *rsc = context;

	rc = mdss_pll_resource_enable(rsc, true);
	if (rc) {
		pr_err("Failed to enable dsi pll resources, rc=%d\n", rc);
		return rc;
	}

	rc = phy_reg_update_bits_sub(rsc, reg, mask, val);
	if (!rc && rsc->slave)
		rc = phy_reg_update_bits_sub(rsc->slave, reg, mask, val);
	(void)mdss_pll_resource_enable(rsc, false);

	return rc;
}

static inline int pclk_mux_read_sel(void *context, unsigned int reg,
					unsigned int *val)
{
	int rc = 0;
	struct mdss_pll_resources *rsc = context;

	rc = mdss_pll_resource_enable(rsc, true);
	if (rc)
		pr_err("Failed to enable dsi pll resources, rc=%d\n", rc);
	else
		*val = (MDSS_PLL_REG_R(rsc->phy_base, reg) & 0x3);

	(void)mdss_pll_resource_enable(rsc, false);
	return rc;
}


static inline int pclk_mux_write_sel_sub(struct mdss_pll_resources *rsc,
				unsigned int reg, unsigned int val)
{
	u32 reg_val;
	int rc = 0;

	reg_val = MDSS_PLL_REG_R(rsc->phy_base, reg);
	reg_val &= ~0x03;
	reg_val |= val;

	MDSS_PLL_REG_W(rsc->phy_base, reg, reg_val);

	return rc;
}

static inline int pclk_mux_write_sel(void *context, unsigned int reg,
					unsigned int val)
{
	int rc = 0;
	struct mdss_pll_resources *rsc = context;

	rc = mdss_pll_resource_enable(rsc, true);
	if (rc) {
		pr_err("Failed to enable dsi pll resources, rc=%d\n", rc);
		return rc;
	}

	rc = pclk_mux_write_sel_sub(rsc, reg, val);
	if (!rc && rsc->slave)
		rc = pclk_mux_write_sel_sub(rsc->slave, reg, val);

	(void)mdss_pll_resource_enable(rsc, false);

	/*
	 * cache the current parent index for cases where parent
	 * is not changing but rate is changing. In that case
	 * clock framework won't call parent_set and hence dsiclk_sel
	 * bit won't be programmed. e.g. dfps update use case.
	 */
	rsc->cached_cfg1 = val;

	return rc;
}

static struct mdss_pll_resources *pll_rsc_db[DSI_PLL_MAX];
static struct dsi_pll_7nm plls[DSI_PLL_MAX];

static void dsi_pll_config_slave(struct mdss_pll_resources *rsc)
{
	u32 reg;
	struct mdss_pll_resources *orsc = pll_rsc_db[DSI_PLL_1];

	if (!rsc)
		return;

	/* Only DSI PLL0 can act as a master */
	if (rsc->index != DSI_PLL_0)
		return;

	/* default configuration: source is either internal or ref clock */
	rsc->slave = NULL;

	if (!orsc) {
		pr_warn("slave PLL unavilable, assuming standalone config\n");
		return;
	}

	/* check to see if the source of DSI1 PLL bitclk is set to external */
	reg = MDSS_PLL_REG_R(orsc->phy_base, PHY_CMN_CLK_CFG1);
	reg &= (BIT(2) | BIT(3));
	if (reg == 0x04)
		rsc->slave = pll_rsc_db[DSI_PLL_1]; /* external source */

	pr_debug("Slave PLL %s\n", rsc->slave ? "configured" : "absent");
}

static void dsi_pll_setup_config(struct dsi_pll_7nm *pll,
				 struct mdss_pll_resources *rsc)
{
	struct dsi_pll_config *config = &pll->pll_configuration;

	config->ref_freq = 19200000;
	config->output_div = 1;
	config->dec_bits = 8;
	config->frac_bits = 18;
	config->lock_timer = 64;
	config->ssc_freq = 31500;
	config->ssc_offset = 5000;
	config->ssc_adj_per = 2;
	config->thresh_cycles = 32;
	config->refclk_cycles = 256;

	config->div_override = false;
	config->ignore_frac = false;
	config->disable_prescaler = false;
	config->enable_ssc = rsc->ssc_en;
	config->ssc_center = rsc->ssc_center;

	if (config->enable_ssc) {
		if (rsc->ssc_freq)
			config->ssc_freq = rsc->ssc_freq;
		if (rsc->ssc_ppm)
			config->ssc_offset = rsc->ssc_ppm;
	}

	dsi_pll_config_slave(rsc);
}

static void dsi_pll_calc_dec_frac(struct dsi_pll_7nm *pll,
				  struct mdss_pll_resources *rsc)
{
	struct dsi_pll_config *config = &pll->pll_configuration;
	struct dsi_pll_regs *regs = &pll->reg_setup;
	u64 fref = rsc->vco_ref_clk_rate;
	u64 pll_freq;
	u64 divider;
	u64 dec, dec_multiple;
	u32 frac;
	u64 multiplier;

	pll_freq = rsc->vco_current_rate;

	if (config->disable_prescaler)
		divider = fref;
	else
		divider = fref * 2;

	multiplier = 1 << config->frac_bits;
	dec_multiple = div_u64(pll_freq * multiplier, divider);
	div_u64_rem(dec_multiple, multiplier, &frac);

	dec = div_u64(dec_multiple, multiplier);

	if (dsi_pll_7nm_is_hw_revision_v1(rsc))
		regs->pll_clock_inverters = 0x0;
	else
		regs->pll_clock_inverters = 0x28;

	regs->pll_lockdet_rate = config->lock_timer;
	regs->decimal_div_start = dec;
	regs->frac_div_start_low = (frac & 0xff);
	regs->frac_div_start_mid = (frac & 0xff00) >> 8;
	regs->frac_div_start_high = (frac & 0x30000) >> 16;
}

static void dsi_pll_calc_ssc(struct dsi_pll_7nm *pll,
		  struct mdss_pll_resources *rsc)
{
	struct dsi_pll_config *config = &pll->pll_configuration;
	struct dsi_pll_regs *regs = &pll->reg_setup;
	u32 ssc_per;
	u32 ssc_mod;
	u64 ssc_step_size;
	u64 frac;

	if (!config->enable_ssc) {
		pr_debug("SSC not enabled\n");
		return;
	}

	ssc_per = DIV_ROUND_CLOSEST(config->ref_freq, config->ssc_freq) / 2 - 1;
	ssc_mod = (ssc_per + 1) % (config->ssc_adj_per + 1);
	ssc_per -= ssc_mod;

	frac = regs->frac_div_start_low |
			(regs->frac_div_start_mid << 8) |
			(regs->frac_div_start_high << 16);
	ssc_step_size = regs->decimal_div_start;
	ssc_step_size *= (1 << config->frac_bits);
	ssc_step_size += frac;
	ssc_step_size *= config->ssc_offset;
	ssc_step_size *= (config->ssc_adj_per + 1);
	ssc_step_size = div_u64(ssc_step_size, (ssc_per + 1));
	ssc_step_size = DIV_ROUND_CLOSEST_ULL(ssc_step_size, 1000000);

	regs->ssc_div_per_low = ssc_per & 0xFF;
	regs->ssc_div_per_high = (ssc_per & 0xFF00) >> 8;
	regs->ssc_stepsize_low = (u32)(ssc_step_size & 0xFF);
	regs->ssc_stepsize_high = (u32)((ssc_step_size & 0xFF00) >> 8);
	regs->ssc_adjper_low = config->ssc_adj_per & 0xFF;
	regs->ssc_adjper_high = (config->ssc_adj_per & 0xFF00) >> 8;

	regs->ssc_control = config->ssc_center ? SSC_CENTER : 0;

	pr_debug("SCC: Dec:%d, frac:%llu, frac_bits:%d\n",
			regs->decimal_div_start, frac, config->frac_bits);
	pr_debug("SSC: div_per:0x%X, stepsize:0x%X, adjper:0x%X\n",
			ssc_per, (u32)ssc_step_size, config->ssc_adj_per);
}

static void dsi_pll_ssc_commit(struct dsi_pll_7nm *pll,
		struct mdss_pll_resources *rsc)
{
	void __iomem *pll_base = rsc->pll_base;
	struct dsi_pll_regs *regs = &pll->reg_setup;

	if (pll->pll_configuration.enable_ssc) {
		pr_debug("SSC is enabled\n");

		MDSS_PLL_REG_W(pll_base, PLL_SSC_STEPSIZE_LOW_1,
				regs->ssc_stepsize_low);
		MDSS_PLL_REG_W(pll_base, PLL_SSC_STEPSIZE_HIGH_1,
				regs->ssc_stepsize_high);
		MDSS_PLL_REG_W(pll_base, PLL_SSC_DIV_PER_LOW_1,
				regs->ssc_div_per_low);
		MDSS_PLL_REG_W(pll_base, PLL_SSC_DIV_PER_HIGH_1,
				regs->ssc_div_per_high);
		MDSS_PLL_REG_W(pll_base, PLL_SSC_ADJPER_LOW_1,
				regs->ssc_adjper_low);
		MDSS_PLL_REG_W(pll_base, PLL_SSC_ADJPER_HIGH_1,
				regs->ssc_adjper_high);
		MDSS_PLL_REG_W(pll_base, PLL_SSC_CONTROL,
				SSC_EN | regs->ssc_control);
	}
}

static void dsi_pll_config_hzindep_reg(struct dsi_pll_7nm *pll,
				  struct mdss_pll_resources *rsc)
{
	void __iomem *pll_base = rsc->pll_base;

	MDSS_PLL_REG_W(pll_base, PLL_ANALOG_CONTROLS_FIVE_1, 0x01);
	MDSS_PLL_REG_W(pll_base, PLL_VCO_CONFIG_1, 0x00);

	if (dsi_pll_7nm_is_hw_revision_v1(rsc))
		MDSS_PLL_REG_W(pll_base, PLL_GEAR_BAND_SELECT_CONTROLS, 0x21);
	else
		MDSS_PLL_REG_W(pll_base, PLL_GEAR_BAND_SELECT_CONTROLS, 0x22);

	MDSS_PLL_REG_W(pll_base, PLL_ANALOG_CONTROLS_FIVE, 0x01);
	MDSS_PLL_REG_W(pll_base, PLL_ANALOG_CONTROLS_TWO, 0x03);
	MDSS_PLL_REG_W(pll_base, PLL_ANALOG_CONTROLS_THREE, 0x00);
	MDSS_PLL_REG_W(pll_base, PLL_DSM_DIVIDER, 0x00);
	MDSS_PLL_REG_W(pll_base, PLL_FEEDBACK_DIVIDER, 0x4e);
	MDSS_PLL_REG_W(pll_base, PLL_CALIBRATION_SETTINGS, 0x40);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_CAL_SETTINGS_THREE, 0xba);
	MDSS_PLL_REG_W(pll_base, PLL_FREQ_DETECT_SETTINGS_ONE, 0x0c);
	MDSS_PLL_REG_W(pll_base, PLL_OUTDIV, 0x00);
	MDSS_PLL_REG_W(pll_base, PLL_CORE_OVERRIDE, 0x00);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_DIGITAL_TIMERS_TWO, 0x08);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_PROP_GAIN_RATE_1, 0x0a);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_BAND_SEL_RATE_1, 0xc0);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_INT_GAIN_IFILT_BAND_1, 0x84);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_INT_GAIN_IFILT_BAND_1, 0x82);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_FL_INT_GAIN_PFILT_BAND_1, 0x4c);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_LOCK_OVERRIDE, 0x80);
	MDSS_PLL_REG_W(pll_base, PLL_PFILT, 0x29);
	MDSS_PLL_REG_W(pll_base, PLL_PFILT, 0x2f);
	MDSS_PLL_REG_W(pll_base, PLL_IFILT, 0x2a);

	if (dsi_pll_7nm_is_hw_revision_v1(rsc))
		MDSS_PLL_REG_W(pll_base, PLL_IFILT, 0x30);
	else
		MDSS_PLL_REG_W(pll_base, PLL_IFILT, 0x22);
}

static void dsi_pll_init_val(struct mdss_pll_resources *rsc)
{
	void __iomem *pll_base = rsc->pll_base;

	MDSS_PLL_REG_W(pll_base, PLL_ANALOG_CONTROLS_ONE, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_INT_LOOP_SETTINGS, 0x0000003F);
	MDSS_PLL_REG_W(pll_base, PLL_INT_LOOP_SETTINGS_TWO, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_ANALOG_CONTROLS_FOUR, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_INT_LOOP_CONTROLS, 0x00000080);
	MDSS_PLL_REG_W(pll_base, PLL_SYSTEM_MUXES, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_FREQ_UPDATE_CONTROL_OVERRIDES, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_CMODE, 0x00000010);
	MDSS_PLL_REG_W(pll_base, PLL_PSM_CTRL, 0x00000020);
	MDSS_PLL_REG_W(pll_base, PLL_RSM_CTRL, 0x00000010);
	MDSS_PLL_REG_W(pll_base, PLL_VCO_TUNE_MAP, 0x00000002);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_CNTRL, 0x0000001C);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_CAL_TIMER_LOW, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_CAL_TIMER_HIGH, 0x00000002);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_CAL_SETTINGS, 0x00000020);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_MIN, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_MAX, 0x000000FF);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_PFILT, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_IFILT, 0x0000000A);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_CAL_SETTINGS_TWO, 0x00000025);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_CAL_SETTINGS_THREE, 0x000000BA);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_CAL_SETTINGS_FOUR, 0x0000004F);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_ICODE_HIGH, 0x0000000A);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_ICODE_LOW, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_FREQ_DETECT_SETTINGS_ONE, 0x0000000C);
	MDSS_PLL_REG_W(pll_base, PLL_FREQ_DETECT_THRESH, 0x00000020);
	MDSS_PLL_REG_W(pll_base, PLL_FREQ_DET_REFCLK_HIGH, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_FREQ_DET_REFCLK_LOW, 0x000000FF);
	MDSS_PLL_REG_W(pll_base, PLL_FREQ_DET_PLLCLK_HIGH, 0x00000010);
	MDSS_PLL_REG_W(pll_base, PLL_FREQ_DET_PLLCLK_LOW, 0x00000046);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_GAIN, 0x00000054);
	MDSS_PLL_REG_W(pll_base, PLL_ICODE_LOW, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_ICODE_HIGH, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_LOCKDET, 0x00000040);
	MDSS_PLL_REG_W(pll_base, PLL_FASTLOCK_CONTROL, 0x00000004);
	MDSS_PLL_REG_W(pll_base, PLL_PASS_OUT_OVERRIDE_ONE, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_PASS_OUT_OVERRIDE_TWO, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_CORE_OVERRIDE, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_CORE_INPUT_OVERRIDE, 0x00000010);
	MDSS_PLL_REG_W(pll_base, PLL_RATE_CHANGE, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_DIGITAL_TIMERS, 0x00000008);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_DIGITAL_TIMERS_TWO, 0x00000008);
	MDSS_PLL_REG_W(pll_base, PLL_DEC_FRAC_MUXES, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_MASH_CONTROL, 0x00000003);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_STEPSIZE_LOW, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_STEPSIZE_HIGH, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_DIV_PER_LOW, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_DIV_PER_HIGH, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_ADJPER_LOW, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_ADJPER_HIGH, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_MUX_CONTROL, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_STEPSIZE_LOW_1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_STEPSIZE_HIGH_1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_DIV_PER_LOW_1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_DIV_PER_HIGH_1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_ADJPER_LOW_1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_ADJPER_HIGH_1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_STEPSIZE_LOW_2, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_STEPSIZE_HIGH_2, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_DIV_PER_LOW_2, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_DIV_PER_HIGH_2, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_ADJPER_LOW_2, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_ADJPER_HIGH_2, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SSC_CONTROL, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_OUTDIV_RATE, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_LOCKDET_RATE_1, 0x00000040);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_LOCKDET_RATE_2, 0x00000040);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_PROP_GAIN_RATE_1, 0x0000000C);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_PROP_GAIN_RATE_2, 0x0000000A);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_BAND_SEL_RATE_1, 0x000000C0);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_BAND_SEL_RATE_2, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_INT_GAIN_IFILT_BAND_1, 0x00000054);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_INT_GAIN_IFILT_BAND_2, 0x00000054);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_FL_INT_GAIN_PFILT_BAND_1, 0x0000004C);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_FL_INT_GAIN_PFILT_BAND_2, 0x0000004C);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_FASTLOCK_EN_BAND, 0x00000003);
	MDSS_PLL_REG_W(pll_base, PLL_FREQ_TUNE_ACCUM_INIT_MID, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_FREQ_TUNE_ACCUM_INIT_HIGH, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_FREQ_TUNE_ACCUM_INIT_MUX, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_LOCK_OVERRIDE, 0x00000080);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_LOCK_DELAY, 0x00000006);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_LOCK_MIN_DELAY, 0x00000019);
	MDSS_PLL_REG_W(pll_base, PLL_CLOCK_INVERTERS, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SPARE_AND_JPC_OVERRIDES, 0x00000000);

	if (dsi_pll_7nm_is_hw_revision_v1(rsc))
		MDSS_PLL_REG_W(pll_base, PLL_BIAS_CONTROL_1, 0x00000066);
	else
		MDSS_PLL_REG_W(pll_base, PLL_BIAS_CONTROL_1, 0x00000040);

	MDSS_PLL_REG_W(pll_base, PLL_BIAS_CONTROL_2, 0x00000020);
	MDSS_PLL_REG_W(pll_base, PLL_ALOG_OBSV_BUS_CTRL_1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_COMMON_STATUS_ONE, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_COMMON_STATUS_TWO, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_BAND_SEL_CAL, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_ICODE_ACCUM_STATUS_LOW, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_ICODE_ACCUM_STATUS_HIGH, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_FD_OUT_LOW, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_FD_OUT_HIGH, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_ALOG_OBSV_BUS_STATUS_1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_MISC_CONFIG, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_FLL_CONFIG, 0x00000002);
	MDSS_PLL_REG_W(pll_base, PLL_FLL_FREQ_ACQ_TIME, 0x00000011);
	MDSS_PLL_REG_W(pll_base, PLL_FLL_CODE0, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_FLL_CODE1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_FLL_GAIN0, 0x00000080);
	MDSS_PLL_REG_W(pll_base, PLL_FLL_GAIN1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_SW_RESET, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_FAST_PWRUP, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_LOCKTIME0, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_LOCKTIME1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_DEBUG_BUS_SEL, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_DEBUG_BUS0, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_DEBUG_BUS1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_DEBUG_BUS2, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_DEBUG_BUS3, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_ANALOG_FLL_CONTROL_OVERRIDES, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_VCO_CONFIG, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_VCO_CAL_CODE1_MODE0_STATUS, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_VCO_CAL_CODE1_MODE1_STATUS, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_RESET_SM_STATUS, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_TDC_OFFSET, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_PS3_PWRDOWN_CONTROLS, 0x0000001D);
	MDSS_PLL_REG_W(pll_base, PLL_PS4_PWRDOWN_CONTROLS, 0x0000001C);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_RST_CONTROLS, 0x000000FF);
	MDSS_PLL_REG_W(pll_base, PLL_GEAR_BAND_SELECT_CONTROLS, 0x00000022);
	MDSS_PLL_REG_W(pll_base, PLL_PSM_CLK_CONTROLS, 0x00000009);
	MDSS_PLL_REG_W(pll_base, PLL_SYSTEM_MUXES_2, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_VCO_CONFIG_1, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_VCO_CONFIG_2, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_CLOCK_INVERTERS_1, 0x00000040);
	MDSS_PLL_REG_W(pll_base, PLL_CLOCK_INVERTERS_2, 0x00000000);
	MDSS_PLL_REG_W(pll_base, PLL_CMODE_1, 0x00000010);
	MDSS_PLL_REG_W(pll_base, PLL_CMODE_2, 0x00000010);
	MDSS_PLL_REG_W(pll_base, PLL_ANALOG_CONTROLS_FIVE_2, 0x00000003);

}

static void dsi_pll_commit(struct dsi_pll_7nm *pll,
			   struct mdss_pll_resources *rsc)
{
	void __iomem *pll_base = rsc->pll_base;
	struct dsi_pll_regs *reg = &pll->reg_setup;

	MDSS_PLL_REG_W(pll_base, PLL_CORE_INPUT_OVERRIDE, 0x12);
	MDSS_PLL_REG_W(pll_base, PLL_DECIMAL_DIV_START_1,
		       reg->decimal_div_start);
	MDSS_PLL_REG_W(pll_base, PLL_FRAC_DIV_START_LOW_1,
		       reg->frac_div_start_low);
	MDSS_PLL_REG_W(pll_base, PLL_FRAC_DIV_START_MID_1,
		       reg->frac_div_start_mid);
	MDSS_PLL_REG_W(pll_base, PLL_FRAC_DIV_START_HIGH_1,
		       reg->frac_div_start_high);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_LOCKDET_RATE_1, 0x40);
	MDSS_PLL_REG_W(pll_base, PLL_PLL_LOCK_DELAY, 0x06);
	MDSS_PLL_REG_W(pll_base, PLL_CMODE_1, 0x10);
	MDSS_PLL_REG_W(pll_base, PLL_CLOCK_INVERTERS_1,
			reg->pll_clock_inverters);
}

static int vco_7nm_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	int rc;
	struct dsi_pll_vco_clk *vco = to_vco_clk_hw(hw);
	struct mdss_pll_resources *rsc = vco->priv;
	struct dsi_pll_7nm *pll;

	if (!rsc) {
		pr_err("pll resource not found\n");
		return -EINVAL;
	}

	if (rsc->pll_on)
		return 0;

	pll = rsc->priv;
	if (!pll) {
		pr_err("pll configuration not found\n");
		return -EINVAL;
	}

	pr_debug("ndx=%d, rate=%lu\n", rsc->index, rate);

	rsc->vco_current_rate = rate;
	rsc->vco_ref_clk_rate = vco->ref_clk_rate;

	rc = mdss_pll_resource_enable(rsc, true);
	if (rc) {
		pr_err("failed to enable mdss dsi pll(%d), rc=%d\n",
		       rsc->index, rc);
		return rc;
	}

	dsi_pll_init_val(rsc);

	dsi_pll_setup_config(pll, rsc);

	dsi_pll_calc_dec_frac(pll, rsc);

	dsi_pll_calc_ssc(pll, rsc);

	dsi_pll_commit(pll, rsc);

	dsi_pll_config_hzindep_reg(pll, rsc);

	dsi_pll_ssc_commit(pll, rsc);

	/* flush, ensure all register writes are done*/
	wmb();

	mdss_pll_resource_enable(rsc, false);

	return 0;
}

static int dsi_pll_7nm_lock_status(struct mdss_pll_resources *pll)
{
	int rc;
	u32 status;
	u32 const delay_us = 100;
	u32 const timeout_us = 5000;

	rc = readl_poll_timeout_atomic(pll->pll_base + PLL_COMMON_STATUS_ONE,
				       status,
				       ((status & BIT(0)) > 0),
				       delay_us,
				       timeout_us);
	if (rc && !pll->handoff_resources)
		pr_err("DSI PLL(%d) lock failed, status=0x%08x\n",
			pll->index, status);

	return rc;
}

static void dsi_pll_disable_pll_bias(struct mdss_pll_resources *rsc)
{
	u32 data = MDSS_PLL_REG_R(rsc->phy_base, PHY_CMN_CTRL_0);

	MDSS_PLL_REG_W(rsc->pll_base, PLL_SYSTEM_MUXES, 0);
	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_CTRL_0, data & ~BIT(5));
	ndelay(250);
}

static void dsi_pll_enable_pll_bias(struct mdss_pll_resources *rsc)
{
	u32 data = MDSS_PLL_REG_R(rsc->phy_base, PHY_CMN_CTRL_0);

	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_CTRL_0, data | BIT(5));
	MDSS_PLL_REG_W(rsc->pll_base, PLL_SYSTEM_MUXES, 0xc0);
	ndelay(250);
}

static void dsi_pll_disable_global_clk(struct mdss_pll_resources *rsc)
{
	u32 data;

	data = MDSS_PLL_REG_R(rsc->phy_base, PHY_CMN_CLK_CFG1);
	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_CLK_CFG1, (data & ~BIT(5)));
}

static void dsi_pll_enable_global_clk(struct mdss_pll_resources *rsc)
{
	u32 data;

	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_CTRL_3, 0x04);

	data = MDSS_PLL_REG_R(rsc->phy_base, PHY_CMN_CLK_CFG1);

	/* Turn on clk_en_sel bit prior to resync toggle fifo */
	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_CLK_CFG1, (data | BIT(5) |
								BIT(4)));
}

static void dsi_pll_phy_dig_reset(struct mdss_pll_resources *rsc)
{
	/*
	 * Reset the PHY digital domain. This would be needed when
	 * coming out of a CX or analog rail power collapse while
	 * ensuring that the pads maintain LP00 or LP11 state
	 */
	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_GLBL_DIGTOP_SPARE4, BIT(0));
	wmb(); /* Ensure that the reset is asserted */
	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_GLBL_DIGTOP_SPARE4, 0x0);
	wmb(); /* Ensure that the reset is deasserted */
}

static int dsi_pll_enable(struct dsi_pll_vco_clk *vco)
{
	int rc;
	struct mdss_pll_resources *rsc = vco->priv;

	dsi_pll_enable_pll_bias(rsc);
	if (rsc->slave)
		dsi_pll_enable_pll_bias(rsc->slave);

	phy_reg_update_bits_sub(rsc, PHY_CMN_CLK_CFG1, 0x03, rsc->cached_cfg1);
	if (rsc->slave)
		phy_reg_update_bits_sub(rsc->slave, PHY_CMN_CLK_CFG1,
				0x03, rsc->cached_cfg1);
	wmb(); /* ensure dsiclk_sel is always programmed before pll start */

	/* Start PLL */
	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_PLL_CNTRL, 0x01);

	/*
	 * ensure all PLL configurations are written prior to checking
	 * for PLL lock.
	 */
	wmb();

	/* Check for PLL lock */
	rc = dsi_pll_7nm_lock_status(rsc);
	if (rc) {
		pr_err("PLL(%d) lock failed\n", rsc->index);
		goto error;
	}

	rsc->pll_on = true;

	/*
	 * assert power on reset for PHY digital in case the PLL is
	 * enabled after CX of analog domain power collapse. This needs
	 * to be done before enabling the global clk.
	 */
	dsi_pll_phy_dig_reset(rsc);
	if (rsc->slave)
		dsi_pll_phy_dig_reset(rsc->slave);

	dsi_pll_enable_global_clk(rsc);
	if (rsc->slave)
		dsi_pll_enable_global_clk(rsc->slave);

error:
	return rc;
}

static void dsi_pll_disable_sub(struct mdss_pll_resources *rsc)
{
	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_RBUF_CTRL, 0);
	dsi_pll_disable_pll_bias(rsc);
}

static void dsi_pll_disable(struct dsi_pll_vco_clk *vco)
{
	struct mdss_pll_resources *rsc = vco->priv;

	if (!rsc->pll_on &&
	    mdss_pll_resource_enable(rsc, true)) {
		pr_err("failed to enable pll (%d) resources\n", rsc->index);
		return;
	}

	rsc->handoff_resources = false;

	pr_debug("stop PLL (%d)\n", rsc->index);

	/*
	 * To avoid any stray glitches while
	 * abruptly powering down the PLL
	 * make sure to gate the clock using
	 * the clock enable bit before powering
	 * down the PLL
	 */
	dsi_pll_disable_global_clk(rsc);
	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_PLL_CNTRL, 0);
	dsi_pll_disable_sub(rsc);
	if (rsc->slave) {
		dsi_pll_disable_global_clk(rsc->slave);
		dsi_pll_disable_sub(rsc->slave);
	}
	/* flush, ensure all register writes are done*/
	wmb();
	rsc->pll_on = false;
}

long vco_7nm_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	unsigned long rrate = rate;
	struct dsi_pll_vco_clk *vco = to_vco_clk_hw(hw);

	if (rate < vco->min_rate)
		rrate = vco->min_rate;
	if (rate > vco->max_rate)
		rrate = vco->max_rate;

	*parent_rate = rrate;

	return rrate;
}

static void vco_7nm_unprepare(struct clk_hw *hw)
{
	struct dsi_pll_vco_clk *vco = to_vco_clk_hw(hw);
	struct mdss_pll_resources *pll = vco->priv;

	if (!pll) {
		pr_err("dsi pll resources not available\n");
		return;
	}

	/*
	 * During unprepare in continuous splash use case we want driver
	 * to pick all dividers instead of retaining bootloader configurations.
	 */
	if (!pll->handoff_resources) {
		pll->cached_cfg0 = MDSS_PLL_REG_R(pll->phy_base,
							PHY_CMN_CLK_CFG0);
		pll->cached_outdiv = MDSS_PLL_REG_R(pll->pll_base,
							PLL_PLL_OUTDIV_RATE);
		pr_debug("cfg0=%d,cfg1=%d, outdiv=%d\n", pll->cached_cfg0,
					pll->cached_cfg1, pll->cached_outdiv);

		pll->vco_cached_rate = clk_hw_get_rate(hw);
	}

	/*
	 * When continuous splash screen feature is enabled, we need to cache
	 * the mux configuration for the pixel_clk_src mux clock. The clock
	 * framework does not call back to re-configure the mux value if it is
	 * does not change.For such usecases, we need to ensure that the cached
	 * value is programmed prior to PLL being locked
	 */
	if (pll->handoff_resources)
		pll->cached_cfg1 = MDSS_PLL_REG_R(pll->phy_base,
							PHY_CMN_CLK_CFG1);

	dsi_pll_disable(vco);
	mdss_pll_resource_enable(pll, false);
}

static int vco_7nm_prepare(struct clk_hw *hw)
{
	int rc = 0;
	struct dsi_pll_vco_clk *vco = to_vco_clk_hw(hw);
	struct mdss_pll_resources *pll = vco->priv;

	if (!pll) {
		pr_err("dsi pll resources are not available\n");
		return -EINVAL;
	}

	/* Skip vco recalculation for continuous splash use case */
	if (pll->handoff_resources == true)
		return 0;

	rc = mdss_pll_resource_enable(pll, true);
	if (rc) {
		pr_err("failed to enable pll (%d) resource, rc=%d\n",
		       pll->index, rc);
		return rc;
	}

	if ((pll->vco_cached_rate != 0) &&
	    (pll->vco_cached_rate == clk_hw_get_rate(hw))) {
		rc = hw->init->ops->set_rate(hw, pll->vco_cached_rate,
				pll->vco_cached_rate);
		if (rc) {
			pr_err("pll(%d) set_rate failed, rc=%d\n",
			       pll->index, rc);
			mdss_pll_resource_enable(pll, false);
			return rc;
		}
		pr_debug("cfg0=%d, cfg1=%d\n", pll->cached_cfg0,
			pll->cached_cfg1);
		MDSS_PLL_REG_W(pll->phy_base, PHY_CMN_CLK_CFG0,
					pll->cached_cfg0);
		MDSS_PLL_REG_W(pll->pll_base, PLL_PLL_OUTDIV_RATE,
					pll->cached_outdiv);
	}

	rc = dsi_pll_enable(vco);
	if (rc) {
		mdss_pll_resource_enable(pll, false);
		pr_err("pll(%d) enable failed, rc=%d\n", pll->index, rc);
		return rc;
	}

	return rc;
}

static unsigned long vco_7nm_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct dsi_pll_vco_clk *vco = to_vco_clk_hw(hw);
	struct mdss_pll_resources *pll = vco->priv;
	int rc;
	u64 ref_clk = vco->ref_clk_rate;
	u64 vco_rate = 0;
	u64 multiplier;
	u32 frac;
	u32 dec;
	u32 outdiv;
	u64 pll_freq, tmp64;

	if (!vco->priv) {
		pr_err("vco priv is null\n");
		return 0;
	}

	/*
	 * Calculate the vco rate from HW registers only for handoff cases.
	 * For other cases where a vco_10nm_set_rate() has already been
	 * called, just return the rate that was set earlier. This is due
	 * to the fact that recalculating VCO rate requires us to read the
	 * correct value of the pll_out_div divider clock, which is only set
	 * afterwards.
	 */
	if (pll->vco_current_rate != 0) {
		pr_debug("returning vco rate = %lld\n", pll->vco_current_rate);
		return pll->vco_current_rate;
	}

	rc = mdss_pll_resource_enable(pll, true);
	if (rc) {
		pr_err("failed to enable pll(%d) resource, rc=%d\n",
		       pll->index, rc);
		return 0;
	}

	pll->handoff_resources = true;
	if (dsi_pll_7nm_lock_status(pll)) {
		pr_debug("PLL not enabled\n");
		pll->handoff_resources = false;
		goto end;
	}

	dec = MDSS_PLL_REG_R(pll->pll_base, PLL_DECIMAL_DIV_START_1);
	dec &= 0xFF;

	frac = MDSS_PLL_REG_R(pll->pll_base, PLL_FRAC_DIV_START_LOW_1);
	frac |= ((MDSS_PLL_REG_R(pll->pll_base, PLL_FRAC_DIV_START_MID_1) &
		  0xFF) <<
		8);
	frac |= ((MDSS_PLL_REG_R(pll->pll_base, PLL_FRAC_DIV_START_HIGH_1) &
		  0x3) <<
		16);

	/* OUTDIV_1:0 field is (log(outdiv, 2)) */
	outdiv = MDSS_PLL_REG_R(pll->pll_base, PLL_PLL_OUTDIV_RATE);
	outdiv &= 0x3;
	outdiv = 1 << outdiv;

	/*
	 * TODO:
	 *	1. Assumes prescaler is disabled
	 *	2. Multiplier is 2^18. it should be 2^(num_of_frac_bits)
	 **/
	multiplier = 1 << 18;
	pll_freq = dec * (ref_clk * 2);
	tmp64 = (ref_clk * 2 * frac);
	pll_freq += div_u64(tmp64, multiplier);

	vco_rate = div_u64(pll_freq, outdiv);

	pr_debug("dec=0x%x, frac=0x%x, outdiv=%d, vco=%llu\n",
		 dec, frac, outdiv, vco_rate);

end:
	(void)mdss_pll_resource_enable(pll, false);
	return (unsigned long)vco_rate;
}

static int pixel_clk_get_div(void *context, unsigned int reg, unsigned int *div)
{
	int rc;
	struct mdss_pll_resources *pll = context;
	u32 reg_val;

	rc = mdss_pll_resource_enable(pll, true);
	if (rc) {
		pr_err("Failed to enable dsi pll resources, rc=%d\n", rc);
		return rc;
	}

	reg_val = MDSS_PLL_REG_R(pll->phy_base, PHY_CMN_CLK_CFG0);
	*div = (reg_val & 0xF0) >> 4;

	/**
	 * Common clock framework the divider value is interpreted as one less
	 * hence we return one less for all dividers except when zero
	 */
	if (*div != 0)
		*div -= 1;

	(void)mdss_pll_resource_enable(pll, false);

	return rc;
}

static void pixel_clk_set_div_sub(struct mdss_pll_resources *pll, int div)
{
	u32 reg_val;

	reg_val = MDSS_PLL_REG_R(pll->phy_base, PHY_CMN_CLK_CFG0);
	reg_val &= ~0xF0;
	reg_val |= (div << 4);
	MDSS_PLL_REG_W(pll->phy_base, PHY_CMN_CLK_CFG0, reg_val);
}

static int pixel_clk_set_div(void *context, unsigned int reg, unsigned int div)
{
	int rc;
	struct mdss_pll_resources *pll = context;

	rc = mdss_pll_resource_enable(pll, true);
	if (rc) {
		pr_err("Failed to enable dsi pll resources, rc=%d\n", rc);
		return rc;
	}
	/**
	 * In common clock framework the divider value provided is one less and
	 * and hence adjusting the divider value by one prior to writing it to
	 * hardware
	 */
	div++;
	pixel_clk_set_div_sub(pll, div);
	if (pll->slave)
		pixel_clk_set_div_sub(pll->slave, div);
	(void)mdss_pll_resource_enable(pll, false);

	return 0;
}

static int bit_clk_get_div(void *context, unsigned int reg, unsigned int *div)
{
	int rc;
	struct mdss_pll_resources *pll = context;
	u32 reg_val;

	rc = mdss_pll_resource_enable(pll, true);
	if (rc) {
		pr_err("Failed to enable dsi pll resources, rc=%d\n", rc);
		return rc;
	}

	reg_val = MDSS_PLL_REG_R(pll->phy_base, PHY_CMN_CLK_CFG0);
	*div = (reg_val & 0x0F);

	/**
	 *Common clock framework the divider value is interpreted as one less
	 * hence we return one less for all dividers except when zero
	 */
	if (*div != 0)
		*div -= 1;
	(void)mdss_pll_resource_enable(pll, false);

	return rc;
}

static void bit_clk_set_div_sub(struct mdss_pll_resources *rsc, int div)
{
	u32 reg_val;

	reg_val = MDSS_PLL_REG_R(rsc->phy_base, PHY_CMN_CLK_CFG0);
	reg_val &= ~0x0F;
	reg_val |= div;
	MDSS_PLL_REG_W(rsc->phy_base, PHY_CMN_CLK_CFG0, reg_val);
}

static int bit_clk_set_div(void *context, unsigned int reg, unsigned int div)
{
	int rc;
	struct mdss_pll_resources *rsc = context;
	struct dsi_pll_8998 *pll;

	if (!rsc) {
		pr_err("pll resource not found\n");
		return -EINVAL;
	}

	pll = rsc->priv;
	if (!pll) {
		pr_err("pll configuration not found\n");
		return -EINVAL;
	}

	rc = mdss_pll_resource_enable(rsc, true);
	if (rc) {
		pr_err("Failed to enable dsi pll resources, rc=%d\n", rc);
		return rc;
	}

	/**
	 * In common clock framework the divider value provided is one less and
	 * and hence adjusting the divider value by one prior to writing it to
	 * hardware
	 */
	div++;

	bit_clk_set_div_sub(rsc, div);
	/* For slave PLL, this divider always should be set to 1 */
	if (rsc->slave)
		bit_clk_set_div_sub(rsc->slave, 1);

	(void)mdss_pll_resource_enable(rsc, false);

	return rc;
}

static struct regmap_config dsi_pll_7nm_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0x7c0,
};

static struct regmap_bus pll_regmap_bus = {
	.reg_write = pll_reg_write,
	.reg_read = pll_reg_read,
};

static struct regmap_bus pclk_src_mux_regmap_bus = {
	.reg_read = pclk_mux_read_sel,
	.reg_write = pclk_mux_write_sel,
};

static struct regmap_bus pclk_src_regmap_bus = {
	.reg_write = pixel_clk_set_div,
	.reg_read = pixel_clk_get_div,
};

static struct regmap_bus bitclk_src_regmap_bus = {
	.reg_write = bit_clk_set_div,
	.reg_read = bit_clk_get_div,
};

static const struct clk_ops clk_ops_vco_7nm = {
	.recalc_rate = vco_7nm_recalc_rate,
	.set_rate = vco_7nm_set_rate,
	.round_rate = vco_7nm_round_rate,
	.prepare = vco_7nm_prepare,
	.unprepare = vco_7nm_unprepare,
};

static struct regmap_bus mdss_mux_regmap_bus = {
	.reg_write = mdss_set_mux_sel,
	.reg_read = mdss_get_mux_sel,
};

/*
 * Clock tree for generating DSI byte and pclk.
 *
 *
 *                  +---------------+
 *                  |    vco_clk    |
 *                  +-------+-------+
 *                          |
 *                          |
 *                  +---------------+
 *                  |  pll_out_div  |
 *                  |  DIV(1,2,4,8) |
 *                  +-------+-------+
 *                          |
 *                          +-----------------------------+--------+
 *                          |                             |        |
 *                  +-------v-------+                     |        |
 *                  |  bitclk_src   |                     |        |
 *                  |  DIV(1..15)   |                     |        |
 *                  +-------+-------+                     |        |
 *                          |                             |        |
 *                          +----------+---------+        |        |
 *   Shadow Path            |          |         |        |        |
 *       +          +-------v-------+  |  +------v------+ | +------v-------+
 *       |          |  byteclk_src  |  |  |post_bit_div | | |post_vco_div  |
 *       |          |  DIV(8)       |  |  |DIV (2)      | | |DIV(4)        |
 *       |          +-------+-------+  |  +------+------+ | +------+-------+
 *       |                  |          |         |      | |        |
 *       |                  |          |         +------+ |        |
 *       |                  |          +-------------+  | |   +----+
 *       |         +--------+                        |  | |   |
 *       |         |                               +-v--v-v---v------+
 *     +-v---------v----+                           \  pclk_src_mux /
 *     \  byteclk_mux /                              \             /
 *      \            /                                +-----+-----+
 *       +----+-----+                                       |        Shadow Path
 *            |                                             |             +
 *            v                                       +-----v------+      |
 *       dsi_byte_clk                                 |  pclk_src  |      |
 *                                                    | DIV(1..15) |      |
 *                                                    +-----+------+      |
 *                                                          |             |
 *                                                          |             |
 *                                                          +--------+    |
 *                                                                   |    |
 *                                                               +---v----v----+
 *                                                                \  pclk_mux /
 *                                                                 \         /
 *                                                                  +---+---+
 *                                                                      |
 *                                                                      |
 *                                                                      v
 *                                                                   dsi_pclk
 *
 */

static struct dsi_pll_vco_clk dsi0pll_vco_clk = {
	.ref_clk_rate = 19200000UL,
	.min_rate = 1000000000UL,
	.max_rate = 3500000000UL,
	.hw.init = &(struct clk_init_data){
			.name = "dsi0pll_vco_clk",
			.parent_names = (const char *[]){"bi_tcxo"},
			.num_parents = 1,
			.ops = &clk_ops_vco_7nm,
			.flags = CLK_GET_RATE_NOCACHE,
	},
};

static struct dsi_pll_vco_clk dsi1pll_vco_clk = {
	.ref_clk_rate = 19200000UL,
	.min_rate = 1000000000UL,
	.max_rate = 3500000000UL,
	.hw.init = &(struct clk_init_data){
			.name = "dsi1pll_vco_clk",
			.parent_names = (const char *[]){"bi_tcxo"},
			.num_parents = 1,
			.ops = &clk_ops_vco_7nm,
			.flags = CLK_GET_RATE_NOCACHE,
	},
};

static struct clk_regmap_div dsi0pll_pll_out_div = {
	.reg = PLL_PLL_OUTDIV_RATE,
	.shift = 0,
	.width = 2,
	.flags = CLK_DIVIDER_POWER_OF_TWO,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi0pll_pll_out_div",
			.parent_names = (const char *[]){"dsi0pll_vco_clk"},
			.num_parents = 1,
			.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
			.ops = &clk_regmap_div_ops,
		},
	},
};

static struct clk_regmap_div dsi1pll_pll_out_div = {
	.reg = PLL_PLL_OUTDIV_RATE,
	.shift = 0,
	.width = 2,
	.flags = CLK_DIVIDER_POWER_OF_TWO,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi1pll_pll_out_div",
			.parent_names = (const char *[]){"dsi1pll_vco_clk"},
			.num_parents = 1,
			.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
			.ops = &clk_regmap_div_ops,
		},
	},
};

static struct clk_regmap_div dsi0pll_bitclk_src = {
	.shift = 0,
	.width = 4,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi0pll_bitclk_src",
			.parent_names = (const char *[]){"dsi0pll_pll_out_div"},
			.num_parents = 1,
			.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
			.ops = &clk_regmap_div_ops,
		},
	},
};

static struct clk_regmap_div dsi1pll_bitclk_src = {
	.shift = 0,
	.width = 4,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi1pll_bitclk_src",
			.parent_names = (const char *[]){"dsi1pll_pll_out_div"},
			.num_parents = 1,
			.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
			.ops = &clk_regmap_div_ops,
		},
	},
};

static struct clk_fixed_factor dsi0pll_post_vco_div = {
	.div = 4,
	.mult = 1,
	.hw.init = &(struct clk_init_data){
		.name = "dsi0pll_post_vco_div",
		.parent_names = (const char *[]){"dsi0pll_pll_out_div"},
		.num_parents = 1,
		.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_fixed_factor dsi1pll_post_vco_div = {
	.div = 4,
	.mult = 1,
	.hw.init = &(struct clk_init_data){
		.name = "dsi1pll_post_vco_div",
		.parent_names = (const char *[]){"dsi1pll_pll_out_div"},
		.num_parents = 1,
		.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_fixed_factor dsi0pll_byteclk_src = {
	.div = 8,
	.mult = 1,
	.hw.init = &(struct clk_init_data){
		.name = "dsi0pll_byteclk_src",
		.parent_names = (const char *[]){"dsi0pll_bitclk_src"},
		.num_parents = 1,
		.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_fixed_factor dsi1pll_byteclk_src = {
	.div = 8,
	.mult = 1,
	.hw.init = &(struct clk_init_data){
		.name = "dsi1pll_byteclk_src",
		.parent_names = (const char *[]){"dsi1pll_bitclk_src"},
		.num_parents = 1,
		.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_fixed_factor dsi0pll_post_bit_div = {
	.div = 2,
	.mult = 1,
	.hw.init = &(struct clk_init_data){
		.name = "dsi0pll_post_bit_div",
		.parent_names = (const char *[]){"dsi0pll_bitclk_src"},
		.num_parents = 1,
		.flags = CLK_GET_RATE_NOCACHE,
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_fixed_factor dsi1pll_post_bit_div = {
	.div = 2,
	.mult = 1,
	.hw.init = &(struct clk_init_data){
		.name = "dsi1pll_post_bit_div",
		.parent_names = (const char *[]){"dsi1pll_bitclk_src"},
		.num_parents = 1,
		.flags = CLK_GET_RATE_NOCACHE,
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_regmap_mux dsi0pll_byteclk_mux = {
	.shift = 0,
	.width = 1,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi0_phy_pll_out_byteclk",
			.parent_names = (const char *[]){"dsi0pll_byteclk_src"},
			.num_parents = 1,
			.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
			.ops = &clk_regmap_mux_closest_ops,
		},
	},
};

static struct clk_regmap_mux dsi1pll_byteclk_mux = {
	.shift = 0,
	.width = 1,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi1_phy_pll_out_byteclk",
			.parent_names = (const char *[]){"dsi1pll_byteclk_src"},
			.num_parents = 1,
			.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
			.ops = &clk_regmap_mux_closest_ops,
		},
	},
};

static struct clk_regmap_mux dsi0pll_pclk_src_mux = {
	.reg = PHY_CMN_CLK_CFG1,
	.shift = 0,
	.width = 2,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi0pll_pclk_src_mux",
			.parent_names = (const char *[]){"dsi0pll_bitclk_src",
					"dsi0pll_post_bit_div",
					"dsi0pll_pll_out_div",
					"dsi0pll_post_vco_div"},
			.num_parents = 4,
			.flags = CLK_GET_RATE_NOCACHE,
			.ops = &clk_regmap_mux_closest_ops,
		},
	},
};

static struct clk_regmap_mux dsi1pll_pclk_src_mux = {
	.reg = PHY_CMN_CLK_CFG1,
	.shift = 0,
	.width = 2,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi1pll_pclk_src_mux",
			.parent_names = (const char *[]){"dsi1pll_bitclk_src",
					"dsi1pll_post_bit_div",
					"dsi1pll_pll_out_div",
					"dsi1pll_post_vco_div"},
			.num_parents = 4,
			.flags = CLK_GET_RATE_NOCACHE,
			.ops = &clk_regmap_mux_closest_ops,
		},
	},
};

static struct clk_regmap_div dsi0pll_pclk_src = {
	.shift = 0,
	.width = 4,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi0pll_pclk_src",
			.parent_names = (const char *[]){
					"dsi0pll_pclk_src_mux"},
			.num_parents = 1,
			.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
			.ops = &clk_regmap_div_ops,
		},
	},
};

static struct clk_regmap_div dsi1pll_pclk_src = {
	.shift = 0,
	.width = 4,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi1pll_pclk_src",
			.parent_names = (const char *[]){
					"dsi1pll_pclk_src_mux"},
			.num_parents = 1,
			.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
			.ops = &clk_regmap_div_ops,
		},
	},
};

static struct clk_regmap_mux dsi0pll_pclk_mux = {
	.shift = 0,
	.width = 1,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi0_phy_pll_out_dsiclk",
			.parent_names = (const char *[]){"dsi0pll_pclk_src"},
			.num_parents = 1,
			.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
			.ops = &clk_regmap_mux_closest_ops,
		},
	},
};

static struct clk_regmap_mux dsi1pll_pclk_mux = {
	.shift = 0,
	.width = 1,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dsi1_phy_pll_out_dsiclk",
			.parent_names = (const char *[]){"dsi1pll_pclk_src"},
			.num_parents = 1,
			.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
			.ops = &clk_regmap_mux_closest_ops,
		},
	},
};

static struct clk_hw *mdss_dsi_pllcc_7nm[] = {
	[VCO_CLK_0] = &dsi0pll_vco_clk.hw,
	[PLL_OUT_DIV_0_CLK] = &dsi0pll_pll_out_div.clkr.hw,
	[BITCLK_SRC_0_CLK] = &dsi0pll_bitclk_src.clkr.hw,
	[BYTECLK_SRC_0_CLK] = &dsi0pll_byteclk_src.hw,
	[POST_BIT_DIV_0_CLK] = &dsi0pll_post_bit_div.hw,
	[POST_VCO_DIV_0_CLK] = &dsi0pll_post_vco_div.hw,
	[BYTECLK_MUX_0_CLK] = &dsi0pll_byteclk_mux.clkr.hw,
	[PCLK_SRC_MUX_0_CLK] = &dsi0pll_pclk_src_mux.clkr.hw,
	[PCLK_SRC_0_CLK] = &dsi0pll_pclk_src.clkr.hw,
	[PCLK_MUX_0_CLK] = &dsi0pll_pclk_mux.clkr.hw,
	[VCO_CLK_1] = &dsi1pll_vco_clk.hw,
	[PLL_OUT_DIV_1_CLK] = &dsi1pll_pll_out_div.clkr.hw,
	[BITCLK_SRC_1_CLK] = &dsi1pll_bitclk_src.clkr.hw,
	[BYTECLK_SRC_1_CLK] = &dsi1pll_byteclk_src.hw,
	[POST_BIT_DIV_1_CLK] = &dsi1pll_post_bit_div.hw,
	[POST_VCO_DIV_1_CLK] = &dsi1pll_post_vco_div.hw,
	[BYTECLK_MUX_1_CLK] = &dsi1pll_byteclk_mux.clkr.hw,
	[PCLK_SRC_MUX_1_CLK] = &dsi1pll_pclk_src_mux.clkr.hw,
	[PCLK_SRC_1_CLK] = &dsi1pll_pclk_src.clkr.hw,
	[PCLK_MUX_1_CLK] = &dsi1pll_pclk_mux.clkr.hw,
};

int dsi_pll_clock_register_7nm(struct platform_device *pdev,
				  struct mdss_pll_resources *pll_res)
{
	int rc = 0, ndx, i;
	struct clk *clk;
	struct clk_onecell_data *clk_data;
	int num_clks = ARRAY_SIZE(mdss_dsi_pllcc_7nm);
	struct regmap *rmap;

	if (!pdev || !pdev->dev.of_node ||
		!pll_res || !pll_res->pll_base || !pll_res->phy_base) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	ndx = pll_res->index;

	if (ndx >= DSI_PLL_MAX) {
		pr_err("pll index(%d) NOT supported\n", ndx);
		return -EINVAL;
	}

	pll_rsc_db[ndx] = pll_res;
	plls[ndx].rsc = pll_res;
	pll_res->priv = &plls[ndx];
	pll_res->vco_delay = VCO_DELAY_USEC;

	clk_data = devm_kzalloc(&pdev->dev, sizeof(struct clk_onecell_data),
					GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->clks = devm_kzalloc(&pdev->dev, (num_clks *
				sizeof(struct clk *)), GFP_KERNEL);
	if (!clk_data->clks) {
		devm_kfree(&pdev->dev, clk_data);
		return -ENOMEM;
	}
	clk_data->clk_num = num_clks;

	/* Establish client data */
	if (ndx == 0) {

		rmap = devm_regmap_init(&pdev->dev, &pll_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi0pll_pll_out_div.clkr.regmap = rmap;

		rmap = devm_regmap_init(&pdev->dev, &bitclk_src_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi0pll_bitclk_src.clkr.regmap = rmap;

		rmap = devm_regmap_init(&pdev->dev, &pclk_src_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi0pll_pclk_src.clkr.regmap = rmap;

		rmap = devm_regmap_init(&pdev->dev, &mdss_mux_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi0pll_pclk_mux.clkr.regmap = rmap;

		rmap = devm_regmap_init(&pdev->dev, &pclk_src_mux_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi0pll_pclk_src_mux.clkr.regmap = rmap;
		rmap = devm_regmap_init(&pdev->dev, &mdss_mux_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi0pll_byteclk_mux.clkr.regmap = rmap;

		dsi0pll_vco_clk.priv = pll_res;
		for (i = VCO_CLK_0; i <= PCLK_MUX_0_CLK; i++) {
			clk = devm_clk_register(&pdev->dev,
						mdss_dsi_pllcc_7nm[i]);
			if (IS_ERR(clk)) {
				pr_err("clk registration failed for DSI clock:%d\n",
							pll_res->index);
				rc = -EINVAL;
				goto clk_register_fail;
			}
			clk_data->clks[i] = clk;

		}

		rc = of_clk_add_provider(pdev->dev.of_node,
				of_clk_src_onecell_get, clk_data);


	} else {
		rmap = devm_regmap_init(&pdev->dev, &pll_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi1pll_pll_out_div.clkr.regmap = rmap;

		rmap = devm_regmap_init(&pdev->dev, &bitclk_src_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi1pll_bitclk_src.clkr.regmap = rmap;

		rmap = devm_regmap_init(&pdev->dev, &pclk_src_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi1pll_pclk_src.clkr.regmap = rmap;

		rmap = devm_regmap_init(&pdev->dev, &mdss_mux_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi1pll_pclk_mux.clkr.regmap = rmap;

		rmap = devm_regmap_init(&pdev->dev, &pclk_src_mux_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi1pll_pclk_src_mux.clkr.regmap = rmap;
		rmap = devm_regmap_init(&pdev->dev, &mdss_mux_regmap_bus,
				pll_res, &dsi_pll_7nm_config);
		dsi1pll_byteclk_mux.clkr.regmap = rmap;
		dsi1pll_vco_clk.priv = pll_res;

		for (i = VCO_CLK_1; i <= PCLK_MUX_1_CLK; i++) {
			clk = devm_clk_register(&pdev->dev,
						mdss_dsi_pllcc_7nm[i]);
			if (IS_ERR(clk)) {
				pr_err("clk registration failed for DSI clock:%d\n",
						pll_res->index);
				rc = -EINVAL;
				goto clk_register_fail;
			}
			clk_data->clks[i] = clk;

		}

		rc = of_clk_add_provider(pdev->dev.of_node,
				of_clk_src_onecell_get, clk_data);
	}
	if (!rc) {
		pr_info("Registered DSI PLL ndx=%d, clocks successfully\n",
				ndx);

		return rc;
	}
clk_register_fail:
	devm_kfree(&pdev->dev, clk_data->clks);
	devm_kfree(&pdev->dev, clk_data);
	return rc;
}
