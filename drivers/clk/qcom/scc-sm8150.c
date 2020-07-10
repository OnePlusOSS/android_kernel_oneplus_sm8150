/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "clk: %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,scc-sm8150.h>

#include "common.h"
#include "clk-regmap.h"
#include "clk-pll.h"
#include "clk-rcg.h"
#include "clk-branch.h"
#include "clk-alpha-pll.h"
#include "vdd-level.h"

#define F(f, s, h, m, n) { (f), (s), (2 * (h) - 1), (m), (n) }

static DEFINE_VDD_REGULATORS(vdd_scc_cx, VDD_NUM, 1, vdd_corner);

enum {
	P_AON_SLEEP_CLK,
	P_AOSS_CC_RO_CLK,
	P_QDSP6SS_PLL_OUT_ODD,
	P_SCC_PLL_OUT_EVEN,
	P_SCC_PLL_OUT_MAIN,
	P_SCC_PLL_OUT_ODD,
	P_SSC_BI_PLL_TEST_SE,
	P_SSC_BI_TCXO,
};

static const struct parent_map scc_parent_map_0[] = {
	{ P_AOSS_CC_RO_CLK, 0 },
	{ P_AON_SLEEP_CLK, 1 },
	{ P_SCC_PLL_OUT_EVEN, 2 },
	{ P_SSC_BI_TCXO, 3 },
	{ P_SCC_PLL_OUT_ODD, 4 },
	{ P_QDSP6SS_PLL_OUT_ODD, 5 },
	{ P_SCC_PLL_OUT_MAIN, 6 },
	{ P_SSC_BI_PLL_TEST_SE, 7 },
};

static const char * const scc_parent_names_0[] = {
	"bi_tcxo",
	"aon_sleep_clk",
	"scc_pll_out_even",
	"bi_tcxo",
	"scc_pll_out_odd",
	"qdsp6ss_pll_out_odd",
	"scc_pll",
	"ssc_bi_pll_test_se",
};

static struct pll_vco trion_vco[] = {
	{ 249600000, 2000000000, 0 },
};

static struct alpha_pll_config scc_pll_config = {
	.l = 0x1F,
	.alpha = 0x4000,
	.config_ctl_val = 0x20485699,
	.config_ctl_hi_val = 0x00002267,
	.config_ctl_hi1_val = 0x00000024,
	.test_ctl_val = 0x00000000,
	.test_ctl_hi_val = 0x00000002,
	.test_ctl_hi1_val = 0x00000000,
	.user_ctl_val = 0x00000100,
	.user_ctl_hi_val = 0x00000805,
	.user_ctl_hi1_val = 0x000000D0,
};

static struct alpha_pll_config scc_pll_config_sm8150_v2 = {
	.l = 0x1E,
	.alpha = 0x0,
	.config_ctl_val = 0x20485699,
	.config_ctl_hi_val = 0x00002267,
	.config_ctl_hi1_val = 0x00000024,
	.user_ctl_val = 0x00000100,
	.user_ctl_hi_val = 0x00000805,
	.user_ctl_hi1_val = 0x000000D0,
};

static struct clk_alpha_pll scc_pll = {
	.offset = 0x0,
	.vco_table = trion_vco,
	.num_vco = ARRAY_SIZE(trion_vco),
	.type = TRION_PLL,
	.config = &scc_pll_config,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "scc_pll",
			.parent_names = (const char *[]){ "bi_tcxo" },
			.num_parents = 1,
			.ops = &clk_trion_pll_ops,
			.vdd_class = &vdd_scc_cx,
			.num_rate_max = VDD_NUM,
			.rate_max = (unsigned long[VDD_NUM]) {
				[VDD_MIN] = 615000000,
				[VDD_LOW] = 1066000000,
				[VDD_LOW_L1] = 1600000000,
				[VDD_NOMINAL] = 2000000000},
		},
	},
};

static const struct clk_div_table post_div_table_trion_even[] = {
	{ 0x0, 1 },
	{ 0x1, 2 },
	{ 0x3, 4 },
	{ 0x7, 8 },
	{ }
};

static struct clk_alpha_pll_postdiv scc_pll_out_even = {
	.offset = 0x0,
	.post_div_shift = 8,
	.post_div_table = post_div_table_trion_even,
	.num_post_div = ARRAY_SIZE(post_div_table_trion_even),
	.width = 4,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "scc_pll_out_even",
		.parent_names = (const char *[]){ "scc_pll" },
		.num_parents = 1,
		.ops = &clk_trion_pll_postdiv_ops,
	},
};

static const struct freq_tbl ftbl_scc_main_rcg_clk_src[] = {
	F(100000000, P_SCC_PLL_OUT_EVEN, 3, 0, 0),
	{ }
};

static const struct freq_tbl ftbl_scc_main_rcg_clk_src_sm8150_v2[] = {
	F(96000000, P_SCC_PLL_OUT_EVEN, 3, 0, 0),
	{ }
};

static struct clk_rcg2 scc_main_rcg_clk_src = {
	.cmd_rcgr = 0x1000,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = scc_parent_map_0,
	.freq_tbl = ftbl_scc_main_rcg_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "scc_main_rcg_clk_src",
		.parent_names = scc_parent_names_0,
		.num_parents = 8,
		.ops = &clk_rcg2_ops,
		.vdd_class = &vdd_scc_cx,
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_MIN] = 100000000,
			[VDD_NOMINAL] = 200000000},
	},
};

static const struct freq_tbl ftbl_scc_qupv3_se0_clk_src[] = {
	F(7372800, P_SCC_PLL_OUT_EVEN, 1, 384, 15625),
	F(14745600, P_SCC_PLL_OUT_EVEN, 1, 768, 15625),
	F(19200000, P_SSC_BI_TCXO, 1, 0, 0),
	F(29491200, P_SCC_PLL_OUT_EVEN, 1, 1536, 15625),
	F(32000000, P_SCC_PLL_OUT_EVEN, 1, 8, 75),
	F(48000000, P_SCC_PLL_OUT_EVEN, 1, 4, 25),
	F(64000000, P_SCC_PLL_OUT_EVEN, 1, 16, 75),
	F(80000000, P_SCC_PLL_OUT_MAIN, 7.5, 0, 0),
	F(96000000, P_SCC_PLL_OUT_EVEN, 1, 8, 25),
	F(100000000, P_SCC_PLL_OUT_EVEN, 3, 0, 0),
	F(102400000, P_SCC_PLL_OUT_EVEN, 1, 128, 375),
	F(112000000, P_SCC_PLL_OUT_EVEN, 1, 28, 75),
	F(117964800, P_SCC_PLL_OUT_EVEN, 1, 6144, 15625),
	F(120000000, P_SCC_PLL_OUT_EVEN, 2.5, 0, 0),
	F(128000000, P_SCC_PLL_OUT_EVEN, 1, 32, 75),
	{ }
};

static const struct freq_tbl ftbl_scc_qupv3_se0_clk_src_sm8150_v2[] = {
	F(7372800, P_SCC_PLL_OUT_EVEN, 1, 16, 625),
	F(14745600, P_SCC_PLL_OUT_EVEN, 1, 32, 625),
	F(19200000, P_SSC_BI_TCXO, 1, 0, 0),
	F(29491200, P_SCC_PLL_OUT_EVEN, 1, 64, 625),
	F(32000000, P_SCC_PLL_OUT_EVEN, 9, 0, 0),
	F(48000000, P_SCC_PLL_OUT_EVEN, 6, 0, 0),
	F(64000000, P_SCC_PLL_OUT_EVEN, 4.5, 0, 0),
	F(96000000, P_SCC_PLL_OUT_MAIN, 6, 0, 0),
	F(100000000, P_SCC_PLL_OUT_MAIN, 1, 25, 144),
	F(102400000, P_SCC_PLL_OUT_MAIN, 1, 8, 45),
	F(112000000, P_SCC_PLL_OUT_MAIN, 1, 7, 36),
	F(117964800, P_SCC_PLL_OUT_MAIN, 1, 128, 625),
	F(120000000, P_SCC_PLL_OUT_MAIN, 1, 5, 24),
	F(128000000, P_SCC_PLL_OUT_MAIN, 4.5, 0, 0),
	F(144000000, P_SCC_PLL_OUT_MAIN, 4, 0, 0),
	F(192000000, P_SCC_PLL_OUT_MAIN, 3, 0, 0),
	{ }
};

static struct clk_rcg2 scc_qupv3_se0_clk_src = {
	.cmd_rcgr = 0x2004,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = scc_parent_map_0,
	.freq_tbl = ftbl_scc_qupv3_se0_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "scc_qupv3_se0_clk_src",
		.parent_names = scc_parent_names_0,
		.num_parents = 8,
		.ops = &clk_rcg2_ops,
		.vdd_class = &vdd_scc_cx,
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_MIN] = 50000000,
			[VDD_LOWER] = 80000000,
			[VDD_LOW] = 120000000,
			[VDD_NOMINAL] = 150000000},
	},
};

static struct clk_rcg2 scc_qupv3_se1_clk_src = {
	.cmd_rcgr = 0x3004,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = scc_parent_map_0,
	.freq_tbl = ftbl_scc_qupv3_se0_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "scc_qupv3_se1_clk_src",
		.parent_names = scc_parent_names_0,
		.num_parents = 8,
		.ops = &clk_rcg2_ops,
		.vdd_class = &vdd_scc_cx,
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_MIN] = 50000000,
			[VDD_LOWER] = 80000000,
			[VDD_LOW] = 120000000,
			[VDD_NOMINAL] = 150000000},
	},
};

static struct clk_rcg2 scc_qupv3_se2_clk_src = {
	.cmd_rcgr = 0x4004,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = scc_parent_map_0,
	.freq_tbl = ftbl_scc_qupv3_se0_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "scc_qupv3_se2_clk_src",
		.parent_names = scc_parent_names_0,
		.num_parents = 8,
		.ops = &clk_rcg2_ops,
		.vdd_class = &vdd_scc_cx,
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_MIN] = 50000000,
			[VDD_LOWER] = 80000000,
			[VDD_LOW] = 120000000,
			[VDD_NOMINAL] = 150000000},
	},
};

static struct clk_rcg2 scc_qupv3_se3_clk_src = {
	.cmd_rcgr = 0xb004,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = scc_parent_map_0,
	.freq_tbl = ftbl_scc_qupv3_se0_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "scc_qupv3_se3_clk_src",
		.parent_names = scc_parent_names_0,
		.num_parents = 8,
		.ops = &clk_rcg2_ops,
		.vdd_class = &vdd_scc_cx,
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_MIN] = 50000000,
			[VDD_LOWER] = 80000000,
			[VDD_LOW] = 120000000,
			[VDD_NOMINAL] = 150000000},
	},
};

static struct clk_rcg2 scc_qupv3_se4_clk_src = {
	.cmd_rcgr = 0xc004,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = scc_parent_map_0,
	.freq_tbl = ftbl_scc_qupv3_se0_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "scc_qupv3_se4_clk_src",
		.parent_names = scc_parent_names_0,
		.num_parents = 8,
		.ops = &clk_rcg2_ops,
		.vdd_class = &vdd_scc_cx,
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_MIN] = 50000000,
			[VDD_LOWER] = 80000000,
			[VDD_LOW] = 120000000,
			[VDD_NOMINAL] = 150000000},
	},
};

static struct clk_rcg2 scc_qupv3_se5_clk_src = {
	.cmd_rcgr = 0xd004,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = scc_parent_map_0,
	.freq_tbl = ftbl_scc_qupv3_se0_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "scc_qupv3_se5_clk_src",
		.parent_names = scc_parent_names_0,
		.num_parents = 8,
		.ops = &clk_rcg2_ops,
		.vdd_class = &vdd_scc_cx,
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_MIN] = 50000000,
			[VDD_LOWER] = 80000000,
			[VDD_LOW] = 120000000,
			[VDD_NOMINAL] = 150000000},
	},
};

static struct clk_branch scc_qupv3_2xcore_clk = {
	.halt_reg = 0x5008,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x21000,
		.enable_mask = BIT(10),
		.hw.init = &(struct clk_init_data){
			.name = "scc_qupv3_2xcore_clk",
			.parent_names = (const char *[]){
				"scc_main_rcg_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch scc_qupv3_core_clk = {
	.halt_reg = 0x5010,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x21000,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "scc_qupv3_core_clk",
			.parent_names = (const char *[]){
				"scc_main_rcg_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch scc_qupv3_m_hclk_clk = {
	.halt_reg = 0x9064,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x9064,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x21000,
		.enable_mask = BIT(1),
		.hw.init = &(struct clk_init_data){
			.name = "scc_qupv3_m_hclk_clk",
			.parent_names = (const char *[]){
				"scc_main_rcg_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch scc_qupv3_s_hclk_clk = {
	.halt_reg = 0x9060,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x9060,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x21000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "scc_qupv3_s_hclk_clk",
			.parent_names = (const char *[]){
				"scc_main_rcg_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch scc_qupv3_se0_clk = {
	.halt_reg = 0x2130,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x21000,
		.enable_mask = BIT(3),
		.hw.init = &(struct clk_init_data){
			.name = "scc_qupv3_se0_clk",
			.parent_names = (const char *[]){
				"scc_qupv3_se0_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch scc_qupv3_se1_clk = {
	.halt_reg = 0x3130,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x21000,
		.enable_mask = BIT(4),
		.hw.init = &(struct clk_init_data){
			.name = "scc_qupv3_se1_clk",
			.parent_names = (const char *[]){
				"scc_qupv3_se1_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch scc_qupv3_se2_clk = {
	.halt_reg = 0x4130,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x21000,
		.enable_mask = BIT(5),
		.hw.init = &(struct clk_init_data){
			.name = "scc_qupv3_se2_clk",
			.parent_names = (const char *[]){
				"scc_qupv3_se2_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch scc_qupv3_se3_clk = {
	.halt_reg = 0xb130,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x21000,
		.enable_mask = BIT(6),
		.hw.init = &(struct clk_init_data){
			.name = "scc_qupv3_se3_clk",
			.parent_names = (const char *[]){
				"scc_qupv3_se3_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch scc_qupv3_se4_clk = {
	.halt_reg = 0xc130,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x21000,
		.enable_mask = BIT(7),
		.hw.init = &(struct clk_init_data){
			.name = "scc_qupv3_se4_clk",
			.parent_names = (const char *[]){
				"scc_qupv3_se4_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch scc_qupv3_se5_clk = {
	.halt_reg = 0xd130,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x21000,
		.enable_mask = BIT(8),
		.hw.init = &(struct clk_init_data){
			.name = "scc_qupv3_se5_clk",
			.parent_names = (const char *[]){
				"scc_qupv3_se5_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_regmap *scc_sm8150_clocks[] = {
	[SCC_MAIN_RCG_CLK_SRC] = &scc_main_rcg_clk_src.clkr,
	[SCC_PLL] = &scc_pll.clkr,
	[SCC_PLL_OUT_EVEN] = &scc_pll_out_even.clkr,
	[SCC_QUPV3_2XCORE_CLK] = &scc_qupv3_2xcore_clk.clkr,
	[SCC_QUPV3_CORE_CLK] = &scc_qupv3_core_clk.clkr,
	[SCC_QUPV3_M_HCLK_CLK] = &scc_qupv3_m_hclk_clk.clkr,
	[SCC_QUPV3_S_HCLK_CLK] = &scc_qupv3_s_hclk_clk.clkr,
	[SCC_QUPV3_SE0_CLK] = &scc_qupv3_se0_clk.clkr,
	[SCC_QUPV3_SE0_CLK_SRC] = &scc_qupv3_se0_clk_src.clkr,
	[SCC_QUPV3_SE1_CLK] = &scc_qupv3_se1_clk.clkr,
	[SCC_QUPV3_SE1_CLK_SRC] = &scc_qupv3_se1_clk_src.clkr,
	[SCC_QUPV3_SE2_CLK] = &scc_qupv3_se2_clk.clkr,
	[SCC_QUPV3_SE2_CLK_SRC] = &scc_qupv3_se2_clk_src.clkr,
	[SCC_QUPV3_SE3_CLK] = &scc_qupv3_se3_clk.clkr,
	[SCC_QUPV3_SE3_CLK_SRC] = &scc_qupv3_se3_clk_src.clkr,
	[SCC_QUPV3_SE4_CLK] = &scc_qupv3_se4_clk.clkr,
	[SCC_QUPV3_SE4_CLK_SRC] = &scc_qupv3_se4_clk_src.clkr,
	[SCC_QUPV3_SE5_CLK] = &scc_qupv3_se5_clk.clkr,
	[SCC_QUPV3_SE5_CLK_SRC] = &scc_qupv3_se5_clk_src.clkr,
};

static struct clk_dfs scc_dfs_clocks[] = {
	{ &scc_qupv3_se0_clk_src, DFS_ENABLE_RCG },
	{ &scc_qupv3_se1_clk_src, DFS_ENABLE_RCG },
	{ &scc_qupv3_se2_clk_src, DFS_ENABLE_RCG },
	{ &scc_qupv3_se3_clk_src, DFS_ENABLE_RCG },
	{ &scc_qupv3_se4_clk_src, DFS_ENABLE_RCG },
	{ &scc_qupv3_se5_clk_src, DFS_ENABLE_RCG },
};

static const struct qcom_cc_dfs_desc scc_sm8150_dfs_desc = {
	.clks = scc_dfs_clocks,
	.num_clks = ARRAY_SIZE(scc_dfs_clocks),
};

static const struct regmap_config scc_sm8150_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x23000,
	.fast_io	= true,
};

static const struct qcom_cc_desc scc_sm8150_desc = {
	.config = &scc_sm8150_regmap_config,
	.clks = scc_sm8150_clocks,
	.num_clks = ARRAY_SIZE(scc_sm8150_clocks),
};

static const struct of_device_id scc_sm8150_match_table[] = {
	{ .compatible = "qcom,scc-sm8150" },
	{ .compatible = "qcom,scc-sm8150-v2" },
	{ }
};
MODULE_DEVICE_TABLE(of, scc_sm8150_match_table);

static void scc_sm8150_fixup_sm8150v2(struct regmap *regmap)
{
	scc_pll.config = &scc_pll_config_sm8150_v2;

	scc_main_rcg_clk_src.freq_tbl = ftbl_scc_main_rcg_clk_src_sm8150_v2;
	scc_main_rcg_clk_src.clkr.hw.init->rate_max[VDD_MIN] = 96000000;
	scc_main_rcg_clk_src.clkr.hw.init->rate_max[VDD_LOW] = 576000000;
	scc_main_rcg_clk_src.clkr.hw.init->rate_max[VDD_NOMINAL] = 576000000;
	scc_qupv3_se0_clk_src.freq_tbl = ftbl_scc_qupv3_se0_clk_src_sm8150_v2;
	scc_qupv3_se0_clk_src.clkr.hw.init->rate_max[VDD_MIN] = 48000000;
	scc_qupv3_se0_clk_src.clkr.hw.init->rate_max[VDD_LOWER] = 96000000;
	scc_qupv3_se0_clk_src.clkr.hw.init->rate_max[VDD_LOW] = 128000000;
	scc_qupv3_se0_clk_src.clkr.hw.init->rate_max[VDD_LOW_L1] = 144000000;
	scc_qupv3_se0_clk_src.clkr.hw.init->rate_max[VDD_NOMINAL] = 192000000;
	scc_qupv3_se1_clk_src.freq_tbl = ftbl_scc_qupv3_se0_clk_src_sm8150_v2;
	scc_qupv3_se1_clk_src.clkr.hw.init->rate_max[VDD_MIN] = 48000000;
	scc_qupv3_se1_clk_src.clkr.hw.init->rate_max[VDD_LOWER] = 96000000;
	scc_qupv3_se1_clk_src.clkr.hw.init->rate_max[VDD_LOW] = 128000000;
	scc_qupv3_se1_clk_src.clkr.hw.init->rate_max[VDD_LOW_L1] = 144000000;
	scc_qupv3_se1_clk_src.clkr.hw.init->rate_max[VDD_NOMINAL] = 192000000;
	scc_qupv3_se2_clk_src.freq_tbl = ftbl_scc_qupv3_se0_clk_src_sm8150_v2;
	scc_qupv3_se2_clk_src.clkr.hw.init->rate_max[VDD_MIN] = 48000000;
	scc_qupv3_se2_clk_src.clkr.hw.init->rate_max[VDD_LOWER] = 96000000;
	scc_qupv3_se2_clk_src.clkr.hw.init->rate_max[VDD_LOW] = 128000000;
	scc_qupv3_se2_clk_src.clkr.hw.init->rate_max[VDD_LOW_L1] = 144000000;
	scc_qupv3_se2_clk_src.clkr.hw.init->rate_max[VDD_NOMINAL] = 192000000;
	scc_qupv3_se3_clk_src.freq_tbl = ftbl_scc_qupv3_se0_clk_src_sm8150_v2;
	scc_qupv3_se3_clk_src.clkr.hw.init->rate_max[VDD_MIN] = 48000000;
	scc_qupv3_se3_clk_src.clkr.hw.init->rate_max[VDD_LOWER] = 96000000;
	scc_qupv3_se3_clk_src.clkr.hw.init->rate_max[VDD_LOW] = 128000000;
	scc_qupv3_se3_clk_src.clkr.hw.init->rate_max[VDD_LOW_L1] = 144000000;
	scc_qupv3_se3_clk_src.clkr.hw.init->rate_max[VDD_NOMINAL] = 192000000;
	scc_qupv3_se4_clk_src.freq_tbl = ftbl_scc_qupv3_se0_clk_src_sm8150_v2;
	scc_qupv3_se4_clk_src.clkr.hw.init->rate_max[VDD_MIN] = 48000000;
	scc_qupv3_se4_clk_src.clkr.hw.init->rate_max[VDD_LOWER] = 96000000;
	scc_qupv3_se4_clk_src.clkr.hw.init->rate_max[VDD_LOW] = 128000000;
	scc_qupv3_se4_clk_src.clkr.hw.init->rate_max[VDD_LOW_L1] = 144000000;
	scc_qupv3_se4_clk_src.clkr.hw.init->rate_max[VDD_NOMINAL] = 192000000;
	scc_qupv3_se5_clk_src.freq_tbl = ftbl_scc_qupv3_se0_clk_src_sm8150_v2;
	scc_qupv3_se5_clk_src.clkr.hw.init->rate_max[VDD_MIN] = 48000000;
	scc_qupv3_se5_clk_src.clkr.hw.init->rate_max[VDD_LOWER] = 96000000;
	scc_qupv3_se5_clk_src.clkr.hw.init->rate_max[VDD_LOW] = 128000000;
	scc_qupv3_se5_clk_src.clkr.hw.init->rate_max[VDD_LOW_L1] = 144000000;
	scc_qupv3_se5_clk_src.clkr.hw.init->rate_max[VDD_NOMINAL] = 192000000;
}

static int scc_sm8150_fixup(struct platform_device *pdev, struct regmap *regmap)
{
	const char *compat = NULL;
	int compatlen = 0;

	compat = of_get_property(pdev->dev.of_node, "compatible", &compatlen);
	if (!compat || (compatlen <= 0))
		return -EINVAL;

	if (!strcmp(compat, "qcom,scc-sm8150-v2"))
		scc_sm8150_fixup_sm8150v2(regmap);

	return 0;
}

static int scc_sm8150_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	int ret;

	regmap = qcom_cc_map(pdev, &scc_sm8150_desc);
	if (IS_ERR(regmap)) {
		pr_err("Failed to map the scc registers\n");
		return PTR_ERR(regmap);
	}

	vdd_scc_cx.regulator[0] = devm_regulator_get(&pdev->dev, "vdd_scc_cx");
	if (IS_ERR(vdd_scc_cx.regulator[0])) {
		ret = PTR_ERR(vdd_scc_cx.regulator[0]);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Unable to get vdd_scc_cx regulator, ret=%d\n",
				ret);
		return ret;
	}

	ret = scc_sm8150_fixup(pdev, regmap);
	if (ret)
		return ret;

	clk_trion_pll_configure(&scc_pll, regmap, scc_pll.config);

	ret = qcom_cc_really_probe(pdev, &scc_sm8150_desc, regmap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register SCC clocks\n");
		return ret;
	}

	ret = qcom_cc_register_rcg_dfs(pdev, &scc_sm8150_dfs_desc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register with DFS\n");
		return ret;
	}

	dev_info(&pdev->dev, "Registered SCC clocks\n");

	return 0;
}

static struct platform_driver scc_sm8150_driver = {
	.probe		= scc_sm8150_probe,
	.driver		= {
		.name	= "scc-sm8150",
		.of_match_table = scc_sm8150_match_table,
	},
};

static int __init scc_sm8150_init(void)
{
	return platform_driver_register(&scc_sm8150_driver);
}
subsys_initcall(scc_sm8150_init);

static void __exit scc_sm8150_exit(void)
{
	platform_driver_unregister(&scc_sm8150_driver);
}
module_exit(scc_sm8150_exit);

MODULE_DESCRIPTION("QTI SCC SM8150 Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:scc-sm8150");
