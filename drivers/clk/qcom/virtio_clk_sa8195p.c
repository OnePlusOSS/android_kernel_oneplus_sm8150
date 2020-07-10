/* Copyright (c) 2019, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <dt-bindings/clock/qcom,gcc-sdmshrike.h>
#include "virtio_clk_common.h"

static const char * const sa8195p_gcc_virtio_clocks[] = {
	[GCC_QUPV3_WRAP0_S0_CLK] = "gcc_qupv3_wrap0_s0_clk",
	[GCC_QUPV3_WRAP0_S1_CLK] = "gcc_qupv3_wrap0_s1_clk",
	[GCC_QUPV3_WRAP0_S2_CLK] = "gcc_qupv3_wrap0_s2_clk",
	[GCC_QUPV3_WRAP0_S3_CLK] = "gcc_qupv3_wrap0_s3_clk",
	[GCC_QUPV3_WRAP0_S4_CLK] = "gcc_qupv3_wrap0_s4_clk",
	[GCC_QUPV3_WRAP0_S5_CLK] = "gcc_qupv3_wrap0_s5_clk",
	[GCC_QUPV3_WRAP0_S6_CLK] = "gcc_qupv3_wrap0_s6_clk",
	[GCC_QUPV3_WRAP0_S7_CLK] = "gcc_qupv3_wrap0_s7_clk",
	[GCC_QUPV3_WRAP1_S0_CLK] = "gcc_qupv3_wrap1_s0_clk",
	[GCC_QUPV3_WRAP1_S1_CLK] = "gcc_qupv3_wrap1_s1_clk",
	[GCC_QUPV3_WRAP1_S2_CLK] = "gcc_qupv3_wrap1_s2_clk",
	[GCC_QUPV3_WRAP1_S3_CLK] = "gcc_qupv3_wrap1_s3_clk",
	[GCC_QUPV3_WRAP1_S4_CLK] = "gcc_qupv3_wrap1_s4_clk",
	[GCC_QUPV3_WRAP1_S5_CLK] = "gcc_qupv3_wrap1_s5_clk",
	[GCC_QUPV3_WRAP2_S0_CLK] = "gcc_qupv3_wrap2_s0_clk",
	[GCC_QUPV3_WRAP2_S1_CLK] = "gcc_qupv3_wrap2_s1_clk",
	[GCC_QUPV3_WRAP2_S2_CLK] = "gcc_qupv3_wrap2_s2_clk",
	[GCC_QUPV3_WRAP2_S3_CLK] = "gcc_qupv3_wrap2_s3_clk",
	[GCC_QUPV3_WRAP2_S4_CLK] = "gcc_qupv3_wrap2_s4_clk",
	[GCC_QUPV3_WRAP2_S5_CLK] = "gcc_qupv3_wrap2_s5_clk",
	[GCC_QUPV3_WRAP_0_M_AHB_CLK] = "gcc_qupv3_wrap_0_m_ahb_clk",
	[GCC_QUPV3_WRAP_0_S_AHB_CLK] = "gcc_qupv3_wrap_0_s_ahb_clk",
	[GCC_QUPV3_WRAP_1_M_AHB_CLK] = "gcc_qupv3_wrap_1_m_ahb_clk",
	[GCC_QUPV3_WRAP_1_S_AHB_CLK] = "gcc_qupv3_wrap_1_s_ahb_clk",
	[GCC_QUPV3_WRAP_2_M_AHB_CLK] = "gcc_qupv3_wrap_2_m_ahb_clk",
	[GCC_QUPV3_WRAP_2_S_AHB_CLK] = "gcc_qupv3_wrap_2_s_ahb_clk",
	[GCC_USB30_PRIM_MASTER_CLK] = "gcc_usb30_prim_master_clk",
	[GCC_CFG_NOC_USB3_PRIM_AXI_CLK] = "gcc_cfg_noc_usb3_prim_axi_clk",
	[GCC_AGGRE_USB3_PRIM_AXI_CLK] = "gcc_aggre_usb3_prim_axi_clk",
	[GCC_USB30_PRIM_MOCK_UTMI_CLK] = "gcc_usb30_prim_mock_utmi_clk",
	[GCC_PCIE_0_PIPE_CLK] = "gcc_pcie_0_pipe_clk",
	[GCC_PCIE_0_AUX_CLK] = "gcc_pcie_0_aux_clk",
	[GCC_PCIE_0_CFG_AHB_CLK] = "gcc_pcie_0_cfg_ahb_clk",
	[GCC_PCIE_0_MSTR_AXI_CLK] = "gcc_pcie_0_mstr_axi_clk",
	[GCC_PCIE_0_SLV_AXI_CLK] = "gcc_pcie_0_slv_axi_clk",
	[GCC_PCIE_0_SLV_Q2A_AXI_CLK] = "gcc_pcie_0_slv_q2a_axi_clk",
	[GCC_AGGRE_NOC_PCIE_TBU_CLK] = "gcc_aggre_noc_pcie_tbu_clk",
	[GCC_PCIE0_PHY_REFGEN_CLK] = "gcc_pcie0_phy_refgen_clk",
	[GCC_PCIE_PHY_AUX_CLK] = "gcc_pcie_phy_aux_clk",
	[GCC_SDCC2_AHB_CLK] = "gcc_sdcc2_ahb_clk",
	[GCC_SDCC2_APPS_CLK] = "gcc_sdcc2_apps_clk",
};

static const char * const sa8195p_gcc_virtio_resets[] = {
	[GCC_QUSB2PHY_PRIM_BCR] = "gcc_qusb2phy_prim_bcr",
	[GCC_USB30_PRIM_BCR] = "gcc_usb30_prim_master_clk",
	[GCC_PCIE_0_BCR] = "gcc_pcie_0_mstr_axi_clk",
	[GCC_PCIE_0_PHY_BCR] = "gcc_pcie_0_phy_bcr",
};

const struct clk_virtio_desc clk_virtio_sa8195p_gcc = {
	.clk_names = sa8195p_gcc_virtio_clocks,
	.num_clks = ARRAY_SIZE(sa8195p_gcc_virtio_clocks),
	.reset_names = sa8195p_gcc_virtio_resets,
	.num_resets = ARRAY_SIZE(sa8195p_gcc_virtio_resets),
};
