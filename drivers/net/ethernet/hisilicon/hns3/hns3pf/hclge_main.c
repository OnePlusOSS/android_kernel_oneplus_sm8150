/*
 * Copyright (c) 2016-2017 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#include "hclge_cmd.h"
#include "hclge_main.h"
#include "hclge_mdio.h"
#include "hclge_tm.h"
#include "hnae3.h"

#define HCLGE_NAME			"hclge"
#define HCLGE_STATS_READ(p, offset) (*((u64 *)((u8 *)(p) + (offset))))
#define HCLGE_MAC_STATS_FIELD_OFF(f) (offsetof(struct hclge_mac_stats, f))
#define HCLGE_64BIT_STATS_FIELD_OFF(f) (offsetof(struct hclge_64_bit_stats, f))
#define HCLGE_32BIT_STATS_FIELD_OFF(f) (offsetof(struct hclge_32_bit_stats, f))

static int hclge_rss_init_hw(struct hclge_dev *hdev);
static int hclge_set_mta_filter_mode(struct hclge_dev *hdev,
				     enum hclge_mta_dmac_sel_type mta_mac_sel,
				     bool enable);
static int hclge_init_vlan_config(struct hclge_dev *hdev);

static struct hnae3_ae_algo ae_algo;

static const struct pci_device_id ae_algo_pci_tbl[] = {
	{PCI_VDEVICE(HUAWEI, HNAE3_DEV_ID_GE), 0},
	{PCI_VDEVICE(HUAWEI, HNAE3_DEV_ID_25GE), 0},
	{PCI_VDEVICE(HUAWEI, HNAE3_DEV_ID_25GE_RDMA), 0},
	{PCI_VDEVICE(HUAWEI, HNAE3_DEV_ID_25GE_RDMA_MACSEC), 0},
	{PCI_VDEVICE(HUAWEI, HNAE3_DEV_ID_50GE_RDMA), 0},
	{PCI_VDEVICE(HUAWEI, HNAE3_DEV_ID_50GE_RDMA_MACSEC), 0},
	{PCI_VDEVICE(HUAWEI, HNAE3_DEV_ID_100G_RDMA_MACSEC), 0},
	/* required last entry */
	{0, }
};

static const char hns3_nic_test_strs[][ETH_GSTRING_LEN] = {
	"Mac    Loopback test",
	"Serdes Loopback test",
	"Phy    Loopback test"
};

static const struct hclge_comm_stats_str g_all_64bit_stats_string[] = {
	{"igu_rx_oversize_pkt",
		HCLGE_64BIT_STATS_FIELD_OFF(igu_rx_oversize_pkt)},
	{"igu_rx_undersize_pkt",
		HCLGE_64BIT_STATS_FIELD_OFF(igu_rx_undersize_pkt)},
	{"igu_rx_out_all_pkt",
		HCLGE_64BIT_STATS_FIELD_OFF(igu_rx_out_all_pkt)},
	{"igu_rx_uni_pkt",
		HCLGE_64BIT_STATS_FIELD_OFF(igu_rx_uni_pkt)},
	{"igu_rx_multi_pkt",
		HCLGE_64BIT_STATS_FIELD_OFF(igu_rx_multi_pkt)},
	{"igu_rx_broad_pkt",
		HCLGE_64BIT_STATS_FIELD_OFF(igu_rx_broad_pkt)},
	{"egu_tx_out_all_pkt",
		HCLGE_64BIT_STATS_FIELD_OFF(egu_tx_out_all_pkt)},
	{"egu_tx_uni_pkt",
		HCLGE_64BIT_STATS_FIELD_OFF(egu_tx_uni_pkt)},
	{"egu_tx_multi_pkt",
		HCLGE_64BIT_STATS_FIELD_OFF(egu_tx_multi_pkt)},
	{"egu_tx_broad_pkt",
		HCLGE_64BIT_STATS_FIELD_OFF(egu_tx_broad_pkt)},
	{"ssu_ppp_mac_key_num",
		HCLGE_64BIT_STATS_FIELD_OFF(ssu_ppp_mac_key_num)},
	{"ssu_ppp_host_key_num",
		HCLGE_64BIT_STATS_FIELD_OFF(ssu_ppp_host_key_num)},
	{"ppp_ssu_mac_rlt_num",
		HCLGE_64BIT_STATS_FIELD_OFF(ppp_ssu_mac_rlt_num)},
	{"ppp_ssu_host_rlt_num",
		HCLGE_64BIT_STATS_FIELD_OFF(ppp_ssu_host_rlt_num)},
	{"ssu_tx_in_num",
		HCLGE_64BIT_STATS_FIELD_OFF(ssu_tx_in_num)},
	{"ssu_tx_out_num",
		HCLGE_64BIT_STATS_FIELD_OFF(ssu_tx_out_num)},
	{"ssu_rx_in_num",
		HCLGE_64BIT_STATS_FIELD_OFF(ssu_rx_in_num)},
	{"ssu_rx_out_num",
		HCLGE_64BIT_STATS_FIELD_OFF(ssu_rx_out_num)}
};

static const struct hclge_comm_stats_str g_all_32bit_stats_string[] = {
	{"igu_rx_err_pkt",
		HCLGE_32BIT_STATS_FIELD_OFF(igu_rx_err_pkt)},
	{"igu_rx_no_eof_pkt",
		HCLGE_32BIT_STATS_FIELD_OFF(igu_rx_no_eof_pkt)},
	{"igu_rx_no_sof_pkt",
		HCLGE_32BIT_STATS_FIELD_OFF(igu_rx_no_sof_pkt)},
	{"egu_tx_1588_pkt",
		HCLGE_32BIT_STATS_FIELD_OFF(egu_tx_1588_pkt)},
	{"ssu_full_drop_num",
		HCLGE_32BIT_STATS_FIELD_OFF(ssu_full_drop_num)},
	{"ssu_part_drop_num",
		HCLGE_32BIT_STATS_FIELD_OFF(ssu_part_drop_num)},
	{"ppp_key_drop_num",
		HCLGE_32BIT_STATS_FIELD_OFF(ppp_key_drop_num)},
	{"ppp_rlt_drop_num",
		HCLGE_32BIT_STATS_FIELD_OFF(ppp_rlt_drop_num)},
	{"ssu_key_drop_num",
		HCLGE_32BIT_STATS_FIELD_OFF(ssu_key_drop_num)},
	{"pkt_curr_buf_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(pkt_curr_buf_cnt)},
	{"qcn_fb_rcv_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(qcn_fb_rcv_cnt)},
	{"qcn_fb_drop_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(qcn_fb_drop_cnt)},
	{"qcn_fb_invaild_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(qcn_fb_invaild_cnt)},
	{"rx_packet_tc0_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc0_in_cnt)},
	{"rx_packet_tc1_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc1_in_cnt)},
	{"rx_packet_tc2_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc2_in_cnt)},
	{"rx_packet_tc3_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc3_in_cnt)},
	{"rx_packet_tc4_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc4_in_cnt)},
	{"rx_packet_tc5_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc5_in_cnt)},
	{"rx_packet_tc6_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc6_in_cnt)},
	{"rx_packet_tc7_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc7_in_cnt)},
	{"rx_packet_tc0_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc0_out_cnt)},
	{"rx_packet_tc1_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc1_out_cnt)},
	{"rx_packet_tc2_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc2_out_cnt)},
	{"rx_packet_tc3_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc3_out_cnt)},
	{"rx_packet_tc4_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc4_out_cnt)},
	{"rx_packet_tc5_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc5_out_cnt)},
	{"rx_packet_tc6_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc6_out_cnt)},
	{"rx_packet_tc7_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_packet_tc7_out_cnt)},
	{"tx_packet_tc0_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc0_in_cnt)},
	{"tx_packet_tc1_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc1_in_cnt)},
	{"tx_packet_tc2_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc2_in_cnt)},
	{"tx_packet_tc3_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc3_in_cnt)},
	{"tx_packet_tc4_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc4_in_cnt)},
	{"tx_packet_tc5_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc5_in_cnt)},
	{"tx_packet_tc6_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc6_in_cnt)},
	{"tx_packet_tc7_in_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc7_in_cnt)},
	{"tx_packet_tc0_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc0_out_cnt)},
	{"tx_packet_tc1_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc1_out_cnt)},
	{"tx_packet_tc2_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc2_out_cnt)},
	{"tx_packet_tc3_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc3_out_cnt)},
	{"tx_packet_tc4_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc4_out_cnt)},
	{"tx_packet_tc5_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc5_out_cnt)},
	{"tx_packet_tc6_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc6_out_cnt)},
	{"tx_packet_tc7_out_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_packet_tc7_out_cnt)},
	{"pkt_curr_buf_tc0_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(pkt_curr_buf_tc0_cnt)},
	{"pkt_curr_buf_tc1_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(pkt_curr_buf_tc1_cnt)},
	{"pkt_curr_buf_tc2_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(pkt_curr_buf_tc2_cnt)},
	{"pkt_curr_buf_tc3_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(pkt_curr_buf_tc3_cnt)},
	{"pkt_curr_buf_tc4_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(pkt_curr_buf_tc4_cnt)},
	{"pkt_curr_buf_tc5_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(pkt_curr_buf_tc5_cnt)},
	{"pkt_curr_buf_tc6_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(pkt_curr_buf_tc6_cnt)},
	{"pkt_curr_buf_tc7_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(pkt_curr_buf_tc7_cnt)},
	{"mb_uncopy_num",
		HCLGE_32BIT_STATS_FIELD_OFF(mb_uncopy_num)},
	{"lo_pri_unicast_rlt_drop_num",
		HCLGE_32BIT_STATS_FIELD_OFF(lo_pri_unicast_rlt_drop_num)},
	{"hi_pri_multicast_rlt_drop_num",
		HCLGE_32BIT_STATS_FIELD_OFF(hi_pri_multicast_rlt_drop_num)},
	{"lo_pri_multicast_rlt_drop_num",
		HCLGE_32BIT_STATS_FIELD_OFF(lo_pri_multicast_rlt_drop_num)},
	{"rx_oq_drop_pkt_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(rx_oq_drop_pkt_cnt)},
	{"tx_oq_drop_pkt_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(tx_oq_drop_pkt_cnt)},
	{"nic_l2_err_drop_pkt_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(nic_l2_err_drop_pkt_cnt)},
	{"roc_l2_err_drop_pkt_cnt",
		HCLGE_32BIT_STATS_FIELD_OFF(roc_l2_err_drop_pkt_cnt)}
};

static const struct hclge_comm_stats_str g_mac_stats_string[] = {
	{"mac_tx_mac_pause_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_mac_pause_num)},
	{"mac_rx_mac_pause_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_mac_pause_num)},
	{"mac_tx_pfc_pri0_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_pfc_pri0_pkt_num)},
	{"mac_tx_pfc_pri1_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_pfc_pri1_pkt_num)},
	{"mac_tx_pfc_pri2_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_pfc_pri2_pkt_num)},
	{"mac_tx_pfc_pri3_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_pfc_pri3_pkt_num)},
	{"mac_tx_pfc_pri4_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_pfc_pri4_pkt_num)},
	{"mac_tx_pfc_pri5_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_pfc_pri5_pkt_num)},
	{"mac_tx_pfc_pri6_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_pfc_pri6_pkt_num)},
	{"mac_tx_pfc_pri7_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_pfc_pri7_pkt_num)},
	{"mac_rx_pfc_pri0_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_pfc_pri0_pkt_num)},
	{"mac_rx_pfc_pri1_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_pfc_pri1_pkt_num)},
	{"mac_rx_pfc_pri2_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_pfc_pri2_pkt_num)},
	{"mac_rx_pfc_pri3_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_pfc_pri3_pkt_num)},
	{"mac_rx_pfc_pri4_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_pfc_pri4_pkt_num)},
	{"mac_rx_pfc_pri5_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_pfc_pri5_pkt_num)},
	{"mac_rx_pfc_pri6_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_pfc_pri6_pkt_num)},
	{"mac_rx_pfc_pri7_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_pfc_pri7_pkt_num)},
	{"mac_tx_total_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_total_pkt_num)},
	{"mac_tx_total_oct_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_total_oct_num)},
	{"mac_tx_good_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_good_pkt_num)},
	{"mac_tx_bad_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_bad_pkt_num)},
	{"mac_tx_good_oct_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_good_oct_num)},
	{"mac_tx_bad_oct_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_bad_oct_num)},
	{"mac_tx_uni_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_uni_pkt_num)},
	{"mac_tx_multi_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_multi_pkt_num)},
	{"mac_tx_broad_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_broad_pkt_num)},
	{"mac_tx_undersize_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_undersize_pkt_num)},
	{"mac_tx_overrsize_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_overrsize_pkt_num)},
	{"mac_tx_64_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_64_oct_pkt_num)},
	{"mac_tx_65_127_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_65_127_oct_pkt_num)},
	{"mac_tx_128_255_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_128_255_oct_pkt_num)},
	{"mac_tx_256_511_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_256_511_oct_pkt_num)},
	{"mac_tx_512_1023_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_512_1023_oct_pkt_num)},
	{"mac_tx_1024_1518_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_1024_1518_oct_pkt_num)},
	{"mac_tx_1519_max_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_tx_1519_max_oct_pkt_num)},
	{"mac_rx_total_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_total_pkt_num)},
	{"mac_rx_total_oct_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_total_oct_num)},
	{"mac_rx_good_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_good_pkt_num)},
	{"mac_rx_bad_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_bad_pkt_num)},
	{"mac_rx_good_oct_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_good_oct_num)},
	{"mac_rx_bad_oct_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_bad_oct_num)},
	{"mac_rx_uni_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_uni_pkt_num)},
	{"mac_rx_multi_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_multi_pkt_num)},
	{"mac_rx_broad_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_broad_pkt_num)},
	{"mac_rx_undersize_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_undersize_pkt_num)},
	{"mac_rx_overrsize_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_overrsize_pkt_num)},
	{"mac_rx_64_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_64_oct_pkt_num)},
	{"mac_rx_65_127_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_65_127_oct_pkt_num)},
	{"mac_rx_128_255_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_128_255_oct_pkt_num)},
	{"mac_rx_256_511_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_256_511_oct_pkt_num)},
	{"mac_rx_512_1023_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_512_1023_oct_pkt_num)},
	{"mac_rx_1024_1518_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_1024_1518_oct_pkt_num)},
	{"mac_rx_1519_max_oct_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rx_1519_max_oct_pkt_num)},

	{"mac_trans_fragment_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_trans_fragment_pkt_num)},
	{"mac_trans_undermin_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_trans_undermin_pkt_num)},
	{"mac_trans_jabber_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_trans_jabber_pkt_num)},
	{"mac_trans_err_all_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_trans_err_all_pkt_num)},
	{"mac_trans_from_app_good_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_trans_from_app_good_pkt_num)},
	{"mac_trans_from_app_bad_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_trans_from_app_bad_pkt_num)},
	{"mac_rcv_fragment_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rcv_fragment_pkt_num)},
	{"mac_rcv_undermin_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rcv_undermin_pkt_num)},
	{"mac_rcv_jabber_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rcv_jabber_pkt_num)},
	{"mac_rcv_fcs_err_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rcv_fcs_err_pkt_num)},
	{"mac_rcv_send_app_good_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rcv_send_app_good_pkt_num)},
	{"mac_rcv_send_app_bad_pkt_num",
		HCLGE_MAC_STATS_FIELD_OFF(mac_rcv_send_app_bad_pkt_num)}
};

static int hclge_64_bit_update_stats(struct hclge_dev *hdev)
{
#define HCLGE_64_BIT_CMD_NUM 5
#define HCLGE_64_BIT_RTN_DATANUM 4
	u64 *data = (u64 *)(&hdev->hw_stats.all_64_bit_stats);
	struct hclge_desc desc[HCLGE_64_BIT_CMD_NUM];
	u64 *desc_data;
	int i, k, n;
	int ret;

	hclge_cmd_setup_basic_desc(&desc[0], HCLGE_OPC_STATS_64_BIT, true);
	ret = hclge_cmd_send(&hdev->hw, desc, HCLGE_64_BIT_CMD_NUM);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Get 64 bit pkt stats fail, status = %d.\n", ret);
		return ret;
	}

	for (i = 0; i < HCLGE_64_BIT_CMD_NUM; i++) {
		if (unlikely(i == 0)) {
			desc_data = (u64 *)(&desc[i].data[0]);
			n = HCLGE_64_BIT_RTN_DATANUM - 1;
		} else {
			desc_data = (u64 *)(&desc[i]);
			n = HCLGE_64_BIT_RTN_DATANUM;
		}
		for (k = 0; k < n; k++) {
			*data++ += cpu_to_le64(*desc_data);
			desc_data++;
		}
	}

	return 0;
}

static void hclge_reset_partial_32bit_counter(struct hclge_32_bit_stats *stats)
{
	stats->pkt_curr_buf_cnt     = 0;
	stats->pkt_curr_buf_tc0_cnt = 0;
	stats->pkt_curr_buf_tc1_cnt = 0;
	stats->pkt_curr_buf_tc2_cnt = 0;
	stats->pkt_curr_buf_tc3_cnt = 0;
	stats->pkt_curr_buf_tc4_cnt = 0;
	stats->pkt_curr_buf_tc5_cnt = 0;
	stats->pkt_curr_buf_tc6_cnt = 0;
	stats->pkt_curr_buf_tc7_cnt = 0;
}

static int hclge_32_bit_update_stats(struct hclge_dev *hdev)
{
#define HCLGE_32_BIT_CMD_NUM 8
#define HCLGE_32_BIT_RTN_DATANUM 8

	struct hclge_desc desc[HCLGE_32_BIT_CMD_NUM];
	struct hclge_32_bit_stats *all_32_bit_stats;
	u32 *desc_data;
	int i, k, n;
	u64 *data;
	int ret;

	all_32_bit_stats = &hdev->hw_stats.all_32_bit_stats;
	data = (u64 *)(&all_32_bit_stats->egu_tx_1588_pkt);

	hclge_cmd_setup_basic_desc(&desc[0], HCLGE_OPC_STATS_32_BIT, true);
	ret = hclge_cmd_send(&hdev->hw, desc, HCLGE_32_BIT_CMD_NUM);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Get 32 bit pkt stats fail, status = %d.\n", ret);

		return ret;
	}

	hclge_reset_partial_32bit_counter(all_32_bit_stats);
	for (i = 0; i < HCLGE_32_BIT_CMD_NUM; i++) {
		if (unlikely(i == 0)) {
			all_32_bit_stats->igu_rx_err_pkt +=
				cpu_to_le32(desc[i].data[0]);
			all_32_bit_stats->igu_rx_no_eof_pkt +=
				cpu_to_le32(desc[i].data[1] & 0xffff);
			all_32_bit_stats->igu_rx_no_sof_pkt +=
				cpu_to_le32((desc[i].data[1] >> 16) & 0xffff);

			desc_data = (u32 *)(&desc[i].data[2]);
			n = HCLGE_32_BIT_RTN_DATANUM - 4;
		} else {
			desc_data = (u32 *)(&desc[i]);
			n = HCLGE_32_BIT_RTN_DATANUM;
		}
		for (k = 0; k < n; k++) {
			*data++ += cpu_to_le32(*desc_data);
			desc_data++;
		}
	}

	return 0;
}

static int hclge_mac_update_stats(struct hclge_dev *hdev)
{
#define HCLGE_MAC_CMD_NUM 17
#define HCLGE_RTN_DATA_NUM 4

	u64 *data = (u64 *)(&hdev->hw_stats.mac_stats);
	struct hclge_desc desc[HCLGE_MAC_CMD_NUM];
	u64 *desc_data;
	int i, k, n;
	int ret;

	hclge_cmd_setup_basic_desc(&desc[0], HCLGE_OPC_STATS_MAC, true);
	ret = hclge_cmd_send(&hdev->hw, desc, HCLGE_MAC_CMD_NUM);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Get MAC pkt stats fail, status = %d.\n", ret);

		return ret;
	}

	for (i = 0; i < HCLGE_MAC_CMD_NUM; i++) {
		if (unlikely(i == 0)) {
			desc_data = (u64 *)(&desc[i].data[0]);
			n = HCLGE_RTN_DATA_NUM - 2;
		} else {
			desc_data = (u64 *)(&desc[i]);
			n = HCLGE_RTN_DATA_NUM;
		}
		for (k = 0; k < n; k++) {
			*data++ += cpu_to_le64(*desc_data);
			desc_data++;
		}
	}

	return 0;
}

static int hclge_tqps_update_stats(struct hnae3_handle *handle)
{
	struct hnae3_knic_private_info *kinfo = &handle->kinfo;
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	struct hnae3_queue *queue;
	struct hclge_desc desc[1];
	struct hclge_tqp *tqp;
	int ret, i;

	for (i = 0; i < kinfo->num_tqps; i++) {
		queue = handle->kinfo.tqp[i];
		tqp = container_of(queue, struct hclge_tqp, q);
		/* command : HCLGE_OPC_QUERY_IGU_STAT */
		hclge_cmd_setup_basic_desc(&desc[0],
					   HCLGE_OPC_QUERY_RX_STATUS,
					   true);

		desc[0].data[0] = (tqp->index & 0x1ff);
		ret = hclge_cmd_send(&hdev->hw, desc, 1);
		if (ret) {
			dev_err(&hdev->pdev->dev,
				"Query tqp stat fail, status = %d,queue = %d\n",
				ret,	i);
			return ret;
		}
		tqp->tqp_stats.rcb_rx_ring_pktnum_rcd +=
			cpu_to_le32(desc[0].data[4]);
	}

	for (i = 0; i < kinfo->num_tqps; i++) {
		queue = handle->kinfo.tqp[i];
		tqp = container_of(queue, struct hclge_tqp, q);
		/* command : HCLGE_OPC_QUERY_IGU_STAT */
		hclge_cmd_setup_basic_desc(&desc[0],
					   HCLGE_OPC_QUERY_TX_STATUS,
					   true);

		desc[0].data[0] = (tqp->index & 0x1ff);
		ret = hclge_cmd_send(&hdev->hw, desc, 1);
		if (ret) {
			dev_err(&hdev->pdev->dev,
				"Query tqp stat fail, status = %d,queue = %d\n",
				ret, i);
			return ret;
		}
		tqp->tqp_stats.rcb_tx_ring_pktnum_rcd +=
			cpu_to_le32(desc[0].data[4]);
	}

	return 0;
}

static u64 *hclge_tqps_get_stats(struct hnae3_handle *handle, u64 *data)
{
	struct hnae3_knic_private_info *kinfo = &handle->kinfo;
	struct hclge_tqp *tqp;
	u64 *buff = data;
	int i;

	for (i = 0; i < kinfo->num_tqps; i++) {
		tqp = container_of(kinfo->tqp[i], struct hclge_tqp, q);
		*buff++ = cpu_to_le64(tqp->tqp_stats.rcb_tx_ring_pktnum_rcd);
	}

	for (i = 0; i < kinfo->num_tqps; i++) {
		tqp = container_of(kinfo->tqp[i], struct hclge_tqp, q);
		*buff++ = cpu_to_le64(tqp->tqp_stats.rcb_rx_ring_pktnum_rcd);
	}

	return buff;
}

static int hclge_tqps_get_sset_count(struct hnae3_handle *handle, int stringset)
{
	struct hnae3_knic_private_info *kinfo = &handle->kinfo;

	return kinfo->num_tqps * (2);
}

static u8 *hclge_tqps_get_strings(struct hnae3_handle *handle, u8 *data)
{
	struct hnae3_knic_private_info *kinfo = &handle->kinfo;
	u8 *buff = data;
	int i = 0;

	for (i = 0; i < kinfo->num_tqps; i++) {
		struct hclge_tqp *tqp = container_of(handle->kinfo.tqp[i],
			struct hclge_tqp, q);
		snprintf(buff, ETH_GSTRING_LEN, "rcb_q%d_tx_pktnum_rcd",
			 tqp->index);
		buff = buff + ETH_GSTRING_LEN;
	}

	for (i = 0; i < kinfo->num_tqps; i++) {
		struct hclge_tqp *tqp = container_of(kinfo->tqp[i],
			struct hclge_tqp, q);
		snprintf(buff, ETH_GSTRING_LEN, "rcb_q%d_rx_pktnum_rcd",
			 tqp->index);
		buff = buff + ETH_GSTRING_LEN;
	}

	return buff;
}

static u64 *hclge_comm_get_stats(void *comm_stats,
				 const struct hclge_comm_stats_str strs[],
				 int size, u64 *data)
{
	u64 *buf = data;
	u32 i;

	for (i = 0; i < size; i++)
		buf[i] = HCLGE_STATS_READ(comm_stats, strs[i].offset);

	return buf + size;
}

static u8 *hclge_comm_get_strings(u32 stringset,
				  const struct hclge_comm_stats_str strs[],
				  int size, u8 *data)
{
	char *buff = (char *)data;
	u32 i;

	if (stringset != ETH_SS_STATS)
		return buff;

	for (i = 0; i < size; i++) {
		snprintf(buff, ETH_GSTRING_LEN, "%s", strs[i].desc);
		buff = buff + ETH_GSTRING_LEN;
	}

	return (u8 *)buff;
}

static void hclge_update_netstat(struct hclge_hw_stats *hw_stats,
				 struct net_device_stats *net_stats)
{
	net_stats->tx_dropped = 0;
	net_stats->rx_dropped = hw_stats->all_32_bit_stats.ssu_full_drop_num;
	net_stats->rx_dropped += hw_stats->all_32_bit_stats.ppp_key_drop_num;
	net_stats->rx_dropped += hw_stats->all_32_bit_stats.ssu_key_drop_num;

	net_stats->rx_errors = hw_stats->mac_stats.mac_rx_overrsize_pkt_num;
	net_stats->rx_errors += hw_stats->mac_stats.mac_rx_undersize_pkt_num;
	net_stats->rx_errors += hw_stats->all_32_bit_stats.igu_rx_err_pkt;
	net_stats->rx_errors += hw_stats->all_32_bit_stats.igu_rx_no_eof_pkt;
	net_stats->rx_errors += hw_stats->all_32_bit_stats.igu_rx_no_sof_pkt;
	net_stats->rx_errors += hw_stats->mac_stats.mac_rcv_fcs_err_pkt_num;

	net_stats->multicast = hw_stats->mac_stats.mac_tx_multi_pkt_num;
	net_stats->multicast += hw_stats->mac_stats.mac_rx_multi_pkt_num;

	net_stats->rx_crc_errors = hw_stats->mac_stats.mac_rcv_fcs_err_pkt_num;
	net_stats->rx_length_errors =
		hw_stats->mac_stats.mac_rx_undersize_pkt_num;
	net_stats->rx_length_errors +=
		hw_stats->mac_stats.mac_rx_overrsize_pkt_num;
	net_stats->rx_over_errors =
		hw_stats->mac_stats.mac_rx_overrsize_pkt_num;
}

static void hclge_update_stats_for_all(struct hclge_dev *hdev)
{
	struct hnae3_handle *handle;
	int status;

	handle = &hdev->vport[0].nic;
	if (handle->client) {
		status = hclge_tqps_update_stats(handle);
		if (status) {
			dev_err(&hdev->pdev->dev,
				"Update TQPS stats fail, status = %d.\n",
				status);
		}
	}

	status = hclge_mac_update_stats(hdev);
	if (status)
		dev_err(&hdev->pdev->dev,
			"Update MAC stats fail, status = %d.\n", status);

	status = hclge_32_bit_update_stats(hdev);
	if (status)
		dev_err(&hdev->pdev->dev,
			"Update 32 bit stats fail, status = %d.\n",
			status);

	hclge_update_netstat(&hdev->hw_stats, &handle->kinfo.netdev->stats);
}

static void hclge_update_stats(struct hnae3_handle *handle,
			       struct net_device_stats *net_stats)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	struct hclge_hw_stats *hw_stats = &hdev->hw_stats;
	int status;

	status = hclge_mac_update_stats(hdev);
	if (status)
		dev_err(&hdev->pdev->dev,
			"Update MAC stats fail, status = %d.\n",
			status);

	status = hclge_32_bit_update_stats(hdev);
	if (status)
		dev_err(&hdev->pdev->dev,
			"Update 32 bit stats fail, status = %d.\n",
			status);

	status = hclge_64_bit_update_stats(hdev);
	if (status)
		dev_err(&hdev->pdev->dev,
			"Update 64 bit stats fail, status = %d.\n",
			status);

	status = hclge_tqps_update_stats(handle);
	if (status)
		dev_err(&hdev->pdev->dev,
			"Update TQPS stats fail, status = %d.\n",
			status);

	hclge_update_netstat(hw_stats, net_stats);
}

static int hclge_get_sset_count(struct hnae3_handle *handle, int stringset)
{
#define HCLGE_LOOPBACK_TEST_FLAGS 0x7

	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	int count = 0;

	/* Loopback test support rules:
	 * mac: only GE mode support
	 * serdes: all mac mode will support include GE/XGE/LGE/CGE
	 * phy: only support when phy device exist on board
	 */
	if (stringset == ETH_SS_TEST) {
		/* clear loopback bit flags at first */
		handle->flags = (handle->flags & (~HCLGE_LOOPBACK_TEST_FLAGS));
		if (hdev->hw.mac.speed == HCLGE_MAC_SPEED_10M ||
		    hdev->hw.mac.speed == HCLGE_MAC_SPEED_100M ||
		    hdev->hw.mac.speed == HCLGE_MAC_SPEED_1G) {
			count += 1;
			handle->flags |= HNAE3_SUPPORT_MAC_LOOPBACK;
		} else {
			count = -EOPNOTSUPP;
		}
	} else if (stringset == ETH_SS_STATS) {
		count = ARRAY_SIZE(g_mac_stats_string) +
			ARRAY_SIZE(g_all_32bit_stats_string) +
			ARRAY_SIZE(g_all_64bit_stats_string) +
			hclge_tqps_get_sset_count(handle, stringset);
	}

	return count;
}

static void hclge_get_strings(struct hnae3_handle *handle,
			      u32 stringset,
			      u8 *data)
{
	u8 *p = (char *)data;
	int size;

	if (stringset == ETH_SS_STATS) {
		size = ARRAY_SIZE(g_mac_stats_string);
		p = hclge_comm_get_strings(stringset,
					   g_mac_stats_string,
					   size,
					   p);
		size = ARRAY_SIZE(g_all_32bit_stats_string);
		p = hclge_comm_get_strings(stringset,
					   g_all_32bit_stats_string,
					   size,
					   p);
		size = ARRAY_SIZE(g_all_64bit_stats_string);
		p = hclge_comm_get_strings(stringset,
					   g_all_64bit_stats_string,
					   size,
					   p);
		p = hclge_tqps_get_strings(handle, p);
	} else if (stringset == ETH_SS_TEST) {
		if (handle->flags & HNAE3_SUPPORT_MAC_LOOPBACK) {
			memcpy(p,
			       hns3_nic_test_strs[HNAE3_MAC_INTER_LOOP_MAC],
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		if (handle->flags & HNAE3_SUPPORT_SERDES_LOOPBACK) {
			memcpy(p,
			       hns3_nic_test_strs[HNAE3_MAC_INTER_LOOP_SERDES],
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		if (handle->flags & HNAE3_SUPPORT_PHY_LOOPBACK) {
			memcpy(p,
			       hns3_nic_test_strs[HNAE3_MAC_INTER_LOOP_PHY],
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
	}
}

static void hclge_get_stats(struct hnae3_handle *handle, u64 *data)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	u64 *p;

	p = hclge_comm_get_stats(&hdev->hw_stats.mac_stats,
				 g_mac_stats_string,
				 ARRAY_SIZE(g_mac_stats_string),
				 data);
	p = hclge_comm_get_stats(&hdev->hw_stats.all_32_bit_stats,
				 g_all_32bit_stats_string,
				 ARRAY_SIZE(g_all_32bit_stats_string),
				 p);
	p = hclge_comm_get_stats(&hdev->hw_stats.all_64_bit_stats,
				 g_all_64bit_stats_string,
				 ARRAY_SIZE(g_all_64bit_stats_string),
				 p);
	p = hclge_tqps_get_stats(handle, p);
}

static int hclge_parse_func_status(struct hclge_dev *hdev,
				   struct hclge_func_status *status)
{
	if (!(status->pf_state & HCLGE_PF_STATE_DONE))
		return -EINVAL;

	/* Set the pf to main pf */
	if (status->pf_state & HCLGE_PF_STATE_MAIN)
		hdev->flag |= HCLGE_FLAG_MAIN;
	else
		hdev->flag &= ~HCLGE_FLAG_MAIN;

	hdev->num_req_vfs = status->vf_num / status->pf_num;
	return 0;
}

static int hclge_query_function_status(struct hclge_dev *hdev)
{
	struct hclge_func_status *req;
	struct hclge_desc desc;
	int timeout = 0;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_QUERY_FUNC_STATUS, true);
	req = (struct hclge_func_status *)desc.data;

	do {
		ret = hclge_cmd_send(&hdev->hw, &desc, 1);
		if (ret) {
			dev_err(&hdev->pdev->dev,
				"query function status failed %d.\n",
				ret);

			return ret;
		}

		/* Check pf reset is done */
		if (req->pf_state)
			break;
		usleep_range(1000, 2000);
	} while (timeout++ < 5);

	ret = hclge_parse_func_status(hdev, req);

	return ret;
}

static int hclge_query_pf_resource(struct hclge_dev *hdev)
{
	struct hclge_pf_res *req;
	struct hclge_desc desc;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_QUERY_PF_RSRC, true);
	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"query pf resource failed %d.\n", ret);
		return ret;
	}

	req = (struct hclge_pf_res *)desc.data;
	hdev->num_tqps = __le16_to_cpu(req->tqp_num);
	hdev->pkt_buf_size = __le16_to_cpu(req->buf_size) << HCLGE_BUF_UNIT_S;

	if (hnae3_dev_roce_supported(hdev)) {
		hdev->num_roce_msix =
		hnae_get_field(__le16_to_cpu(req->pf_intr_vector_number),
			       HCLGE_PF_VEC_NUM_M, HCLGE_PF_VEC_NUM_S);

		/* PF should have NIC vectors and Roce vectors,
		 * NIC vectors are queued before Roce vectors.
		 */
		hdev->num_msi = hdev->num_roce_msix  + HCLGE_ROCE_VECTOR_OFFSET;
	} else {
		hdev->num_msi =
		hnae_get_field(__le16_to_cpu(req->pf_intr_vector_number),
			       HCLGE_PF_VEC_NUM_M, HCLGE_PF_VEC_NUM_S);
	}

	return 0;
}

static int hclge_parse_speed(int speed_cmd, int *speed)
{
	switch (speed_cmd) {
	case 6:
		*speed = HCLGE_MAC_SPEED_10M;
		break;
	case 7:
		*speed = HCLGE_MAC_SPEED_100M;
		break;
	case 0:
		*speed = HCLGE_MAC_SPEED_1G;
		break;
	case 1:
		*speed = HCLGE_MAC_SPEED_10G;
		break;
	case 2:
		*speed = HCLGE_MAC_SPEED_25G;
		break;
	case 3:
		*speed = HCLGE_MAC_SPEED_40G;
		break;
	case 4:
		*speed = HCLGE_MAC_SPEED_50G;
		break;
	case 5:
		*speed = HCLGE_MAC_SPEED_100G;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void hclge_parse_cfg(struct hclge_cfg *cfg, struct hclge_desc *desc)
{
	struct hclge_cfg_param *req;
	u64 mac_addr_tmp_high;
	u64 mac_addr_tmp;
	int i;

	req = (struct hclge_cfg_param *)desc[0].data;

	/* get the configuration */
	cfg->vmdq_vport_num = hnae_get_field(__le32_to_cpu(req->param[0]),
					     HCLGE_CFG_VMDQ_M,
					     HCLGE_CFG_VMDQ_S);
	cfg->tc_num = hnae_get_field(__le32_to_cpu(req->param[0]),
				     HCLGE_CFG_TC_NUM_M, HCLGE_CFG_TC_NUM_S);
	cfg->tqp_desc_num = hnae_get_field(__le32_to_cpu(req->param[0]),
					   HCLGE_CFG_TQP_DESC_N_M,
					   HCLGE_CFG_TQP_DESC_N_S);

	cfg->phy_addr = hnae_get_field(__le32_to_cpu(req->param[1]),
				       HCLGE_CFG_PHY_ADDR_M,
				       HCLGE_CFG_PHY_ADDR_S);
	cfg->media_type = hnae_get_field(__le32_to_cpu(req->param[1]),
					 HCLGE_CFG_MEDIA_TP_M,
					 HCLGE_CFG_MEDIA_TP_S);
	cfg->rx_buf_len = hnae_get_field(__le32_to_cpu(req->param[1]),
					 HCLGE_CFG_RX_BUF_LEN_M,
					 HCLGE_CFG_RX_BUF_LEN_S);
	/* get mac_address */
	mac_addr_tmp = __le32_to_cpu(req->param[2]);
	mac_addr_tmp_high = hnae_get_field(__le32_to_cpu(req->param[3]),
					   HCLGE_CFG_MAC_ADDR_H_M,
					   HCLGE_CFG_MAC_ADDR_H_S);

	mac_addr_tmp |= (mac_addr_tmp_high << 31) << 1;

	cfg->default_speed = hnae_get_field(__le32_to_cpu(req->param[3]),
					    HCLGE_CFG_DEFAULT_SPEED_M,
					    HCLGE_CFG_DEFAULT_SPEED_S);
	for (i = 0; i < ETH_ALEN; i++)
		cfg->mac_addr[i] = (mac_addr_tmp >> (8 * i)) & 0xff;

	req = (struct hclge_cfg_param *)desc[1].data;
	cfg->numa_node_map = __le32_to_cpu(req->param[0]);
}

/* hclge_get_cfg: query the static parameter from flash
 * @hdev: pointer to struct hclge_dev
 * @hcfg: the config structure to be getted
 */
static int hclge_get_cfg(struct hclge_dev *hdev, struct hclge_cfg *hcfg)
{
	struct hclge_desc desc[HCLGE_PF_CFG_DESC_NUM];
	struct hclge_cfg_param *req;
	int i, ret;

	for (i = 0; i < HCLGE_PF_CFG_DESC_NUM; i++) {
		req = (struct hclge_cfg_param *)desc[i].data;
		hclge_cmd_setup_basic_desc(&desc[i], HCLGE_OPC_GET_CFG_PARAM,
					   true);
		hnae_set_field(req->offset, HCLGE_CFG_OFFSET_M,
			       HCLGE_CFG_OFFSET_S, i * HCLGE_CFG_RD_LEN_BYTES);
		/* Len should be united by 4 bytes when send to hardware */
		hnae_set_field(req->offset, HCLGE_CFG_RD_LEN_M,
			       HCLGE_CFG_RD_LEN_S,
			       HCLGE_CFG_RD_LEN_BYTES / HCLGE_CFG_RD_LEN_UNIT);
		req->offset = cpu_to_le32(req->offset);
	}

	ret = hclge_cmd_send(&hdev->hw, desc, HCLGE_PF_CFG_DESC_NUM);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"get config failed %d.\n", ret);
		return ret;
	}

	hclge_parse_cfg(hcfg, desc);
	return 0;
}

static int hclge_get_cap(struct hclge_dev *hdev)
{
	int ret;

	ret = hclge_query_function_status(hdev);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"query function status error %d.\n", ret);
		return ret;
	}

	/* get pf resource */
	ret = hclge_query_pf_resource(hdev);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"query pf resource error %d.\n", ret);
		return ret;
	}

	return 0;
}

static int hclge_configure(struct hclge_dev *hdev)
{
	struct hclge_cfg cfg;
	int ret, i;

	ret = hclge_get_cfg(hdev, &cfg);
	if (ret) {
		dev_err(&hdev->pdev->dev, "get mac mode error %d.\n", ret);
		return ret;
	}

	hdev->num_vmdq_vport = cfg.vmdq_vport_num;
	hdev->base_tqp_pid = 0;
	hdev->rss_size_max = 1;
	hdev->rx_buf_len = cfg.rx_buf_len;
	ether_addr_copy(hdev->hw.mac.mac_addr, cfg.mac_addr);
	hdev->hw.mac.media_type = cfg.media_type;
	hdev->hw.mac.phy_addr = cfg.phy_addr;
	hdev->num_desc = cfg.tqp_desc_num;
	hdev->tm_info.num_pg = 1;
	hdev->tm_info.num_tc = cfg.tc_num;
	hdev->tm_info.hw_pfc_map = 0;

	ret = hclge_parse_speed(cfg.default_speed, &hdev->hw.mac.speed);
	if (ret) {
		dev_err(&hdev->pdev->dev, "Get wrong speed ret=%d.\n", ret);
		return ret;
	}

	if ((hdev->tm_info.num_tc > HNAE3_MAX_TC) ||
	    (hdev->tm_info.num_tc < 1)) {
		dev_warn(&hdev->pdev->dev, "TC num = %d.\n",
			 hdev->tm_info.num_tc);
		hdev->tm_info.num_tc = 1;
	}

	/* Currently not support uncontiuous tc */
	for (i = 0; i < cfg.tc_num; i++)
		hnae_set_bit(hdev->hw_tc_map, i, 1);

	if (!hdev->num_vmdq_vport && !hdev->num_req_vfs)
		hdev->tx_sch_mode = HCLGE_FLAG_TC_BASE_SCH_MODE;
	else
		hdev->tx_sch_mode = HCLGE_FLAG_VNET_BASE_SCH_MODE;

	return ret;
}

static int hclge_config_tso(struct hclge_dev *hdev, int tso_mss_min,
			    int tso_mss_max)
{
	struct hclge_cfg_tso_status *req;
	struct hclge_desc desc;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_TSO_GENERIC_CONFIG, false);

	req = (struct hclge_cfg_tso_status *)desc.data;
	hnae_set_field(req->tso_mss_min, HCLGE_TSO_MSS_MIN_M,
		       HCLGE_TSO_MSS_MIN_S, tso_mss_min);
	hnae_set_field(req->tso_mss_max, HCLGE_TSO_MSS_MIN_M,
		       HCLGE_TSO_MSS_MIN_S, tso_mss_max);

	return hclge_cmd_send(&hdev->hw, &desc, 1);
}

static int hclge_alloc_tqps(struct hclge_dev *hdev)
{
	struct hclge_tqp *tqp;
	int i;

	hdev->htqp = devm_kcalloc(&hdev->pdev->dev, hdev->num_tqps,
				  sizeof(struct hclge_tqp), GFP_KERNEL);
	if (!hdev->htqp)
		return -ENOMEM;

	tqp = hdev->htqp;

	for (i = 0; i < hdev->num_tqps; i++) {
		tqp->dev = &hdev->pdev->dev;
		tqp->index = i;

		tqp->q.ae_algo = &ae_algo;
		tqp->q.buf_size = hdev->rx_buf_len;
		tqp->q.desc_num = hdev->num_desc;
		tqp->q.io_base = hdev->hw.io_base + HCLGE_TQP_REG_OFFSET +
			i * HCLGE_TQP_REG_SIZE;

		tqp++;
	}

	return 0;
}

static int hclge_map_tqps_to_func(struct hclge_dev *hdev, u16 func_id,
				  u16 tqp_pid, u16 tqp_vid, bool is_pf)
{
	struct hclge_tqp_map *req;
	struct hclge_desc desc;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_SET_TQP_MAP, false);

	req = (struct hclge_tqp_map *)desc.data;
	req->tqp_id = cpu_to_le16(tqp_pid);
	req->tqp_vf = cpu_to_le16(func_id);
	req->tqp_flag = !is_pf << HCLGE_TQP_MAP_TYPE_B |
			1 << HCLGE_TQP_MAP_EN_B;
	req->tqp_vid = cpu_to_le16(tqp_vid);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev, "TQP map failed %d.\n",
			ret);
		return ret;
	}

	return 0;
}

static int  hclge_assign_tqp(struct hclge_vport *vport,
			     struct hnae3_queue **tqp, u16 num_tqps)
{
	struct hclge_dev *hdev = vport->back;
	int i, alloced, func_id, ret;
	bool is_pf;

	func_id = vport->vport_id;
	is_pf = (vport->vport_id == 0) ? true : false;

	for (i = 0, alloced = 0; i < hdev->num_tqps &&
	     alloced < num_tqps; i++) {
		if (!hdev->htqp[i].alloced) {
			hdev->htqp[i].q.handle = &vport->nic;
			hdev->htqp[i].q.tqp_index = alloced;
			tqp[alloced] = &hdev->htqp[i].q;
			hdev->htqp[i].alloced = true;
			ret = hclge_map_tqps_to_func(hdev, func_id,
						     hdev->htqp[i].index,
						     alloced, is_pf);
			if (ret)
				return ret;

			alloced++;
		}
	}
	vport->alloc_tqps = num_tqps;

	return 0;
}

static int hclge_knic_setup(struct hclge_vport *vport, u16 num_tqps)
{
	struct hnae3_handle *nic = &vport->nic;
	struct hnae3_knic_private_info *kinfo = &nic->kinfo;
	struct hclge_dev *hdev = vport->back;
	int i, ret;

	kinfo->num_desc = hdev->num_desc;
	kinfo->rx_buf_len = hdev->rx_buf_len;
	kinfo->num_tc = min_t(u16, num_tqps, hdev->tm_info.num_tc);
	kinfo->rss_size
		= min_t(u16, hdev->rss_size_max, num_tqps / kinfo->num_tc);
	kinfo->num_tqps = kinfo->rss_size * kinfo->num_tc;

	for (i = 0; i < HNAE3_MAX_TC; i++) {
		if (hdev->hw_tc_map & BIT(i)) {
			kinfo->tc_info[i].enable = true;
			kinfo->tc_info[i].tqp_offset = i * kinfo->rss_size;
			kinfo->tc_info[i].tqp_count = kinfo->rss_size;
			kinfo->tc_info[i].tc = i;
		} else {
			/* Set to default queue if TC is disable */
			kinfo->tc_info[i].enable = false;
			kinfo->tc_info[i].tqp_offset = 0;
			kinfo->tc_info[i].tqp_count = 1;
			kinfo->tc_info[i].tc = 0;
		}
	}

	kinfo->tqp = devm_kcalloc(&hdev->pdev->dev, kinfo->num_tqps,
				  sizeof(struct hnae3_queue *), GFP_KERNEL);
	if (!kinfo->tqp)
		return -ENOMEM;

	ret = hclge_assign_tqp(vport, kinfo->tqp, kinfo->num_tqps);
	if (ret) {
		dev_err(&hdev->pdev->dev, "fail to assign TQPs %d.\n", ret);
		return -EINVAL;
	}

	return 0;
}

static void hclge_unic_setup(struct hclge_vport *vport, u16 num_tqps)
{
	/* this would be initialized later */
}

static int hclge_vport_setup(struct hclge_vport *vport, u16 num_tqps)
{
	struct hnae3_handle *nic = &vport->nic;
	struct hclge_dev *hdev = vport->back;
	int ret;

	nic->pdev = hdev->pdev;
	nic->ae_algo = &ae_algo;
	nic->numa_node_mask = hdev->numa_node_mask;

	if (hdev->ae_dev->dev_type == HNAE3_DEV_KNIC) {
		ret = hclge_knic_setup(vport, num_tqps);
		if (ret) {
			dev_err(&hdev->pdev->dev, "knic setup failed %d\n",
				ret);
			return ret;
		}
	} else {
		hclge_unic_setup(vport, num_tqps);
	}

	return 0;
}

static int hclge_alloc_vport(struct hclge_dev *hdev)
{
	struct pci_dev *pdev = hdev->pdev;
	struct hclge_vport *vport;
	u32 tqp_main_vport;
	u32 tqp_per_vport;
	int num_vport, i;
	int ret;

	/* We need to alloc a vport for main NIC of PF */
	num_vport = hdev->num_vmdq_vport + hdev->num_req_vfs + 1;

	if (hdev->num_tqps < num_vport) {
		dev_err(&hdev->pdev->dev, "tqps(%d) is less than vports(%d)",
			hdev->num_tqps, num_vport);
		return -EINVAL;
	}

	/* Alloc the same number of TQPs for every vport */
	tqp_per_vport = hdev->num_tqps / num_vport;
	tqp_main_vport = tqp_per_vport + hdev->num_tqps % num_vport;

	vport = devm_kcalloc(&pdev->dev, num_vport, sizeof(struct hclge_vport),
			     GFP_KERNEL);
	if (!vport)
		return -ENOMEM;

	hdev->vport = vport;
	hdev->num_alloc_vport = num_vport;

#ifdef CONFIG_PCI_IOV
	/* Enable SRIOV */
	if (hdev->num_req_vfs) {
		dev_info(&pdev->dev, "active VFs(%d) found, enabling SRIOV\n",
			 hdev->num_req_vfs);
		ret = pci_enable_sriov(hdev->pdev, hdev->num_req_vfs);
		if (ret) {
			hdev->num_alloc_vfs = 0;
			dev_err(&pdev->dev, "SRIOV enable failed %d\n",
				ret);
			return ret;
		}
	}
	hdev->num_alloc_vfs = hdev->num_req_vfs;
#endif

	for (i = 0; i < num_vport; i++) {
		vport->back = hdev;
		vport->vport_id = i;

		if (i == 0)
			ret = hclge_vport_setup(vport, tqp_main_vport);
		else
			ret = hclge_vport_setup(vport, tqp_per_vport);
		if (ret) {
			dev_err(&pdev->dev,
				"vport setup failed for vport %d, %d\n",
				i, ret);
			return ret;
		}

		vport++;
	}

	return 0;
}

static int  hclge_cmd_alloc_tx_buff(struct hclge_dev *hdev, u16 buf_size)
{
/* TX buffer size is unit by 128 byte */
#define HCLGE_BUF_SIZE_UNIT_SHIFT	7
#define HCLGE_BUF_SIZE_UPDATE_EN_MSK	BIT(15)
	struct hclge_tx_buff_alloc *req;
	struct hclge_desc desc;
	int ret;
	u8 i;

	req = (struct hclge_tx_buff_alloc *)desc.data;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_TX_BUFF_ALLOC, 0);
	for (i = 0; i < HCLGE_TC_NUM; i++)
		req->tx_pkt_buff[i] =
			cpu_to_le16((buf_size >> HCLGE_BUF_SIZE_UNIT_SHIFT) |
				     HCLGE_BUF_SIZE_UPDATE_EN_MSK);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev, "tx buffer alloc cmd failed %d.\n",
			ret);
		return ret;
	}

	return 0;
}

static int hclge_tx_buffer_alloc(struct hclge_dev *hdev, u32 buf_size)
{
	int ret = hclge_cmd_alloc_tx_buff(hdev, buf_size);

	if (ret) {
		dev_err(&hdev->pdev->dev,
			"tx buffer alloc failed %d\n", ret);
		return ret;
	}

	return 0;
}

static int hclge_get_tc_num(struct hclge_dev *hdev)
{
	int i, cnt = 0;

	for (i = 0; i < HCLGE_MAX_TC_NUM; i++)
		if (hdev->hw_tc_map & BIT(i))
			cnt++;
	return cnt;
}

static int hclge_get_pfc_enalbe_num(struct hclge_dev *hdev)
{
	int i, cnt = 0;

	for (i = 0; i < HCLGE_MAX_TC_NUM; i++)
		if (hdev->hw_tc_map & BIT(i) &&
		    hdev->tm_info.hw_pfc_map & BIT(i))
			cnt++;
	return cnt;
}

/* Get the number of pfc enabled TCs, which have private buffer */
static int hclge_get_pfc_priv_num(struct hclge_dev *hdev)
{
	struct hclge_priv_buf *priv;
	int i, cnt = 0;

	for (i = 0; i < HCLGE_MAX_TC_NUM; i++) {
		priv = &hdev->priv_buf[i];
		if ((hdev->tm_info.hw_pfc_map & BIT(i)) &&
		    priv->enable)
			cnt++;
	}

	return cnt;
}

/* Get the number of pfc disabled TCs, which have private buffer */
static int hclge_get_no_pfc_priv_num(struct hclge_dev *hdev)
{
	struct hclge_priv_buf *priv;
	int i, cnt = 0;

	for (i = 0; i < HCLGE_MAX_TC_NUM; i++) {
		priv = &hdev->priv_buf[i];
		if (hdev->hw_tc_map & BIT(i) &&
		    !(hdev->tm_info.hw_pfc_map & BIT(i)) &&
		    priv->enable)
			cnt++;
	}

	return cnt;
}

static u32 hclge_get_rx_priv_buff_alloced(struct hclge_dev *hdev)
{
	struct hclge_priv_buf *priv;
	u32 rx_priv = 0;
	int i;

	for (i = 0; i < HCLGE_MAX_TC_NUM; i++) {
		priv = &hdev->priv_buf[i];
		if (priv->enable)
			rx_priv += priv->buf_size;
	}
	return rx_priv;
}

static bool  hclge_is_rx_buf_ok(struct hclge_dev *hdev, u32 rx_all)
{
	u32 shared_buf_min, shared_buf_tc, shared_std;
	int tc_num, pfc_enable_num;
	u32 shared_buf;
	u32 rx_priv;
	int i;

	tc_num = hclge_get_tc_num(hdev);
	pfc_enable_num = hclge_get_pfc_enalbe_num(hdev);

	if (hnae3_dev_dcb_supported(hdev))
		shared_buf_min = 2 * hdev->mps + HCLGE_DEFAULT_DV;
	else
		shared_buf_min = 2 * hdev->mps + HCLGE_DEFAULT_NON_DCB_DV;

	shared_buf_tc = pfc_enable_num * hdev->mps +
			(tc_num - pfc_enable_num) * hdev->mps / 2 +
			hdev->mps;
	shared_std = max_t(u32, shared_buf_min, shared_buf_tc);

	rx_priv = hclge_get_rx_priv_buff_alloced(hdev);
	if (rx_all <= rx_priv + shared_std)
		return false;

	shared_buf = rx_all - rx_priv;
	hdev->s_buf.buf_size = shared_buf;
	hdev->s_buf.self.high = shared_buf;
	hdev->s_buf.self.low =  2 * hdev->mps;

	for (i = 0; i < HCLGE_MAX_TC_NUM; i++) {
		if ((hdev->hw_tc_map & BIT(i)) &&
		    (hdev->tm_info.hw_pfc_map & BIT(i))) {
			hdev->s_buf.tc_thrd[i].low = hdev->mps;
			hdev->s_buf.tc_thrd[i].high = 2 * hdev->mps;
		} else {
			hdev->s_buf.tc_thrd[i].low = 0;
			hdev->s_buf.tc_thrd[i].high = hdev->mps;
		}
	}

	return true;
}

/* hclge_rx_buffer_calc: calculate the rx private buffer size for all TCs
 * @hdev: pointer to struct hclge_dev
 * @tx_size: the allocated tx buffer for all TCs
 * @return: 0: calculate sucessful, negative: fail
 */
int hclge_rx_buffer_calc(struct hclge_dev *hdev, u32 tx_size)
{
	u32 rx_all = hdev->pkt_buf_size - tx_size;
	int no_pfc_priv_num, pfc_priv_num;
	struct hclge_priv_buf *priv;
	int i;

	/* When DCB is not supported, rx private
	 * buffer is not allocated.
	 */
	if (!hnae3_dev_dcb_supported(hdev)) {
		if (!hclge_is_rx_buf_ok(hdev, rx_all))
			return -ENOMEM;

		return 0;
	}

	/* step 1, try to alloc private buffer for all enabled tc */
	for (i = 0; i < HCLGE_MAX_TC_NUM; i++) {
		priv = &hdev->priv_buf[i];
		if (hdev->hw_tc_map & BIT(i)) {
			priv->enable = 1;
			if (hdev->tm_info.hw_pfc_map & BIT(i)) {
				priv->wl.low = hdev->mps;
				priv->wl.high = priv->wl.low + hdev->mps;
				priv->buf_size = priv->wl.high +
						HCLGE_DEFAULT_DV;
			} else {
				priv->wl.low = 0;
				priv->wl.high = 2 * hdev->mps;
				priv->buf_size = priv->wl.high;
			}
		} else {
			priv->enable = 0;
			priv->wl.low = 0;
			priv->wl.high = 0;
			priv->buf_size = 0;
		}
	}

	if (hclge_is_rx_buf_ok(hdev, rx_all))
		return 0;

	/* step 2, try to decrease the buffer size of
	 * no pfc TC's private buffer
	 */
	for (i = 0; i < HCLGE_MAX_TC_NUM; i++) {
		priv = &hdev->priv_buf[i];

		priv->enable = 0;
		priv->wl.low = 0;
		priv->wl.high = 0;
		priv->buf_size = 0;

		if (!(hdev->hw_tc_map & BIT(i)))
			continue;

		priv->enable = 1;

		if (hdev->tm_info.hw_pfc_map & BIT(i)) {
			priv->wl.low = 128;
			priv->wl.high = priv->wl.low + hdev->mps;
			priv->buf_size = priv->wl.high + HCLGE_DEFAULT_DV;
		} else {
			priv->wl.low = 0;
			priv->wl.high = hdev->mps;
			priv->buf_size = priv->wl.high;
		}
	}

	if (hclge_is_rx_buf_ok(hdev, rx_all))
		return 0;

	/* step 3, try to reduce the number of pfc disabled TCs,
	 * which have private buffer
	 */
	/* get the total no pfc enable TC number, which have private buffer */
	no_pfc_priv_num = hclge_get_no_pfc_priv_num(hdev);

	/* let the last to be cleared first */
	for (i = HCLGE_MAX_TC_NUM - 1; i >= 0; i--) {
		priv = &hdev->priv_buf[i];

		if (hdev->hw_tc_map & BIT(i) &&
		    !(hdev->tm_info.hw_pfc_map & BIT(i))) {
			/* Clear the no pfc TC private buffer */
			priv->wl.low = 0;
			priv->wl.high = 0;
			priv->buf_size = 0;
			priv->enable = 0;
			no_pfc_priv_num--;
		}

		if (hclge_is_rx_buf_ok(hdev, rx_all) ||
		    no_pfc_priv_num == 0)
			break;
	}

	if (hclge_is_rx_buf_ok(hdev, rx_all))
		return 0;

	/* step 4, try to reduce the number of pfc enabled TCs
	 * which have private buffer.
	 */
	pfc_priv_num = hclge_get_pfc_priv_num(hdev);

	/* let the last to be cleared first */
	for (i = HCLGE_MAX_TC_NUM - 1; i >= 0; i--) {
		priv = &hdev->priv_buf[i];

		if (hdev->hw_tc_map & BIT(i) &&
		    hdev->tm_info.hw_pfc_map & BIT(i)) {
			/* Reduce the number of pfc TC with private buffer */
			priv->wl.low = 0;
			priv->enable = 0;
			priv->wl.high = 0;
			priv->buf_size = 0;
			pfc_priv_num--;
		}

		if (hclge_is_rx_buf_ok(hdev, rx_all) ||
		    pfc_priv_num == 0)
			break;
	}
	if (hclge_is_rx_buf_ok(hdev, rx_all))
		return 0;

	return -ENOMEM;
}

static int hclge_rx_priv_buf_alloc(struct hclge_dev *hdev)
{
	struct hclge_rx_priv_buff *req;
	struct hclge_desc desc;
	int ret;
	int i;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_RX_PRIV_BUFF_ALLOC, false);
	req = (struct hclge_rx_priv_buff *)desc.data;

	/* Alloc private buffer TCs */
	for (i = 0; i < HCLGE_MAX_TC_NUM; i++) {
		struct hclge_priv_buf *priv = &hdev->priv_buf[i];

		req->buf_num[i] =
			cpu_to_le16(priv->buf_size >> HCLGE_BUF_UNIT_S);
		req->buf_num[i] |=
			cpu_to_le16(true << HCLGE_TC0_PRI_BUF_EN_B);
	}

	req->shared_buf =
		cpu_to_le16((hdev->s_buf.buf_size >> HCLGE_BUF_UNIT_S) |
			    (1 << HCLGE_TC0_PRI_BUF_EN_B));

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"rx private buffer alloc cmd failed %d\n", ret);
		return ret;
	}

	return 0;
}

#define HCLGE_PRIV_ENABLE(a) ((a) > 0 ? 1 : 0)

static int hclge_rx_priv_wl_config(struct hclge_dev *hdev)
{
	struct hclge_rx_priv_wl_buf *req;
	struct hclge_priv_buf *priv;
	struct hclge_desc desc[2];
	int i, j;
	int ret;

	for (i = 0; i < 2; i++) {
		hclge_cmd_setup_basic_desc(&desc[i], HCLGE_OPC_RX_PRIV_WL_ALLOC,
					   false);
		req = (struct hclge_rx_priv_wl_buf *)desc[i].data;

		/* The first descriptor set the NEXT bit to 1 */
		if (i == 0)
			desc[i].flag |= cpu_to_le16(HCLGE_CMD_FLAG_NEXT);
		else
			desc[i].flag &= ~cpu_to_le16(HCLGE_CMD_FLAG_NEXT);

		for (j = 0; j < HCLGE_TC_NUM_ONE_DESC; j++) {
			priv = &hdev->priv_buf[i * HCLGE_TC_NUM_ONE_DESC + j];
			req->tc_wl[j].high =
				cpu_to_le16(priv->wl.high >> HCLGE_BUF_UNIT_S);
			req->tc_wl[j].high |=
				cpu_to_le16(HCLGE_PRIV_ENABLE(priv->wl.high) <<
					    HCLGE_RX_PRIV_EN_B);
			req->tc_wl[j].low =
				cpu_to_le16(priv->wl.low >> HCLGE_BUF_UNIT_S);
			req->tc_wl[j].low |=
				cpu_to_le16(HCLGE_PRIV_ENABLE(priv->wl.low) <<
					    HCLGE_RX_PRIV_EN_B);
		}
	}

	/* Send 2 descriptor at one time */
	ret = hclge_cmd_send(&hdev->hw, desc, 2);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"rx private waterline config cmd failed %d\n",
			ret);
		return ret;
	}
	return 0;
}

static int hclge_common_thrd_config(struct hclge_dev *hdev)
{
	struct hclge_shared_buf *s_buf = &hdev->s_buf;
	struct hclge_rx_com_thrd *req;
	struct hclge_desc desc[2];
	struct hclge_tc_thrd *tc;
	int i, j;
	int ret;

	for (i = 0; i < 2; i++) {
		hclge_cmd_setup_basic_desc(&desc[i],
					   HCLGE_OPC_RX_COM_THRD_ALLOC, false);
		req = (struct hclge_rx_com_thrd *)&desc[i].data;

		/* The first descriptor set the NEXT bit to 1 */
		if (i == 0)
			desc[i].flag |= cpu_to_le16(HCLGE_CMD_FLAG_NEXT);
		else
			desc[i].flag &= ~cpu_to_le16(HCLGE_CMD_FLAG_NEXT);

		for (j = 0; j < HCLGE_TC_NUM_ONE_DESC; j++) {
			tc = &s_buf->tc_thrd[i * HCLGE_TC_NUM_ONE_DESC + j];

			req->com_thrd[j].high =
				cpu_to_le16(tc->high >> HCLGE_BUF_UNIT_S);
			req->com_thrd[j].high |=
				cpu_to_le16(HCLGE_PRIV_ENABLE(tc->high) <<
					    HCLGE_RX_PRIV_EN_B);
			req->com_thrd[j].low =
				cpu_to_le16(tc->low >> HCLGE_BUF_UNIT_S);
			req->com_thrd[j].low |=
				cpu_to_le16(HCLGE_PRIV_ENABLE(tc->low) <<
					    HCLGE_RX_PRIV_EN_B);
		}
	}

	/* Send 2 descriptors at one time */
	ret = hclge_cmd_send(&hdev->hw, desc, 2);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"common threshold config cmd failed %d\n", ret);
		return ret;
	}
	return 0;
}

static int hclge_common_wl_config(struct hclge_dev *hdev)
{
	struct hclge_shared_buf *buf = &hdev->s_buf;
	struct hclge_rx_com_wl *req;
	struct hclge_desc desc;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_RX_COM_WL_ALLOC, false);

	req = (struct hclge_rx_com_wl *)desc.data;
	req->com_wl.high = cpu_to_le16(buf->self.high >> HCLGE_BUF_UNIT_S);
	req->com_wl.high |=
		cpu_to_le16(HCLGE_PRIV_ENABLE(buf->self.high) <<
			    HCLGE_RX_PRIV_EN_B);

	req->com_wl.low = cpu_to_le16(buf->self.low >> HCLGE_BUF_UNIT_S);
	req->com_wl.low |=
		cpu_to_le16(HCLGE_PRIV_ENABLE(buf->self.low) <<
			    HCLGE_RX_PRIV_EN_B);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"common waterline config cmd failed %d\n", ret);
		return ret;
	}

	return 0;
}

int hclge_buffer_alloc(struct hclge_dev *hdev)
{
	u32 tx_buf_size = HCLGE_DEFAULT_TX_BUF;
	int ret;

	hdev->priv_buf = devm_kmalloc_array(&hdev->pdev->dev, HCLGE_MAX_TC_NUM,
					    sizeof(struct hclge_priv_buf),
					    GFP_KERNEL | __GFP_ZERO);
	if (!hdev->priv_buf)
		return -ENOMEM;

	ret = hclge_tx_buffer_alloc(hdev, tx_buf_size);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"could not alloc tx buffers %d\n", ret);
		return ret;
	}

	ret = hclge_rx_buffer_calc(hdev, tx_buf_size);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"could not calc rx priv buffer size for all TCs %d\n",
			ret);
		return ret;
	}

	ret = hclge_rx_priv_buf_alloc(hdev);
	if (ret) {
		dev_err(&hdev->pdev->dev, "could not alloc rx priv buffer %d\n",
			ret);
		return ret;
	}

	if (hnae3_dev_dcb_supported(hdev)) {
		ret = hclge_rx_priv_wl_config(hdev);
		if (ret) {
			dev_err(&hdev->pdev->dev,
				"could not configure rx private waterline %d\n",
				ret);
			return ret;
		}

		ret = hclge_common_thrd_config(hdev);
		if (ret) {
			dev_err(&hdev->pdev->dev,
				"could not configure common threshold %d\n",
				ret);
			return ret;
		}
	}

	ret = hclge_common_wl_config(hdev);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"could not configure common waterline %d\n", ret);
		return ret;
	}

	return 0;
}

static int hclge_init_roce_base_info(struct hclge_vport *vport)
{
	struct hnae3_handle *roce = &vport->roce;
	struct hnae3_handle *nic = &vport->nic;

	roce->rinfo.num_vectors = vport->back->num_roce_msix;

	if (vport->back->num_msi_left < vport->roce.rinfo.num_vectors ||
	    vport->back->num_msi_left == 0)
		return -EINVAL;

	roce->rinfo.base_vector = vport->back->roce_base_vector;

	roce->rinfo.netdev = nic->kinfo.netdev;
	roce->rinfo.roce_io_base = vport->back->hw.io_base;

	roce->pdev = nic->pdev;
	roce->ae_algo = nic->ae_algo;
	roce->numa_node_mask = nic->numa_node_mask;

	return 0;
}

static int hclge_init_msix(struct hclge_dev *hdev)
{
	struct pci_dev *pdev = hdev->pdev;
	int ret, i;

	hdev->msix_entries = devm_kcalloc(&pdev->dev, hdev->num_msi,
					  sizeof(struct msix_entry),
					  GFP_KERNEL);
	if (!hdev->msix_entries)
		return -ENOMEM;

	hdev->vector_status = devm_kcalloc(&pdev->dev, hdev->num_msi,
					   sizeof(u16), GFP_KERNEL);
	if (!hdev->vector_status)
		return -ENOMEM;

	for (i = 0; i < hdev->num_msi; i++) {
		hdev->msix_entries[i].entry = i;
		hdev->vector_status[i] = HCLGE_INVALID_VPORT;
	}

	hdev->num_msi_left = hdev->num_msi;
	hdev->base_msi_vector = hdev->pdev->irq;
	hdev->roce_base_vector = hdev->base_msi_vector +
				HCLGE_ROCE_VECTOR_OFFSET;

	ret = pci_enable_msix_range(hdev->pdev, hdev->msix_entries,
				    hdev->num_msi, hdev->num_msi);
	if (ret < 0) {
		dev_info(&hdev->pdev->dev,
			 "MSI-X vector alloc failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int hclge_init_msi(struct hclge_dev *hdev)
{
	struct pci_dev *pdev = hdev->pdev;
	int vectors;
	int i;

	hdev->vector_status = devm_kcalloc(&pdev->dev, hdev->num_msi,
					   sizeof(u16), GFP_KERNEL);
	if (!hdev->vector_status)
		return -ENOMEM;

	for (i = 0; i < hdev->num_msi; i++)
		hdev->vector_status[i] = HCLGE_INVALID_VPORT;

	vectors = pci_alloc_irq_vectors(pdev, 1, hdev->num_msi, PCI_IRQ_MSI);
	if (vectors < 0) {
		dev_err(&pdev->dev, "MSI vectors enable failed %d\n", vectors);
		return -EINVAL;
	}
	hdev->num_msi = vectors;
	hdev->num_msi_left = vectors;
	hdev->base_msi_vector = pdev->irq;
	hdev->roce_base_vector = hdev->base_msi_vector +
				HCLGE_ROCE_VECTOR_OFFSET;

	return 0;
}

static void hclge_check_speed_dup(struct hclge_dev *hdev, int duplex, int speed)
{
	struct hclge_mac *mac = &hdev->hw.mac;

	if ((speed == HCLGE_MAC_SPEED_10M) || (speed == HCLGE_MAC_SPEED_100M))
		mac->duplex = (u8)duplex;
	else
		mac->duplex = HCLGE_MAC_FULL;

	mac->speed = speed;
}

int hclge_cfg_mac_speed_dup(struct hclge_dev *hdev, int speed, u8 duplex)
{
	struct hclge_config_mac_speed_dup *req;
	struct hclge_desc desc;
	int ret;

	req = (struct hclge_config_mac_speed_dup *)desc.data;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_CONFIG_SPEED_DUP, false);

	hnae_set_bit(req->speed_dup, HCLGE_CFG_DUPLEX_B, !!duplex);

	switch (speed) {
	case HCLGE_MAC_SPEED_10M:
		hnae_set_field(req->speed_dup, HCLGE_CFG_SPEED_M,
			       HCLGE_CFG_SPEED_S, 6);
		break;
	case HCLGE_MAC_SPEED_100M:
		hnae_set_field(req->speed_dup, HCLGE_CFG_SPEED_M,
			       HCLGE_CFG_SPEED_S, 7);
		break;
	case HCLGE_MAC_SPEED_1G:
		hnae_set_field(req->speed_dup, HCLGE_CFG_SPEED_M,
			       HCLGE_CFG_SPEED_S, 0);
		break;
	case HCLGE_MAC_SPEED_10G:
		hnae_set_field(req->speed_dup, HCLGE_CFG_SPEED_M,
			       HCLGE_CFG_SPEED_S, 1);
		break;
	case HCLGE_MAC_SPEED_25G:
		hnae_set_field(req->speed_dup, HCLGE_CFG_SPEED_M,
			       HCLGE_CFG_SPEED_S, 2);
		break;
	case HCLGE_MAC_SPEED_40G:
		hnae_set_field(req->speed_dup, HCLGE_CFG_SPEED_M,
			       HCLGE_CFG_SPEED_S, 3);
		break;
	case HCLGE_MAC_SPEED_50G:
		hnae_set_field(req->speed_dup, HCLGE_CFG_SPEED_M,
			       HCLGE_CFG_SPEED_S, 4);
		break;
	case HCLGE_MAC_SPEED_100G:
		hnae_set_field(req->speed_dup, HCLGE_CFG_SPEED_M,
			       HCLGE_CFG_SPEED_S, 5);
		break;
	default:
		dev_err(&hdev->pdev->dev, "invalid speed (%d)\n", speed);
		return -EINVAL;
	}

	hnae_set_bit(req->mac_change_fec_en, HCLGE_CFG_MAC_SPEED_CHANGE_EN_B,
		     1);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"mac speed/duplex config cmd failed %d.\n", ret);
		return ret;
	}

	hclge_check_speed_dup(hdev, duplex, speed);

	return 0;
}

static int hclge_cfg_mac_speed_dup_h(struct hnae3_handle *handle, int speed,
				     u8 duplex)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	return hclge_cfg_mac_speed_dup(hdev, speed, duplex);
}

static int hclge_query_mac_an_speed_dup(struct hclge_dev *hdev, int *speed,
					u8 *duplex)
{
	struct hclge_query_an_speed_dup *req;
	struct hclge_desc desc;
	int speed_tmp;
	int ret;

	req = (struct hclge_query_an_speed_dup *)desc.data;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_QUERY_AN_RESULT, true);
	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"mac speed/autoneg/duplex query cmd failed %d\n",
			ret);
		return ret;
	}

	*duplex = hnae_get_bit(req->an_syn_dup_speed, HCLGE_QUERY_DUPLEX_B);
	speed_tmp = hnae_get_field(req->an_syn_dup_speed, HCLGE_QUERY_SPEED_M,
				   HCLGE_QUERY_SPEED_S);

	ret = hclge_parse_speed(speed_tmp, speed);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"could not parse speed(=%d), %d\n", speed_tmp, ret);
		return -EIO;
	}

	return 0;
}

static int hclge_query_autoneg_result(struct hclge_dev *hdev)
{
	struct hclge_mac *mac = &hdev->hw.mac;
	struct hclge_query_an_speed_dup *req;
	struct hclge_desc desc;
	int ret;

	req = (struct hclge_query_an_speed_dup *)desc.data;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_QUERY_AN_RESULT, true);
	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"autoneg result query cmd failed %d.\n", ret);
		return ret;
	}

	mac->autoneg = hnae_get_bit(req->an_syn_dup_speed, HCLGE_QUERY_AN_B);

	return 0;
}

static int hclge_set_autoneg_en(struct hclge_dev *hdev, bool enable)
{
	struct hclge_config_auto_neg *req;
	struct hclge_desc desc;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_CONFIG_AN_MODE, false);

	req = (struct hclge_config_auto_neg *)desc.data;
	hnae_set_bit(req->cfg_an_cmd_flag, HCLGE_MAC_CFG_AN_EN_B, !!enable);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev, "auto neg set cmd failed %d.\n",
			ret);
		return ret;
	}

	return 0;
}

static int hclge_set_autoneg(struct hnae3_handle *handle, bool enable)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	return hclge_set_autoneg_en(hdev, enable);
}

static int hclge_get_autoneg(struct hnae3_handle *handle)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	struct phy_device *phydev = hdev->hw.mac.phydev;

	if (phydev)
		return phydev->autoneg;

	hclge_query_autoneg_result(hdev);

	return hdev->hw.mac.autoneg;
}

static int hclge_mac_init(struct hclge_dev *hdev)
{
	struct hclge_mac *mac = &hdev->hw.mac;
	int ret;

	ret = hclge_cfg_mac_speed_dup(hdev, hdev->hw.mac.speed, HCLGE_MAC_FULL);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Config mac speed dup fail ret=%d\n", ret);
		return ret;
	}

	mac->link = 0;

	ret = hclge_mac_mdio_config(hdev);
	if (ret) {
		dev_warn(&hdev->pdev->dev,
			 "mdio config fail ret=%d\n", ret);
		return ret;
	}

	/* Initialize the MTA table work mode */
	hdev->accept_mta_mc	= true;
	hdev->enable_mta	= true;
	hdev->mta_mac_sel_type	= HCLGE_MAC_ADDR_47_36;

	ret = hclge_set_mta_filter_mode(hdev,
					hdev->mta_mac_sel_type,
					hdev->enable_mta);
	if (ret) {
		dev_err(&hdev->pdev->dev, "set mta filter mode failed %d\n",
			ret);
		return ret;
	}

	return hclge_cfg_func_mta_filter(hdev, 0, hdev->accept_mta_mc);
}

static void hclge_task_schedule(struct hclge_dev *hdev)
{
	if (!test_bit(HCLGE_STATE_DOWN, &hdev->state) &&
	    !test_bit(HCLGE_STATE_REMOVING, &hdev->state) &&
	    !test_and_set_bit(HCLGE_STATE_SERVICE_SCHED, &hdev->state))
		(void)schedule_work(&hdev->service_task);
}

static int hclge_get_mac_link_status(struct hclge_dev *hdev)
{
	struct hclge_link_status *req;
	struct hclge_desc desc;
	int link_status;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_QUERY_LINK_STATUS, true);
	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev, "get link status cmd failed %d\n",
			ret);
		return ret;
	}

	req = (struct hclge_link_status *)desc.data;
	link_status = req->status & HCLGE_LINK_STATUS;

	return !!link_status;
}

static int hclge_get_mac_phy_link(struct hclge_dev *hdev)
{
	int mac_state;
	int link_stat;

	mac_state = hclge_get_mac_link_status(hdev);

	if (hdev->hw.mac.phydev) {
		if (hdev->hw.mac.phydev->state == PHY_RUNNING)
			link_stat = mac_state &
				hdev->hw.mac.phydev->link;
		else
			link_stat = 0;

	} else {
		link_stat = mac_state;
	}

	return !!link_stat;
}

static void hclge_update_link_status(struct hclge_dev *hdev)
{
	struct hnae3_client *client = hdev->nic_client;
	struct hnae3_handle *handle;
	int state;
	int i;

	if (!client)
		return;
	state = hclge_get_mac_phy_link(hdev);
	if (state != hdev->hw.mac.link) {
		for (i = 0; i < hdev->num_vmdq_vport + 1; i++) {
			handle = &hdev->vport[i].nic;
			client->ops->link_status_change(handle, state);
		}
		hdev->hw.mac.link = state;
	}
}

static int hclge_update_speed_duplex(struct hclge_dev *hdev)
{
	struct hclge_mac mac = hdev->hw.mac;
	u8 duplex;
	int speed;
	int ret;

	/* get the speed and duplex as autoneg'result from mac cmd when phy
	 * doesn't exit.
	 */
	if (mac.phydev)
		return 0;

	/* update mac->antoneg. */
	ret = hclge_query_autoneg_result(hdev);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"autoneg result query failed %d\n", ret);
		return ret;
	}

	if (!mac.autoneg)
		return 0;

	ret = hclge_query_mac_an_speed_dup(hdev, &speed, &duplex);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"mac autoneg/speed/duplex query failed %d\n", ret);
		return ret;
	}

	if ((mac.speed != speed) || (mac.duplex != duplex)) {
		ret = hclge_cfg_mac_speed_dup(hdev, speed, duplex);
		if (ret) {
			dev_err(&hdev->pdev->dev,
				"mac speed/duplex config failed %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int hclge_update_speed_duplex_h(struct hnae3_handle *handle)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	return hclge_update_speed_duplex(hdev);
}

static int hclge_get_status(struct hnae3_handle *handle)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	hclge_update_link_status(hdev);

	return hdev->hw.mac.link;
}

static void hclge_service_timer(unsigned long data)
{
	struct hclge_dev *hdev = (struct hclge_dev *)data;
	(void)mod_timer(&hdev->service_timer, jiffies + HZ);

	hclge_task_schedule(hdev);
}

static void hclge_service_complete(struct hclge_dev *hdev)
{
	WARN_ON(!test_bit(HCLGE_STATE_SERVICE_SCHED, &hdev->state));

	/* Flush memory before next watchdog */
	smp_mb__before_atomic();
	clear_bit(HCLGE_STATE_SERVICE_SCHED, &hdev->state);
}

static void hclge_service_task(struct work_struct *work)
{
	struct hclge_dev *hdev =
		container_of(work, struct hclge_dev, service_task);

	hclge_update_speed_duplex(hdev);
	hclge_update_link_status(hdev);
	hclge_update_stats_for_all(hdev);
	hclge_service_complete(hdev);
}

static void hclge_disable_sriov(struct hclge_dev *hdev)
{
	/* If our VFs are assigned we cannot shut down SR-IOV
	 * without causing issues, so just leave the hardware
	 * available but disabled
	 */
	if (pci_vfs_assigned(hdev->pdev)) {
		dev_warn(&hdev->pdev->dev,
			 "disabling driver while VFs are assigned\n");
		return;
	}

	pci_disable_sriov(hdev->pdev);
}

struct hclge_vport *hclge_get_vport(struct hnae3_handle *handle)
{
	/* VF handle has no client */
	if (!handle->client)
		return container_of(handle, struct hclge_vport, nic);
	else if (handle->client->type == HNAE3_CLIENT_ROCE)
		return container_of(handle, struct hclge_vport, roce);
	else
		return container_of(handle, struct hclge_vport, nic);
}

static int hclge_get_vector(struct hnae3_handle *handle, u16 vector_num,
			    struct hnae3_vector_info *vector_info)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hnae3_vector_info *vector = vector_info;
	struct hclge_dev *hdev = vport->back;
	int alloc = 0;
	int i, j;

	vector_num = min(hdev->num_msi_left, vector_num);

	for (j = 0; j < vector_num; j++) {
		for (i = 1; i < hdev->num_msi; i++) {
			if (hdev->vector_status[i] == HCLGE_INVALID_VPORT) {
				vector->vector = pci_irq_vector(hdev->pdev, i);
				vector->io_addr = hdev->hw.io_base +
					HCLGE_VECTOR_REG_BASE +
					(i - 1) * HCLGE_VECTOR_REG_OFFSET +
					vport->vport_id *
					HCLGE_VECTOR_VF_OFFSET;
				hdev->vector_status[i] = vport->vport_id;

				vector++;
				alloc++;

				break;
			}
		}
	}
	hdev->num_msi_left -= alloc;
	hdev->num_msi_used += alloc;

	return alloc;
}

static int hclge_get_vector_index(struct hclge_dev *hdev, int vector)
{
	int i;

	for (i = 0; i < hdev->num_msi; i++) {
		if (hdev->msix_entries) {
			if (vector == hdev->msix_entries[i].vector)
				return i;
		} else {
			if (vector == (hdev->base_msi_vector + i))
				return i;
		}
	}
	return -EINVAL;
}

static u32 hclge_get_rss_key_size(struct hnae3_handle *handle)
{
	return HCLGE_RSS_KEY_SIZE;
}

static u32 hclge_get_rss_indir_size(struct hnae3_handle *handle)
{
	return HCLGE_RSS_IND_TBL_SIZE;
}

static int hclge_get_rss_algo(struct hclge_dev *hdev)
{
	struct hclge_rss_config *req;
	struct hclge_desc desc;
	int rss_hash_algo;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_RSS_GENERIC_CONFIG, true);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Get link status error, status =%d\n", ret);
		return ret;
	}

	req = (struct hclge_rss_config *)desc.data;
	rss_hash_algo = (req->hash_config & HCLGE_RSS_HASH_ALGO_MASK);

	if (rss_hash_algo == HCLGE_RSS_HASH_ALGO_TOEPLITZ)
		return ETH_RSS_HASH_TOP;

	return -EINVAL;
}

static int hclge_set_rss_algo_key(struct hclge_dev *hdev,
				  const u8 hfunc, const u8 *key)
{
	struct hclge_rss_config *req;
	struct hclge_desc desc;
	int key_offset;
	int key_size;
	int ret;

	req = (struct hclge_rss_config *)desc.data;

	for (key_offset = 0; key_offset < 3; key_offset++) {
		hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_RSS_GENERIC_CONFIG,
					   false);

		req->hash_config |= (hfunc & HCLGE_RSS_HASH_ALGO_MASK);
		req->hash_config |= (key_offset << HCLGE_RSS_HASH_KEY_OFFSET_B);

		if (key_offset == 2)
			key_size =
			HCLGE_RSS_KEY_SIZE - HCLGE_RSS_HASH_KEY_NUM * 2;
		else
			key_size = HCLGE_RSS_HASH_KEY_NUM;

		memcpy(req->hash_key,
		       key + key_offset * HCLGE_RSS_HASH_KEY_NUM, key_size);

		ret = hclge_cmd_send(&hdev->hw, &desc, 1);
		if (ret) {
			dev_err(&hdev->pdev->dev,
				"Configure RSS config fail, status = %d\n",
				ret);
			return ret;
		}
	}
	return 0;
}

static int hclge_set_rss_indir_table(struct hclge_dev *hdev, const u32 *indir)
{
	struct hclge_rss_indirection_table *req;
	struct hclge_desc desc;
	int i, j;
	int ret;

	req = (struct hclge_rss_indirection_table *)desc.data;

	for (i = 0; i < HCLGE_RSS_CFG_TBL_NUM; i++) {
		hclge_cmd_setup_basic_desc
			(&desc, HCLGE_OPC_RSS_INDIR_TABLE, false);

		req->start_table_index = i * HCLGE_RSS_CFG_TBL_SIZE;
		req->rss_set_bitmap = HCLGE_RSS_SET_BITMAP_MSK;

		for (j = 0; j < HCLGE_RSS_CFG_TBL_SIZE; j++)
			req->rss_result[j] =
				indir[i * HCLGE_RSS_CFG_TBL_SIZE + j];

		ret = hclge_cmd_send(&hdev->hw, &desc, 1);
		if (ret) {
			dev_err(&hdev->pdev->dev,
				"Configure rss indir table fail,status = %d\n",
				ret);
			return ret;
		}
	}
	return 0;
}

static int hclge_set_rss_tc_mode(struct hclge_dev *hdev, u16 *tc_valid,
				 u16 *tc_size, u16 *tc_offset)
{
	struct hclge_rss_tc_mode *req;
	struct hclge_desc desc;
	int ret;
	int i;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_RSS_TC_MODE, false);
	req = (struct hclge_rss_tc_mode *)desc.data;

	for (i = 0; i < HCLGE_MAX_TC_NUM; i++) {
		hnae_set_bit(req->rss_tc_mode[i], HCLGE_RSS_TC_VALID_B,
			     (tc_valid[i] & 0x1));
		hnae_set_field(req->rss_tc_mode[i], HCLGE_RSS_TC_SIZE_M,
			       HCLGE_RSS_TC_SIZE_S, tc_size[i]);
		hnae_set_field(req->rss_tc_mode[i], HCLGE_RSS_TC_OFFSET_M,
			       HCLGE_RSS_TC_OFFSET_S, tc_offset[i]);
	}

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Configure rss tc mode fail, status = %d\n", ret);
		return ret;
	}

	return 0;
}

static int hclge_set_rss_input_tuple(struct hclge_dev *hdev)
{
#define HCLGE_RSS_INPUT_TUPLE_OTHER		0xf
#define HCLGE_RSS_INPUT_TUPLE_SCTP		0x1f
	struct hclge_rss_input_tuple *req;
	struct hclge_desc desc;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_RSS_INPUT_TUPLE, false);

	req = (struct hclge_rss_input_tuple *)desc.data;
	req->ipv4_tcp_en = HCLGE_RSS_INPUT_TUPLE_OTHER;
	req->ipv4_udp_en = HCLGE_RSS_INPUT_TUPLE_OTHER;
	req->ipv4_sctp_en = HCLGE_RSS_INPUT_TUPLE_SCTP;
	req->ipv4_fragment_en = HCLGE_RSS_INPUT_TUPLE_OTHER;
	req->ipv6_tcp_en = HCLGE_RSS_INPUT_TUPLE_OTHER;
	req->ipv6_udp_en = HCLGE_RSS_INPUT_TUPLE_OTHER;
	req->ipv6_sctp_en = HCLGE_RSS_INPUT_TUPLE_SCTP;
	req->ipv6_fragment_en = HCLGE_RSS_INPUT_TUPLE_OTHER;
	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Configure rss input fail, status = %d\n", ret);
		return ret;
	}

	return 0;
}

static int hclge_get_rss(struct hnae3_handle *handle, u32 *indir,
			 u8 *key, u8 *hfunc)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	int i;

	/* Get hash algorithm */
	if (hfunc)
		*hfunc = hclge_get_rss_algo(hdev);

	/* Get the RSS Key required by the user */
	if (key)
		memcpy(key, vport->rss_hash_key, HCLGE_RSS_KEY_SIZE);

	/* Get indirect table */
	if (indir)
		for (i = 0; i < HCLGE_RSS_IND_TBL_SIZE; i++)
			indir[i] =  vport->rss_indirection_tbl[i];

	return 0;
}

static int hclge_set_rss(struct hnae3_handle *handle, const u32 *indir,
			 const  u8 *key, const  u8 hfunc)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	u8 hash_algo;
	int ret, i;

	/* Set the RSS Hash Key if specififed by the user */
	if (key) {
		/* Update the shadow RSS key with user specified qids */
		memcpy(vport->rss_hash_key, key, HCLGE_RSS_KEY_SIZE);

		if (hfunc == ETH_RSS_HASH_TOP ||
		    hfunc == ETH_RSS_HASH_NO_CHANGE)
			hash_algo = HCLGE_RSS_HASH_ALGO_TOEPLITZ;
		else
			return -EINVAL;
		ret = hclge_set_rss_algo_key(hdev, hash_algo, key);
		if (ret)
			return ret;
	}

	/* Update the shadow RSS table with user specified qids */
	for (i = 0; i < HCLGE_RSS_IND_TBL_SIZE; i++)
		vport->rss_indirection_tbl[i] = indir[i];

	/* Update the hardware */
	ret = hclge_set_rss_indir_table(hdev, indir);
	return ret;
}

static int hclge_get_tc_size(struct hnae3_handle *handle)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	return hdev->rss_size_max;
}

static int hclge_rss_init_hw(struct hclge_dev *hdev)
{
	const  u8 hfunc = HCLGE_RSS_HASH_ALGO_TOEPLITZ;
	struct hclge_vport *vport = hdev->vport;
	u16 tc_offset[HCLGE_MAX_TC_NUM];
	u8 rss_key[HCLGE_RSS_KEY_SIZE];
	u16 tc_valid[HCLGE_MAX_TC_NUM];
	u16 tc_size[HCLGE_MAX_TC_NUM];
	u32 *rss_indir = NULL;
	u16 rss_size = 0, roundup_size;
	const u8 *key;
	int i, ret, j;

	rss_indir = kcalloc(HCLGE_RSS_IND_TBL_SIZE, sizeof(u32), GFP_KERNEL);
	if (!rss_indir)
		return -ENOMEM;

	/* Get default RSS key */
	netdev_rss_key_fill(rss_key, HCLGE_RSS_KEY_SIZE);

	/* Initialize RSS indirect table for each vport */
	for (j = 0; j < hdev->num_vmdq_vport + 1; j++) {
		for (i = 0; i < HCLGE_RSS_IND_TBL_SIZE; i++) {
			vport[j].rss_indirection_tbl[i] =
				i % vport[j].alloc_rss_size;

			/* vport 0 is for PF */
			if (j != 0)
				continue;

			rss_size = vport[j].alloc_rss_size;
			rss_indir[i] = vport[j].rss_indirection_tbl[i];
		}
	}
	ret = hclge_set_rss_indir_table(hdev, rss_indir);
	if (ret)
		goto err;

	key = rss_key;
	ret = hclge_set_rss_algo_key(hdev, hfunc, key);
	if (ret)
		goto err;

	ret = hclge_set_rss_input_tuple(hdev);
	if (ret)
		goto err;

	/* Each TC have the same queue size, and tc_size set to hardware is
	 * the log2 of roundup power of two of rss_size, the acutal queue
	 * size is limited by indirection table.
	 */
	if (rss_size > HCLGE_RSS_TC_SIZE_7 || rss_size == 0) {
		dev_err(&hdev->pdev->dev,
			"Configure rss tc size failed, invalid TC_SIZE = %d\n",
			rss_size);
		ret = -EINVAL;
		goto err;
	}

	roundup_size = roundup_pow_of_two(rss_size);
	roundup_size = ilog2(roundup_size);

	for (i = 0; i < HCLGE_MAX_TC_NUM; i++) {
		tc_valid[i] = 0;

		if (!(hdev->hw_tc_map & BIT(i)))
			continue;

		tc_valid[i] = 1;
		tc_size[i] = roundup_size;
		tc_offset[i] = rss_size * i;
	}

	ret = hclge_set_rss_tc_mode(hdev, tc_valid, tc_size, tc_offset);

err:
	kfree(rss_indir);

	return ret;
}

int hclge_map_vport_ring_to_vector(struct hclge_vport *vport, int vector_id,
				   struct hnae3_ring_chain_node *ring_chain)
{
	struct hclge_dev *hdev = vport->back;
	struct hclge_ctrl_vector_chain *req;
	struct hnae3_ring_chain_node *node;
	struct hclge_desc desc;
	int ret;
	int i;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_ADD_RING_TO_VECTOR, false);

	req = (struct hclge_ctrl_vector_chain *)desc.data;
	req->int_vector_id = vector_id;

	i = 0;
	for (node = ring_chain; node; node = node->next) {
		hnae_set_field(req->tqp_type_and_id[i], HCLGE_INT_TYPE_M,
			       HCLGE_INT_TYPE_S,
			       hnae_get_bit(node->flag, HNAE3_RING_TYPE_B));
		hnae_set_field(req->tqp_type_and_id[i], HCLGE_TQP_ID_M,
			       HCLGE_TQP_ID_S,	node->tqp_index);
		hnae_set_field(req->tqp_type_and_id[i], HCLGE_INT_GL_IDX_M,
			       HCLGE_INT_GL_IDX_S,
			       hnae_get_bit(node->flag, HNAE3_RING_TYPE_B));
		req->tqp_type_and_id[i] = cpu_to_le16(req->tqp_type_and_id[i]);
		req->vfid = vport->vport_id;

		if (++i >= HCLGE_VECTOR_ELEMENTS_PER_CMD) {
			req->int_cause_num = HCLGE_VECTOR_ELEMENTS_PER_CMD;

			ret = hclge_cmd_send(&hdev->hw, &desc, 1);
			if (ret) {
				dev_err(&hdev->pdev->dev,
					"Map TQP fail, status is %d.\n",
					ret);
				return ret;
			}
			i = 0;

			hclge_cmd_setup_basic_desc(&desc,
						   HCLGE_OPC_ADD_RING_TO_VECTOR,
						   false);
			req->int_vector_id = vector_id;
		}
	}

	if (i > 0) {
		req->int_cause_num = i;

		ret = hclge_cmd_send(&hdev->hw, &desc, 1);
		if (ret) {
			dev_err(&hdev->pdev->dev,
				"Map TQP fail, status is %d.\n", ret);
			return ret;
		}
	}

	return 0;
}

int hclge_map_handle_ring_to_vector(struct hnae3_handle *handle,
				    int vector,
				    struct hnae3_ring_chain_node *ring_chain)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	int vector_id;

	vector_id = hclge_get_vector_index(hdev, vector);
	if (vector_id < 0) {
		dev_err(&hdev->pdev->dev,
			"Get vector index fail. ret =%d\n", vector_id);
		return vector_id;
	}

	return hclge_map_vport_ring_to_vector(vport, vector_id, ring_chain);
}

static int hclge_unmap_ring_from_vector(
	struct hnae3_handle *handle, int vector,
	struct hnae3_ring_chain_node *ring_chain)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	struct hclge_ctrl_vector_chain *req;
	struct hnae3_ring_chain_node *node;
	struct hclge_desc desc;
	int i, vector_id;
	int ret;

	vector_id = hclge_get_vector_index(hdev, vector);
	if (vector_id < 0) {
		dev_err(&handle->pdev->dev,
			"Get vector index fail. ret =%d\n", vector_id);
		return vector_id;
	}

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_DEL_RING_TO_VECTOR, false);

	req = (struct hclge_ctrl_vector_chain *)desc.data;
	req->int_vector_id = vector_id;

	i = 0;
	for (node = ring_chain; node; node = node->next) {
		hnae_set_field(req->tqp_type_and_id[i], HCLGE_INT_TYPE_M,
			       HCLGE_INT_TYPE_S,
			       hnae_get_bit(node->flag, HNAE3_RING_TYPE_B));
		hnae_set_field(req->tqp_type_and_id[i], HCLGE_TQP_ID_M,
			       HCLGE_TQP_ID_S,	node->tqp_index);
		hnae_set_field(req->tqp_type_and_id[i], HCLGE_INT_GL_IDX_M,
			       HCLGE_INT_GL_IDX_S,
			       hnae_get_bit(node->flag, HNAE3_RING_TYPE_B));

		req->tqp_type_and_id[i] = cpu_to_le16(req->tqp_type_and_id[i]);
		req->vfid = vport->vport_id;

		if (++i >= HCLGE_VECTOR_ELEMENTS_PER_CMD) {
			req->int_cause_num = HCLGE_VECTOR_ELEMENTS_PER_CMD;

			ret = hclge_cmd_send(&hdev->hw, &desc, 1);
			if (ret) {
				dev_err(&hdev->pdev->dev,
					"Unmap TQP fail, status is %d.\n",
					ret);
				return ret;
			}
			i = 0;
			hclge_cmd_setup_basic_desc(&desc,
						   HCLGE_OPC_DEL_RING_TO_VECTOR,
						   false);
			req->int_vector_id = vector_id;
		}
	}

	if (i > 0) {
		req->int_cause_num = i;

		ret = hclge_cmd_send(&hdev->hw, &desc, 1);
		if (ret) {
			dev_err(&hdev->pdev->dev,
				"Unmap TQP fail, status is %d.\n", ret);
			return ret;
		}
	}

	return 0;
}

int hclge_cmd_set_promisc_mode(struct hclge_dev *hdev,
			       struct hclge_promisc_param *param)
{
	struct hclge_promisc_cfg *req;
	struct hclge_desc desc;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_CFG_PROMISC_MODE, false);

	req = (struct hclge_promisc_cfg *)desc.data;
	req->vf_id = param->vf_id;
	req->flag = (param->enable << HCLGE_PROMISC_EN_B);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Set promisc mode fail, status is %d.\n", ret);
		return ret;
	}
	return 0;
}

void hclge_promisc_param_init(struct hclge_promisc_param *param, bool en_uc,
			      bool en_mc, bool en_bc, int vport_id)
{
	if (!param)
		return;

	memset(param, 0, sizeof(struct hclge_promisc_param));
	if (en_uc)
		param->enable = HCLGE_PROMISC_EN_UC;
	if (en_mc)
		param->enable |= HCLGE_PROMISC_EN_MC;
	if (en_bc)
		param->enable |= HCLGE_PROMISC_EN_BC;
	param->vf_id = vport_id;
}

static void hclge_set_promisc_mode(struct hnae3_handle *handle, u32 en)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	struct hclge_promisc_param param;

	hclge_promisc_param_init(&param, en, en, true, vport->vport_id);
	hclge_cmd_set_promisc_mode(hdev, &param);
}

static void hclge_cfg_mac_mode(struct hclge_dev *hdev, bool enable)
{
	struct hclge_desc desc;
	struct hclge_config_mac_mode *req =
		(struct hclge_config_mac_mode *)desc.data;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_CONFIG_MAC_MODE, false);
	hnae_set_bit(req->txrx_pad_fcs_loop_en, HCLGE_MAC_TX_EN_B, enable);
	hnae_set_bit(req->txrx_pad_fcs_loop_en, HCLGE_MAC_RX_EN_B, enable);
	hnae_set_bit(req->txrx_pad_fcs_loop_en, HCLGE_MAC_PAD_TX_B, enable);
	hnae_set_bit(req->txrx_pad_fcs_loop_en, HCLGE_MAC_PAD_RX_B, enable);
	hnae_set_bit(req->txrx_pad_fcs_loop_en, HCLGE_MAC_1588_TX_B, 0);
	hnae_set_bit(req->txrx_pad_fcs_loop_en, HCLGE_MAC_1588_RX_B, 0);
	hnae_set_bit(req->txrx_pad_fcs_loop_en, HCLGE_MAC_APP_LP_B, 0);
	hnae_set_bit(req->txrx_pad_fcs_loop_en, HCLGE_MAC_LINE_LP_B, 0);
	hnae_set_bit(req->txrx_pad_fcs_loop_en, HCLGE_MAC_FCS_TX_B, enable);
	hnae_set_bit(req->txrx_pad_fcs_loop_en, HCLGE_MAC_RX_FCS_B, enable);
	hnae_set_bit(req->txrx_pad_fcs_loop_en,
		     HCLGE_MAC_RX_FCS_STRIP_B, enable);
	hnae_set_bit(req->txrx_pad_fcs_loop_en,
		     HCLGE_MAC_TX_OVERSIZE_TRUNCATE_B, enable);
	hnae_set_bit(req->txrx_pad_fcs_loop_en,
		     HCLGE_MAC_RX_OVERSIZE_TRUNCATE_B, enable);
	hnae_set_bit(req->txrx_pad_fcs_loop_en,
		     HCLGE_MAC_TX_UNDER_MIN_ERR_B, enable);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret)
		dev_err(&hdev->pdev->dev,
			"mac enable fail, ret =%d.\n", ret);
}

static int hclge_tqp_enable(struct hclge_dev *hdev, int tqp_id,
			    int stream_id, bool enable)
{
	struct hclge_desc desc;
	struct hclge_cfg_com_tqp_queue *req =
		(struct hclge_cfg_com_tqp_queue *)desc.data;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_CFG_COM_TQP_QUEUE, false);
	req->tqp_id = cpu_to_le16(tqp_id & HCLGE_RING_ID_MASK);
	req->stream_id = cpu_to_le16(stream_id);
	req->enable |= enable << HCLGE_TQP_ENABLE_B;

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret)
		dev_err(&hdev->pdev->dev,
			"Tqp enable fail, status =%d.\n", ret);
	return ret;
}

static void hclge_reset_tqp_stats(struct hnae3_handle *handle)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hnae3_queue *queue;
	struct hclge_tqp *tqp;
	int i;

	for (i = 0; i < vport->alloc_tqps; i++) {
		queue = handle->kinfo.tqp[i];
		tqp = container_of(queue, struct hclge_tqp, q);
		memset(&tqp->tqp_stats, 0, sizeof(tqp->tqp_stats));
	}
}

static int hclge_ae_start(struct hnae3_handle *handle)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	int i, queue_id, ret;

	for (i = 0; i < vport->alloc_tqps; i++) {
		/* todo clear interrupt */
		/* ring enable */
		queue_id = hclge_get_queue_id(handle->kinfo.tqp[i]);
		if (queue_id < 0) {
			dev_warn(&hdev->pdev->dev,
				 "Get invalid queue id, ignore it\n");
			continue;
		}

		hclge_tqp_enable(hdev, queue_id, 0, true);
	}
	/* mac enable */
	hclge_cfg_mac_mode(hdev, true);
	clear_bit(HCLGE_STATE_DOWN, &hdev->state);
	(void)mod_timer(&hdev->service_timer, jiffies + HZ);

	ret = hclge_mac_start_phy(hdev);
	if (ret)
		return ret;

	/* reset tqp stats */
	hclge_reset_tqp_stats(handle);

	return 0;
}

static void hclge_ae_stop(struct hnae3_handle *handle)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	int i, queue_id;

	for (i = 0; i < vport->alloc_tqps; i++) {
		/* Ring disable */
		queue_id = hclge_get_queue_id(handle->kinfo.tqp[i]);
		if (queue_id < 0) {
			dev_warn(&hdev->pdev->dev,
				 "Get invalid queue id, ignore it\n");
			continue;
		}

		hclge_tqp_enable(hdev, queue_id, 0, false);
	}
	/* Mac disable */
	hclge_cfg_mac_mode(hdev, false);

	hclge_mac_stop_phy(hdev);

	/* reset tqp stats */
	hclge_reset_tqp_stats(handle);
}

static int hclge_get_mac_vlan_cmd_status(struct hclge_vport *vport,
					 u16 cmdq_resp, u8  resp_code,
					 enum hclge_mac_vlan_tbl_opcode op)
{
	struct hclge_dev *hdev = vport->back;
	int return_status = -EIO;

	if (cmdq_resp) {
		dev_err(&hdev->pdev->dev,
			"cmdq execute failed for get_mac_vlan_cmd_status,status=%d.\n",
			cmdq_resp);
		return -EIO;
	}

	if (op == HCLGE_MAC_VLAN_ADD) {
		if ((!resp_code) || (resp_code == 1)) {
			return_status = 0;
		} else if (resp_code == 2) {
			return_status = -EIO;
			dev_err(&hdev->pdev->dev,
				"add mac addr failed for uc_overflow.\n");
		} else if (resp_code == 3) {
			return_status = -EIO;
			dev_err(&hdev->pdev->dev,
				"add mac addr failed for mc_overflow.\n");
		} else {
			dev_err(&hdev->pdev->dev,
				"add mac addr failed for undefined, code=%d.\n",
				resp_code);
		}
	} else if (op == HCLGE_MAC_VLAN_REMOVE) {
		if (!resp_code) {
			return_status = 0;
		} else if (resp_code == 1) {
			return_status = -EIO;
			dev_dbg(&hdev->pdev->dev,
				"remove mac addr failed for miss.\n");
		} else {
			dev_err(&hdev->pdev->dev,
				"remove mac addr failed for undefined, code=%d.\n",
				resp_code);
		}
	} else if (op == HCLGE_MAC_VLAN_LKUP) {
		if (!resp_code) {
			return_status = 0;
		} else if (resp_code == 1) {
			return_status = -EIO;
			dev_dbg(&hdev->pdev->dev,
				"lookup mac addr failed for miss.\n");
		} else {
			dev_err(&hdev->pdev->dev,
				"lookup mac addr failed for undefined, code=%d.\n",
				resp_code);
		}
	} else {
		return_status = -EIO;
		dev_err(&hdev->pdev->dev,
			"unknown opcode for get_mac_vlan_cmd_status,opcode=%d.\n",
			op);
	}

	return return_status;
}

static int hclge_update_desc_vfid(struct hclge_desc *desc, int vfid, bool clr)
{
	int word_num;
	int bit_num;

	if (vfid > 255 || vfid < 0)
		return -EIO;

	if (vfid >= 0 && vfid <= 191) {
		word_num = vfid / 32;
		bit_num  = vfid % 32;
		if (clr)
			desc[1].data[word_num] &= ~(1 << bit_num);
		else
			desc[1].data[word_num] |= (1 << bit_num);
	} else {
		word_num = (vfid - 192) / 32;
		bit_num  = vfid % 32;
		if (clr)
			desc[2].data[word_num] &= ~(1 << bit_num);
		else
			desc[2].data[word_num] |= (1 << bit_num);
	}

	return 0;
}

static bool hclge_is_all_function_id_zero(struct hclge_desc *desc)
{
#define HCLGE_DESC_NUMBER 3
#define HCLGE_FUNC_NUMBER_PER_DESC 6
	int i, j;

	for (i = 1; i < HCLGE_DESC_NUMBER; i++)
		for (j = 0; j < HCLGE_FUNC_NUMBER_PER_DESC; j++)
			if (desc[i].data[j])
				return false;

	return true;
}

static void hclge_prepare_mac_addr(struct hclge_mac_vlan_tbl_entry *new_req,
				   const u8 *addr)
{
	const unsigned char *mac_addr = addr;
	u32 high_val = mac_addr[2] << 16 | (mac_addr[3] << 24) |
		       (mac_addr[0]) | (mac_addr[1] << 8);
	u32 low_val  = mac_addr[4] | (mac_addr[5] << 8);

	new_req->mac_addr_hi32 = cpu_to_le32(high_val);
	new_req->mac_addr_lo16 = cpu_to_le16(low_val & 0xffff);
}

u16 hclge_get_mac_addr_to_mta_index(struct hclge_vport *vport,
				    const u8 *addr)
{
	u16 high_val = addr[1] | (addr[0] << 8);
	struct hclge_dev *hdev = vport->back;
	u32 rsh = 4 - hdev->mta_mac_sel_type;
	u16 ret_val = (high_val >> rsh) & 0xfff;

	return ret_val;
}

static int hclge_set_mta_filter_mode(struct hclge_dev *hdev,
				     enum hclge_mta_dmac_sel_type mta_mac_sel,
				     bool enable)
{
	struct hclge_mta_filter_mode *req;
	struct hclge_desc desc;
	int ret;

	req = (struct hclge_mta_filter_mode *)desc.data;
	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_MTA_MAC_MODE_CFG, false);

	hnae_set_bit(req->dmac_sel_en, HCLGE_CFG_MTA_MAC_EN_B,
		     enable);
	hnae_set_field(req->dmac_sel_en, HCLGE_CFG_MTA_MAC_SEL_M,
		       HCLGE_CFG_MTA_MAC_SEL_S, mta_mac_sel);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Config mat filter mode failed for cmd_send, ret =%d.\n",
			ret);
		return ret;
	}

	return 0;
}

int hclge_cfg_func_mta_filter(struct hclge_dev *hdev,
			      u8 func_id,
			      bool enable)
{
	struct hclge_cfg_func_mta_filter *req;
	struct hclge_desc desc;
	int ret;

	req = (struct hclge_cfg_func_mta_filter *)desc.data;
	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_MTA_MAC_FUNC_CFG, false);

	hnae_set_bit(req->accept, HCLGE_CFG_FUNC_MTA_ACCEPT_B,
		     enable);
	req->function_id = func_id;

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Config func_id enable failed for cmd_send, ret =%d.\n",
			ret);
		return ret;
	}

	return 0;
}

static int hclge_set_mta_table_item(struct hclge_vport *vport,
				    u16 idx,
				    bool enable)
{
	struct hclge_dev *hdev = vport->back;
	struct hclge_cfg_func_mta_item *req;
	struct hclge_desc desc;
	int ret;

	req = (struct hclge_cfg_func_mta_item *)desc.data;
	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_MTA_TBL_ITEM_CFG, false);
	hnae_set_bit(req->accept, HCLGE_CFG_MTA_ITEM_ACCEPT_B, enable);

	hnae_set_field(req->item_idx, HCLGE_CFG_MTA_ITEM_IDX_M,
		       HCLGE_CFG_MTA_ITEM_IDX_S, idx);
	req->item_idx = cpu_to_le16(req->item_idx);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Config mta table item failed for cmd_send, ret =%d.\n",
			ret);
		return ret;
	}

	return 0;
}

static int hclge_remove_mac_vlan_tbl(struct hclge_vport *vport,
				     struct hclge_mac_vlan_tbl_entry *req)
{
	struct hclge_dev *hdev = vport->back;
	struct hclge_desc desc;
	u8 resp_code;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_MAC_VLAN_REMOVE, false);

	memcpy(desc.data, req, sizeof(struct hclge_mac_vlan_tbl_entry));

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"del mac addr failed for cmd_send, ret =%d.\n",
			ret);
		return ret;
	}
	resp_code = (desc.data[0] >> 8) & 0xff;

	return hclge_get_mac_vlan_cmd_status(vport, desc.retval, resp_code,
					     HCLGE_MAC_VLAN_REMOVE);
}

static int hclge_lookup_mac_vlan_tbl(struct hclge_vport *vport,
				     struct hclge_mac_vlan_tbl_entry *req,
				     struct hclge_desc *desc,
				     bool is_mc)
{
	struct hclge_dev *hdev = vport->back;
	u8 resp_code;
	int ret;

	hclge_cmd_setup_basic_desc(&desc[0], HCLGE_OPC_MAC_VLAN_ADD, true);
	if (is_mc) {
		desc[0].flag |= cpu_to_le16(HCLGE_CMD_FLAG_NEXT);
		memcpy(desc[0].data,
		       req,
		       sizeof(struct hclge_mac_vlan_tbl_entry));
		hclge_cmd_setup_basic_desc(&desc[1],
					   HCLGE_OPC_MAC_VLAN_ADD,
					   true);
		desc[1].flag |= cpu_to_le16(HCLGE_CMD_FLAG_NEXT);
		hclge_cmd_setup_basic_desc(&desc[2],
					   HCLGE_OPC_MAC_VLAN_ADD,
					   true);
		ret = hclge_cmd_send(&hdev->hw, desc, 3);
	} else {
		memcpy(desc[0].data,
		       req,
		       sizeof(struct hclge_mac_vlan_tbl_entry));
		ret = hclge_cmd_send(&hdev->hw, desc, 1);
	}
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"lookup mac addr failed for cmd_send, ret =%d.\n",
			ret);
		return ret;
	}
	resp_code = (desc[0].data[0] >> 8) & 0xff;

	return hclge_get_mac_vlan_cmd_status(vport, desc[0].retval, resp_code,
					     HCLGE_MAC_VLAN_LKUP);
}

static int hclge_add_mac_vlan_tbl(struct hclge_vport *vport,
				  struct hclge_mac_vlan_tbl_entry *req,
				  struct hclge_desc *mc_desc)
{
	struct hclge_dev *hdev = vport->back;
	int cfg_status;
	u8 resp_code;
	int ret;

	if (!mc_desc) {
		struct hclge_desc desc;

		hclge_cmd_setup_basic_desc(&desc,
					   HCLGE_OPC_MAC_VLAN_ADD,
					   false);
		memcpy(desc.data, req, sizeof(struct hclge_mac_vlan_tbl_entry));
		ret = hclge_cmd_send(&hdev->hw, &desc, 1);
		resp_code = (desc.data[0] >> 8) & 0xff;
		cfg_status = hclge_get_mac_vlan_cmd_status(vport, desc.retval,
							   resp_code,
							   HCLGE_MAC_VLAN_ADD);
	} else {
		mc_desc[0].flag &= cpu_to_le16(~HCLGE_CMD_FLAG_WR);
		mc_desc[0].flag |= cpu_to_le16(HCLGE_CMD_FLAG_NEXT);
		mc_desc[1].flag &= cpu_to_le16(~HCLGE_CMD_FLAG_WR);
		mc_desc[1].flag |= cpu_to_le16(HCLGE_CMD_FLAG_NEXT);
		mc_desc[2].flag &= cpu_to_le16(~HCLGE_CMD_FLAG_WR);
		mc_desc[2].flag &= cpu_to_le16(~HCLGE_CMD_FLAG_NEXT);
		memcpy(mc_desc[0].data, req,
		       sizeof(struct hclge_mac_vlan_tbl_entry));
		ret = hclge_cmd_send(&hdev->hw, mc_desc, 3);
		resp_code = (mc_desc[0].data[0] >> 8) & 0xff;
		cfg_status = hclge_get_mac_vlan_cmd_status(vport,
							   mc_desc[0].retval,
							   resp_code,
							   HCLGE_MAC_VLAN_ADD);
	}

	if (ret) {
		dev_err(&hdev->pdev->dev,
			"add mac addr failed for cmd_send, ret =%d.\n",
			ret);
		return ret;
	}

	return cfg_status;
}

static int hclge_add_uc_addr(struct hnae3_handle *handle,
			     const unsigned char *addr)
{
	struct hclge_vport *vport = hclge_get_vport(handle);

	return hclge_add_uc_addr_common(vport, addr);
}

int hclge_add_uc_addr_common(struct hclge_vport *vport,
			     const unsigned char *addr)
{
	struct hclge_dev *hdev = vport->back;
	struct hclge_mac_vlan_tbl_entry req;
	enum hclge_cmd_status status;

	/* mac addr check */
	if (is_zero_ether_addr(addr) ||
	    is_broadcast_ether_addr(addr) ||
	    is_multicast_ether_addr(addr)) {
		dev_err(&hdev->pdev->dev,
			"Set_uc mac err! invalid mac:%pM. is_zero:%d,is_br=%d,is_mul=%d\n",
			 addr,
			 is_zero_ether_addr(addr),
			 is_broadcast_ether_addr(addr),
			 is_multicast_ether_addr(addr));
		return -EINVAL;
	}

	memset(&req, 0, sizeof(req));
	hnae_set_bit(req.flags, HCLGE_MAC_VLAN_BIT0_EN_B, 1);
	hnae_set_bit(req.entry_type, HCLGE_MAC_VLAN_BIT0_EN_B, 0);
	hnae_set_bit(req.entry_type, HCLGE_MAC_VLAN_BIT1_EN_B, 0);
	hnae_set_bit(req.mc_mac_en, HCLGE_MAC_VLAN_BIT0_EN_B, 0);
	hnae_set_bit(req.egress_port,
		     HCLGE_MAC_EPORT_SW_EN_B, 0);
	hnae_set_bit(req.egress_port,
		     HCLGE_MAC_EPORT_TYPE_B, 0);
	hnae_set_field(req.egress_port, HCLGE_MAC_EPORT_VFID_M,
		       HCLGE_MAC_EPORT_VFID_S, vport->vport_id);
	hnae_set_field(req.egress_port, HCLGE_MAC_EPORT_PFID_M,
		       HCLGE_MAC_EPORT_PFID_S, 0);
	req.egress_port = cpu_to_le16(req.egress_port);

	hclge_prepare_mac_addr(&req, addr);

	status = hclge_add_mac_vlan_tbl(vport, &req, NULL);

	return status;
}

static int hclge_rm_uc_addr(struct hnae3_handle *handle,
			    const unsigned char *addr)
{
	struct hclge_vport *vport = hclge_get_vport(handle);

	return hclge_rm_uc_addr_common(vport, addr);
}

int hclge_rm_uc_addr_common(struct hclge_vport *vport,
			    const unsigned char *addr)
{
	struct hclge_dev *hdev = vport->back;
	struct hclge_mac_vlan_tbl_entry req;
	enum hclge_cmd_status status;

	/* mac addr check */
	if (is_zero_ether_addr(addr) ||
	    is_broadcast_ether_addr(addr) ||
	    is_multicast_ether_addr(addr)) {
		dev_dbg(&hdev->pdev->dev,
			"Remove mac err! invalid mac:%pM.\n",
			 addr);
		return -EINVAL;
	}

	memset(&req, 0, sizeof(req));
	hnae_set_bit(req.flags, HCLGE_MAC_VLAN_BIT0_EN_B, 1);
	hnae_set_bit(req.entry_type, HCLGE_MAC_VLAN_BIT0_EN_B, 0);
	hclge_prepare_mac_addr(&req, addr);
	status = hclge_remove_mac_vlan_tbl(vport, &req);

	return status;
}

static int hclge_add_mc_addr(struct hnae3_handle *handle,
			     const unsigned char *addr)
{
	struct hclge_vport *vport = hclge_get_vport(handle);

	return	hclge_add_mc_addr_common(vport, addr);
}

int hclge_add_mc_addr_common(struct hclge_vport *vport,
			     const unsigned char *addr)
{
	struct hclge_dev *hdev = vport->back;
	struct hclge_mac_vlan_tbl_entry req;
	struct hclge_desc desc[3];
	u16 tbl_idx;
	int status;

	/* mac addr check */
	if (!is_multicast_ether_addr(addr)) {
		dev_err(&hdev->pdev->dev,
			"Add mc mac err! invalid mac:%pM.\n",
			 addr);
		return -EINVAL;
	}
	memset(&req, 0, sizeof(req));
	hnae_set_bit(req.flags, HCLGE_MAC_VLAN_BIT0_EN_B, 1);
	hnae_set_bit(req.entry_type, HCLGE_MAC_VLAN_BIT0_EN_B, 0);
	hnae_set_bit(req.entry_type, HCLGE_MAC_VLAN_BIT1_EN_B, 1);
	hnae_set_bit(req.mc_mac_en, HCLGE_MAC_VLAN_BIT0_EN_B, 0);
	hclge_prepare_mac_addr(&req, addr);
	status = hclge_lookup_mac_vlan_tbl(vport, &req, desc, true);
	if (!status) {
		/* This mac addr exist, update VFID for it */
		hclge_update_desc_vfid(desc, vport->vport_id, false);
		status = hclge_add_mac_vlan_tbl(vport, &req, desc);
	} else {
		/* This mac addr do not exist, add new entry for it */
		memset(desc[0].data, 0, sizeof(desc[0].data));
		memset(desc[1].data, 0, sizeof(desc[0].data));
		memset(desc[2].data, 0, sizeof(desc[0].data));
		hclge_update_desc_vfid(desc, vport->vport_id, false);
		status = hclge_add_mac_vlan_tbl(vport, &req, desc);
	}

	/* Set MTA table for this MAC address */
	tbl_idx = hclge_get_mac_addr_to_mta_index(vport, addr);
	status = hclge_set_mta_table_item(vport, tbl_idx, true);

	return status;
}

static int hclge_rm_mc_addr(struct hnae3_handle *handle,
			    const unsigned char *addr)
{
	struct hclge_vport *vport = hclge_get_vport(handle);

	return hclge_rm_mc_addr_common(vport, addr);
}

int hclge_rm_mc_addr_common(struct hclge_vport *vport,
			    const unsigned char *addr)
{
	struct hclge_dev *hdev = vport->back;
	struct hclge_mac_vlan_tbl_entry req;
	enum hclge_cmd_status status;
	struct hclge_desc desc[3];
	u16 tbl_idx;

	/* mac addr check */
	if (!is_multicast_ether_addr(addr)) {
		dev_dbg(&hdev->pdev->dev,
			"Remove mc mac err! invalid mac:%pM.\n",
			 addr);
		return -EINVAL;
	}

	memset(&req, 0, sizeof(req));
	hnae_set_bit(req.flags, HCLGE_MAC_VLAN_BIT0_EN_B, 1);
	hnae_set_bit(req.entry_type, HCLGE_MAC_VLAN_BIT0_EN_B, 0);
	hnae_set_bit(req.entry_type, HCLGE_MAC_VLAN_BIT1_EN_B, 1);
	hnae_set_bit(req.mc_mac_en, HCLGE_MAC_VLAN_BIT0_EN_B, 0);
	hclge_prepare_mac_addr(&req, addr);
	status = hclge_lookup_mac_vlan_tbl(vport, &req, desc, true);
	if (!status) {
		/* This mac addr exist, remove this handle's VFID for it */
		hclge_update_desc_vfid(desc, vport->vport_id, true);

		if (hclge_is_all_function_id_zero(desc))
			/* All the vfid is zero, so need to delete this entry */
			status = hclge_remove_mac_vlan_tbl(vport, &req);
		else
			/* Not all the vfid is zero, update the vfid */
			status = hclge_add_mac_vlan_tbl(vport, &req, desc);

	} else {
		/* This mac addr do not exist, can't delete it */
		dev_err(&hdev->pdev->dev,
			"Rm multicast mac addr failed, ret = %d.\n",
			status);
		return -EIO;
	}

	/* Set MTB table for this MAC address */
	tbl_idx = hclge_get_mac_addr_to_mta_index(vport, addr);
	status = hclge_set_mta_table_item(vport, tbl_idx, false);

	return status;
}

static void hclge_get_mac_addr(struct hnae3_handle *handle, u8 *p)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	ether_addr_copy(p, hdev->hw.mac.mac_addr);
}

static int hclge_set_mac_addr(struct hnae3_handle *handle, void *p)
{
	const unsigned char *new_addr = (const unsigned char *)p;
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	/* mac addr check */
	if (is_zero_ether_addr(new_addr) ||
	    is_broadcast_ether_addr(new_addr) ||
	    is_multicast_ether_addr(new_addr)) {
		dev_err(&hdev->pdev->dev,
			"Change uc mac err! invalid mac:%p.\n",
			 new_addr);
		return -EINVAL;
	}

	hclge_rm_uc_addr(handle, hdev->hw.mac.mac_addr);

	if (!hclge_add_uc_addr(handle, new_addr)) {
		ether_addr_copy(hdev->hw.mac.mac_addr, new_addr);
		return 0;
	}

	return -EIO;
}

static int hclge_set_vlan_filter_ctrl(struct hclge_dev *hdev, u8 vlan_type,
				      bool filter_en)
{
	struct hclge_vlan_filter_ctrl *req;
	struct hclge_desc desc;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_VLAN_FILTER_CTRL, false);

	req = (struct hclge_vlan_filter_ctrl *)desc.data;
	req->vlan_type = vlan_type;
	req->vlan_fe = filter_en;

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev, "set vlan filter fail, ret =%d.\n",
			ret);
		return ret;
	}

	return 0;
}

int hclge_set_vf_vlan_common(struct hclge_dev *hdev, int vfid,
			     bool is_kill, u16 vlan, u8 qos, __be16 proto)
{
#define HCLGE_MAX_VF_BYTES  16
	struct hclge_vlan_filter_vf_cfg *req0;
	struct hclge_vlan_filter_vf_cfg *req1;
	struct hclge_desc desc[2];
	u8 vf_byte_val;
	u8 vf_byte_off;
	int ret;

	hclge_cmd_setup_basic_desc(&desc[0],
				   HCLGE_OPC_VLAN_FILTER_VF_CFG, false);
	hclge_cmd_setup_basic_desc(&desc[1],
				   HCLGE_OPC_VLAN_FILTER_VF_CFG, false);

	desc[0].flag |= cpu_to_le16(HCLGE_CMD_FLAG_NEXT);

	vf_byte_off = vfid / 8;
	vf_byte_val = 1 << (vfid % 8);

	req0 = (struct hclge_vlan_filter_vf_cfg *)desc[0].data;
	req1 = (struct hclge_vlan_filter_vf_cfg *)desc[1].data;

	req0->vlan_id  = vlan;
	req0->vlan_cfg = is_kill;

	if (vf_byte_off < HCLGE_MAX_VF_BYTES)
		req0->vf_bitmap[vf_byte_off] = vf_byte_val;
	else
		req1->vf_bitmap[vf_byte_off - HCLGE_MAX_VF_BYTES] = vf_byte_val;

	ret = hclge_cmd_send(&hdev->hw, desc, 2);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Send vf vlan command fail, ret =%d.\n",
			ret);
		return ret;
	}

	if (!is_kill) {
		if (!req0->resp_code || req0->resp_code == 1)
			return 0;

		dev_err(&hdev->pdev->dev,
			"Add vf vlan filter fail, ret =%d.\n",
			req0->resp_code);
	} else {
		if (!req0->resp_code)
			return 0;

		dev_err(&hdev->pdev->dev,
			"Kill vf vlan filter fail, ret =%d.\n",
			req0->resp_code);
	}

	return -EIO;
}

static int hclge_set_port_vlan_filter(struct hnae3_handle *handle,
				      __be16 proto, u16 vlan_id,
				      bool is_kill)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	struct hclge_vlan_filter_pf_cfg *req;
	struct hclge_desc desc;
	u8 vlan_offset_byte_val;
	u8 vlan_offset_byte;
	u8 vlan_offset_160;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_VLAN_FILTER_PF_CFG, false);

	vlan_offset_160 = vlan_id / 160;
	vlan_offset_byte = (vlan_id % 160) / 8;
	vlan_offset_byte_val = 1 << (vlan_id % 8);

	req = (struct hclge_vlan_filter_pf_cfg *)desc.data;
	req->vlan_offset = vlan_offset_160;
	req->vlan_cfg = is_kill;
	req->vlan_offset_bitmap[vlan_offset_byte] = vlan_offset_byte_val;

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"port vlan command, send fail, ret =%d.\n",
			ret);
		return ret;
	}

	ret = hclge_set_vf_vlan_common(hdev, 0, is_kill, vlan_id, 0, proto);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Set pf vlan filter config fail, ret =%d.\n",
			ret);
		return -EIO;
	}

	return 0;
}

static int hclge_set_vf_vlan_filter(struct hnae3_handle *handle, int vfid,
				    u16 vlan, u8 qos, __be16 proto)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	if ((vfid >= hdev->num_alloc_vfs) || (vlan > 4095) || (qos > 7))
		return -EINVAL;
	if (proto != htons(ETH_P_8021Q))
		return -EPROTONOSUPPORT;

	return hclge_set_vf_vlan_common(hdev, vfid, false, vlan, qos, proto);
}

static int hclge_init_vlan_config(struct hclge_dev *hdev)
{
#define HCLGE_VLAN_TYPE_VF_TABLE   0
#define HCLGE_VLAN_TYPE_PORT_TABLE 1
	struct hnae3_handle *handle;
	int ret;

	ret = hclge_set_vlan_filter_ctrl(hdev, HCLGE_VLAN_TYPE_VF_TABLE,
					 true);
	if (ret)
		return ret;

	ret = hclge_set_vlan_filter_ctrl(hdev, HCLGE_VLAN_TYPE_PORT_TABLE,
					 true);
	if (ret)
		return ret;

	handle = &hdev->vport[0].nic;
	return hclge_set_port_vlan_filter(handle, htons(ETH_P_8021Q), 0, false);
}

static int hclge_set_mtu(struct hnae3_handle *handle, int new_mtu)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_config_max_frm_size *req;
	struct hclge_dev *hdev = vport->back;
	struct hclge_desc desc;
	int ret;

	if ((new_mtu < HCLGE_MAC_MIN_MTU) || (new_mtu > HCLGE_MAC_MAX_MTU))
		return -EINVAL;

	hdev->mps = new_mtu;
	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_CONFIG_MAX_FRM_SIZE, false);

	req = (struct hclge_config_max_frm_size *)desc.data;
	req->max_frm_size = cpu_to_le16(new_mtu);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev, "set mtu fail, ret =%d.\n", ret);
		return ret;
	}

	return 0;
}

static int hclge_send_reset_tqp_cmd(struct hclge_dev *hdev, u16 queue_id,
				    bool enable)
{
	struct hclge_reset_tqp_queue *req;
	struct hclge_desc desc;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_RESET_TQP_QUEUE, false);

	req = (struct hclge_reset_tqp_queue *)desc.data;
	req->tqp_id = cpu_to_le16(queue_id & HCLGE_RING_ID_MASK);
	hnae_set_bit(req->reset_req, HCLGE_TQP_RESET_B, enable);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Send tqp reset cmd error, status =%d\n", ret);
		return ret;
	}

	return 0;
}

static int hclge_get_reset_status(struct hclge_dev *hdev, u16 queue_id)
{
	struct hclge_reset_tqp_queue *req;
	struct hclge_desc desc;
	int ret;

	hclge_cmd_setup_basic_desc(&desc, HCLGE_OPC_RESET_TQP_QUEUE, true);

	req = (struct hclge_reset_tqp_queue *)desc.data;
	req->tqp_id = cpu_to_le16(queue_id & HCLGE_RING_ID_MASK);

	ret = hclge_cmd_send(&hdev->hw, &desc, 1);
	if (ret) {
		dev_err(&hdev->pdev->dev,
			"Get reset status error, status =%d\n", ret);
		return ret;
	}

	return hnae_get_bit(req->ready_to_reset, HCLGE_TQP_RESET_B);
}

static void hclge_reset_tqp(struct hnae3_handle *handle, u16 queue_id)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	int reset_try_times = 0;
	int reset_status;
	int ret;

	ret = hclge_tqp_enable(hdev, queue_id, 0, false);
	if (ret) {
		dev_warn(&hdev->pdev->dev, "Disable tqp fail, ret = %d\n", ret);
		return;
	}

	ret = hclge_send_reset_tqp_cmd(hdev, queue_id, true);
	if (ret) {
		dev_warn(&hdev->pdev->dev,
			 "Send reset tqp cmd fail, ret = %d\n", ret);
		return;
	}

	reset_try_times = 0;
	while (reset_try_times++ < HCLGE_TQP_RESET_TRY_TIMES) {
		/* Wait for tqp hw reset */
		msleep(20);
		reset_status = hclge_get_reset_status(hdev, queue_id);
		if (reset_status)
			break;
	}

	if (reset_try_times >= HCLGE_TQP_RESET_TRY_TIMES) {
		dev_warn(&hdev->pdev->dev, "Reset TQP fail\n");
		return;
	}

	ret = hclge_send_reset_tqp_cmd(hdev, queue_id, false);
	if (ret) {
		dev_warn(&hdev->pdev->dev,
			 "Deassert the soft reset fail, ret = %d\n", ret);
		return;
	}
}

static u32 hclge_get_fw_version(struct hnae3_handle *handle)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	return hdev->fw_version;
}

static void hclge_get_pauseparam(struct hnae3_handle *handle, u32 *auto_neg,
				 u32 *rx_en, u32 *tx_en)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	*auto_neg = hclge_get_autoneg(handle);

	if (hdev->tm_info.fc_mode == HCLGE_FC_PFC) {
		*rx_en = 0;
		*tx_en = 0;
		return;
	}

	if (hdev->tm_info.fc_mode == HCLGE_FC_RX_PAUSE) {
		*rx_en = 1;
		*tx_en = 0;
	} else if (hdev->tm_info.fc_mode == HCLGE_FC_TX_PAUSE) {
		*tx_en = 1;
		*rx_en = 0;
	} else if (hdev->tm_info.fc_mode == HCLGE_FC_FULL) {
		*rx_en = 1;
		*tx_en = 1;
	} else {
		*rx_en = 0;
		*tx_en = 0;
	}
}

static void hclge_get_ksettings_an_result(struct hnae3_handle *handle,
					  u8 *auto_neg, u32 *speed, u8 *duplex)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	if (speed)
		*speed = hdev->hw.mac.speed;
	if (duplex)
		*duplex = hdev->hw.mac.duplex;
	if (auto_neg)
		*auto_neg = hdev->hw.mac.autoneg;
}

static void hclge_get_media_type(struct hnae3_handle *handle, u8 *media_type)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;

	if (media_type)
		*media_type = hdev->hw.mac.media_type;
}

static void hclge_get_mdix_mode(struct hnae3_handle *handle,
				u8 *tp_mdix_ctrl, u8 *tp_mdix)
{
	struct hclge_vport *vport = hclge_get_vport(handle);
	struct hclge_dev *hdev = vport->back;
	struct phy_device *phydev = hdev->hw.mac.phydev;
	int mdix_ctrl, mdix, retval, is_resolved;

	if (!phydev) {
		*tp_mdix_ctrl = ETH_TP_MDI_INVALID;
		*tp_mdix = ETH_TP_MDI_INVALID;
		return;
	}

	phy_write(phydev, HCLGE_PHY_PAGE_REG, HCLGE_PHY_PAGE_MDIX);

	retval = phy_read(phydev, HCLGE_PHY_CSC_REG);
	mdix_ctrl = hnae_get_field(retval, HCLGE_PHY_MDIX_CTRL_M,
				   HCLGE_PHY_MDIX_CTRL_S);

	retval = phy_read(phydev, HCLGE_PHY_CSS_REG);
	mdix = hnae_get_bit(retval, HCLGE_PHY_MDIX_STATUS_B);
	is_resolved = hnae_get_bit(retval, HCLGE_PHY_SPEED_DUP_RESOLVE_B);

	phy_write(phydev, HCLGE_PHY_PAGE_REG, HCLGE_PHY_PAGE_COPPER);

	switch (mdix_ctrl) {
	case 0x0:
		*tp_mdix_ctrl = ETH_TP_MDI;
		break;
	case 0x1:
		*tp_mdix_ctrl = ETH_TP_MDI_X;
		break;
	case 0x3:
		*tp_mdix_ctrl = ETH_TP_MDI_AUTO;
		break;
	default:
		*tp_mdix_ctrl = ETH_TP_MDI_INVALID;
		break;
	}

	if (!is_resolved)
		*tp_mdix = ETH_TP_MDI_INVALID;
	else if (mdix)
		*tp_mdix = ETH_TP_MDI_X;
	else
		*tp_mdix = ETH_TP_MDI;
}

static int hclge_init_client_instance(struct hnae3_client *client,
				      struct hnae3_ae_dev *ae_dev)
{
	struct hclge_dev *hdev = ae_dev->priv;
	struct hclge_vport *vport;
	int i, ret;

	for (i = 0; i <  hdev->num_vmdq_vport + 1; i++) {
		vport = &hdev->vport[i];

		switch (client->type) {
		case HNAE3_CLIENT_KNIC:

			hdev->nic_client = client;
			vport->nic.client = client;
			ret = client->ops->init_instance(&vport->nic);
			if (ret)
				goto err;

			if (hdev->roce_client &&
			    hnae3_dev_roce_supported(hdev)) {
				struct hnae3_client *rc = hdev->roce_client;

				ret = hclge_init_roce_base_info(vport);
				if (ret)
					goto err;

				ret = rc->ops->init_instance(&vport->roce);
				if (ret)
					goto err;
			}

			break;
		case HNAE3_CLIENT_UNIC:
			hdev->nic_client = client;
			vport->nic.client = client;

			ret = client->ops->init_instance(&vport->nic);
			if (ret)
				goto err;

			break;
		case HNAE3_CLIENT_ROCE:
			if (hnae3_dev_roce_supported(hdev)) {
				hdev->roce_client = client;
				vport->roce.client = client;
			}

			if (hdev->roce_client && hdev->nic_client) {
				ret = hclge_init_roce_base_info(vport);
				if (ret)
					goto err;

				ret = client->ops->init_instance(&vport->roce);
				if (ret)
					goto err;
			}
		}
	}

	return 0;
err:
	return ret;
}

static void hclge_uninit_client_instance(struct hnae3_client *client,
					 struct hnae3_ae_dev *ae_dev)
{
	struct hclge_dev *hdev = ae_dev->priv;
	struct hclge_vport *vport;
	int i;

	for (i = 0; i < hdev->num_vmdq_vport + 1; i++) {
		vport = &hdev->vport[i];
		if (hdev->roce_client) {
			hdev->roce_client->ops->uninit_instance(&vport->roce,
								0);
			hdev->roce_client = NULL;
			vport->roce.client = NULL;
		}
		if (client->type == HNAE3_CLIENT_ROCE)
			return;
		if (client->ops->uninit_instance) {
			client->ops->uninit_instance(&vport->nic, 0);
			hdev->nic_client = NULL;
			vport->nic.client = NULL;
		}
	}
}

static int hclge_pci_init(struct hclge_dev *hdev)
{
	struct pci_dev *pdev = hdev->pdev;
	struct hclge_hw *hw;
	int ret;

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable PCI device\n");
		goto err_no_drvdata;
	}

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret) {
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(&pdev->dev,
				"can't set consistent PCI DMA");
			goto err_disable_device;
		}
		dev_warn(&pdev->dev, "set DMA mask to 32 bits\n");
	}

	ret = pci_request_regions(pdev, HCLGE_DRIVER_NAME);
	if (ret) {
		dev_err(&pdev->dev, "PCI request regions failed %d\n", ret);
		goto err_disable_device;
	}

	pci_set_master(pdev);
	hw = &hdev->hw;
	hw->back = hdev;
	hw->io_base = pcim_iomap(pdev, 2, 0);
	if (!hw->io_base) {
		dev_err(&pdev->dev, "Can't map configuration register space\n");
		ret = -ENOMEM;
		goto err_clr_master;
	}

	return 0;
err_clr_master:
	pci_clear_master(pdev);
	pci_release_regions(pdev);
err_disable_device:
	pci_disable_device(pdev);
err_no_drvdata:
	pci_set_drvdata(pdev, NULL);

	return ret;
}

static void hclge_pci_uninit(struct hclge_dev *hdev)
{
	struct pci_dev *pdev = hdev->pdev;

	if (hdev->flag & HCLGE_FLAG_USE_MSIX) {
		pci_disable_msix(pdev);
		devm_kfree(&pdev->dev, hdev->msix_entries);
		hdev->msix_entries = NULL;
	} else {
		pci_disable_msi(pdev);
	}

	pci_clear_master(pdev);
	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

static int hclge_init_ae_dev(struct hnae3_ae_dev *ae_dev)
{
	struct pci_dev *pdev = ae_dev->pdev;
	struct hclge_dev *hdev;
	int ret;

	hdev = devm_kzalloc(&pdev->dev, sizeof(*hdev), GFP_KERNEL);
	if (!hdev) {
		ret = -ENOMEM;
		goto err_hclge_dev;
	}

	hdev->flag |= HCLGE_FLAG_USE_MSIX;
	hdev->pdev = pdev;
	hdev->ae_dev = ae_dev;
	ae_dev->priv = hdev;

	ret = hclge_pci_init(hdev);
	if (ret) {
		dev_err(&pdev->dev, "PCI init failed\n");
		goto err_pci_init;
	}

	/* Command queue initialize */
	ret = hclge_cmd_init(hdev);
	if (ret)
		goto err_cmd_init;

	ret = hclge_get_cap(hdev);
	if (ret) {
		dev_err(&pdev->dev, "get hw capability error, ret = %d.\n",
			ret);
		return ret;
	}

	ret = hclge_configure(hdev);
	if (ret) {
		dev_err(&pdev->dev, "Configure dev error, ret = %d.\n", ret);
		return ret;
	}

	if (hdev->flag & HCLGE_FLAG_USE_MSIX)
		ret = hclge_init_msix(hdev);
	else
		ret = hclge_init_msi(hdev);
	if (ret) {
		dev_err(&pdev->dev, "Init msix/msi error, ret = %d.\n", ret);
		return ret;
	}

	ret = hclge_alloc_tqps(hdev);
	if (ret) {
		dev_err(&pdev->dev, "Allocate TQPs error, ret = %d.\n", ret);
		return ret;
	}

	ret = hclge_alloc_vport(hdev);
	if (ret) {
		dev_err(&pdev->dev, "Allocate vport error, ret = %d.\n", ret);
		return ret;
	}

	ret = hclge_mac_init(hdev);
	if (ret) {
		dev_err(&pdev->dev, "Mac init error, ret = %d\n", ret);
		return ret;
	}
	ret = hclge_buffer_alloc(hdev);
	if (ret) {
		dev_err(&pdev->dev, "Buffer allocate fail, ret =%d\n", ret);
		return  ret;
	}

	ret = hclge_config_tso(hdev, HCLGE_TSO_MSS_MIN, HCLGE_TSO_MSS_MAX);
	if (ret) {
		dev_err(&pdev->dev, "Enable tso fail, ret =%d\n", ret);
		return ret;
	}

	ret = hclge_init_vlan_config(hdev);
	if (ret) {
		dev_err(&pdev->dev, "VLAN init fail, ret =%d\n", ret);
		return  ret;
	}

	ret = hclge_tm_schd_init(hdev);
	if (ret) {
		dev_err(&pdev->dev, "tm schd init fail, ret =%d\n", ret);
		return ret;
	}

	ret = hclge_rss_init_hw(hdev);
	if (ret) {
		dev_err(&pdev->dev, "Rss init fail, ret =%d\n", ret);
		return ret;
	}

	setup_timer(&hdev->service_timer, hclge_service_timer,
		    (unsigned long)hdev);
	INIT_WORK(&hdev->service_task, hclge_service_task);

	set_bit(HCLGE_STATE_SERVICE_INITED, &hdev->state);
	set_bit(HCLGE_STATE_DOWN, &hdev->state);

	pr_info("%s driver initialization finished.\n", HCLGE_DRIVER_NAME);
	return 0;

err_cmd_init:
	pci_release_regions(pdev);
err_pci_init:
	pci_set_drvdata(pdev, NULL);
err_hclge_dev:
	return ret;
}

static void hclge_uninit_ae_dev(struct hnae3_ae_dev *ae_dev)
{
	struct hclge_dev *hdev = ae_dev->priv;
	struct hclge_mac *mac = &hdev->hw.mac;

	set_bit(HCLGE_STATE_DOWN, &hdev->state);

	if (IS_ENABLED(CONFIG_PCI_IOV))
		hclge_disable_sriov(hdev);

	if (hdev->service_timer.data)
		del_timer_sync(&hdev->service_timer);
	if (hdev->service_task.func)
		cancel_work_sync(&hdev->service_task);

	if (mac->phydev)
		mdiobus_unregister(mac->mdio_bus);

	hclge_destroy_cmd_queue(&hdev->hw);
	hclge_pci_uninit(hdev);
	ae_dev->priv = NULL;
}

static const struct hnae3_ae_ops hclge_ops = {
	.init_ae_dev = hclge_init_ae_dev,
	.uninit_ae_dev = hclge_uninit_ae_dev,
	.init_client_instance = hclge_init_client_instance,
	.uninit_client_instance = hclge_uninit_client_instance,
	.map_ring_to_vector = hclge_map_handle_ring_to_vector,
	.unmap_ring_from_vector = hclge_unmap_ring_from_vector,
	.get_vector = hclge_get_vector,
	.set_promisc_mode = hclge_set_promisc_mode,
	.start = hclge_ae_start,
	.stop = hclge_ae_stop,
	.get_status = hclge_get_status,
	.get_ksettings_an_result = hclge_get_ksettings_an_result,
	.update_speed_duplex_h = hclge_update_speed_duplex_h,
	.cfg_mac_speed_dup_h = hclge_cfg_mac_speed_dup_h,
	.get_media_type = hclge_get_media_type,
	.get_rss_key_size = hclge_get_rss_key_size,
	.get_rss_indir_size = hclge_get_rss_indir_size,
	.get_rss = hclge_get_rss,
	.set_rss = hclge_set_rss,
	.get_tc_size = hclge_get_tc_size,
	.get_mac_addr = hclge_get_mac_addr,
	.set_mac_addr = hclge_set_mac_addr,
	.add_uc_addr = hclge_add_uc_addr,
	.rm_uc_addr = hclge_rm_uc_addr,
	.add_mc_addr = hclge_add_mc_addr,
	.rm_mc_addr = hclge_rm_mc_addr,
	.set_autoneg = hclge_set_autoneg,
	.get_autoneg = hclge_get_autoneg,
	.get_pauseparam = hclge_get_pauseparam,
	.set_mtu = hclge_set_mtu,
	.reset_queue = hclge_reset_tqp,
	.get_stats = hclge_get_stats,
	.update_stats = hclge_update_stats,
	.get_strings = hclge_get_strings,
	.get_sset_count = hclge_get_sset_count,
	.get_fw_version = hclge_get_fw_version,
	.get_mdix_mode = hclge_get_mdix_mode,
	.set_vlan_filter = hclge_set_port_vlan_filter,
	.set_vf_vlan_filter = hclge_set_vf_vlan_filter,
};

static struct hnae3_ae_algo ae_algo = {
	.ops = &hclge_ops,
	.name = HCLGE_NAME,
	.pdev_id_table = ae_algo_pci_tbl,
};

static int hclge_init(void)
{
	pr_info("%s is initializing\n", HCLGE_NAME);

	return hnae3_register_ae_algo(&ae_algo);
}

static void hclge_exit(void)
{
	hnae3_unregister_ae_algo(&ae_algo);
}
module_init(hclge_init);
module_exit(hclge_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Huawei Tech. Co., Ltd.");
MODULE_DESCRIPTION("HCLGE Driver");
MODULE_VERSION(HCLGE_MOD_VERSION);
