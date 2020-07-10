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

#ifndef _RMNET_QMI_I_H
#define _RMNET_QMI_I_H

#include <linux/netdevice.h>
#include <linux/skbuff.h>

#define IP_VER_4 4
#define IP_VER_6 6

#define MAX_CLIENT_NUM 2
#define MAX_FLOW_NUM 32
#define DEFAULT_GRANT 1
#define DFC_MAX_BEARERS_V01 16

struct rmnet_flow_map {
	struct list_head list;
	u8 bearer_id;
	u32 flow_id;
	int ip_type;
	u32 tcm_handle;
};

struct rmnet_bearer_map {
	struct list_head list;
	u8 bearer_id;
	int flow_ref;
	u32 grant_size;
	u32 grant_thresh;
	u16 seq;
	u8  ack_req;
	u32 last_grant;
	u16 last_seq;
	bool tcp_bidir;
	bool rat_switch;
	bool tx_off;
};

struct svc_info {
	u32 instance;
	u32 ep_type;
	u32 iface_id;
};

struct qos_info {
	u8 mux_id;
	struct net_device *real_dev;
	struct list_head flow_head;
	struct list_head bearer_head;
	u32 default_grant;
	u32 tran_num;
	spinlock_t qos_lock;
};

struct flow_info {
	struct net_device *dev;
	struct rmnet_flow_map *itm;
};

struct qmi_info {
	int flag;
	void *wda_client;
	void *wda_pending;
	void *dfc_clients[MAX_CLIENT_NUM];
	void *dfc_pending[MAX_CLIENT_NUM];
	unsigned long ps_work_active;
	bool ps_enabled;
	bool dl_msg_active;
	bool ps_ignore_grant;
};

enum data_ep_type_enum_v01 {
	DATA_EP_TYPE_ENUM_MIN_ENUM_VAL_V01 = INT_MIN,
	DATA_EP_TYPE_RESERVED_V01 = 0x00,
	DATA_EP_TYPE_HSIC_V01 = 0x01,
	DATA_EP_TYPE_HSUSB_V01 = 0x02,
	DATA_EP_TYPE_PCIE_V01 = 0x03,
	DATA_EP_TYPE_EMBEDDED_V01 = 0x04,
	DATA_EP_TYPE_ENUM_MAX_ENUM_VAL_V01 = INT_MAX
};

struct data_ep_id_type_v01 {

	enum data_ep_type_enum_v01 ep_type;
	u32 iface_id;
};

extern struct qmi_elem_info data_ep_id_type_v01_ei[];

void *qmi_rmnet_has_dfc_client(struct qmi_info *qmi);

#ifdef CONFIG_QCOM_QMI_DFC
struct rmnet_flow_map *
qmi_rmnet_get_flow_map(struct qos_info *qos_info,
		       u32 flow_id, int ip_type);

struct rmnet_bearer_map *
qmi_rmnet_get_bearer_map(struct qos_info *qos_info, u8 bearer_id);

unsigned int qmi_rmnet_grant_per(unsigned int grant);

int dfc_qmi_client_init(void *port, int index, struct svc_info *psvc,
			struct qmi_info *qmi);

void dfc_qmi_client_exit(void *dfc_data);

void dfc_qmi_burst_check(struct net_device *dev, struct qos_info *qos,
			 int ip_type, u32 mark, unsigned int len);

int qmi_rmnet_flow_control(struct net_device *dev, u32 tcm_handle, int enable);

void dfc_qmi_query_flow(void *dfc_data);

int dfc_bearer_flow_ctl(struct net_device *dev,
			struct rmnet_bearer_map *bearer,
			struct qos_info *qos);
#else
static inline struct rmnet_flow_map *
qmi_rmnet_get_flow_map(struct qos_info *qos_info,
		       uint32_t flow_id, int ip_type)
{
	return NULL;
}

static inline struct rmnet_bearer_map *
qmi_rmnet_get_bearer_map(struct qos_info *qos_info, u8 bearer_id)
{
	return NULL;
}

static inline int
dfc_qmi_client_init(void *port, int index, struct svc_info *psvc,
		    struct qmi_info *qmi)
{
	return -EINVAL;
}

static inline void dfc_qmi_client_exit(void *dfc_data)
{
}

static inline void
dfc_qmi_burst_check(struct net_device *dev, struct qos_info *qos,
		    int ip_type, u32 mark, unsigned int len)
{
}

static inline void
dfc_qmi_query_flow(void *dfc_data)
{
}

static inline int
dfc_bearer_flow_ctl(struct net_device *dev,
		    struct rmnet_bearer_map *bearer,
		    struct qos_info *qos)
{
	return 0;
}
#endif

#ifdef CONFIG_QCOM_QMI_POWER_COLLAPSE
int
wda_qmi_client_init(void *port, struct svc_info *psvc, struct qmi_info *qmi);
void wda_qmi_client_exit(void *wda_data);
int wda_set_powersave_mode(void *wda_data, u8 enable);
void qmi_rmnet_flush_ps_wq(void);
#else
static inline int
wda_qmi_client_init(void *port, struct svc_info *psvc, struct qmi_info *qmi)
{
	return -EINVAL;
}

static inline void wda_qmi_client_exit(void *wda_data)
{
}

static inline int wda_set_powersave_mode(void *wda_data, u8 enable)
{
	return -EINVAL;
}
static inline void qmi_rmnet_flush_ps_wq(void)
{
}
#endif
#endif /*_RMNET_QMI_I_H*/
