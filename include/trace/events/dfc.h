/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM dfc

#if !defined(_TRACE_DFC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_DFC_H

#include <linux/tracepoint.h>

TRACE_EVENT(dfc_qmi_tc,

	TP_PROTO(const char *name, u8 bearer_id, u32 flow_id, u32 grant,
		 int qlen, u32 tcm_handle, int enable),

	TP_ARGS(name, bearer_id, flow_id, grant, qlen, tcm_handle, enable),

	TP_STRUCT__entry(
		__string(dev_name, name)
		__field(u8, bid)
		__field(u32, fid)
		__field(u32, grant)
		__field(int, qlen)
		__field(u32, tcm_handle)
		__field(int, enable)
	),

	TP_fast_assign(
		__assign_str(dev_name, name);
		__entry->bid = bearer_id;
		__entry->fid = flow_id;
		__entry->grant = grant;
		__entry->qlen = qlen;
		__entry->tcm_handle = tcm_handle;
		__entry->enable = enable;
	),

	TP_printk("dev=%s bearer_id=%u grant=%u len=%d flow_id=%u q=%d %s",
		__get_str(dev_name),
		__entry->bid, __entry->grant, __entry->qlen, __entry->fid,
		__entry->tcm_handle,
		__entry->enable ? "enable" : "disable")
);

TRACE_EVENT(dfc_flow_ind,

	TP_PROTO(int src, int idx, u8 mux_id, u8 bearer_id, u32 grant,
		 u16 seq_num, u8 ack_req, u32 ancillary),

	TP_ARGS(src, idx, mux_id, bearer_id, grant, seq_num, ack_req,
		ancillary),

	TP_STRUCT__entry(
		__field(int, src)
		__field(int, idx)
		__field(u8, mid)
		__field(u8, bid)
		__field(u32, grant)
		__field(u16, seq)
		__field(u8, ack_req)
		__field(u32, ancillary)
	),

	TP_fast_assign(
		__entry->src = src;
		__entry->idx = idx;
		__entry->mid = mux_id;
		__entry->bid = bearer_id;
		__entry->grant = grant;
		__entry->seq = seq_num;
		__entry->ack_req = ack_req;
		__entry->ancillary = ancillary;
	),

	TP_printk("src=%d [%d]: mid=%u bid=%u grant=%u seq=%u ack=%u anc=%u",
		__entry->src, __entry->idx, __entry->mid, __entry->bid,
		__entry->grant, __entry->seq, __entry->ack_req,
		__entry->ancillary)
);

TRACE_EVENT(dfc_flow_check,

	TP_PROTO(const char *name, u8 bearer_id, unsigned int len, u32 grant),

	TP_ARGS(name, bearer_id, len, grant),

	TP_STRUCT__entry(
		__string(dev_name, name)
		__field(u8, bearer_id)
		__field(unsigned int, len)
		__field(u32, grant)
	),

	TP_fast_assign(
		__assign_str(dev_name, name)
		__entry->bearer_id = bearer_id;
		__entry->len = len;
		__entry->grant = grant;
	),

	TP_printk("dev=%s bearer_id=%u skb_len=%u current_grant=%u",
		__get_str(dev_name),
		__entry->bearer_id, __entry->len, __entry->grant)
);

TRACE_EVENT(dfc_flow_info,

	TP_PROTO(const char *name, u8 bearer_id, u32 flow_id, int ip_type,
		 u32 handle, int add),

	TP_ARGS(name, bearer_id, flow_id, ip_type, handle, add),

	TP_STRUCT__entry(
		__string(dev_name, name)
		__field(u8, bid)
		__field(u32, fid)
		__field(int, ip)
		__field(u32, handle)
		__field(int, action)
	),

	TP_fast_assign(
		__assign_str(dev_name, name)
		__entry->bid = bearer_id;
		__entry->fid = flow_id;
		__entry->ip = ip_type;
		__entry->handle = handle;
		__entry->action = add;
	),

	TP_printk("%s: dev=%s bearer_id=%u flow_id=%u ip_type=%d q=%d",
		__entry->action ? "add flow" : "delete flow",
		__get_str(dev_name),
		__entry->bid, __entry->fid, __entry->ip, __entry->handle)
);

TRACE_EVENT(dfc_client_state_up,

	TP_PROTO(int idx, u32 instance, u32 ep_type, u32 iface),

	TP_ARGS(idx, instance, ep_type, iface),

	TP_STRUCT__entry(
		__field(int, idx)
		__field(u32, instance)
		__field(u32, ep_type)
		__field(u32, iface)
	),

	TP_fast_assign(
		__entry->idx = idx;
		__entry->instance = instance;
		__entry->ep_type = ep_type;
		__entry->iface = iface;
	),

	TP_printk("Client[%d]: Connection established with DFC Service "
		  "instance=%u ep_type=%u iface_id=%u",
		__entry->idx, __entry->instance,
		__entry->ep_type, __entry->iface)
);

TRACE_EVENT(dfc_client_state_down,

	TP_PROTO(int idx, int from_cb),

	TP_ARGS(idx, from_cb),

	TP_STRUCT__entry(
		__field(int, idx)
		__field(int, from_cb)
	),

	TP_fast_assign(
		__entry->idx = idx;
		__entry->from_cb = from_cb;
	),

	TP_printk("Client[%d]: Connection with DFC service lost. "
		  "Exit by callback %d",
		  __entry->idx, __entry->from_cb)
);

TRACE_EVENT(dfc_qmap_cmd,

	TP_PROTO(u8 mux_id, u8 bearer_id, u16 seq_num, u8 type, u32 tran),

	TP_ARGS(mux_id, bearer_id, seq_num, type, tran),

	TP_STRUCT__entry(
		__field(u8, mid)
		__field(u8, bid)
		__field(u16, seq)
		__field(u8, type)
		__field(u32, tran)
	),

	TP_fast_assign(
		__entry->mid = mux_id;
		__entry->bid = bearer_id;
		__entry->seq = seq_num;
		__entry->type = type;
		__entry->tran = tran;
	),

	TP_printk("mux_id=%u bearer_id=%u seq_num=%u type=%u tran=%u",
		__entry->mid, __entry->bid, __entry->seq,
		__entry->type, __entry->tran)
);

TRACE_EVENT(dfc_tx_link_status_ind,

	TP_PROTO(int src, int idx, u8 status, u8 mux_id, u8 bearer_id),

	TP_ARGS(src, idx, status, mux_id, bearer_id),

	TP_STRUCT__entry(
		__field(int, src)
		__field(int, idx)
		__field(u8, status)
		__field(u8, mid)
		__field(u8, bid)
	),

	TP_fast_assign(
		__entry->src = src;
		__entry->idx = idx;
		__entry->status = status;
		__entry->mid = mux_id;
		__entry->bid = bearer_id;
	),

	TP_printk("src=%d [%d]: status=%u mux_id=%u bearer_id=%u",
		__entry->src, __entry->idx, __entry->status,
		__entry->mid, __entry->bid)
);

#endif /* _TRACE_DFC_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
