/* Copyright (c) 2013-2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * RMNET configuration engine
 *
 */

#include <net/sock.h>
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/netdevice.h>
#include "rmnet_config.h"
#include "rmnet_handlers.h"
#include "rmnet_vnd.h"
#include "rmnet_private.h"
#include "rmnet_map.h"
#include "rmnet_descriptor.h"
#include <soc/qcom/rmnet_qmi.h>
#include <soc/qcom/qmi_rmnet.h>

/* Locking scheme -
 * The shared resource which needs to be protected is realdev->rx_handler_data.
 * For the writer path, this is using rtnl_lock(). The writer paths are
 * rmnet_newlink(), rmnet_dellink() and rmnet_force_unassociate_device(). These
 * paths are already called with rtnl_lock() acquired in. There is also an
 * ASSERT_RTNL() to ensure that we are calling with rtnl acquired. For
 * dereference here, we will need to use rtnl_dereference(). Dev list writing
 * needs to happen with rtnl_lock() acquired for netdev_master_upper_dev_link().
 * For the reader path, the real_dev->rx_handler_data is called in the TX / RX
 * path. We only need rcu_read_lock() for these scenarios. In these cases,
 * the rcu_read_lock() is held in __dev_queue_xmit() and
 * netif_receive_skb_internal(), so readers need to use rcu_dereference_rtnl()
 * to get the relevant information. For dev list reading, we again acquire
 * rcu_read_lock() in rmnet_dellink() for netdev_master_upper_dev_get_rcu().
 * We also use unregister_netdevice_many() to free all rmnet devices in
 * rmnet_force_unassociate_device() so we dont lose the rtnl_lock() and free in
 * same context.
 */

/* Local Definitions and Declarations */

enum {
	IFLA_RMNET_DFC_QOS = __IFLA_RMNET_MAX,
	IFLA_RMNET_UL_AGG_PARAMS,
	__IFLA_RMNET_EXT_MAX,
};

static const struct nla_policy rmnet_policy[__IFLA_RMNET_EXT_MAX] = {
	[IFLA_RMNET_MUX_ID] = {
		.type = NLA_U16
	},
	[IFLA_RMNET_FLAGS] = {
		.len = sizeof(struct ifla_rmnet_flags)
	},
	[IFLA_RMNET_DFC_QOS] = {
		.len = sizeof(struct tcmsg)
	},
	[IFLA_RMNET_UL_AGG_PARAMS] = {
		.len = sizeof(struct rmnet_egress_agg_params)
	},
};

int rmnet_is_real_dev_registered(const struct net_device *real_dev)
{
	return rcu_access_pointer(real_dev->rx_handler) == rmnet_rx_handler;
}
EXPORT_SYMBOL(rmnet_is_real_dev_registered);

/* Needs rtnl lock */
static struct rmnet_port*
rmnet_get_port_rtnl(const struct net_device *real_dev)
{
	return rtnl_dereference(real_dev->rx_handler_data);
}

static int rmnet_unregister_real_device(struct net_device *real_dev,
					struct rmnet_port *port)
{
	if (port->nr_rmnet_devs)
		return -EINVAL;

<<<<<<< HEAD
	rmnet_map_cmd_exit(port);
	rmnet_map_tx_aggregate_exit(port);

	rmnet_descriptor_deinit(port);

	kfree(port);

=======
>>>>>>> v4.14.158
	netdev_rx_handler_unregister(real_dev);

	kfree(port);

	/* release reference on real_dev */
	dev_put(real_dev);

	netdev_dbg(real_dev, "Removed from rmnet\n");
	return 0;
}

static int rmnet_register_real_device(struct net_device *real_dev)
{
	struct rmnet_port *port;
	int rc, entry;

	ASSERT_RTNL();

	if (rmnet_is_real_dev_registered(real_dev))
		return 0;

	port = kzalloc(sizeof(*port), GFP_ATOMIC);
	if (!port)
		return -ENOMEM;

	port->dev = real_dev;
	rc = netdev_rx_handler_register(real_dev, rmnet_rx_handler, port);
	if (rc) {
		kfree(port);
		return -EBUSY;
	}
	/* hold on to real dev for MAP data */
	dev_hold(real_dev);

	for (entry = 0; entry < RMNET_MAX_LOGICAL_EP; entry++)
		INIT_HLIST_HEAD(&port->muxed_ep[entry]);

	rc = rmnet_descriptor_init(port);
	if (rc) {
		rmnet_descriptor_deinit(port);
		return rc;
	}

	rmnet_map_tx_aggregate_init(port);
	rmnet_map_cmd_init(port);

	netdev_dbg(real_dev, "registered with rmnet\n");
	return 0;
}

static void rmnet_unregister_bridge(struct net_device *dev,
				    struct rmnet_port *port)
{
	struct rmnet_port *bridge_port;
	struct net_device *bridge_dev;

	if (port->rmnet_mode != RMNET_EPMODE_BRIDGE)
		return;

	/* bridge slave handling */
	if (!port->nr_rmnet_devs) {
		bridge_dev = port->bridge_ep;

		bridge_port = rmnet_get_port_rtnl(bridge_dev);
		bridge_port->bridge_ep = NULL;
		bridge_port->rmnet_mode = RMNET_EPMODE_VND;
	} else {
		bridge_dev = port->bridge_ep;

		bridge_port = rmnet_get_port_rtnl(bridge_dev);
		rmnet_unregister_real_device(bridge_dev, bridge_port);
	}
}

static int rmnet_newlink(struct net *src_net, struct net_device *dev,
			 struct nlattr *tb[], struct nlattr *data[],
			 struct netlink_ext_ack *extack)
{
	u32 data_format = RMNET_FLAGS_INGRESS_DEAGGREGATION;
	struct net_device *real_dev;
	int mode = RMNET_EPMODE_VND;
	struct rmnet_endpoint *ep;
	struct rmnet_port *port;
	int err = 0;
	u16 mux_id;

	real_dev = __dev_get_by_index(src_net, nla_get_u32(tb[IFLA_LINK]));
	if (!real_dev || !dev)
		return -ENODEV;

	if (!data[IFLA_RMNET_MUX_ID])
		return -EINVAL;

	ep = kzalloc(sizeof(*ep), GFP_ATOMIC);
	if (!ep)
		return -ENOMEM;

	mux_id = nla_get_u16(data[IFLA_RMNET_MUX_ID]);

	err = rmnet_register_real_device(real_dev);
	if (err)
		goto err0;

	port = rmnet_get_port_rtnl(real_dev);
	err = rmnet_vnd_newlink(mux_id, dev, port, real_dev, ep);
	if (err)
		goto err1;

	port->rmnet_mode = mode;

	hlist_add_head_rcu(&ep->hlnode, &port->muxed_ep[mux_id]);

	if (data[IFLA_RMNET_FLAGS]) {
		struct ifla_rmnet_flags *flags;

		flags = nla_data(data[IFLA_RMNET_FLAGS]);
		data_format = flags->flags & flags->mask;
	}

	netdev_dbg(dev, "data format [0x%08X]\n", data_format);
	port->data_format = data_format;

	if (data[IFLA_RMNET_UL_AGG_PARAMS]) {
		void *agg_params;
		unsigned long irq_flags;

		agg_params = nla_data(data[IFLA_RMNET_UL_AGG_PARAMS]);
		spin_lock_irqsave(&port->agg_lock, irq_flags);
		memcpy(&port->egress_agg_params, agg_params,
		       sizeof(port->egress_agg_params));
		spin_unlock_irqrestore(&port->agg_lock, irq_flags);
	}

	return 0;

err1:
	rmnet_unregister_real_device(real_dev, port);
err0:
	kfree(ep);
	return err;
}

static void rmnet_dellink(struct net_device *dev, struct list_head *head)
{
	struct rmnet_priv *priv = netdev_priv(dev);
	struct net_device *real_dev;
	struct rmnet_endpoint *ep;
	struct rmnet_port *port;
	u8 mux_id;

	real_dev = priv->real_dev;

	if (!real_dev || !rmnet_is_real_dev_registered(real_dev))
		return;

	port = rmnet_get_port_rtnl(real_dev);

	mux_id = rmnet_vnd_get_mux(dev);

	ep = rmnet_get_endpoint(port, mux_id);
	if (ep) {
		hlist_del_init_rcu(&ep->hlnode);
		rmnet_unregister_bridge(dev, port);
		rmnet_vnd_dellink(mux_id, port, ep);
		synchronize_rcu();
		kfree(ep);
	}

	if (!port->nr_rmnet_devs)
		qmi_rmnet_qmi_exit(port->qmi_info, port);

	rmnet_unregister_real_device(real_dev, port);

	unregister_netdevice_queue(dev, head);
}

static void rmnet_force_unassociate_device(struct net_device *dev)
{
	struct net_device *real_dev = dev;
	struct hlist_node *tmp_ep;
	struct rmnet_endpoint *ep;
	struct rmnet_port *port;
	unsigned long bkt_ep;
	LIST_HEAD(list);

	if (!rmnet_is_real_dev_registered(real_dev))
		return;

	ASSERT_RTNL();

	port = rmnet_get_port_rtnl(dev);
	qmi_rmnet_qmi_exit(port->qmi_info, port);

	rmnet_unregister_bridge(dev, port);

	hash_for_each_safe(port->muxed_ep, bkt_ep, tmp_ep, ep, hlnode) {
		unregister_netdevice_queue(ep->egress_dev, &list);
		rmnet_vnd_dellink(ep->mux_id, port, ep);

		hlist_del_init_rcu(&ep->hlnode);
		synchronize_rcu();
		kfree(ep);
	}

	unregister_netdevice_many(&list);

	rmnet_unregister_real_device(real_dev, port);
}

static int rmnet_config_notify_cb(struct notifier_block *nb,
				  unsigned long event, void *data)
{
	struct net_device *dev = netdev_notifier_info_to_dev(data);

	if (!dev)
		return NOTIFY_DONE;

	switch (event) {
	case NETDEV_UNREGISTER:
		netdev_dbg(dev, "Kernel unregister\n");
		rmnet_force_unassociate_device(dev);
		break;

	default:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block rmnet_dev_notifier __read_mostly = {
	.notifier_call = rmnet_config_notify_cb,
};

static int rmnet_rtnl_validate(struct nlattr *tb[], struct nlattr *data[],
			       struct netlink_ext_ack *extack)
{
	struct rmnet_egress_agg_params *agg_params;
	u16 mux_id;

	if (!data) {
		return -EINVAL;
	} else {
		if (data[IFLA_RMNET_MUX_ID]) {
			mux_id = nla_get_u16(data[IFLA_RMNET_MUX_ID]);
			if (mux_id > (RMNET_MAX_LOGICAL_EP - 1))
				return -ERANGE;
		}

		if (data[IFLA_RMNET_UL_AGG_PARAMS]) {
			agg_params = nla_data(data[IFLA_RMNET_UL_AGG_PARAMS]);
			if (agg_params->agg_time < 3000000)
				return -EINVAL;
		}
	}

	return 0;
}

static int rmnet_changelink(struct net_device *dev, struct nlattr *tb[],
			    struct nlattr *data[],
			    struct netlink_ext_ack *extack)
{
	struct rmnet_priv *priv = netdev_priv(dev);
	struct net_device *real_dev;
	struct rmnet_endpoint *ep;
	struct rmnet_port *port;
	u16 mux_id;

	real_dev = __dev_get_by_index(dev_net(dev),
				      nla_get_u32(tb[IFLA_LINK]));

	if (!real_dev || !dev || !rmnet_is_real_dev_registered(real_dev))
		return -ENODEV;

	port = rmnet_get_port_rtnl(real_dev);

	if (data[IFLA_RMNET_MUX_ID]) {
		mux_id = nla_get_u16(data[IFLA_RMNET_MUX_ID]);
		ep = rmnet_get_endpoint(port, priv->mux_id);
		if (!ep)
			return -ENODEV;

		hlist_del_init_rcu(&ep->hlnode);
		hlist_add_head_rcu(&ep->hlnode, &port->muxed_ep[mux_id]);

		ep->mux_id = mux_id;
		priv->mux_id = mux_id;
	}

	if (data[IFLA_RMNET_FLAGS]) {
		struct ifla_rmnet_flags *flags;

		flags = nla_data(data[IFLA_RMNET_FLAGS]);
		port->data_format = flags->flags & flags->mask;
	}

	if (data[IFLA_RMNET_DFC_QOS]) {
		struct tcmsg *tcm;

		tcm = nla_data(data[IFLA_RMNET_DFC_QOS]);
		qmi_rmnet_change_link(dev, port, tcm);
	}

	if (data[IFLA_RMNET_UL_AGG_PARAMS]) {
		void *agg_params;
		unsigned long irq_flags;

		agg_params = nla_data(data[IFLA_RMNET_UL_AGG_PARAMS]);
		spin_lock_irqsave(&port->agg_lock, irq_flags);
		memcpy(&port->egress_agg_params, agg_params,
		       sizeof(port->egress_agg_params));
		spin_unlock_irqrestore(&port->agg_lock, irq_flags);
	}

	return 0;
}

static size_t rmnet_get_size(const struct net_device *dev)
{
	return
		/* IFLA_RMNET_MUX_ID */
		nla_total_size(2) +
		/* IFLA_RMNET_FLAGS */
		nla_total_size(sizeof(struct ifla_rmnet_flags)) +
		/* IFLA_RMNET_DFC_QOS */
		nla_total_size(sizeof(struct tcmsg)) +
		/* IFLA_RMNET_UL_AGG_PARAMS */
		nla_total_size(sizeof(struct rmnet_egress_agg_params));
}

static int rmnet_fill_info(struct sk_buff *skb, const struct net_device *dev)
{
	struct rmnet_priv *priv = netdev_priv(dev);
	struct net_device *real_dev;
	struct ifla_rmnet_flags f;
	struct rmnet_port *port = NULL;

	real_dev = priv->real_dev;

	if (nla_put_u16(skb, IFLA_RMNET_MUX_ID, priv->mux_id))
		goto nla_put_failure;

	if (rmnet_is_real_dev_registered(real_dev)) {
		port = rmnet_get_port_rtnl(real_dev);
		f.flags = port->data_format;
	} else {
		f.flags = 0;
	}

	f.mask  = ~0;

	if (nla_put(skb, IFLA_RMNET_FLAGS, sizeof(f), &f))
		goto nla_put_failure;

	if (port) {
		if (nla_put(skb, IFLA_RMNET_UL_AGG_PARAMS,
			    sizeof(port->egress_agg_params),
			    &port->egress_agg_params))
			goto nla_put_failure;
	}

	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

struct rtnl_link_ops rmnet_link_ops __read_mostly = {
	.kind		= "rmnet",
	.maxtype	= __IFLA_RMNET_EXT_MAX,
	.priv_size	= sizeof(struct rmnet_priv),
	.setup		= rmnet_vnd_setup,
	.validate	= rmnet_rtnl_validate,
	.newlink	= rmnet_newlink,
	.dellink	= rmnet_dellink,
	.get_size	= rmnet_get_size,
	.changelink     = rmnet_changelink,
	.policy		= rmnet_policy,
	.fill_info	= rmnet_fill_info,
};

/* Needs either rcu_read_lock() or rtnl lock */
struct rmnet_port *rmnet_get_port(struct net_device *real_dev)
{
	if (rmnet_is_real_dev_registered(real_dev))
		return rcu_dereference_rtnl(real_dev->rx_handler_data);
	else
		return NULL;
}
EXPORT_SYMBOL(rmnet_get_port);

struct rmnet_endpoint *rmnet_get_endpoint(struct rmnet_port *port, u8 mux_id)
{
	struct rmnet_endpoint *ep;

	hlist_for_each_entry_rcu(ep, &port->muxed_ep[mux_id], hlnode) {
		if (ep->mux_id == mux_id)
			return ep;
	}

	return NULL;
}
EXPORT_SYMBOL(rmnet_get_endpoint);

int rmnet_add_bridge(struct net_device *rmnet_dev,
		     struct net_device *slave_dev)
{
	struct rmnet_priv *priv = netdev_priv(rmnet_dev);
	struct net_device *real_dev = priv->real_dev;
	struct rmnet_port *port, *slave_port;
	int err;

	port = rmnet_get_port(real_dev);

	/* If there is more than one rmnet dev attached, its probably being
	 * used for muxing. Skip the briding in that case
	 */
	if (port->nr_rmnet_devs > 1)
		return -EINVAL;

	if (rmnet_is_real_dev_registered(slave_dev))
		return -EBUSY;

	err = rmnet_register_real_device(slave_dev);
	if (err)
		return -EBUSY;

	slave_port = rmnet_get_port(slave_dev);
	slave_port->rmnet_mode = RMNET_EPMODE_BRIDGE;
	slave_port->bridge_ep = real_dev;

	port->rmnet_mode = RMNET_EPMODE_BRIDGE;
	port->bridge_ep = slave_dev;

	netdev_dbg(slave_dev, "registered with rmnet as slave\n");
	return 0;
}

int rmnet_del_bridge(struct net_device *rmnet_dev,
		     struct net_device *slave_dev)
{
	struct rmnet_priv *priv = netdev_priv(rmnet_dev);
	struct net_device *real_dev = priv->real_dev;
	struct rmnet_port *port, *slave_port;

	port = rmnet_get_port(real_dev);
	port->rmnet_mode = RMNET_EPMODE_VND;
	port->bridge_ep = NULL;

	slave_port = rmnet_get_port(slave_dev);
	rmnet_unregister_real_device(slave_dev, slave_port);

	netdev_dbg(slave_dev, "removed from rmnet as slave\n");
	return 0;
}

#ifdef CONFIG_QCOM_QMI_RMNET
void *rmnet_get_qmi_pt(void *port)
{
	if (port)
		return ((struct rmnet_port *)port)->qmi_info;

	return NULL;
}
EXPORT_SYMBOL(rmnet_get_qmi_pt);

void *rmnet_get_qos_pt(struct net_device *dev)
{
	if (dev)
		return rcu_dereference(
			((struct rmnet_priv *)netdev_priv(dev))->qos_info);

	return NULL;
}
EXPORT_SYMBOL(rmnet_get_qos_pt);

void *rmnet_get_rmnet_port(struct net_device *dev)
{
	struct rmnet_priv *priv;

	if (dev) {
		priv = netdev_priv(dev);
		return (void *)rmnet_get_port(priv->real_dev);
	}

	return NULL;
}
EXPORT_SYMBOL(rmnet_get_rmnet_port);

struct net_device *rmnet_get_rmnet_dev(void *port, u8 mux_id)
{
	struct rmnet_endpoint *ep;

	if (port) {
		ep = rmnet_get_endpoint((struct rmnet_port *)port, mux_id);
		if (ep)
			return ep->egress_dev;
	}

	return NULL;
}
EXPORT_SYMBOL(rmnet_get_rmnet_dev);

void rmnet_reset_qmi_pt(void *port)
{
	if (port)
		((struct rmnet_port *)port)->qmi_info = NULL;
}
EXPORT_SYMBOL(rmnet_reset_qmi_pt);

void rmnet_init_qmi_pt(void *port, void *qmi)
{
	if (port)
		((struct rmnet_port *)port)->qmi_info = qmi;
}
EXPORT_SYMBOL(rmnet_init_qmi_pt);

void rmnet_get_packets(void *port, u64 *rx, u64 *tx)
{
	struct rmnet_priv *priv;
	struct rmnet_pcpu_stats *ps;
	unsigned int cpu, start;

	struct rmnet_endpoint *ep;
	unsigned long bkt;

	if (!port || !tx || !rx)
		return;

	*tx = 0;
	*rx = 0;
	rcu_read_lock();
	hash_for_each(((struct rmnet_port *)port)->muxed_ep, bkt, ep, hlnode) {
		priv = netdev_priv(ep->egress_dev);
		for_each_possible_cpu(cpu) {
			ps = per_cpu_ptr(priv->pcpu_stats, cpu);
			do {
				start = u64_stats_fetch_begin_irq(&ps->syncp);
				*tx += ps->stats.tx_pkts;
				*rx += ps->stats.rx_pkts;
			} while (u64_stats_fetch_retry_irq(&ps->syncp, start));
		}
	}
	rcu_read_unlock();
}
EXPORT_SYMBOL(rmnet_get_packets);

void  rmnet_set_powersave_format(void *port)
{
	if (!port)
		return;
	((struct rmnet_port *)port)->data_format |= RMNET_INGRESS_FORMAT_PS;
}
EXPORT_SYMBOL(rmnet_set_powersave_format);

void  rmnet_clear_powersave_format(void *port)
{
	if (!port)
		return;
	((struct rmnet_port *)port)->data_format &= ~RMNET_INGRESS_FORMAT_PS;
}
EXPORT_SYMBOL(rmnet_clear_powersave_format);

void rmnet_enable_all_flows(void *port)
{
	struct rmnet_endpoint *ep;
	unsigned long bkt;

	if (unlikely(!port))
		return;

	rcu_read_lock();
	hash_for_each_rcu(((struct rmnet_port *)port)->muxed_ep,
			  bkt, ep, hlnode) {
		qmi_rmnet_enable_all_flows(ep->egress_dev);
	}
	rcu_read_unlock();
}
EXPORT_SYMBOL(rmnet_enable_all_flows);

bool rmnet_all_flows_enabled(void *port)
{
	struct rmnet_endpoint *ep;
	unsigned long bkt;
	bool ret = true;

	if (unlikely(!port))
		return true;

	rcu_read_lock();
	hash_for_each_rcu(((struct rmnet_port *)port)->muxed_ep,
			  bkt, ep, hlnode) {
		if (!qmi_rmnet_all_flows_enabled(ep->egress_dev)) {
			ret = false;
			goto out;
		}
	}
out:
	rcu_read_unlock();

	return ret;
}
EXPORT_SYMBOL(rmnet_all_flows_enabled);

int rmnet_get_powersave_notif(void *port)
{
	if (!port)
		return 0;
	return ((struct rmnet_port *)port)->data_format & RMNET_FORMAT_PS_NOTIF;
}
EXPORT_SYMBOL(rmnet_get_powersave_notif);
#endif

/* Startup/Shutdown */

static int __init rmnet_init(void)
{
	int rc;

	rc = register_netdevice_notifier(&rmnet_dev_notifier);
	if (rc != 0)
		return rc;

	rc = rtnl_link_register(&rmnet_link_ops);
	if (rc != 0) {
		unregister_netdevice_notifier(&rmnet_dev_notifier);
		return rc;
	}
	return rc;
}

static void __exit rmnet_exit(void)
{
	unregister_netdevice_notifier(&rmnet_dev_notifier);
	rtnl_link_unregister(&rmnet_link_ops);
}

module_init(rmnet_init)
module_exit(rmnet_exit)
MODULE_LICENSE("GPL v2");
