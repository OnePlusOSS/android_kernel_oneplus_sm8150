/*
 * net/dsa/dsa.c - Hardware switch handling
 * Copyright (c) 2008-2009 Marvell Semiconductor
 * Copyright (c) 2013 Florian Fainelli <florian@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/of_gpio.h>
#include <linux/netdevice.h>
#include <linux/sysfs.h>
#include <linux/phy_fixed.h>
#include <linux/gpio/consumer.h>
#include <linux/etherdevice.h>

#include "dsa_priv.h"

static struct sk_buff *dsa_slave_notag_xmit(struct sk_buff *skb,
					    struct net_device *dev)
{
	/* Just return the original SKB */
	return skb;
}

static const struct dsa_device_ops none_ops = {
	.xmit	= dsa_slave_notag_xmit,
	.rcv	= NULL,
};

const struct dsa_device_ops *dsa_device_ops[DSA_TAG_LAST] = {
#ifdef CONFIG_NET_DSA_TAG_BRCM
	[DSA_TAG_PROTO_BRCM] = &brcm_netdev_ops,
#endif
#ifdef CONFIG_NET_DSA_TAG_DSA
	[DSA_TAG_PROTO_DSA] = &dsa_netdev_ops,
#endif
#ifdef CONFIG_NET_DSA_TAG_EDSA
	[DSA_TAG_PROTO_EDSA] = &edsa_netdev_ops,
#endif
#ifdef CONFIG_NET_DSA_TAG_KSZ
	[DSA_TAG_PROTO_KSZ] = &ksz_netdev_ops,
#endif
#ifdef CONFIG_NET_DSA_TAG_LAN9303
	[DSA_TAG_PROTO_LAN9303] = &lan9303_netdev_ops,
#endif
#ifdef CONFIG_NET_DSA_TAG_MTK
	[DSA_TAG_PROTO_MTK] = &mtk_netdev_ops,
#endif
#ifdef CONFIG_NET_DSA_TAG_QCA
	[DSA_TAG_PROTO_QCA] = &qca_netdev_ops,
#endif
#ifdef CONFIG_NET_DSA_TAG_TRAILER
	[DSA_TAG_PROTO_TRAILER] = &trailer_netdev_ops,
#endif
	[DSA_TAG_PROTO_NONE] = &none_ops,
};

int dsa_cpu_dsa_setup(struct dsa_port *port)
{
	struct device_node *port_dn = port->dn;
	struct dsa_switch *ds = port->ds;
	struct phy_device *phydev;
	int ret, mode;

	if (of_phy_is_fixed_link(port_dn)) {
		ret = of_phy_register_fixed_link(port_dn);
		if (ret) {
			dev_err(ds->dev, "failed to register fixed PHY\n");
			return ret;
		}
		phydev = of_phy_find_device(port_dn);

		mode = of_get_phy_mode(port_dn);
		if (mode < 0)
			mode = PHY_INTERFACE_MODE_NA;
		phydev->interface = mode;

		genphy_config_init(phydev);
		genphy_read_status(phydev);
		if (ds->ops->adjust_link)
			ds->ops->adjust_link(ds, port->index, phydev);

		put_device(&phydev->mdio.dev);
	}

	return 0;
}

const struct dsa_device_ops *dsa_resolve_tag_protocol(int tag_protocol)
{
	const struct dsa_device_ops *ops;

	if (tag_protocol >= DSA_TAG_LAST)
		return ERR_PTR(-EINVAL);
	ops = dsa_device_ops[tag_protocol];

	if (!ops)
		return ERR_PTR(-ENOPROTOOPT);

	return ops;
}

int dsa_cpu_port_ethtool_setup(struct dsa_port *cpu_dp)
{
	struct dsa_switch *ds = cpu_dp->ds;
	struct net_device *master;
	struct ethtool_ops *cpu_ops;

	master = cpu_dp->netdev;

	cpu_ops = devm_kzalloc(ds->dev, sizeof(*cpu_ops), GFP_KERNEL);
	if (!cpu_ops)
		return -ENOMEM;

	memcpy(&cpu_dp->ethtool_ops, master->ethtool_ops,
	       sizeof(struct ethtool_ops));
	cpu_dp->orig_ethtool_ops = master->ethtool_ops;
	memcpy(cpu_ops, &cpu_dp->ethtool_ops,
	       sizeof(struct ethtool_ops));
	dsa_cpu_port_ethtool_init(cpu_ops);
	master->ethtool_ops = cpu_ops;

	return 0;
}

void dsa_cpu_port_ethtool_restore(struct dsa_port *cpu_dp)
{
	cpu_dp->netdev->ethtool_ops = cpu_dp->orig_ethtool_ops;
}

void dsa_cpu_dsa_destroy(struct dsa_port *port)
{
	struct device_node *port_dn = port->dn;

	if (of_phy_is_fixed_link(port_dn))
		of_phy_deregister_fixed_link(port_dn);
}

static int dev_is_class(struct device *dev, void *class)
{
	if (dev->class != NULL && !strcmp(dev->class->name, class))
		return 1;

	return 0;
}

static struct device *dev_find_class(struct device *parent, char *class)
{
	if (dev_is_class(parent, class)) {
		get_device(parent);
		return parent;
	}

	return device_find_child(parent, class, dev_is_class);
}

struct net_device *dsa_dev_to_net_device(struct device *dev)
{
	struct device *d;

	d = dev_find_class(dev, "net");
	if (d != NULL) {
		struct net_device *nd;

		nd = to_net_dev(d);
		dev_hold(nd);
		put_device(d);

		return nd;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(dsa_dev_to_net_device);

static int dsa_switch_rcv(struct sk_buff *skb, struct net_device *dev,
			  struct packet_type *pt, struct net_device *unused)
{
	struct dsa_switch_tree *dst = dev->dsa_ptr;
	struct sk_buff *nskb = NULL;
	struct pcpu_sw_netstats *s;
	struct dsa_slave_priv *p;

	if (unlikely(dst == NULL)) {
		kfree_skb(skb);
		return 0;
	}

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb)
		return 0;

	nskb = dst->rcv(skb, dev, pt);
	if (!nskb) {
		kfree_skb(skb);
		return 0;
	}

	skb = nskb;
	p = netdev_priv(skb->dev);
	skb_push(skb, ETH_HLEN);
	skb->pkt_type = PACKET_HOST;
	skb->protocol = eth_type_trans(skb, skb->dev);

	s = this_cpu_ptr(p->stats64);
	u64_stats_update_begin(&s->syncp);
	s->rx_packets++;
	s->rx_bytes += skb->len;
	u64_stats_update_end(&s->syncp);

	netif_receive_skb(skb);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static bool dsa_is_port_initialized(struct dsa_switch *ds, int p)
{
	return ds->enabled_port_mask & (1 << p) && ds->ports[p].netdev;
}

int dsa_switch_suspend(struct dsa_switch *ds)
{
	int i, ret = 0;

	/* Suspend slave network devices */
	for (i = 0; i < ds->num_ports; i++) {
		if (!dsa_is_port_initialized(ds, i))
			continue;

		ret = dsa_slave_suspend(ds->ports[i].netdev);
		if (ret)
			return ret;
	}

	if (ds->ops->suspend)
		ret = ds->ops->suspend(ds);

	return ret;
}
EXPORT_SYMBOL_GPL(dsa_switch_suspend);

int dsa_switch_resume(struct dsa_switch *ds)
{
	int i, ret = 0;

	if (ds->ops->resume)
		ret = ds->ops->resume(ds);

	if (ret)
		return ret;

	/* Resume slave network devices */
	for (i = 0; i < ds->num_ports; i++) {
		if (!dsa_is_port_initialized(ds, i))
			continue;

		ret = dsa_slave_resume(ds->ports[i].netdev);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(dsa_switch_resume);
#endif

static struct packet_type dsa_pack_type __read_mostly = {
	.type	= cpu_to_be16(ETH_P_XDSA),
	.func	= dsa_switch_rcv,
};

static struct workqueue_struct *dsa_owq;

bool dsa_schedule_work(struct work_struct *work)
{
	return queue_work(dsa_owq, work);
}

static int __init dsa_init_module(void)
{
	int rc;

	dsa_owq = alloc_ordered_workqueue("dsa_ordered",
					  WQ_MEM_RECLAIM);
	if (!dsa_owq)
		return -ENOMEM;

	rc = dsa_slave_register_notifier();
	if (rc)
		goto register_notifier_fail;

	rc = dsa_legacy_register();
	if (rc)
		goto legacy_register_fail;

	dev_add_pack(&dsa_pack_type);

	return 0;

legacy_register_fail:
	dsa_slave_unregister_notifier();
register_notifier_fail:
	destroy_workqueue(dsa_owq);

	return rc;
}
module_init(dsa_init_module);

static void __exit dsa_cleanup_module(void)
{
	dsa_slave_unregister_notifier();
	dev_remove_pack(&dsa_pack_type);
	dsa_legacy_unregister();
	destroy_workqueue(dsa_owq);
}
module_exit(dsa_cleanup_module);

MODULE_AUTHOR("Lennert Buytenhek <buytenh@wantstofly.org>");
MODULE_DESCRIPTION("Driver for Distributed Switch Architecture switch chips");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dsa");
