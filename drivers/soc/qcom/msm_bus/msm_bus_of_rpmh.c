/* Copyright (c) 2014-2018, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "AXI: %s(): " fmt, __func__

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#include <linux/msm_bus_rules.h>
#include "msm_bus_core.h"
#include "msm_bus_rpmh.h"

#define DEFAULT_QOS_FREQ	19200
#define DEFAULT_UTIL_FACT	100
#define DEFAULT_VRAIL_COMP	100
#define DEFAULT_AGG_SCHEME	AGG_SCHEME_LEG

static int *get_arr(struct platform_device *pdev,
		struct device_node *node, const char *prop,
		int *nports)
{
	int size = 0, ret;
	int *arr = NULL;

	if (of_get_property(node, prop, &size)) {
		*nports = size / sizeof(int);
	} else {
		dev_dbg(&pdev->dev, "Property %s not available\n", prop);
		*nports = 0;
		return NULL;
	}

	arr = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(arr)) {
		dev_err(&pdev->dev, "Error: Failed to alloc mem for %s\n",
				prop);
		return NULL;
	}

	ret = of_property_read_u32_array(node, prop, (u32 *)arr, *nports);
	if (ret) {
		dev_err(&pdev->dev, "Error in reading property: %s\n", prop);
		goto arr_err;
	}

	return arr;
arr_err:
	devm_kfree(&pdev->dev, arr);
	return NULL;
}

static struct msm_bus_rsc_device_type *get_rsc_device_info(
		struct device_node *dev_node,
		struct platform_device *pdev)
{
	struct msm_bus_rsc_device_type *rsc_dev;
	int ret;

	rsc_dev = devm_kzalloc(&pdev->dev,
			sizeof(struct msm_bus_rsc_device_type),
			GFP_KERNEL);
	if (!rsc_dev) {
		dev_err(&pdev->dev,
			"Error: Unable to allocate memory for rsc_dev\n");
		goto rsc_dev_err;
	}

	ret = of_property_read_u32(dev_node, "qcom,req_state",
			&rsc_dev->req_state);
	if (ret) {
		dev_dbg(&pdev->dev, "req_state missing, using default\n");
		rsc_dev->req_state = 2;
	}

	return rsc_dev;

rsc_dev_err:
	devm_kfree(&pdev->dev, rsc_dev);
	rsc_dev = 0;
	return NULL;
}

static struct msm_bus_bcm_device_type *get_bcm_device_info(
		struct device_node *dev_node,
		struct platform_device *pdev)
{
	struct msm_bus_bcm_device_type *bcm_dev;
	unsigned int ret;

	bcm_dev = devm_kzalloc(&pdev->dev,
			sizeof(struct msm_bus_bcm_device_type),
			GFP_KERNEL);
	if (!bcm_dev) {
		dev_err(&pdev->dev,
			"Error: Unable to allocate memory for bcm_dev\n");
		goto bcm_dev_err;
	}

	ret = of_property_read_string(dev_node, "qcom,bcm-name",
							&bcm_dev->name);
	if (ret) {
		dev_warn(&pdev->dev, "Bcm node is missing name\n");
		goto bcm_dev_err;
	}

	return bcm_dev;

bcm_dev_err:
	devm_kfree(&pdev->dev, bcm_dev);
	bcm_dev = 0;
	return NULL;
}

static struct msm_bus_fab_device_type *get_fab_device_info(
		struct device_node *dev_node,
		struct platform_device *pdev)
{
	struct msm_bus_fab_device_type *fab_dev;
	unsigned int ret;
	struct resource *res;
	const char *base_name;

	fab_dev = devm_kzalloc(&pdev->dev,
			sizeof(struct msm_bus_fab_device_type),
			GFP_KERNEL);
	if (!fab_dev) {
		dev_err(&pdev->dev,
			"Error: Unable to allocate memory for fab_dev\n");
		return NULL;
	}

	ret = of_property_read_string(dev_node, "qcom,base-name", &base_name);
	if (ret) {
		dev_err(&pdev->dev, "Error: Unable to get base address name\n");
		goto fab_dev_err;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, base_name);
	if (!res) {
		dev_err(&pdev->dev, "Error getting qos base addr %s\n",
								base_name);
		goto fab_dev_err;
	}
	fab_dev->pqos_base = res->start;
	fab_dev->qos_range = resource_size(res);
	fab_dev->bypass_qos_prg = of_property_read_bool(dev_node,
						"qcom,bypass-qos-prg");

	ret = of_property_read_u32(dev_node, "qcom,base-offset",
			&fab_dev->base_offset);
	if (ret)
		dev_dbg(&pdev->dev, "Bus base offset is missing\n");

	ret = of_property_read_u32(dev_node, "qcom,sbm-offset",
			&fab_dev->sbm_offset);
	if (ret)
		dev_dbg(&pdev->dev, "sbm disable offset is missing\n");

	ret = of_property_read_u32(dev_node, "qcom,qos-off",
			&fab_dev->qos_off);
	if (ret)
		dev_dbg(&pdev->dev, "Bus qos off is missing\n");


	ret = of_property_read_u32(dev_node, "qcom,bus-type",
						&fab_dev->bus_type);
	if (ret)
		dev_warn(&pdev->dev, "Bus type is missing\n");

	ret = of_property_read_u32(dev_node, "qcom,qos-freq",
						&fab_dev->qos_freq);
	if (ret) {
		dev_dbg(&pdev->dev, "Bus qos freq is missing\n");
		fab_dev->qos_freq = DEFAULT_QOS_FREQ;
	}

	return fab_dev;

fab_dev_err:
	devm_kfree(&pdev->dev, fab_dev);
	fab_dev = 0;
	return NULL;
}

static void get_qos_params(
		struct device_node * const dev_node,
		struct platform_device * const pdev,
		struct msm_bus_node_info_type *node_info)
{
	const uint32_t *vec_arr = NULL;
	int len;

	of_property_read_u32(dev_node, "qcom,prio",
					&node_info->qos_params.prio_dflt);

	vec_arr = of_get_property(dev_node, "qcom,lim-params", &len);
	if (vec_arr != NULL && len == sizeof(uint32_t) * 2) {
		node_info->qos_params.limiter.bw = be32_to_cpu(vec_arr[0]);
		node_info->qos_params.limiter.sat = be32_to_cpu(vec_arr[1]);
	} else {
		node_info->qos_params.limiter.bw = 0;
		node_info->qos_params.limiter.sat = 0;
	}

	node_info->qos_params.limiter_en = of_property_read_bool(dev_node,
						"qcom,lim-en");

	vec_arr = of_get_property(dev_node, "qcom,qos-reg-params", &len);
	if (vec_arr != NULL && len == sizeof(uint32_t) * 4) {
		node_info->qos_params.reg.low_prio = be32_to_cpu(vec_arr[0]);
		node_info->qos_params.reg.hi_prio = be32_to_cpu(vec_arr[1]);
		node_info->qos_params.reg.bw = be32_to_cpu(vec_arr[2]);
		node_info->qos_params.reg.sat = be32_to_cpu(vec_arr[3]);
	} else {
		node_info->qos_params.reg.low_prio = 0;
		node_info->qos_params.reg.hi_prio = 0;
		node_info->qos_params.reg.bw = 0;
		node_info->qos_params.reg.sat = 0;
	}

	vec_arr = of_get_property(dev_node, "qcom,qos-reg-mode", &len);
	if (vec_arr != NULL && len == sizeof(uint32_t) * 2) {
		node_info->qos_params.reg_mode.read = be32_to_cpu(vec_arr[0]);
		node_info->qos_params.reg_mode.write = be32_to_cpu(vec_arr[1]);
	} else {
		node_info->qos_params.reg_mode.read = 0;
		node_info->qos_params.reg_mode.write = 0;
	}

	node_info->qos_params.urg_fwd_en = of_property_read_bool(dev_node,
						"qcom,forwarding");

	node_info->qos_params.defer_init_qos = of_property_read_bool(dev_node,
						"qcom,defer-init-qos");
}

static int msm_bus_of_parse_clk_array(struct device_node *dev_node,
			struct device_node *gdsc_node,
			struct platform_device *pdev, struct nodeclk **clk_arr,
			int *num_clks, int id)
{
	int ret = 0;
	int idx = 0;
	struct property *prop;
	const char *clk_name;
	int clks = 0;

	clks = of_property_count_strings(dev_node, "clock-names");
	if (clks < 0) {
		dev_err(&pdev->dev, "No qos clks node %d\n", id);
		ret = clks;
		goto exit_of_parse_clk_array;
	}

	*num_clks = clks;
	*clk_arr = devm_kzalloc(&pdev->dev,
			(clks * sizeof(struct nodeclk)), GFP_KERNEL);

	if (!(*clk_arr)) {
		dev_err(&pdev->dev, "Error allocating clk nodes for %d\n", id);
		ret = -ENOMEM;
		*num_clks = 0;
		goto exit_of_parse_clk_array;
	}

	of_property_for_each_string(dev_node, "clock-names", prop, clk_name) {
		char gdsc_string[MAX_REG_NAME];

		(*clk_arr)[idx].clk = of_clk_get_by_name(dev_node, clk_name);
		if (IS_ERR_OR_NULL((*clk_arr)[idx].clk))
			goto exit_of_parse_clk_array;

		if (strnstr(clk_name, "no-rate", strlen(clk_name)))
			(*clk_arr)[idx].enable_only_clk = true;

		scnprintf(gdsc_string, MAX_REG_NAME, "%s-supply", clk_name);

		if (of_find_property(gdsc_node, gdsc_string, NULL))
			scnprintf((*clk_arr)[idx].reg_name,
				MAX_REG_NAME, "%s", clk_name);
		else
			scnprintf((*clk_arr)[idx].reg_name,
					MAX_REG_NAME, "%c", '\0');

		idx++;
	}
exit_of_parse_clk_array:
	return ret;
}

static void get_agg_params(
		struct device_node * const dev_node,
		struct platform_device * const pdev,
		struct msm_bus_node_info_type *node_info)
{
	int ret;


	ret = of_property_read_u32(dev_node, "qcom,buswidth",
					&node_info->agg_params.buswidth);
	if (ret) {
		dev_dbg(&pdev->dev, "Using default 8 bytes %d", node_info->id);
		node_info->agg_params.buswidth = 8;
	}

	ret = of_property_read_u32(dev_node, "qcom,agg-ports",
				   &node_info->agg_params.num_aggports);
	if (ret)
		node_info->agg_params.num_aggports = node_info->num_qports;

	ret = of_property_read_u32(dev_node, "qcom,agg-scheme",
					&node_info->agg_params.agg_scheme);
	if (ret) {
		if (node_info->is_fab_dev)
			node_info->agg_params.agg_scheme = DEFAULT_AGG_SCHEME;
		else
			node_info->agg_params.agg_scheme = AGG_SCHEME_NONE;
	}

	ret = of_property_read_u32(dev_node, "qcom,vrail-comp",
					&node_info->agg_params.vrail_comp);
	if (ret) {
		if (node_info->is_fab_dev)
			node_info->agg_params.vrail_comp = DEFAULT_VRAIL_COMP;
		else
			node_info->agg_params.vrail_comp = 0;
	}
}



static struct msm_bus_node_info_type *get_node_info_data(
		struct device_node * const dev_node,
		struct platform_device * const pdev)
{
	struct msm_bus_node_info_type *node_info;
	unsigned int ret;
	int size;
	int i;
	struct device_node *con_node;
	struct device_node *bus_dev;
	struct device_node *bcm_dev;
	struct device_node *rsc_dev;

	node_info = devm_kzalloc(&pdev->dev,
			sizeof(struct msm_bus_node_info_type),
			GFP_KERNEL);
	if (!node_info) {
		dev_err(&pdev->dev,
			"Error: Unable to allocate memory for node_info\n");
		return NULL;
	}

	ret = of_property_read_u32(dev_node, "cell-id", &node_info->id);
	if (ret) {
		dev_warn(&pdev->dev, "Bus node is missing cell-id\n");
		goto node_info_err;
	}
	ret = of_property_read_string(dev_node, "label", &node_info->name);
	if (ret) {
		dev_warn(&pdev->dev, "Bus node is missing name\n");
		goto node_info_err;
	}
	node_info->qport = get_arr(pdev, dev_node, "qcom,qport",
			&node_info->num_qports);

	node_info->num_disable_ports = of_property_count_elems_of_size(dev_node,
			 "qcom,disable-ports", sizeof(uint32_t));

	if (node_info->num_disable_ports < 0) {
		node_info->num_disable_ports = 0;
		dev_dbg(&pdev->dev, "no disable ports\n");
	}

	if (node_info->num_disable_ports) {
		node_info->disable_ports = devm_kcalloc(&pdev->dev,
			node_info->num_disable_ports, sizeof(uint32_t),
							GFP_KERNEL);
		if (!node_info->disable_ports)
			return NULL;
		ret = of_property_read_u32_array(dev_node, "qcom,disable-ports",
					node_info->disable_ports,
					node_info->num_disable_ports);
	}

	if (of_get_property(dev_node, "qcom,connections", &size)) {
		node_info->num_connections = size / sizeof(int);
		node_info->connections = devm_kzalloc(&pdev->dev, size,
				GFP_KERNEL);
	} else {
		node_info->num_connections = 0;
		node_info->connections = 0;
	}

	for (i = 0; i < node_info->num_connections; i++) {
		con_node = of_parse_phandle(dev_node, "qcom,connections", i);
		if (IS_ERR_OR_NULL(con_node))
			goto node_info_err;

		if (of_property_read_u32(con_node, "cell-id",
				&node_info->connections[i]))
			goto node_info_err;
		of_node_put(con_node);
	}

	if (of_get_property(dev_node, "qcom,blacklist", &size)) {
		node_info->num_blist = size/sizeof(u32);
		node_info->bl_cons = devm_kzalloc(&pdev->dev,
		size, GFP_KERNEL);
	} else {
		node_info->num_blist = 0;
		node_info->bl_cons = 0;
	}

	for (i = 0; i < node_info->num_blist; i++) {
		con_node = of_parse_phandle(dev_node, "qcom,blacklist", i);
		if (IS_ERR_OR_NULL(con_node))
			goto node_info_err;

		if (of_property_read_u32(con_node, "cell-id",
				&node_info->bl_cons[i]))
			goto node_info_err;
		of_node_put(con_node);
	}

	bus_dev = of_parse_phandle(dev_node, "qcom,bus-dev", 0);
	if (!IS_ERR_OR_NULL(bus_dev)) {
		if (of_property_read_u32(bus_dev, "cell-id",
			&node_info->bus_device_id)) {
			dev_err(&pdev->dev, "Can't find bus device. Node %d",
					node_info->id);
			goto node_info_err;
		}

		of_node_put(bus_dev);
	} else
		dev_dbg(&pdev->dev, "Can't find bdev phandle for %d",
					node_info->id);

	if (of_get_property(dev_node, "qcom,bcms", &size)) {
		node_info->num_bcm_devs = size / sizeof(int);
		node_info->bcm_dev_ids = devm_kzalloc(&pdev->dev, size,
				GFP_KERNEL);
	} else {
		node_info->num_bcm_devs = 0;
		node_info->bcm_devs = 0;
	}

	for (i = 0; i < node_info->num_bcm_devs; i++) {
		bcm_dev = of_parse_phandle(dev_node, "qcom,bcms", i);
		if (IS_ERR_OR_NULL(bcm_dev))
			goto node_info_err;

		if (of_property_read_u32(bcm_dev, "cell-id",
				&node_info->bcm_dev_ids[i])){
			dev_err(&pdev->dev, "Can't find bcm device. Node %d",
					node_info->id);
			goto node_info_err;
		}
		of_node_put(bcm_dev);
	}

	if (of_get_property(dev_node, "qcom,rscs", &size)) {
		node_info->num_rsc_devs = size / sizeof(int);
		node_info->rsc_dev_ids = devm_kzalloc(&pdev->dev, size,
				GFP_KERNEL);
	} else {
		node_info->num_rsc_devs = 0;
		node_info->rsc_devs = 0;
	}

	for (i = 0; i < node_info->num_rsc_devs; i++) {
		rsc_dev = of_parse_phandle(dev_node, "qcom,rscs", i);
		if (IS_ERR_OR_NULL(rsc_dev))
			goto node_info_err;

		if (of_property_read_u32(rsc_dev, "cell-id",
				&node_info->rsc_dev_ids[i])){
			dev_err(&pdev->dev, "Can't find rsc device. Node %d",
					node_info->id);
			goto node_info_err;
		}
		of_node_put(rsc_dev);
	}

	node_info->is_bcm_dev = of_property_read_bool(dev_node, "qcom,bcm-dev");
	node_info->is_rsc_dev = of_property_read_bool(dev_node, "qcom,rsc-dev");
	node_info->is_fab_dev = of_property_read_bool(dev_node, "qcom,fab-dev");
	node_info->virt_dev = of_property_read_bool(dev_node, "qcom,virt-dev");

	get_agg_params(dev_node, pdev, node_info);
	get_qos_params(dev_node, pdev, node_info);

	return node_info;

node_info_err:
	devm_kfree(&pdev->dev, node_info);
	node_info = 0;
	return NULL;
}

static int get_bus_node_device_data(
		struct device_node * const dev_node,
		struct platform_device * const pdev,
		struct msm_bus_node_device_type * const node_device)
{
	bool enable_only;
	bool setrate_only;
	int num_elems = 0, num_bcms = 0, i = 0, ret = 0, num_regs = 0;
	uint32_t *vec_arr = NULL;
	struct qos_bcm_type *qos_bcms = NULL;
	struct device_node *qos_clk_node = NULL;
	const char *reg_name;
	struct property *prop;

	node_device->node_info = get_node_info_data(dev_node, pdev);
	if (IS_ERR_OR_NULL(node_device->node_info)) {
		dev_err(&pdev->dev, "Error: Node info missing\n");
		return -ENODATA;
	}
	node_device->ap_owned = of_property_read_bool(dev_node,
							"qcom,ap-owned");

	if (node_device->node_info->is_bcm_dev) {

		node_device->bcmdev = get_bcm_device_info(dev_node, pdev);

		if (IS_ERR_OR_NULL(node_device->bcmdev)) {
			dev_err(&pdev->dev,
				"Error: BCM device info missing\n");
			devm_kfree(&pdev->dev, node_device->node_info);
			return -ENODATA;
		}
	}

	if (node_device->node_info->is_rsc_dev) {

		node_device->rscdev = get_rsc_device_info(dev_node, pdev);

		if (IS_ERR_OR_NULL(node_device->rscdev)) {
			dev_err(&pdev->dev,
				"Error: RSC device info missing\n");
			devm_kfree(&pdev->dev, node_device->node_info);
			return -ENODATA;
		}
	}

	if (node_device->node_info->is_fab_dev) {
		dev_dbg(&pdev->dev, "Dev %d\n", node_device->node_info->id);

		if (!node_device->node_info->virt_dev) {
			node_device->fabdev =
				get_fab_device_info(dev_node, pdev);
			if (IS_ERR_OR_NULL(node_device->fabdev)) {
				dev_err(&pdev->dev,
					"Error: Fabric device info missing\n");
				devm_kfree(&pdev->dev, node_device->node_info);
				return -ENODATA;
			}
		}

		node_device->bus_qos_clk.clk = of_clk_get_by_name(dev_node,
							"bus_qos_clk");

		if (IS_ERR_OR_NULL(node_device->bus_qos_clk.clk)) {
			dev_dbg(&pdev->dev,
				"%s:Failed to get bus qos clk for %d",
				__func__, node_device->node_info->id);
			scnprintf(node_device->bus_qos_clk.reg_name,
					MAX_REG_NAME, "%c", '\0');
		} else {
			if (of_find_property(dev_node, "bus-qos-gdsc-supply",
								NULL))
				scnprintf(node_device->bus_qos_clk.reg_name,
					MAX_REG_NAME, "%s", "bus-qos-gdsc");
			else
				scnprintf(node_device->bus_qos_clk.reg_name,
					MAX_REG_NAME, "%c", '\0');
		}

		qos_clk_node = of_get_child_by_name(dev_node,
						"qcom,node-qos-clks");

		if (qos_clk_node) {
			if (msm_bus_of_parse_clk_array(qos_clk_node, dev_node,
						pdev,
						&node_device->node_qos_clks,
						&node_device->num_node_qos_clks,
						node_device->node_info->id)) {
				dev_info(&pdev->dev, "Bypass QoS programming");
				node_device->fabdev->bypass_qos_prg = true;
			}
			of_node_put(qos_clk_node);
		}
	} else {
		num_elems = of_property_count_elems_of_size(dev_node,
					"qcom,node-qos-bcms", sizeof(uint32_t));

		if (num_elems > 0) {
			if (num_elems % 3 != 0) {
				pr_err("Error: Length-error on getting vectors\n");
				return -ENODATA;
			}

			vec_arr = devm_kzalloc(&pdev->dev, (sizeof(uint32_t) *
							num_elems), GFP_KERNEL);
			if (!vec_arr)
				return -ENOMEM;

			ret = of_property_read_u32_array(dev_node,
						"qcom,node-qos-bcms", vec_arr,
								num_elems);
			if (ret) {
				pr_err("Error: problem reading qos-bcm vectors\n");
				return ret;
			}
			num_bcms = num_elems / 3;
			node_device->num_qos_bcms = num_bcms;

			qos_bcms = devm_kzalloc(&pdev->dev,
						(sizeof(struct qos_bcm_type) *
						num_bcms), GFP_KERNEL);
			if (!qos_bcms)
				return -ENOMEM;

			for (i = 0; i < num_bcms; i++) {
				int index = i * 3;

				qos_bcms[i].qos_bcm_id = vec_arr[index];
				qos_bcms[i].vec.vec_a =
					(uint64_t)KBTOB(vec_arr[index + 1]);
				qos_bcms[i].vec.vec_b =
					(uint64_t)KBTOB(vec_arr[index + 2]);
			}
			node_device->qos_bcms = qos_bcms;
		}

		enable_only = of_property_read_bool(dev_node,
							"qcom,enable-only-clk");
		node_device->clk[DUAL_CTX].enable_only_clk = enable_only;

		/*
		 * Doesn't make sense to have a clk handle you can't enable or
		 * set rate on.
		 */
		if (!enable_only) {
			setrate_only = of_property_read_bool(dev_node,
						"qcom,setrate-only-clk");
			node_device->clk[DUAL_CTX].setrate_only_clk =
								setrate_only;
			node_device->clk[ACTIVE_CTX].setrate_only_clk =
								setrate_only;
		}

		qos_clk_node = of_get_child_by_name(dev_node,
						"qcom,node-qos-clks");

		if (qos_clk_node) {
			if (msm_bus_of_parse_clk_array(qos_clk_node, dev_node,
						pdev,
						&node_device->node_qos_clks,
						&node_device->num_node_qos_clks,
						node_device->node_info->id)) {
				dev_dbg(&pdev->dev, "Bypass QoS programming");
				node_device->fabdev->bypass_qos_prg = true;
			}
			of_node_put(qos_clk_node);
		}
		node_device->clk[DUAL_CTX].clk = of_clk_get_by_name(dev_node,
							"node_clk");

		if (IS_ERR_OR_NULL(node_device->clk[DUAL_CTX].clk))
			dev_dbg(&pdev->dev,
				"%s:Failed to get bus clk for bus%d ctx%d",
				__func__, node_device->node_info->id,
								DUAL_CTX);

		if (of_find_property(dev_node, "node-gdsc-supply", NULL))
			scnprintf(node_device->clk[DUAL_CTX].reg_name,
				MAX_REG_NAME, "%s", "node-gdsc");
		else
			scnprintf(node_device->clk[DUAL_CTX].reg_name,
				MAX_REG_NAME, "%c", '\0');

		num_regs = of_property_count_strings(dev_node,
							"node-reg-names");
		if (num_regs < 0)
			node_device->num_regs = 0;
		else {
			i = 0;
			node_device->num_regs = num_regs;
			node_device->node_regs = devm_kzalloc(&pdev->dev,
				(num_regs * sizeof(struct node_regulator)),
								GFP_KERNEL);

			of_property_for_each_string(dev_node, "node-reg-names",
							prop, reg_name) {
				scnprintf(node_device->node_regs[i].name,
					MAX_REG_NAME, "%s", reg_name);
				i++;
			}
		}
	}
	return 0;
}

struct msm_bus_device_node_registration
	*msm_bus_of_to_pdata(struct platform_device *pdev)
{
	struct device_node *of_node, *child_node;
	struct msm_bus_device_node_registration *pdata;
	unsigned int i = 0, j;
	unsigned int ret;

	if (!pdev) {
		pr_err("Error: Null platform device\n");
		return NULL;
	}

	of_node = pdev->dev.of_node;

	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct msm_bus_device_node_registration),
			GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev,
				"Error: Memory allocation for pdata failed\n");
		return NULL;
	}

	pdata->num_devices = of_get_child_count(of_node);

	pdata->info = devm_kzalloc(&pdev->dev,
			sizeof(struct msm_bus_node_device_type) *
			pdata->num_devices, GFP_KERNEL);

	if (!pdata->info) {
		dev_err(&pdev->dev,
			"Error: Memory allocation for pdata->info failed\n");
		goto node_reg_err;
	}

	ret = 0;
	for_each_child_of_node(of_node, child_node) {
		ret = get_bus_node_device_data(child_node, pdev,
				&pdata->info[i]);
		if (ret) {
			dev_err(&pdev->dev, "Error: unable to initialize bus nodes\n");
			goto node_reg_err_1;
		}
		pdata->info[i].of_node = child_node;
		i++;
	}

	dev_dbg(&pdev->dev, "bus topology:\n");
	for (i = 0; i < pdata->num_devices; i++) {
		dev_dbg(&pdev->dev, "id %d\nnum_qports %d\nnum_connections %d",
				pdata->info[i].node_info->id,
				pdata->info[i].node_info->num_qports,
				pdata->info[i].node_info->num_connections);
		dev_dbg(&pdev->dev, "\nbus_device_id %d\n buswidth %d\n",
				pdata->info[i].node_info->bus_device_id,
				pdata->info[i].node_info->agg_params.buswidth);
		for (j = 0; j < pdata->info[i].node_info->num_connections;
									j++) {
			dev_dbg(&pdev->dev, "connection[%d]: %d\n", j,
				pdata->info[i].node_info->connections[j]);
		}
		for (j = 0; j < pdata->info[i].node_info->num_blist;
									 j++) {
			dev_dbg(&pdev->dev, "black_listed_node[%d]: %d\n", j,
				pdata->info[i].node_info->bl_cons[j]);
		}
		if (pdata->info[i].fabdev)
			dev_dbg(&pdev->dev, "base_addr %zu\nbus_type %d\n",
				(size_t)pdata->info[i].fabdev->pqos_base,
					pdata->info[i].fabdev->bus_type);
	}
	return pdata;

node_reg_err_1:
	devm_kfree(&pdev->dev, pdata->info);
node_reg_err:
	devm_kfree(&pdev->dev, pdata);
	pdata = NULL;
	return NULL;
}

static int msm_bus_of_get_ids(struct platform_device *pdev,
			struct device_node *dev_node, int **dev_ids,
			int *num_ids, char *prop_name)
{
	int ret = 0;
	int size, i;
	struct device_node *rule_node;
	int *ids = NULL;

	if (of_get_property(dev_node, prop_name, &size)) {
		*num_ids = size / sizeof(int);
		ids = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	} else {
		dev_err(&pdev->dev, "No rule nodes, skipping node");
		ret = -ENXIO;
		goto exit_get_ids;
	}

	*dev_ids = ids;
	for (i = 0; i < *num_ids; i++) {
		rule_node = of_parse_phandle(dev_node, prop_name, i);
		if (IS_ERR_OR_NULL(rule_node)) {
			dev_err(&pdev->dev, "Can't get rule node id");
			ret = -ENXIO;
			goto err_get_ids;
		}

		if (of_property_read_u32(rule_node, "cell-id",
				&ids[i])) {
			dev_err(&pdev->dev, "Can't get rule node id");
			ret = -ENXIO;
			goto err_get_ids;
		}
		of_node_put(rule_node);
	}
exit_get_ids:
	return ret;
err_get_ids:
	devm_kfree(&pdev->dev, ids);
	of_node_put(rule_node);
	ids = NULL;
	return ret;
}

int msm_bus_of_get_static_rules(struct platform_device *pdev,
					struct bus_rule_type **static_rules)
{
	int ret = 0;
	struct device_node *of_node, *child_node;
	int num_rules = 0;
	int rule_idx = 0;
	int bw_fld = 0;
	int i;
	struct bus_rule_type *local_rule = NULL;

	of_node = pdev->dev.of_node;
	num_rules = of_get_child_count(of_node);
	local_rule = devm_kzalloc(&pdev->dev,
				sizeof(struct bus_rule_type) * num_rules,
				GFP_KERNEL);

	if (IS_ERR_OR_NULL(local_rule)) {
		ret = -ENOMEM;
		goto exit_static_rules;
	}

	*static_rules = local_rule;
	for_each_child_of_node(of_node, child_node) {
		ret = msm_bus_of_get_ids(pdev, child_node,
			&local_rule[rule_idx].src_id,
			&local_rule[rule_idx].num_src,
			"qcom,src-nodes");

		ret = msm_bus_of_get_ids(pdev, child_node,
			&local_rule[rule_idx].dst_node,
			&local_rule[rule_idx].num_dst,
			"qcom,dest-node");

		ret = of_property_read_u32(child_node, "qcom,src-field",
				&local_rule[rule_idx].src_field);
		if (ret) {
			dev_err(&pdev->dev, "src-field missing");
			ret = -ENXIO;
			goto err_static_rules;
		}

		ret = of_property_read_u32(child_node, "qcom,src-op",
				&local_rule[rule_idx].op);
		if (ret) {
			dev_err(&pdev->dev, "src-op missing");
			ret = -ENXIO;
			goto err_static_rules;
		}

		ret = of_property_read_u32(child_node, "qcom,mode",
				&local_rule[rule_idx].mode);
		if (ret) {
			dev_err(&pdev->dev, "mode missing");
			ret = -ENXIO;
			goto err_static_rules;
		}

		ret = of_property_read_u32(child_node, "qcom,thresh", &bw_fld);
		if (ret) {
			dev_err(&pdev->dev, "thresh missing");
			ret = -ENXIO;
			goto err_static_rules;
		} else
			local_rule[rule_idx].thresh = KBTOB(bw_fld);

		ret = of_property_read_u32(child_node, "qcom,dest-bw",
								&bw_fld);
		if (ret)
			local_rule[rule_idx].dst_bw = 0;
		else
			local_rule[rule_idx].dst_bw = KBTOB(bw_fld);

		rule_idx++;
	}
	ret = rule_idx;
exit_static_rules:
	return ret;
err_static_rules:
	for (i = 0; i < num_rules; i++) {
		if (!IS_ERR_OR_NULL(local_rule)) {
			if (!IS_ERR_OR_NULL(local_rule[i].src_id))
				devm_kfree(&pdev->dev,
						local_rule[i].src_id);
			if (!IS_ERR_OR_NULL(local_rule[i].dst_node))
				devm_kfree(&pdev->dev,
						local_rule[i].dst_node);
			devm_kfree(&pdev->dev, local_rule);
		}
	}
	*static_rules = NULL;
	return ret;
}
