/* Copyright (c) 2014-2019, The Linux Foundation. All rights reserved.
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

#ifndef __MSM_PCIE_H
#define __MSM_PCIE_H

#include <linux/types.h>
#include <linux/pci.h>

enum msm_pcie_config {
	MSM_PCIE_CONFIG_INVALID = 0,
	MSM_PCIE_CONFIG_NO_CFG_RESTORE = 0x1,
	MSM_PCIE_CONFIG_LINKDOWN = 0x2,
	MSM_PCIE_CONFIG_NO_RECOVERY = 0x4,
};

enum msm_pcie_pm_opt {
	MSM_PCIE_SUSPEND,
	MSM_PCIE_RESUME,
	MSM_PCIE_DISABLE_PC,
	MSM_PCIE_ENABLE_PC,
};

enum msm_pcie_event {
	MSM_PCIE_EVENT_INVALID = 0,
	MSM_PCIE_EVENT_LINKDOWN = 0x1,
	MSM_PCIE_EVENT_LINKUP = 0x2,
	MSM_PCIE_EVENT_WAKEUP = 0x4,
	MSM_PCIE_EVENT_L1SS_TIMEOUT = BIT(3),
};

enum msm_pcie_trigger {
	MSM_PCIE_TRIGGER_CALLBACK,
	MSM_PCIE_TRIGGER_COMPLETION,
};

struct msm_pcie_notify {
	enum msm_pcie_event event;
	void *user;
	void *data;
	u32 options;
};

struct msm_pcie_register_event {
	u32 events;
	void *user;
	enum msm_pcie_trigger mode;
	void (*callback)(struct msm_pcie_notify *notify);
	struct msm_pcie_notify notify;
	struct completion *completion;
	u32 options;
};

#ifdef CONFIG_PCI_MSM_MSI
void msm_msi_config(struct irq_domain *domain);
int msm_msi_init(struct device *dev);
#else
static inline void msm_msi_config(struct irq_domain *domain)
{
}

static inline int msm_msi_init(struct device *dev)
{
	return -EINVAL;
}
#endif

#ifdef CONFIG_PCI_MSM

/**
 * msm_pcie_allow_l1 - allow PCIe link to enter L1
 * @pci_dev:          client's pci device structure
 *
 * This function gives PCIe clients the control to allow the link to enter L1.
 */
void msm_pcie_allow_l1(struct pci_dev *pci_dev);

/**
 * msm_pcie_prevent_l1 - keeps PCIe link out of L1
 * @pci_dev:          client's pci device structure
 *
 * This function gives PCIe clients the control to exit and prevent the link
 * from entering L1.
 *
 * Return 0 on success, negative value on error
 */
int msm_pcie_prevent_l1(struct pci_dev *pci_dev);

/**
 * msm_pcie_set_link_bandwidth - updates the number of lanes and speed of PCIe
 * link.
 * @pci_dev:		client's pci device structure
 * @target_link_speed:	gen speed
 * @target_link_width:	number of lanes
 *
 * This function gives PCIe clients the control to update the number of lanes
 * and gen speed of the link.
 *
 * Return: 0 on success, negative value on error
 */
int msm_pcie_set_link_bandwidth(struct pci_dev *pci_dev, u16 target_link_speed,
				u16 target_link_width);

/**
 * msm_pcie_l1ss_timeout_disable - disable L1ss timeout feature
 * @pci_dev:	client's pci device structure
 *
 * This function gives PCIe clients the control to disable L1ss timeout
 * feature.
 */
void msm_pcie_l1ss_timeout_disable(struct pci_dev *pci_dev);

/**
 * msm_pcie_l1ss_timeout_enable - enable L1ss timeout feature
 * @pci_dev:	client's pci device structure
 *
 * This function gives PCIe clients the control to enable L1ss timeout
 * feature.
 */
void msm_pcie_l1ss_timeout_enable(struct pci_dev *pci_dev);

/**
 * msm_pcie_pm_control - control the power state of a PCIe link.
 * @pm_opt:	power management operation
 * @busnr:	bus number of PCIe endpoint
 * @user:	handle of the caller
 * @data:	private data from the caller
 * @options:	options for pm control
 *
 * This function gives PCIe endpoint device drivers the control to change
 * the power state of a PCIe link for their device.
 *
 * Return: 0 on success, negative value on error
 */
int msm_pcie_pm_control(enum msm_pcie_pm_opt pm_opt, u32 busnr, void *user,
			void *data, u32 options);

/**
 * msm_pcie_register_event - register an event with PCIe bus driver.
 * @reg:	event structure
 *
 * This function gives PCIe endpoint device drivers an option to register
 * events with PCIe bus driver.
 *
 * Return: 0 on success, negative value on error
 */
int msm_pcie_register_event(struct msm_pcie_register_event *reg);

/**
 * msm_pcie_deregister_event - deregister an event with PCIe bus driver.
 * @reg:	event structure
 *
 * This function gives PCIe endpoint device drivers an option to deregister
 * events with PCIe bus driver.
 *
 * Return: 0 on success, negative value on error
 */
int msm_pcie_deregister_event(struct msm_pcie_register_event *reg);

/**
 * msm_pcie_recover_config - recover config space.
 * @dev:	pci device structure
 *
 * This function recovers the config space of both RC and Endpoint.
 *
 * Return: 0 on success, negative value on error
 */
int msm_pcie_recover_config(struct pci_dev *dev);

/**
 * msm_pcie_enumerate - enumerate Endpoints.
 * @rc_idx:	RC that Endpoints connect to.
 *
 * This function enumerates Endpoints connected to RC.
 *
 * Return: 0 on success, negative value on error
 */
int msm_pcie_enumerate(u32 rc_idx);

/**
 * msm_pcie_recover_config - recover config space.
 * @dev:	pci device structure
 *
 * This function recovers the config space of both RC and Endpoint.
 *
 * Return: 0 on success, negative value on error
 */
int msm_pcie_recover_config(struct pci_dev *dev);

/**
 * msm_pcie_shadow_control - control the shadowing of PCIe config space.
 * @dev:	pci device structure
 * @enable:	shadowing should be enabled or disabled
 *
 * This function gives PCIe endpoint device drivers the control to enable
 * or disable the shadowing of PCIe config space.
 *
 * Return: 0 on success, negative value on error
 */
int msm_pcie_shadow_control(struct pci_dev *dev, bool enable);

/*
 * msm_pcie_debug_info - run a PCIe specific debug testcase.
 * @dev:	pci device structure
 * @option:	specifies which PCIe debug testcase to execute
 * @base:	PCIe specific range
 * @offset:	offset of destination register
 * @mask:	mask the bit(s) of destination register
 * @value:	value to be written to destination register
 *
 * This function gives PCIe endpoint device drivers the control to
 * run a debug testcase.
 *
 * Return: 0 on success, negative value on error
 */
int msm_pcie_debug_info(struct pci_dev *dev, u32 option, u32 base,
			u32 offset, u32 mask, u32 value);

#else /* !CONFIG_PCI_MSM */
static inline int msm_pcie_pm_control(enum msm_pcie_pm_opt pm_opt, u32 busnr,
			void *user, void *data, u32 options)
{
	return -ENODEV;
}

static inline void msm_pcie_allow_l1(struct pci_dev *pci_dev)
{
}

static inline int msm_pcie_prevent_l1(struct pci_dev *pci_dev)
{
	return -ENODEV;
}

static inline int msm_pcie_l1ss_timeout_disable(struct pci_dev *pci_dev)
{
	return -ENODEV;
}

static inline int msm_pcie_l1ss_timeout_enable(struct pci_dev *pci_dev)
{
	return -ENODEV;
}

static inline int msm_pcie_register_event(struct msm_pcie_register_event *reg)
{
	return -ENODEV;
}

static inline int msm_pcie_deregister_event(struct msm_pcie_register_event *reg)
{
	return -ENODEV;
}

static inline int msm_pcie_recover_config(struct pci_dev *dev)
{
	return -ENODEV;
}

static inline int msm_pcie_enumerate(u32 rc_idx)
{
	return -ENODEV;
}

static inline int msm_pcie_shadow_control(struct pci_dev *dev, bool enable)
{
	return -ENODEV;
}

static inline int msm_pcie_debug_info(struct pci_dev *dev, u32 option, u32 base,
			u32 offset, u32 mask, u32 value)
{
	return -ENODEV;
}
#endif /* CONFIG_PCI_MSM */

#endif /* __MSM_PCIE_H */
