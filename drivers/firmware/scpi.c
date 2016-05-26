/*
 * System Control and Power Interface (SCPI) Message Protocol framework
 *
 * SCPI Message Protocol is used between the System Control Processor(SCP)
 * and the Application Processors(AP). The Message Handling Unit(MHU)
 * provides a mechanism for inter-processor communication between SCP's
 * Cortex M3 and AP.
 *
 * SCP offers control and management of the core/cluster power states,
 * various power domain DVFS including the core/cluster, certain system
 * clocks configuration, thermal sensors and many others.
 *
 * Copyright (C) 2015 ARM Ltd.
 * Copyright (C) 2016 BayLibre, SAS.
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/scpi_protocol.h>
#include <linux/spinlock.h>

static DEFINE_MUTEX(scpi_list_mutex);
static LIST_HEAD(scpi_drivers_list);

struct scpi_ops *of_scpi_ops_get(struct device_node *node)
{
	struct scpi_ops *ops = NULL;
	struct scpi_driver *r;

	if (!node)
		return ERR_PTR(-EINVAL);

	mutex_lock(&scpi_list_mutex);
	list_for_each_entry(r, &scpi_drivers_list, list) {
		if (node == r->node) {
			ops = r->ops;
			break;
		}
	}
	mutex_unlock(&scpi_list_mutex);

	if (!ops)
		return ERR_PTR(-EPROBE_DEFER);

	return ops;
}
EXPORT_SYMBOL_GPL(of_scpi_ops_get);

int scpi_driver_register(struct scpi_driver *drv)
{
	mutex_lock(&scpi_list_mutex);
	list_add(&drv->list, &scpi_drivers_list);
	mutex_unlock(&scpi_list_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(scpi_driver_register);

void scpi_driver_unregister(struct scpi_driver *drv)
{
	mutex_lock(&scpi_list_mutex);
	list_del(&drv->list);
	mutex_unlock(&scpi_list_mutex);
}
EXPORT_SYMBOL_GPL(scpi_driver_unregister);

static void devm_scpi_driver_unregister(struct device *dev, void *res)
{
	scpi_driver_unregister(*(struct scpi_driver **)res);
}

int devm_scpi_driver_register(struct device *dev,
				struct scpi_driver *drv)
{
	struct scpi_driver **rcdrv;
	int ret;

	rcdrv = devres_alloc(devm_scpi_driver_unregister, sizeof(*drv),
			     GFP_KERNEL);
	if (!rcdrv)
		return -ENOMEM;

	ret = scpi_driver_register(drv);
	if (!ret) {
		*rcdrv = drv;
		devres_add(dev, rcdrv);
	} else
		devres_free(rcdrv);

	return ret;
}
EXPORT_SYMBOL_GPL(devm_scpi_driver_register);
