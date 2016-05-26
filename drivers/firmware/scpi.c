/*
 * System Control and Power Interface (SCPI) Message Protocol registry
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

static struct scpi_ops *g_ops;

struct scpi_ops *get_scpi_ops(void)
{
	return g_ops;
}
EXPORT_SYMBOL_GPL(get_scpi_ops);

int scpi_ops_register(struct scpi_ops *ops)
{
	if (!ops)
		return -EINVAL;

	if (g_ops)
		return -EEXIST;

	g_ops = ops;

	return 0;
}
EXPORT_SYMBOL_GPL(scpi_ops_register);

void scpi_ops_unregister(struct scpi_ops *ops)
{
	if (g_ops == ops)
		g_ops = NULL;
}
EXPORT_SYMBOL_GPL(scpi_ops_unregister);

static void devm_scpi_ops_unregister(struct device *dev, void *res)
{
	scpi_ops_unregister(*(struct scpi_ops **)res);
}

int devm_scpi_ops_register(struct device *dev,
				struct scpi_ops *ops)
{
	struct scpi_ops **rcops;
	int ret;

	rcops = devres_alloc(devm_scpi_ops_unregister, sizeof(*ops),
			     GFP_KERNEL);
	if (!rcops)
		return -ENOMEM;

	ret = scpi_ops_register(ops);
	if (!ret) {
		*rcops = ops;
		devres_add(dev, rcops);
	} else
		devres_free(rcops);

	return ret;
}
EXPORT_SYMBOL_GPL(devm_scpi_ops_register);
