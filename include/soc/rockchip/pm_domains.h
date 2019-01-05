/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2022, The Chromium OS Authors. All rights reserved.
 */

#ifndef __SOC_ROCKCHIP_PM_DOMAINS_H__
#define __SOC_ROCKCHIP_PM_DOMAINS_H__

#ifdef CONFIG_ROCKCHIP_PM_DOMAINS

int rockchip_pmu_block(void);
void rockchip_pmu_unblock(void);
int rockchip_pmu_idle_request(struct device *dev, bool idle);

#else /* CONFIG_ROCKCHIP_PM_DOMAINS */

static inline int rockchip_pmu_block(void)
{
	return 0;
}

static inline void rockchip_pmu_unblock(void) { }

static inline int rockchip_pmu_idle_request(struct device *dev, bool idle)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_ROCKCHIP_PM_DOMAINS */

#endif /* __SOC_ROCKCHIP_PM_DOMAINS_H__ */
