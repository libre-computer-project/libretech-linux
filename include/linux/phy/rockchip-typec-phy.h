/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Rockchip Type-C PHY DP alt-mode interface
 *
 * Shared between phy-rockchip-typec.c and cdn-dp-core.c to allow the PHY
 * to notify the DP controller of HPD events driven by the USB Type-C
 * typec_mux/typec_switch API, without requiring extcon.
 */

#ifndef __PHY_ROCKCHIP_TYPEC_H
#define __PHY_ROCKCHIP_TYPEC_H

#include <linux/notifier.h>

/* Notification events from PHY to DP controller */
#define ROCKCHIP_TYPEC_PHY_EVENT_HPD		1

/**
 * struct rockchip_typec_phy_dp_state - DP alt-mode state from PHY
 * @dp_connected: true if DP alt-mode is active and HPD is asserted
 * @lanes: number of DP lanes (2 or 4)
 * @flip: true if connector is in flipped orientation
 */
struct rockchip_typec_phy_dp_state {
	bool dp_connected;
	u8 lanes;
	bool flip;
};

#ifdef CONFIG_PHY_ROCKCHIP_TYPEC

int rockchip_typec_phy_get_dp_state(struct phy *phy,
				    struct rockchip_typec_phy_dp_state *state);
int rockchip_typec_phy_register_notify(struct phy *phy,
				       struct notifier_block *nb);
int rockchip_typec_phy_unregister_notify(struct phy *phy,
					 struct notifier_block *nb);

#else

static inline int
rockchip_typec_phy_get_dp_state(struct phy *phy,
				struct rockchip_typec_phy_dp_state *state)
{
	return -EOPNOTSUPP;
}

static inline int
rockchip_typec_phy_register_notify(struct phy *phy,
				   struct notifier_block *nb)
{
	return -EOPNOTSUPP;
}

static inline int
rockchip_typec_phy_unregister_notify(struct phy *phy,
				     struct notifier_block *nb)
{
	return -EOPNOTSUPP;
}

#endif /* CONFIG_PHY_ROCKCHIP_TYPEC */

#endif /* __PHY_ROCKCHIP_TYPEC_H */
