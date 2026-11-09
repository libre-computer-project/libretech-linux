#ifndef __PHY_MESON_GXL_H__
#define __PHY_MESON_GXL_H__

#include <linux/phy.h>
#include <linux/phy/phy.h>

irqreturn_t meson_gxl_phy_handle_interrupt(struct phy_device *phydev);

#endif /* __PHY_MESON_GXL_H__ */
