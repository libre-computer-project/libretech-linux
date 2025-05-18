// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson GXL Internal PHY Driver
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 * Copyright (C) 2016 BayLibre, SAS. All rights reserved.
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/bitfield.h>
#include <linux/smscphy.h>

#define ANEG_ADV	4
#define TSTCNTL		20
#define  TSTCNTL_READ		BIT(15)
#define  TSTCNTL_WRITE		BIT(14)
#define  TSTCNTL_REG_BANK_SEL	GENMASK(12, 11)
#define  TSTCNTL_TEST_MODE	BIT(10)
#define  TSTCNTL_READ_ADDRESS	GENMASK(9, 5)
#define  TSTCNTL_WRITE_ADDRESS	GENMASK(4, 0)
#define TSTREAD1	21
#define TSTWRITE	23

#define PHY_INT_SRC	29 // 0x1D
#define PHY_INT_MASK	30 // 0x1E
#define PHY_INT_WOL			GENMASK(11,9)
#define PHY_INT_WOL_MP			BIT(11)
#define PHY_INT_ENERGYON		BIT(7)
#define PHY_INT_ANEG_COMP		BIT(6)
#define PHY_INT_REMOTE_FAULT		BIT(5)
#define PHY_INT_LINK_DOWN		BIT(4)
#define PHY_INT_ANEG_LP_ACK		BIT(3)
#define PHY_INT_PARALLEL_DET_FAULT	BIT(2)
#define PHY_INT_ANEG_PAGE		BIT(1)
#define PHY_INT_DEFAULT	(PHY_INT_ANEG_COMP | PHY_INT_LINK_DOWN | PHY_INT_ENERGYON)

#define BANK_ANALOG_DSP		0
#define BANK_WOL		1
#define BANK_BIST		3

/* WOL Registers */
#define LPI_STATUS	0xc
#define  LPI_STATUS_RSV12	BIT(12)

/* BIST Registers */
#define FR_PLL_CONTROL	0x1b
#define FR_PLL_DIV0	0x1c
#define FR_PLL_DIV1	0x1d

static int meson_gxl_open_banks(struct phy_device *phydev)
{
	int ret;

	/* Enable Analog and DSP register Bank access by
	 * toggling TSTCNTL_TEST_MODE bit in the TSTCNTL register
	 */
	ret = phy_write(phydev, TSTCNTL, 0);
	if (ret)
		return ret;
	ret = phy_write(phydev, TSTCNTL, TSTCNTL_TEST_MODE);
	if (ret)
		return ret;
	ret = phy_write(phydev, TSTCNTL, 0);
	if (ret)
		return ret;
	return phy_write(phydev, TSTCNTL, TSTCNTL_TEST_MODE);
}

static void meson_gxl_close_banks(struct phy_device *phydev)
{
	phy_write(phydev, TSTCNTL, 0);
}

static int meson_gxl_read_reg(struct phy_device *phydev,
			      unsigned int bank, unsigned int reg)
{
	int ret;

	ret = meson_gxl_open_banks(phydev);
	if (ret)
		goto out;

	ret = phy_write(phydev, TSTCNTL, TSTCNTL_READ |
			FIELD_PREP(TSTCNTL_REG_BANK_SEL, bank) |
			TSTCNTL_TEST_MODE |
			FIELD_PREP(TSTCNTL_READ_ADDRESS, reg));
	if (ret)
		goto out;

	ret = phy_read(phydev, TSTREAD1);
out:
	/* Close the bank access on our way out */
	meson_gxl_close_banks(phydev);
	return ret;
}

static int meson_gxl_write_reg(struct phy_device *phydev,
			       unsigned int bank, unsigned int reg,
			       uint16_t value)
{
	int ret;

	ret = meson_gxl_open_banks(phydev);
	if (ret)
		goto out;

	ret = phy_write(phydev, TSTWRITE, value);
	if (ret)
		goto out;

	ret = phy_write(phydev, TSTCNTL, TSTCNTL_WRITE |
			FIELD_PREP(TSTCNTL_REG_BANK_SEL, bank) |
			TSTCNTL_TEST_MODE |
			FIELD_PREP(TSTCNTL_WRITE_ADDRESS, reg));

out:
	/* Close the bank access on our way out */
	meson_gxl_close_banks(phydev);
	return ret;
}

static int meson_gxl_config_init(struct phy_device *phydev)
{
	int ret;

	/* Enable fractional PLL */
	ret = meson_gxl_write_reg(phydev, BANK_BIST, FR_PLL_CONTROL, 0x5);
	if (ret)
		return ret;

	/* Program fraction FR_PLL_DIV1 */
	ret = meson_gxl_write_reg(phydev, BANK_BIST, FR_PLL_DIV1, 0x029a);
	if (ret)
		return ret;

	/* Program fraction FR_PLL_DIV1 */
	ret = meson_gxl_write_reg(phydev, BANK_BIST, FR_PLL_DIV0, 0xaaaa);
	if (ret)
		return ret;

	return 0;
}

static int meson_gxl_wol_status(struct phy_device *phydev){
	return (phy_read(phydev, PHY_INT_MASK) & PHY_INT_WOL) > 0;
}

static void meson_gxl_get_wol(struct phy_device *phydev, struct ethtool_wolinfo *wol)
{
	wol->supported = WAKE_MAGIC;
	wol->wolopts = 0;

	int int_mask = phy_read(phydev, PHY_INT_MASK) & PHY_INT_WOL;
	if (int_mask < 0) {
		wol->supported = 0;
		return;
	}

	if (int_mask & PHY_INT_WOL)
		wol->wolopts |= WAKE_MAGIC;

	memset(wol->sopass, 0, sizeof(wol->sopass));
}

static int meson_gxl_set_wol(struct phy_device *phydev, struct ethtool_wolinfo *wol)
{
	struct net_device *ndev = phydev->attached_dev;
	const unsigned char *mac_addr;

	if (!ndev) return -ENODEV;

	mac_addr = ndev->dev_addr;
	pr_debug("meson_gxl_set_wol: mac %pM\n", mac_addr);

	if (wol->wolopts & ~WAKE_MAGIC) return -EOPNOTSUPP;

	if (wol->wolopts & WAKE_MAGIC){
		/* MAC address */
		meson_gxl_write_reg(phydev, BANK_WOL, 0, mac_addr[5] | mac_addr[4] << 8);
		meson_gxl_write_reg(phydev, BANK_WOL, 1, mac_addr[3] | mac_addr[2] << 8);
		meson_gxl_write_reg(phydev, BANK_WOL, 2, mac_addr[1] | mac_addr[0] << 8);
		meson_gxl_write_reg(phydev, BANK_WOL, 3, 0x9);

		/* Enable interrupt */
		phy_write(phydev, PHY_INT_MASK, phy_read(phydev, PHY_INT_MASK) | PHY_INT_WOL);
		pr_debug("meson_gxl_set_wol: enabled\n");
	} else {
		phy_write(phydev, PHY_INT_MASK, phy_read(phydev, PHY_INT_MASK) & ~PHY_INT_WOL);
		pr_debug("meson_gxl_set_wol: disabled\n");
	}

	return 0;
}

static int meson_gxl_phy_suspend(struct phy_device *phydev)
{
	int wol = meson_gxl_wol_status(phydev);
	pr_debug("%s: wol %d\n", __func__, wol);

	if (wol){
		// disable aneg comp and link down interrupts
		pr_debug("%s: int src %x mask %x\n", __func__, phy_read(phydev, PHY_INT_SRC), phy_read(phydev, PHY_INT_MASK));
		phy_write(phydev, PHY_INT_MASK, phy_read(phydev, PHY_INT_MASK) & ~PHY_INT_DEFAULT);
		pr_debug("%s: int src %x mask %x\n", __func__, phy_read(phydev, PHY_INT_SRC), phy_read(phydev, PHY_INT_MASK));
		//genphy_suspend(phydev);
	} else {
		pr_debug("%s: generic\n", __func__);
		genphy_suspend(phydev);
	}
	return 0;
}

static int meson_gxl_phy_resume(struct phy_device *phydev)
{
	int ret;
	int wol = meson_gxl_wol_status(phydev);
	pr_debug("%s: wol %d\n", __func__, wol);

	if (wol){
		// disable aneg comp and link down interrupts
		pr_debug("%s: int mask %x\n", __func__, phy_read(phydev, PHY_INT_MASK));
		phy_write(phydev, PHY_INT_MASK, phy_read(phydev, PHY_INT_MASK) | PHY_INT_DEFAULT);
		pr_debug("%s: int mask %x\n", __func__, phy_read(phydev, PHY_INT_MASK));
	} else {
		ret = genphy_resume(phydev); // is this ok for wol case where only the interrupt mask is modified?
		pr_debug("%s: generic\n", __func__);
		if (ret)
			return ret;
	}

	ret = meson_gxl_config_init(phydev);
	if (ret) return ret;

	return 0;
}

/* This function is provided to cope with the possible failures of this phy
 * during aneg process. When aneg fails, the PHY reports that aneg is done
 * but the value found in MII_LPA is wrong:
 *  - Early failures: MII_LPA is just 0x0001. if MII_EXPANSION reports that
 *    the link partner (LP) supports aneg but the LP never acked our base
 *    code word, it is likely that we never sent it to begin with.
 *  - Late failures: MII_LPA is filled with a value which seems to make sense
 *    but it actually is not what the LP is advertising. It seems that we
 *    can detect this using a magic bit in the WOL bank (reg 12 - bit 12).
 *    If this particular bit is not set when aneg is reported being done,
 *    it means MII_LPA is likely to be wrong.
 *
 * In both case, forcing a restart of the aneg process solve the problem.
 * When this failure happens, the first retry is usually successful but,
 * in some cases, it may take up to 6 retries to get a decent result
 */
static int meson_gxl_read_status(struct phy_device *phydev)
{
	int ret, wol, lpa, exp;

	if (phydev->autoneg == AUTONEG_ENABLE) {
		ret = genphy_aneg_done(phydev);
		if (ret < 0)
			return ret;
		else if (!ret)
			goto read_status_continue;

		/* Aneg is done, let's check everything is fine */
		wol = meson_gxl_read_reg(phydev, BANK_WOL, LPI_STATUS);
		if (wol < 0)
			return wol;

		lpa = phy_read(phydev, MII_LPA);
		if (lpa < 0)
			return lpa;

		exp = phy_read(phydev, MII_EXPANSION);
		if (exp < 0)
			return exp;

		if (!(wol & LPI_STATUS_RSV12) ||
		    ((exp & EXPANSION_NWAY) && !(lpa & LPA_LPACK))) {
			/* Looks like aneg failed after all */
			phydev_dbg(phydev, "LPA corruption - aneg restart\n");
			return genphy_restart_aneg(phydev);
		} else {
			phydev->autoneg_complete = 1;
		}
	}

read_status_continue:
	return genphy_read_status(phydev);
}

irqreturn_t meson_gxl_phy_handle_interrupt(struct phy_device *phydev)
{
	int irq_status;

	irq_status = phy_read(phydev, PHY_INT_SRC);
	pr_debug("%s: interrupt source %x\n", __func__, irq_status);
	if (irq_status < 0) {
		if (irq_status != -ENODEV)
			phy_error(phydev);

		return IRQ_NONE;
	}

	if (!(irq_status & PHY_INT_DEFAULT)){
		pr_debug("%s: ignoring non-default interrupts\n", __func__);
		return IRQ_NONE;
	}

	if (phydev->autoneg == AUTONEG_ENABLE){
		pr_debug("%s: interrupt during aneg\n", __func__);
		if (irq_status == PHY_INT_LINK_DOWN){
			pr_debug("%s: ignoring link down interrupt while aneg\n", __func__);
			return IRQ_HANDLED;
		}
	}

	if (irq_status & PHY_INT_WOL){
		pr_debug("%s: ignoring wol interrupt\n", __func__);
		return IRQ_HANDLED;
	}

	phy_trigger_machine(phydev);

	return IRQ_HANDLED;
}

EXPORT_SYMBOL_GPL(meson_gxl_phy_handle_interrupt);

static struct phy_driver meson_gxl_phy[] = {
	{
		PHY_ID_MATCH_EXACT(0x01814400),
		.name		= "Meson GXL Internal PHY",
		/* PHY_BASIC_FEATURES */
		.flags		= PHY_IS_INTERNAL | PHY_ALWAYS_CALL_SUSPEND,
		.soft_reset	= genphy_soft_reset,
		.config_init	= meson_gxl_config_init,
		.read_status	= meson_gxl_read_status,
		.config_intr	= smsc_phy_config_intr,
		.handle_interrupt = meson_gxl_phy_handle_interrupt,
		.suspend	= meson_gxl_phy_suspend,
		.resume		= meson_gxl_phy_resume,
		.read_mmd	= genphy_read_mmd_unsupported,
		.write_mmd	= genphy_write_mmd_unsupported,
		.get_wol	= meson_gxl_get_wol,
		.set_wol	= meson_gxl_set_wol,
	}, {
		PHY_ID_MATCH_EXACT(0x01803301),
		.name		= "Meson G12A Internal PHY",
		/* PHY_BASIC_FEATURES */
		.flags		= PHY_IS_INTERNAL,
		.probe		= smsc_phy_probe,
		.config_init	= smsc_phy_config_init,
		.soft_reset	= genphy_soft_reset,
		.read_status	= lan87xx_read_status,
		.config_intr	= smsc_phy_config_intr,
		.handle_interrupt = smsc_phy_handle_interrupt,

		.get_tunable	= smsc_phy_get_tunable,
		.set_tunable	= smsc_phy_set_tunable,

		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.read_mmd	= genphy_read_mmd_unsupported,
		.write_mmd	= genphy_write_mmd_unsupported,
	},
};

static struct mdio_device_id __maybe_unused meson_gxl_tbl[] = {
	{ PHY_ID_MATCH_VENDOR(0x01814400) },
	{ PHY_ID_MATCH_VENDOR(0x01803301) },
	{ }
};

module_phy_driver(meson_gxl_phy);

MODULE_DEVICE_TABLE(mdio, meson_gxl_tbl);

MODULE_DESCRIPTION("Amlogic Meson GXL Internal PHY driver");
MODULE_AUTHOR("Baoqi wang");
MODULE_AUTHOR("Neil Armstrong <narmstrong@baylibre.com>");
MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_LICENSE("GPL");
