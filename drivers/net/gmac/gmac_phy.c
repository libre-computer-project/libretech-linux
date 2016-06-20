/*
 * linux/arch/arm/mach-oxnas/gmac_phy.c
 *
 * Copyright (C) 2005 Oxford Semiconductor Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/delay.h>

//#define GMAC_DEBUG
#undef GMAC_DEBUG

#include "gmac.h"
#include "gmac_phy.h"
#include "gmac_reg.h"

static const int PHY_TRANSFER_TIMEOUT_MS = 100;

/*
 * Reads a register from the MII Management serial interface
 */
int phy_read(struct net_device *dev, int phyaddr, int phyreg)
{
    int data = 0;
#ifndef ARMULATING
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    unsigned long end;

    u32 addr = (phyaddr << MAC_GMII_ADR_PA_BIT) |
               (phyreg << MAC_GMII_ADR_GR_BIT) |
               (priv->gmii_csr_clk_range << MAC_GMII_ADR_CR_BIT) |
               (1UL << MAC_GMII_ADR_GB_BIT);

    mac_reg_write(priv, MAC_GMII_ADR_REG, addr);

    end = jiffies + MS_TO_JIFFIES(PHY_TRANSFER_TIMEOUT_MS);
    while (time_before(jiffies, end)) {
        if (!(mac_reg_read(priv, MAC_GMII_ADR_REG) & (1UL << MAC_GMII_ADR_GB_BIT))) {
            // Successfully read from PHY
            data = mac_reg_read(priv, MAC_GMII_DATA_REG) & 0xFFFF;
            break;
        }
    }

    DBG(1, KERN_INFO "phy_read() %s: phyaddr=0x%x, phyreg=0x%x, phydata=0x%x\n", dev->name, phyaddr, phyreg, data);
#endif // ARMULATING

    return data;
}

/*
 * Writes a register to the MII Management serial interface
 */
void phy_write(struct net_device *dev, int phyaddr, int phyreg, int phydata)
{
#ifndef ARMULATING
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    unsigned long end;

    u32 addr = (phyaddr << MAC_GMII_ADR_PA_BIT) |
               (phyreg << MAC_GMII_ADR_GR_BIT) |
               (priv->gmii_csr_clk_range << MAC_GMII_ADR_CR_BIT) |
               (1UL << MAC_GMII_ADR_GW_BIT) |
               (1UL << MAC_GMII_ADR_GB_BIT);

    mac_reg_write(priv, MAC_GMII_DATA_REG, phydata);
    mac_reg_write(priv, MAC_GMII_ADR_REG, addr);

    end = jiffies + MS_TO_JIFFIES(PHY_TRANSFER_TIMEOUT_MS);
    while (time_before(jiffies, end)) {
        if (!(mac_reg_read(priv, MAC_GMII_ADR_REG) & (1UL << MAC_GMII_ADR_GB_BIT))) {
            break;
        }
    }

    DBG(1, KERN_INFO "phy_write() %s: phyaddr=0x%x, phyreg=0x%x, phydata=0x%x\n", dev->name, phyaddr, phyreg, phydata);
#endif // ARMULATING
}

/*
 * Finds and reports the PHY address
 */
void phy_detect(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
#ifdef ARMULATING
    priv->mii.phy_id = 0;
    priv->phy_type = 0x22 << 16 | 0x1619;
    priv->phy_addr = 0;
#else // ARMULATING
    int phyaddr;

    DBG(2, KERN_INFO "phy_detect() %s: Entered\n", priv->netdev->name);

    // Scan all 32 PHY addresses if necessary
    priv->phy_type = 0;
    for (phyaddr = 1; phyaddr < 33; ++phyaddr) {
        unsigned int id1, id2;

        // Read the PHY identifiers
        id1 = phy_read(priv->netdev, phyaddr & 31, MII_PHYSID1);
        id2 = phy_read(priv->netdev, phyaddr & 31, MII_PHYSID2);

        DBG(2, KERN_INFO "phy_detect() %s: PHY adr = %u -> phy_id1=0x%x, phy_id2=0x%x\n", priv->netdev->name, phyaddr, id1, id2);

        // Make sure it is a valid identifier
        if (id1 != 0x0000 && id1 != 0xffff && id1 != 0x8000 &&
            id2 != 0x0000 && id2 != 0xffff && id2 != 0x8000) {
            DBG(2, KERN_NOTICE "phy_detect() %s: Found PHY at address = %u\n", priv->netdev->name, phyaddr);
            priv->mii.phy_id = phyaddr & 31;
            priv->phy_type = id1 << 16 | id2;
            priv->phy_addr = phyaddr;
            break;
        }
    }
#endif // ARMULATING
}

void start_phy_reset(gmac_priv_t* priv)
{
    // Ask the PHY to reset
    phy_write(priv->netdev, priv->phy_addr, MII_BMCR, BMCR_RESET);
}

int is_phy_reset_complete(gmac_priv_t* priv)
{
#ifdef ARMULATING
    return 1;
#else // ARMULATING
    int complete = 0;
    int bmcr;

    // Read back the status until it indicates reset, or we timeout
    bmcr = phy_read(priv->netdev, priv->phy_addr, MII_BMCR);
    if (!(bmcr & BMCR_RESET)) {
        complete = 1;
    }

    return complete;
#endif // ARMULATING
}

int phy_reset(struct net_device *dev)
{
#ifdef ARMULATING
    return 0;
#else // ARMULATING
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    int complete = 0;
    unsigned long end;

    // Start the reset operation
    start_phy_reset(priv);

    // Total time to wait for reset to complete
    end = jiffies + MS_TO_JIFFIES(PHY_TRANSFER_TIMEOUT_MS);

    // Should apparently wait at least 50mS before reading back from PHY; this
    // could just be a nasty feature of the SMC91x MAC/PHY and not apply to us
    msleep(50);

    // Read back the status until it indicates reset, or we timeout
    while (!(complete = is_phy_reset_complete(priv)) && time_before(jiffies, end)) {
        msleep(1);
    }

    return !complete;
#endif // ARMULATING
}

void phy_powerdown(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);

    unsigned int bmcr = phy_read(dev, priv->phy_addr, MII_BMCR);
    phy_write(dev, priv->phy_addr, MII_BMCR, bmcr | BMCR_PDOWN);

	if (priv->phy_type == PHY_TYPE_ICPLUS_IP1001) {
		// Cope with weird ICPlus PHY behaviour
		phy_read(dev, priv->phy_addr, MII_BMCR);
	}
}

void set_phy_negotiate_mode(struct net_device *dev)
{
    gmac_priv_t        *priv = (gmac_priv_t*)netdev_priv(dev);
    struct mii_if_info *mii = &priv->mii;
    struct ethtool_cmd *ecmd = &priv->ethtool_cmd;
    u32                 bmcr;

	bmcr = mii->mdio_read(dev, mii->phy_id, MII_BMCR);

    if (ecmd->autoneg == AUTONEG_ENABLE) {
        u32 advert, tmp;
        u32 advert2 = 0, tmp2 = 0;

//printk("set_phy_negotiate_mode() Auto negotiating link mode\n");
        // Advertise only what has been requested
        advert = mii->mdio_read(dev, mii->phy_id, MII_ADVERTISE);
        tmp = advert & ~(ADVERTISE_ALL | ADVERTISE_100BASE4 |
		                 ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

        if (ecmd->supported & (SUPPORTED_1000baseT_Full | ADVERTISE_1000HALF)) {
            advert2 = mii->mdio_read(dev, mii->phy_id, MII_CTRL1000);
            tmp2 = advert2 & ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
        }

        if (ecmd->advertising & ADVERTISED_10baseT_Half) {
            tmp |= ADVERTISE_10HALF;
        }
        if (ecmd->advertising & ADVERTISED_10baseT_Full) {
            tmp |= ADVERTISE_10FULL;
        }
        if (ecmd->advertising & ADVERTISED_100baseT_Half) {
            tmp |= ADVERTISE_100HALF;
        }
        if (ecmd->advertising & ADVERTISED_100baseT_Full) {
            tmp |= ADVERTISE_100FULL;
        }
        if ((ecmd->supported & SUPPORTED_1000baseT_Half) &&
            (ecmd->advertising & ADVERTISED_1000baseT_Half)) {
                tmp2 |= ADVERTISE_1000HALF;
        }
        if ((ecmd->supported & SUPPORTED_1000baseT_Full) &&
            (ecmd->advertising & ADVERTISED_1000baseT_Full)) {
                tmp2 |= ADVERTISE_1000FULL;
        }

        if (ecmd->advertising & ADVERTISED_Pause) {
            tmp |= ADVERTISE_PAUSE_CAP;
        }
        if (ecmd->advertising & ADVERTISED_Asym_Pause) {
            tmp |= ADVERTISE_PAUSE_ASYM;
        }

        if (advert != tmp) {
//printk("set_phy_negotiate_mode() Setting MII_ADVERTISE to 0x%08x\n", tmp);
            mii->mdio_write(dev, mii->phy_id, MII_ADVERTISE, tmp);
            mii->advertising = tmp;
        }
        if (advert2 != tmp2) {
//printk("set_phy_negotiate_mode() Setting MII_CTRL1000 to 0x%08x\n", tmp2);
            mii->mdio_write(dev, mii->phy_id, MII_CTRL1000, tmp2);
        }

        // Auto-negotiate the link state
        bmcr |= (BMCR_ANRESTART | BMCR_ANENABLE);
        mii->mdio_write(dev, mii->phy_id, MII_BMCR, bmcr);
    } else {
        u32 tmp;
//printk("set_phy_negotiate_mode() Unilaterally setting link mode\n");

        // Turn off auto negotiation, set speed and duplicitly unilaterally
        tmp = bmcr & ~(BMCR_ANENABLE | BMCR_SPEED100 | BMCR_SPEED1000 | BMCR_FULLDPLX);
        if (ecmd->speed == SPEED_1000) {
            tmp |= BMCR_SPEED1000;
        } else if (ecmd->speed == SPEED_100) {
            tmp |= BMCR_SPEED100;
        }

        if (ecmd->duplex == DUPLEX_FULL) {
            tmp |= BMCR_FULLDPLX;
            mii->full_duplex = 1;
        } else {
            mii->full_duplex = 0;
        }

        if (bmcr != tmp) {
            mii->mdio_write(dev, mii->phy_id, MII_BMCR, tmp);
        }
    }
}

u32 get_phy_capabilies(gmac_priv_t* priv)
{
    struct mii_if_info *mii = &priv->mii;

	// Ask the PHY for it's capabilities
	u32 reg = mii->mdio_read(priv->netdev, mii->phy_id, MII_BMSR);

	// Assume PHY has MII interface
	u32 features = SUPPORTED_MII;

	if (reg & BMSR_ANEGCAPABLE) {
		features |= SUPPORTED_Autoneg;
	}
	if (reg & BMSR_100FULL) {
		features |= SUPPORTED_100baseT_Full;
	}
	if (reg & BMSR_100HALF) {
		features |= SUPPORTED_100baseT_Half;
	}
	if (reg & BMSR_10FULL) {
		features |= SUPPORTED_10baseT_Full;
	}
	if (reg & BMSR_10HALF) {
		features |= SUPPORTED_10baseT_Half;
	}

	// Does the PHY have the extended status register?
	if (reg & BMSR_ESTATEN) {
		reg = mii->mdio_read(priv->netdev, mii->phy_id, MII_ESTATUS);

		if (reg & ESTATUS_1000_TFULL)
			features |= SUPPORTED_1000baseT_Full;
		if (reg & ESTATUS_1000_THALF)
			features |= SUPPORTED_1000baseT_Half;
	}

	return features;
}
