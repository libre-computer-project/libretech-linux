// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2014 Felix Fietkau <nbd@openwrt.org>
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 */

#include <linux/debugfs.h>
#include <linux/delay.h>

#include "mt7601u.h"
#include "mcu.h"
#include "eeprom.h"
#include "mac.h"

static int
mt76_reg_set(void *data, u64 val)
{
	struct mt7601u_dev *dev = data;

	mt76_wr(dev, dev->debugfs_reg, val);
	return 0;
}

static int
mt76_reg_get(void *data, u64 *val)
{
	struct mt7601u_dev *dev = data;

	*val = mt76_rr(dev, dev->debugfs_reg);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_regval, mt76_reg_get, mt76_reg_set, "0x%08llx\n");

static int
mt7601u_mcu_reg_get(void *data, u64 *val)
{
	struct mt7601u_dev *dev = data;
	u32 raw[4] = {};
	int ret;

	ret = mt7601u_mcu_reg_read(dev, MT_MCU_MEMMAP_BBP | dev->debugfs_mcu_reg,
				   raw, ARRAY_SIZE(raw));
	if (ret)
		return ret;
	/* response = [rxfce_hdr, echoed_addr, VALUE, hdr]; value in raw[2]
	 * (silicon-pinned -- NOT raw[1], which echoes the request address). */
	*val = raw[2];
	return 0;
}

static int
mt7601u_mcu_reg_set(void *data, u64 val)
{
#if IS_ENABLED(CONFIG_MT7601U_RESEARCH)
	struct mt7601u_dev *dev = data;
	struct mt76_reg_pair rp = { .reg = dev->debugfs_mcu_reg, .value = (u32)val };

	return mt7601u_write_reg_pairs(dev, MT_MCU_MEMMAP_BBP, &rp, 1);
#else
	return -EPERM;		/* MCU-private write can poke live peripherals */
#endif
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_mcu_regval, mt7601u_mcu_reg_get,
			 mt7601u_mcu_reg_set, "0x%08llx\n");

static int
mt7601u_ampdu_stat_show(struct seq_file *file, void *data)
{
	struct mt7601u_dev *dev = file->private;
	int i, j;

#define stat_printf(grp, off, name)					\
	seq_printf(file, #name ":\t%llu\n", dev->stats.grp[off])

	stat_printf(rx_stat, 0, rx_crc_err);
	stat_printf(rx_stat, 1, rx_phy_err);
	stat_printf(rx_stat, 2, rx_false_cca);
	stat_printf(rx_stat, 3, rx_plcp_err);
	stat_printf(rx_stat, 4, rx_fifo_overflow);
	stat_printf(rx_stat, 5, rx_duplicate);

	stat_printf(tx_stat, 0, tx_fail_cnt);
	stat_printf(tx_stat, 1, tx_bcn_cnt);
	stat_printf(tx_stat, 2, tx_success);
	stat_printf(tx_stat, 3, tx_retransmit);
	stat_printf(tx_stat, 4, tx_zero_len);
	stat_printf(tx_stat, 5, tx_underflow);

	stat_printf(aggr_stat, 0, non_aggr_tx);
	stat_printf(aggr_stat, 1, aggr_tx);

	stat_printf(zero_len_del, 0, tx_zero_len_del);
	stat_printf(zero_len_del, 1, rx_zero_len_del);
#undef stat_printf

	seq_puts(file, "Aggregations stats:\n");
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 8; j++)
			seq_printf(file, "%08llx ",
				   dev->stats.aggr_n[i * 8 + j]);
		seq_putc(file, '\n');
	}

	seq_printf(file, "recent average AMPDU len: %d\n",
		   atomic_read(&dev->avg_ampdu_len));

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mt7601u_ampdu_stat);

static int
mt7601u_eeprom_param_show(struct seq_file *file, void *data)
{
	struct mt7601u_dev *dev = file->private;
	struct mt7601u_rate_power *rp = &dev->ee->power_rate_table;
	struct tssi_data *td = &dev->ee->tssi_data;
	int i;

	seq_printf(file, "RF freq offset: %hhx\n", dev->ee->rf_freq_off);
	seq_printf(file, "RSSI offset: %hhx %hhx\n",
		   dev->ee->rssi_offset[0], dev->ee->rssi_offset[1]);
	seq_printf(file, "Reference temp: %hhx\n", dev->ee->ref_temp);
	seq_printf(file, "LNA gain: %hhx\n", dev->ee->lna_gain);
	seq_printf(file, "Reg channels: %hhu-%d\n", dev->ee->reg.start,
		   dev->ee->reg.start + dev->ee->reg.num - 1);

	seq_puts(file, "Per rate power:\n");
	for (i = 0; i < 2; i++)
		seq_printf(file, "\t raw:%02hhx bw20:%02hhx bw40:%02hhx\n",
			   rp->cck[i].raw, rp->cck[i].bw20, rp->cck[i].bw40);
	for (i = 0; i < 4; i++)
		seq_printf(file, "\t raw:%02hhx bw20:%02hhx bw40:%02hhx\n",
			   rp->ofdm[i].raw, rp->ofdm[i].bw20, rp->ofdm[i].bw40);
	for (i = 0; i < 4; i++)
		seq_printf(file, "\t raw:%02hhx bw20:%02hhx bw40:%02hhx\n",
			   rp->ht[i].raw, rp->ht[i].bw20, rp->ht[i].bw40);

	seq_puts(file, "Per channel power:\n");
	for (i = 0; i < 7; i++)
		seq_printf(file, "\t tx_power  ch%u:%02hhx ch%u:%02hhx\n",
			   i * 2 + 1, dev->ee->chan_pwr[i * 2],
			   i * 2 + 2, dev->ee->chan_pwr[i * 2 + 1]);

	if (!dev->ee->tssi_enabled)
		return 0;

	seq_puts(file, "TSSI:\n");
	seq_printf(file, "\t slope:%02hhx\n", td->slope);
	seq_printf(file, "\t offset=%02hhx %02hhx %02hhx\n",
		   td->offset[0], td->offset[1], td->offset[2]);
	seq_printf(file, "\t delta_off:%08x\n", td->tx0_delta_offset);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mt7601u_eeprom_param);

/* Per-channel energy survey (feature #4, RX-only): park the PHY on each 2.4 GHz
 * channel for a short dwell and sample the MAC's clear-on-read RX MIB counters
 * (MT_RX_STA_CNT0/1). The false-CCA count (low 16 bits of MT_RX_STA_CNT1) is the
 * channel-occupancy / energy proxy: the PHY bumps it whenever it sees medium
 * energy above the CCA threshold that does not decode into a valid frame, so it
 * is live during an idle dwell -- unlike the per-frame AGC RSSI latch, which is
 * only written while a frame is actively being received. phy_err/crc_err (from
 * MT_RX_STA_CNT0) are reported alongside as secondary energy indicators. Restore
 * the last mac80211 channel before returning.
 *
 * Note: MT_RX_STA_CNT* are clear-on-read; the counts consumed here are ones
 * mt7601u_mac_work() would otherwise fold into dev->stats.
 */
#define MT7601U_SURVEY_DWELL_MS		50

static int
mt7601u_survey_show(struct seq_file *file, void *data)
{
	struct mt7601u_dev *dev = file->private;
	struct ieee80211_supported_band *sband = dev->sband_2g;
	struct cfg80211_chan_def orig, chandef;
	int i, ret = 0;

	if (!sband)
		return -ENODEV;

	mutex_lock(&dev->mutex);

	orig = dev->chandef;

	seq_puts(file, "chan\tfreq\tfalse_cca\tphy_err\tcrc_err\n");

	for (i = 0; i < sband->n_channels; i++) {
		struct ieee80211_channel *chan = &sband->channels[i];
		u32 cnt0, cnt1;

		cfg80211_chandef_create(&chandef, chan, NL80211_CHAN_NO_HT);

		ret = mt7601u_phy_set_channel(dev, &chandef);
		if (ret)
			break;

		/* clear-on-read: prime a baseline, dwell, then read the delta */
		mt76_rr(dev, MT_RX_STA_CNT0);
		mt76_rr(dev, MT_RX_STA_CNT1);

		msleep(MT7601U_SURVEY_DWELL_MS);

		cnt0 = mt76_rr(dev, MT_RX_STA_CNT0);
		cnt1 = mt76_rr(dev, MT_RX_STA_CNT1);

		seq_printf(file, "%d\t%d\t%u\t\t%u\t%u\n",
			   chan->hw_value, chan->center_freq,
			   cnt1 & 0xffff,	/* false CCA  */
			   cnt0 >> 16,		/* phy err    */
			   cnt0 & 0xffff);	/* crc err    */
	}

	/* restore the channel mac80211 last selected */
	if (orig.chan)
		mt7601u_phy_set_channel(dev, &orig);

	mutex_unlock(&dev->mutex);

	return ret;
}

DEFINE_SHOW_ATTRIBUTE(mt7601u_survey);

#if IS_ENABLED(CONFIG_MT7601U_RESEARCH)
/*
 * Stealth / silent capture (feature #5): clearing MT_AUTO_RSP_CFG (0x1404,
 * init default 0x13) stops the MAC auto-ACK/response engine, so in monitor mode
 * the device receives while transmitting nothing.
 *   echo 1 > stealth -> auto-response OFF ; echo 0 > stealth -> restore 0x13
 */
static int
mt7601u_stealth_set(void *data, u64 val)
{
	struct mt7601u_dev *dev = data;

	if (val)
		mt76_wr(dev, MT_AUTO_RSP_CFG, 0);
	else
		mt76_wr(dev, MT_AUTO_RSP_CFG, 0x00000013);
	return 0;
}

static int
mt7601u_stealth_get(void *data, u64 *val)
{
	struct mt7601u_dev *dev = data;

	*val = (mt76_rr(dev, MT_AUTO_RSP_CFG) == 0);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_stealth, mt7601u_stealth_get,
			 mt7601u_stealth_set, "%llu\n");

/*
 * Fast MAC randomization / multi-BSSID (feature #9): write a new station MAC
 * straight into MT_MAC_ADDR_DW0/DW1 via mt7601u_set_macaddr() (re-applies the
 * U2ME mask, no re-init). 48-bit MAC MSB-first; writing 0 installs a random one.
 */
static int
mt7601u_macaddr_set(void *data, u64 val)
{
	struct mt7601u_dev *dev = data;
	u8 addr[ETH_ALEN];

	addr[0] = val >> 40;
	addr[1] = val >> 32;
	addr[2] = val >> 24;
	addr[3] = val >> 16;
	addr[4] = val >> 8;
	addr[5] = val;

	mt7601u_set_macaddr(dev, addr);
	return 0;
}

static int
mt7601u_macaddr_get(void *data, u64 *val)
{
	struct mt7601u_dev *dev = data;

	*val = ((u64)dev->macaddr[0] << 40) | ((u64)dev->macaddr[1] << 32) |
	       ((u64)dev->macaddr[2] << 24) | ((u64)dev->macaddr[3] << 16) |
	       ((u64)dev->macaddr[4] << 8)  |  (u64)dev->macaddr[5];
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_macaddr, mt7601u_macaddr_get,
			 mt7601u_macaddr_set, "%012llx\n");

/*
 * RF/baseband test-mode controls -- LAB/SHIELDED-ENVIRONMENT ONLY.
 *
 * These files drive the transceiver into a continuous or tone emission for RF
 * characterization (PA / EVM / spectral-mask / LO-leakage). Emitting a carrier
 * or continuous tone on 2.4 GHz outside an RF-shielded chamber violates RF
 * regulations. Park the PHY on a channel first (`iw dev <wlan> set channel N`).
 */
static int mt76_rf_test_get(void *data, u64 *val)
{
	*val = 0;
	return 0;
}

/* rf_test_tx: 0=off, 1=single-tone/CW carrier, 2=continuous modulated TX.
 * LAB/SHIELDED-ENVIRONMENT ONLY -- emits on air. */
static int mt76_rf_test_tx_set(void *data, u64 val)
{
	return mt7601u_rf_test_set_tx(data, val);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_rf_test_tx, mt76_rf_test_get,
			 mt76_rf_test_tx_set, "%llu\n");

/* rf_test_power: 6-bit per-chain TX power (ALC CH_INIT). LAB ONLY. */
static int mt76_rf_test_power_set(void *data, u64 val)
{
	mt7601u_rf_test_set_power(data, val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_rf_test_power, mt76_rf_test_get,
			 mt76_rf_test_power_set, "%llu\n");

/* rf_test_rf: one raw RF-transceiver write, val=(bank<<16)|(reg<<8)|data,
 * for PLL/PA/LO poking via MT_RF_CSR_CFG. LAB/SHIELDED-ENVIRONMENT ONLY. */
static int mt76_rf_test_rf_set(void *data, u64 val)
{
	return mt7601u_rf_test_reg_wr(data, val);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_rf_test_rf, mt76_rf_test_get,
			 mt76_rf_test_rf_set, "0x%06llx\n");

/*
 * BBP-block dump (feature #10, CSI-feasibility GO/NO-GO probe, RX-only).
 *
 * The RX-side BBP/DSP blocks (RXFE 0x2800, RXO 0x2900, CAL 0x2c00, DSC 0x2e00,
 * and the 1T1R-vestigial PFMU 0x2f00) live in the 16-bit BBP internal address
 * space, which the everyday 8-bit BBP CSR accessor (MT_BBP_CSR_CFG, R0..R255)
 * cannot reach. The MCU memmap (MT_MCU_MEMMAP_BBP | addr, via CMD_RANDOM_READ)
 * can -- it is the same window the init path already WRITES through
 * (mt7601u_write_reg_pairs(dev, MT_MCU_MEMMAP_BBP, ...)). This file makes those
 * blocks READABLE so an operator can sweep them against a known CW tone and look
 * for a subcarrier-indexed complex array that tracks it. See the internal
 * MT7601U CSI feasibility study.
 *
 * READ-ONLY: issues CMD_RANDOM_READ only -- never writes, never emits. The raw
 * MCU response words are printed next to the decoded value so the response
 * framing can be validated on-silicon against a known register BEFORE any
 * RX-block interpretation. This is a probe, not a CSI decoder.
 */
#define MT7601U_BBP_BLOCK_WORDS		64	/* 64 words / 256 B per block */

static const struct {
	const char *name;
	u32 base;
} mt7601u_bbp_blocks[] = {
	{ "RXFE", MT_BBP_RXFE_BASE },
	{ "RXO",  MT_BBP_RXO_BASE },
	{ "CAL",  MT_BBP_CAL_BASE },
	{ "DSC",  MT_BBP_DSC_BASE },
	{ "PFMU", MT_BBP_PFMU_BASE },	/* 1T1R-vestigial: least likely */
};

static int
mt7601u_bbp_block_dump_show(struct seq_file *file, void *data)
{
	struct mt7601u_dev *dev = file->private;
	int b, n, ret = 0;

	if (!test_bit(MT7601U_STATE_MCU_RUNNING, &dev->state))
		return -ENODEV;

	mutex_lock(&dev->mutex);

	for (b = 0; b < ARRAY_SIZE(mt7601u_bbp_blocks); b++) {
		u32 base = mt7601u_bbp_blocks[b].base;

		seq_printf(file, "# %s block @ 0x%04x (MCU memmap 0x%08x)\n",
			   mt7601u_bbp_blocks[b].name, base,
			   MT_MCU_MEMMAP_BBP | base);
		seq_puts(file, "# idx\taddr\t\tlo16\thi16\traw[0..3]\n");

		for (n = 0; n < MT7601U_BBP_BLOCK_WORDS; n++) {
			u32 addr = MT_MCU_MEMMAP_BBP | (base + (n << 2));
			u32 raw[4] = {};

			ret = mt7601u_mcu_reg_read(dev, addr, raw,
						   ARRAY_SIZE(raw));
			if (ret < 0) {
				seq_printf(file, "%2d\t0x%08x\tERR %d\n",
					   n, addr, ret);
				goto out;
			}

			/* raw[0] is the MCU response header (rxfce); the
			 * register value is EXPECTED in raw[1], but the full
			 * raw window is printed so the framing (stride/offset)
			 * can be pinned down against a known register on-air.
			 */
			seq_printf(file,
				   "%2d\t0x%08x\t0x%04x\t0x%04x\t%08x %08x %08x %08x\n",
				   n, addr,
				   (u32)(raw[1] & 0xffff),
				   (u32)((raw[1] >> 16) & 0xffff),
				   raw[0], raw[1], raw[2], raw[3]);
		}
		seq_putc(file, '\n');
	}
out:
	mutex_unlock(&dev->mutex);
	return ret < 0 ? ret : 0;
}

DEFINE_SHOW_ATTRIBUTE(mt7601u_bbp_block_dump);
#endif /* CONFIG_MT7601U_RESEARCH */

void mt7601u_init_debugfs(struct mt7601u_dev *dev)
{
	struct dentry *dir;

	dir = debugfs_create_dir("mt7601u", dev->hw->wiphy->debugfsdir);

	debugfs_create_u8("temperature", 0400, dir, &dev->raw_temp);
	debugfs_create_u32("temp_mode", 0400, dir, &dev->temp_mode);

	debugfs_create_u32("regidx", 0600, dir, &dev->debugfs_reg);
	debugfs_create_file("regval", 0600, dir, dev, &fops_regval);
	debugfs_create_u32("mcu_regidx", 0600, dir, &dev->debugfs_mcu_reg);
	debugfs_create_file("mcu_regval", 0600, dir, dev, &fops_mcu_regval);
	debugfs_create_file("ampdu_stat", 0400, dir, dev, &mt7601u_ampdu_stat_fops);
	debugfs_create_file("eeprom_param", 0400, dir, dev, &mt7601u_eeprom_param_fops);
	debugfs_create_file("survey", 0400, dir, dev, &mt7601u_survey_fops);

#if IS_ENABLED(CONFIG_MT7601U_RESEARCH)
	/* research features (soft8021x MT7601U) -- gated, default n */
	debugfs_create_file("stealth", 0600, dir, dev, &fops_stealth);
	debugfs_create_file("macaddr", 0600, dir, dev, &fops_macaddr);

	/* RF/baseband test modes -- LAB/SHIELDED-ENVIRONMENT ONLY (emit on air). */
	debugfs_create_file("rf_test_tx", 0600, dir, dev, &fops_rf_test_tx);
	debugfs_create_file("rf_test_power", 0600, dir, dev, &fops_rf_test_power);
	debugfs_create_file("rf_test_rf", 0600, dir, dev, &fops_rf_test_rf);

	debugfs_create_file("bbp_block_dump", 0400, dir, dev,
			    &mt7601u_bbp_block_dump_fops);
#endif /* CONFIG_MT7601U_RESEARCH */
}
