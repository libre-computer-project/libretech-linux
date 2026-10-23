// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASUS Vivobook S15 (X1E80100) Embedded Controller Driver
 *
 * The EC at I2C 0x76 uses single-byte I2C register addressing.  Fan
 * profile (FC24) and temperature push (FC20) are direct writes.  Fan
 * RPM and other internal EC registers are accessed through the FC52
 * RECM mailbox (AttribBytes(5)):
 *
 *   Write to reg 0x52: [addrHi, addrLo, count, 0x00, 0x00]
 *   Read  from reg 0x52: 5 bytes; result is the first byte.
 *
 * Battery charge limit uses the FC51/FC54 WECB mailbox:
 *
 *   FC51 (reg 0x51, 21-byte AttribBytes): pack ECWM opcode + args
 *   FC54 (reg 0x54, 5-byte AttribBytes):  trigger EC at internal addr
 *   FC52 RECM: poll internal addr 0x0C7C until EC clears it (=0)
 *
 * Protocol derived from DSDT ECWM/WECB/WECM methods (WMIT device).
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

/* Direct I2C register addresses at device 0x76 */
#define EC_IREG_FIFO		0x10	/* reads 0x01 when EC is ready */
#define EC_IREG_FANP		0x24	/* FC24: 1-byte fan profile */
#define EC_IREG_FANTRIP		0x25	/* F25S: 5-byte fan trip point set */
#define EC_IREG_LUT		0x28	/* F28G/F28S: fan thermistor LUT read/write */
#define EC_IREG_RECM		0x52	/* FC52: RECM mailbox (5-byte AttribBytes) */
#define EC_IREG_WECB		0x51	/* FC51: WECB mailbox (21-byte AttribBytes) */
#define EC_IREG_WECM		0x54	/* FC54: WECM trigger (5-byte AttribBytes) */
#define EC_IREG_TEMP		0x00	/* FC20: 5-byte temperature push */
#define EC_IREG_SCI		0x35	/* FC35: write 0x01 to enable EC SCI notifications */
#define EC_IREG_FC30		0x30	/* FC30: direct fan RPM override */

/*
 * F28G/F28S LUT constants.
 *
 * F28G (read):  write [EC_IREG_LUT, profile_id, lut_index], read EC_LUT_RD_SIZE bytes.
 * F28S (write): write [EC_IREG_LUT, profile_id, lut_index, EC_LUT_WR_SIZE bytes].
 *
 * The 65th byte in F28S vs 64 bytes from F28G is unconfirmed from RE — it may
 * be a checksum, commit flag, or padding.  Use 0x00 until empirically verified.
 * profile_id uses FANP encoding (0x11=Silent, 0x12=Balanced, etc.).
 * lut_index: 0 is the primary LUT for each profile; other indices unknown.
 *
 * DANGER: F28S writes the fan thermistor LUT directly.  An incorrect payload
 * disables fan ramp control until the EC is power-cycled.  Always F28G-read
 * first and verify the format before writing.  Do NOT confuse with F32S
 * (register 0x32), which writes thermistor trip thresholds and caused a
 * prior corruption incident (thresholds zeroed, fans lost all ramp response).
 */
#define EC_LUT_RD_SIZE		64	/* bytes returned by F28G */
#define EC_LUT_WR_SIZE		65	/* bytes consumed by F28S (65th byte TBD) */
#define EC_LUT_DEFAULT_PROFILE	EC_FANP_BALANCED	/* profile for sysfs default */
#define EC_LUT_DEFAULT_INDEX	0

/* FC20 payload constants: [0x20, 0x00, 0x02, temp_lo, temp_hi] */
#define EC_TEMP_CMD		0x20
#define EC_TEMP_FLAG		0x02

/* EC internal addresses accessed via RECM mailbox */
#define EC_IADDR_FAN1_RPM_LO	0x0602
#define EC_IADDR_FAN1_RPM_HI	0x0603
#define EC_IADDR_FAN2_RPM_LO	0x0624
#define EC_IADDR_FAN2_RPM_HI	0x0625
#define EC_IADDR_CMD_STATUS	0x0C7C	/* WMI command completion status */

/*
 * EC thermal sensor block: 15 single-byte readings, each in Celsius.
 * Addresses 0x0B45–0x0B53 (confirmed from DSDT RECM/WECM region map).
 * Mapping to named sensors (chassis thermistors, VRM, etc.) requires
 * empirical correlation — use ec_thermal_dump sysfs to read all 15.
 */
#define EC_IADDR_THERM_BASE	0x0B45
#define EC_NUM_THERM		15

/*
 * Additional thermal sensors at non-contiguous RECM addresses (from RE).
 * Physical identity unknown; exposed via ec_thermal_dump for empirical mapping.
 */
#define EC_IADDR_THERM_EXTRA0	0x0669
#define EC_IADDR_THERM_EXTRA1	0x06B1
#define EC_IADDR_THERM_EXTRA2	0x06E0
#define EC_IADDR_THERM_EXTRA3	0x06F0

/*
 * Chassis thermistor trip points from DSDT TZ39/TZ40 (milliCelsius).
 * _PSV = 47.85°C: fan LUT onset — fans begin ramping at this threshold.
 * _HOT = 61.85°C: emergency throttle threshold.
 * Used as hwmon temp1_max / temp1_crit for sensors(1) alarm output.
 */
#define EC_THERM_PSV_MC		47850	/* _PSV: fan onset */
#define EC_THERM_HOT_MC		61850	/* _HOT: throttle threshold */

/* FC51 (WECB) parameters */
#define EC_WECB_LEN		21	/* total payload bytes for WECB */
#define EC_WECM_OPCODE_BATT	0x83	/* ECWM opcode: battery charge limit */

/* Battery charge limit: DeviceID 0x0012008E in DSDT DEVS table */
#define EC_BATT_DEV_ID		0x0012008EUL
#define EC_BATT_LIMIT_MIN	40
#define EC_BATT_LIMIT_MAX	100
#define EC_BATT_LIMIT_DEF	80

/* Poll parameters for WECM completion at EC_IADDR_CMD_STATUS */
#define EC_POLL_TRIES		20
#define EC_POLL_DELAY_MS	10

/* Hardware reset via reset-gpios: pulse duration and FIFO poll timeout */
#define EC_RESET_PULSE_MS	50
#define EC_RESET_WAIT_TRIES	50
#define EC_RESET_WAIT_MS	10

/*
 * F25S fan trip point IDs (from DSDT FAN1/FAN2._DSM FF04 calls):
 *   0x11 = FAN1 LRPM, 0x21 = FAN1 URPM
 *   0x12 = FAN2 LRPM, 0x22 = FAN2 URPM
 * Write mode byte is always 0x02.  Payload: [reg=0x25, id, 0x02, rpm_lo, rpm_hi]
 */
#define EC_FANTRIP_FAN1_LRPM	0x11
#define EC_FANTRIP_FAN1_URPM	0x21
#define EC_FANTRIP_FAN2_LRPM	0x12
#define EC_FANTRIP_FAN2_URPM	0x22

/* FANP values (FC24 profile bytes, RE-verified): */
/* 0x11=Silent, 0x12=Balanced-lo, 0x13=Balanced-hi, 0x14=Performance-lo */
/* 0x15=Performance-hi, 0x16=Turbo-lo, 0x17=Turbo-hi */
#define EC_FANP_SILENT		0x11
#define EC_FANP_BALANCED	0x12
#define EC_FANP_BALANCED_HI	0x13
#define EC_FANP_PERF		0x14
#define EC_FANP_PERF_HI		0x15
#define EC_FANP_TURBO		0x16
#define EC_FANP_TURBO_HI	0x17

#define EC_NUM_FANS		2
#define EC_DEFAULT_TEMP_C	60
#define EC_TEMP_UPDATE_MS	30000

/*
 * WECB opcode 0x85: set fan RPM range. DSDT FAN0.CRNF() sends:
 *   ECWM(0, (URPM<<16)|LRPM, 0, 0, 0, 0x85)
 * Without this the EC has no RPM target and idles at minimum speed
 * regardless of FANP profile. URPM values come from DSDT FAN0.FOPR:
 *   {3000, 3420, 3840, 6000} RPM for states 0-3.
 */
#define EC_WECB_CMD_FAN_RPM_RANGE	0x85

/*
 * I2C 0x5B (SL5B) — ECCW mailbox for the keyboard backlight AND the fan
 * test-mode actuator (EC-side manual fan-duty force).
 *
 * ECCW uses the two-step ECWB (FC10/FC11) GenericSerialBus writes to the
 * C4:30/31/32 mailbox.  Wire framing (AttribBytes: command byte + N data
 * bytes only, NO SMBus length byte — see ec_sl5b_write/ec_sl5b_read):
 *   SELECT  C4:reg  -> {0x10, page, reg}
 *   WRITE   C4:reg  -> {0x11, val}
 *   READ    C4:reg  -> select, then read 1 byte (result at index 0)
 *
 * Sequence per op: write func (C4:31) → data (C4:32) → trigger/class (C4:30);
 * the trigger byte is the operation CLASS (fan = 0x01, kbd/bus = 0x02) and the
 * EC clears C4:30 to 0 on completion.  RE + hardware-confirmed (Phase 2.6,
 * asus/re/fan-actuator-re.md): duty 60→780 RPM, 100→2700 RPM (units 0–100%).
 */
#define EC_SL5B_ADDR		0x5B
#define EC_ECCW_PAGE		0xC4
#define EC_ECCW_REG_TRIGGER	0x30	/* write class byte; EC clears on completion */
#define EC_ECCW_REG_FUNC	0x31	/* function code */
#define EC_ECCW_REG_DATA	0x32	/* data byte */
/* C4:30 trigger byte = operation class (disjoint namespaces per class) */
#define EC_ECCW_CLASS_FAN	0x01	/* fan test-mode / duty actuator */
#define EC_ECCW_CLASS_KBD	0x02	/* keyboard backlight + ECCW bus-enable */
#define EC_ECCW_FUNC_BUS_INIT	0x83	/* enable ECCW bus; must precede KBL writes */
#define EC_ECCW_FUNC_KBL	0x82	/* set keyboard backlight PWM duty */
#define EC_ECCW_INIT_VAL	0x01
/*
 * Completion/idle poll budget.  The EC can take up to ~300 ms to consume the
 * C4:30 mailbox (Phase 2.6 observed arm→clear at +0.3 s), so allow generously.
 */
#define EC_ECCW_POLL_TRIES	40
#define EC_ECCW_POLL_MS		10

/*
 * Fan test-mode actuator (ECCW class 0x01), RE'd from AsusWinIO64.dll and
 * hardware-confirmed (Phase 2.6).  Each is func (C4:31) + data (C4:32):
 *   SetFanTestMode(en) : func 0x82, data (en?0x02:0x00)
 *   SetFanIndex(idx)   : func 0x8C, data idx
 *   SetFanPwmDuty(d)   : func 0x8A, data d (0–100%)
 */
#define EC_FAN_FUNC_TESTMODE	0x82
#define EC_FAN_FUNC_INDEX	0x8C
#define EC_FAN_FUNC_PWMDUTY	0x8A
#define EC_FAN_TESTMODE_ON	0x02	/* (1 & 0x7F) << 1 */
#define EC_FAN_TESTMODE_OFF	0x00
#define EC_FAN_INDEX_1		0x01	/* fan #1 (drives fan2_input tach) */
#define EC_FAN_DUTY_MAX		100	/* duty units are 0–100 (percent) */
/*
 * Spin-up floor: measured 30→0 RPM (below floor), 60→780, 100→2700.  Any
 * FORCED duty must be ≥ this or the fan won't actually spin.
 */
#define EC_FAN_DUTY_MIN_SPIN	60

/*
 * I2C 0x5B page 0xC9 — WEBC/REBC fan duty curve mailbox.
 * Protocol reverse-engineered from DSDT \_SB.I2C6.WEBC/REBC AML (DSDT.dsl:49210).
 *
 * Slot selectors (from DSDT EC0 WMI dispatch, DSDT.dsl:56905-56968):
 *   0x80-0x82: three fan-curve slots for WMI command 0x00110024
 *   0x84-0x86: three fan-curve slots for WMI command 0x00110025
 *   0x88-0x8A: three fan-curve slots for WMI command 0x00110032
 * Command bytes (DSDT GDFC/SUFC):
 *   WEBC cmd 0x20 = select fan curve slot (data byte = slot selector)
 *   REBC cmd 0x21 = read 16 bytes
 *   WEBC cmd 0x22 = write 16 bytes
 *
 * Note: the ACPI GDFC path (acpi_evaluate_object "\_SB.EC0.GDFC") cannot be
 * used because this kernel boots via DT; the ACPI namespace is not initialised.
 */
#define EC_CURVE_PAGE		0xC9
#define EC_CURVE_REG_CMD	0x6E	/* write command byte here LAST */
#define EC_CURVE_REG_STATUS	0x6F	/* busy/ready flags */
#define EC_CURVE_STATUS_READY	0x80	/* bit7: data written / EC data ready */
#define EC_CURVE_STATUS_HOST	0x20	/* bit5: host ready to receive */
#define EC_CURVE_STATUS_ACK	0x40	/* bit6: ack (written after read or timeout) */
#define EC_CURVE_REG_DATA	0x40	/* 16-byte data buffer: 0x40..0x4F */
#define EC_CURVE_DATA_LEN	16
#define EC_CURVE_SEL_FAN1	0x40	/* WEBC write slot: Fan1 (DSDT SUFC Arg4) */
#define EC_CURVE_SEL_FAN2	0x44	/* WEBC write slot: Fan2 */
#define EC_CURVE_SEL_FAN3	0x48	/* WEBC write slot: Fan3 */
#define EC_CURVE_CMD_SELECT	0x20
#define EC_CURVE_CMD_READ	0x21
#define EC_CURVE_CMD_WRITE	0x22	/* WEBC: write 16-byte duty curve */
#define EC_CURVE_POLL_TRIES	0xC8	/* busy poll limit (DSDT BMTR init = 0xC8) */
#define EC_CURVE_DATA_POLL	0x32	/* data-ready poll limit (DSDT second BMTR) */

/* Keyboard backlight brightness levels 0–3 → PWM duty byte */
static const u8 ec_kbl_pwm[] = { 0x00, 0x55, 0xAA, 0xFF };
#define EC_KBL_MAX_BRIGHTNESS	(ARRAY_SIZE(ec_kbl_pwm) - 1)
#define EC_KBL_DEFAULT		1	/* dim on first boot */

/* Fan operating points (URPM per cooling state), from DSDT FAN0.FOPR.
 * RE: asus/re/bios_analysis.txt:71 — FOPR={0x0BB8,0x0D5C,0x0F00,0x1770} */
static const u16 fan_urpm_by_state[] = { 3000, 3420, 3840, 6000 };

/* Cooling states: 0=silent 1=balanced 2=performance 3=turbo */
static const u8 fanp_by_state[] = {
	EC_FANP_SILENT,
	EC_FANP_BALANCED,
	EC_FANP_PERF,
	EC_FANP_TURBO,
};

/*
 * Cooling-state → forced fan duty (%), for the ECCW test-mode actuator.
 * State 0 (CPU below the first passive trip) is handled separately: the fan
 * is RELEASED to EC-auto so the EC's chassis loop keeps the low end quiet.
 * States 1–3 map to the 60/72/84 °C passive trips and force duty the EC
 * otherwise ignores; all are ≥ the measured ~60% spin-up floor (30→0 RPM,
 * 60→780, 100→2700), so the fan always actually spins when forced.
 */
static const u8 fan_duty_by_state[] = { 0, 60, 80, 100 };

/* EC SCI event codes drained from CMD5 (reg 0x05) by the periodic poll */
#define EC_EVENT_FAN1_STAT	0x30
#define EC_EVENT_FAN2_STAT	0x31
#define EC_EVENT_FAN1_SPEED	0x32
#define EC_EVENT_FAN2_SPEED	0x33
#define EC_EVENT_LUT_DONE	0x34
#define EC_EVENT_PROFILE_DONE	0x35
#define EC_EVENT_THERM_BASE	0x36	/* 0x36–0x3C: thermistor 1–7 threshold */
#define EC_EVENT_EC_RESET	0x3D
#define EC_EVENT_PWR		0xDA	/* power/battery event */
#define EC_EVENT_FAN_HIB	0xDB	/* fan hibernate */

/* Max CMD5 event codes to drain per poll cycle (bounds an event burst). */
#define EC_EVENT_DRAIN_MAX	16

struct asus_ec {
	struct i2c_client		*client;
	struct i2c_client		*sl5b;		/* 0x5B client for ECCW mailbox */
	struct mutex			lock;
	struct thermal_cooling_device	*cdev;
	struct delayed_work		temp_work;
	struct gpio_desc		*reset_gpio;
	u8				fanp;
	unsigned long			cooling_state;
	u8				batt_limit;	/* charge limit in % */
	u16				fan_lrpm[EC_NUM_FANS]; /* last written LRPM per fan */
	u8				kbl_level;	/* keyboard backlight 0–3 */
	bool				fan_test_mode;	/* true when the ECCW fan actuator holds test mode */
	u8				fan_duty;	/* last forced duty (0–100) while in test mode */
};

/* Read 1 byte from direct I2C register (single-byte addressing). */
static int ec_direct_read(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0,        .len = 1, .buf = &reg  },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = 1, .buf = val   },
	};
	int ret = i2c_transfer(client->adapter, msgs, 2);
	return ret == 2 ? 0 : (ret < 0 ? ret : -EIO);
}

/* Write 1 byte to a direct I2C register. */
static int ec_direct_write(struct i2c_client *client, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	struct i2c_msg msg = { .addr = client->addr, .flags = 0, .len = 2, .buf = buf };
	int ret = i2c_transfer(client->adapter, &msg, 1);
	return ret == 1 ? 0 : (ret < 0 ? ret : -EIO);
}

/* Write multiple bytes starting at a direct I2C register. */
static int ec_direct_write_buf(struct i2c_client *client, u8 reg,
				const u8 *data, size_t len)
{
	u8 buf[32];
	struct i2c_msg msg;
	int ret;

	if (WARN_ON(len + 1 > sizeof(buf)))
		return -EINVAL;

	buf[0] = reg;
	memcpy(buf + 1, data, len);

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = len + 1;
	msg.buf   = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	return ret == 1 ? 0 : (ret < 0 ? ret : -EIO);
}

/*
 * Read 1 byte from an EC internal 16-bit address via the FC52 RECM mailbox.
 *
 * AttribBytes(5) protocol: write 5 bytes to reg 0x52, then read 5 bytes
 * back.  The result is the first byte of the read response.
 *
 * DSDT RECM() maps its buffer offsets as:
 *   buf[0] = GSB status  (not transmitted)
 *   buf[1] = GSB count   (not transmitted; ACPI sets to 0x04)
 *   buf[2] = addrHi      (wire byte 0)
 *   buf[3] = addrLo      (wire byte 1)
 *   buf[4] = byte count  (wire byte 2)
 *   buf[5,6] = 0x00      (wire bytes 3,4)
 * Response: buf[2] = result byte.
 */
static int ec_recm_read(struct i2c_client *client, u16 addr, u8 *val)
{
	u8 wbuf[6] = { EC_IREG_RECM, addr >> 8, addr & 0xff, 1, 0, 0 };
	u8 rreg = EC_IREG_RECM;
	u8 rbuf[5];
	struct i2c_msg wmsg = {
		.addr = client->addr, .flags = 0, .len = 6, .buf = wbuf,
	};
	struct i2c_msg rmsgs[2] = {
		{ .addr = client->addr, .flags = 0,        .len = 1, .buf = &rreg },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = 5, .buf = rbuf  },
	};
	int ret;

	ret = i2c_transfer(client->adapter, &wmsg, 1);
	if (ret != 1)
		return ret < 0 ? ret : -EIO;

	ret = i2c_transfer(client->adapter, rmsgs, 2);
	if (ret != 2)
		return ret < 0 ? ret : -EIO;

	*val = rbuf[0];
	return 0;
}

/*
 * Write a 21-byte WECB payload to the EC via FC51 (reg 0x51).
 *
 * DSDT WMIT.WECB() uses an ACPI GenericSerialBus buffer (23 bytes total):
 *   buf[0] = GSB status  (not transmitted on wire)
 *   buf[1] = GSB count = 0x17 (not transmitted on wire; ACPI internal)
 *   buf[2..22] = 21 actual wire bytes: [opcode, arg0_LE32 .. arg4_LE32]
 *
 * The 22-byte I2C write is: reg (0x51) followed by these 21 wire bytes.
 */
static int ec_wecb_write(struct i2c_client *client, u8 opcode,
			  u32 arg0, u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	u8 payload[EC_WECB_LEN] = {
		opcode,			/* wire[0]  = opcode (buf[2] in DSDT) */
		(u8)(arg0),		/* wire[1]  */
		(u8)(arg0 >> 8),	/* wire[2]  */
		(u8)(arg0 >> 16),	/* wire[3]  */
		(u8)(arg0 >> 24),	/* wire[4]  */
		(u8)(arg1),		/* wire[5]  */
		(u8)(arg1 >> 8),	/* wire[6]  */
		(u8)(arg1 >> 16),	/* wire[7]  */
		(u8)(arg1 >> 24),	/* wire[8]  */
		(u8)(arg2),		/* wire[9]  */
		(u8)(arg2 >> 8),	/* wire[10] */
		(u8)(arg2 >> 16),	/* wire[11] */
		(u8)(arg2 >> 24),	/* wire[12] */
		(u8)(arg3),		/* wire[13] */
		(u8)(arg3 >> 8),	/* wire[14] */
		(u8)(arg3 >> 16),	/* wire[15] */
		(u8)(arg3 >> 24),	/* wire[16] */
		(u8)(arg4),		/* wire[17] */
		(u8)(arg4 >> 8),	/* wire[18] */
		(u8)(arg4 >> 16),	/* wire[19] */
		(u8)(arg4 >> 24),	/* wire[20] */
	};

	return ec_direct_write_buf(client, EC_IREG_WECB, payload, sizeof(payload));
}

/*
 * Trigger EC command execution via FC54 (reg 0x54, WECM AttribBytes(5)).
 *
 * DSDT WMIT.WECM(val, addr): writes 5 bytes at reg 0x54:
 *   [addrHi, addrLo, count=1, value, pad=0]
 * Used after WECB to poke EC_IADDR_CMD_STATUS (0x0C7C) with value=1.
 */
static int ec_wecm_trigger(struct i2c_client *client, u16 addr, u8 value)
{
	u8 payload[5] = {
		(u8)(addr >> 8),
		(u8)(addr & 0xff),
		0x01,
		value,
		0x00,
	};

	return ec_direct_write_buf(client, EC_IREG_WECM, payload, sizeof(payload));
}

/*
 * Poll EC_IADDR_CMD_STATUS via RECM until it reads 0 (command complete).
 * Returns 0 on success, -ETIMEDOUT if EC does not respond.
 */
static int ec_wecm_wait(struct i2c_client *client)
{
	int i;
	u8 status;
	int ret;

	for (i = 0; i < EC_POLL_TRIES; i++) {
		ret = ec_recm_read(client, EC_IADDR_CMD_STATUS, &status);
		if (ret)
			return ret;
		if (status == 0)
			return 0;
		msleep(EC_POLL_DELAY_MS);
	}
	return -ETIMEDOUT;
}

/*
 * Set the battery charge limit (%).
 *
 * Maps to WMI DEVS(0x0012008E, limit%) which calls:
 *   ECWM(arg0=0x0012008E, arg1=limit%, arg2=0, arg3=0, arg4=0, opcode=0x83)
 * which calls WMIT.WECB(opcode=0x83, 0x0012008E, limit%, 0, 0, 0)
 * then WMIT.WECM(1, 0x0C7C).
 *
 * Caller must hold ec->lock.
 */
static int ec_set_batt_limit(struct asus_ec *ec, u8 limit)
{
	int ret;

	ret = ec_wecb_write(ec->client, EC_WECM_OPCODE_BATT,
			    (u32)EC_BATT_DEV_ID, (u32)limit, 0, 0, 0);
	if (ret)
		return ret;

	ret = ec_wecm_trigger(ec->client, EC_IADDR_CMD_STATUS, 0x01);
	if (ret)
		return ret;

	ret = ec_wecm_wait(ec->client);
	if (!ret)
		ec->batt_limit = limit;
	return ret;
}

/*
 * Push current SoC temperature to the EC (FC20, AttribRawBytes(5)).
 * 5-byte payload at I2C register 0x00: [0x20, 0x00, 0x02, temp_lo, temp_hi]
 * Temperature unit: tens of Celsius (Qualcomm PEP ETW: 55°C → send 5).
 */
static int ec_temp_push(struct asus_ec *ec, int temp_c)
{
	u16 ec_val = (u16)(temp_c / 10);
	u8 payload[5] = {
		EC_TEMP_CMD,
		0x00,
		EC_TEMP_FLAG,
		(u8)(ec_val & 0xff),
		(u8)(ec_val >> 8),
	};

	return ec_direct_write_buf(ec->client, EC_IREG_TEMP, payload, sizeof(payload));
}

/*
 * ECWB — write one byte to I2C 0x5B page:reg (FC10/FC11 two-step protocol).
 *
 * GenericSerialBus AttribBytes framing: the command byte is followed by
 * EXACTLY the data bytes — the AML buffer's [status][length] header is NOT
 * transmitted on the wire.  So SELECT = {0x10, page, reg} (3 bytes) and
 * WRITE = {0x11, val} (2 bytes) — no SMBus length byte.  The earlier
 * {0x10, 0x02, page, reg}/{0x11, 0x01, val} framing put that length byte on
 * the wire, corrupting the (page,reg) select so C4:30 never armed (Phase 2.5
 * root-cause; matches the confirmed-working RECM AttribBytes(5) path).
 */
static int ec_sl5b_write(struct i2c_client *sl5b, u8 page, u8 reg, u8 val)
{
	u8 sel[] = { 0x10, page, reg };
	u8 wr[]  = { 0x11, val };
	int ret;

	ret = i2c_master_send(sl5b, sel, sizeof(sel));
	if (ret != sizeof(sel))
		return ret < 0 ? ret : -EIO;
	ret = i2c_master_send(sl5b, wr, sizeof(wr));
	return ret == sizeof(wr) ? 0 : (ret < 0 ? ret : -EIO);
}

/*
 * ECRB — read one byte from I2C 0x5B page:reg (FC10/FC11 two-step protocol).
 * SELECT {0x10, page, reg}, then a 1-byte read; the result is the first (only)
 * data byte — no length byte precedes it (see ec_sl5b_write framing note).
 */
static int ec_sl5b_read(struct i2c_client *sl5b, u8 page, u8 reg, u8 *val)
{
	u8 sel[] = { 0x10, page, reg };
	u8 cmd = 0x11;
	u8 rbuf[1];
	struct i2c_msg rmsgs[2] = {
		{ .addr = sl5b->addr, .flags = 0,        .len = 1, .buf = &cmd  },
		{ .addr = sl5b->addr, .flags = I2C_M_RD, .len = 1, .buf = rbuf  },
	};
	int ret;

	ret = i2c_master_send(sl5b, sel, sizeof(sel));
	if (ret != sizeof(sel))
		return ret < 0 ? ret : -EIO;
	ret = i2c_transfer(sl5b->adapter, rmsgs, 2);
	if (ret != 2)
		return ret < 0 ? ret : -EIO;
	*val = rbuf[0];
	return 0;
}

/*
 * ECCW — trigger the C4:30/31/32 command mailbox on I2C 0x5B.
 * Sequence (RE: asus/re/fan-actuator-re.md, asus_sci_analysis.txt:63):
 *   1. Pre-poll C4:30 == 0 (mailbox idle)
 *   2. Write C4:31 = func
 *   3. Write C4:32 = val (data)
 *   4. Write C4:30 = class (0x01 fan / 0x02 kbd) — the trigger
 *   5. Post-poll C4:30 == 0 (completion)
 *
 * The I2C register writes (steps 2–4) are the load-bearing effect and are
 * fatal on failure.  Step 1 (idle pre-poll) is best-effort — a stale C4:30
 * readback must not block a fresh command.  Step 5 (completion poll) is
 * authoritative for the FAN class only: with the Phase 2.6-corrected framing
 * the EC arms C4:30 to the class byte on trigger and clears it to 0x00 once it
 * consumes the mailbox, so for a fan write a completion timeout means the EC
 * did NOT act — return -ETIMEDOUT so ec_fan_force_duty releases the fan to
 * EC-auto instead of leaving it in test mode at an unconfirmed duty (a silent
 * success there could strand the fan off).  The kbd class (backlight / bus
 * init) has no such safety dependency and its C4:30 clear is unverified on
 * real silicon, so a kbd completion timeout stays best-effort (warn + return
 * 0) — it must NOT turn a landed backlight write into a spurious failure.
 */
static int ec_eccw(struct i2c_client *sl5b, u8 class, u8 func, u8 val)
{
	u8 status;
	int ret, i;

	/* Step 1: pre-poll — wait for mailbox idle (best-effort) */
	for (i = 0; i < EC_ECCW_POLL_TRIES; i++) {
		ret = ec_sl5b_read(sl5b, EC_ECCW_PAGE, EC_ECCW_REG_TRIGGER,
				   &status);
		if (!ret && status == 0x00)
			break;
		msleep(EC_ECCW_POLL_MS);
	}

	/* Steps 2–4: func first, then data, then trigger/class */
	ret = ec_sl5b_write(sl5b, EC_ECCW_PAGE, EC_ECCW_REG_FUNC, func);
	if (ret)
		return ret;
	ret = ec_sl5b_write(sl5b, EC_ECCW_PAGE, EC_ECCW_REG_DATA, val);
	if (ret)
		return ret;
	ret = ec_sl5b_write(sl5b, EC_ECCW_PAGE, EC_ECCW_REG_TRIGGER, class);
	if (ret)
		return ret;

	/*
	 * Step 5: post-poll — C4:30 must clear to 0x00 (EC consumed the mailbox).
	 * The strict -ETIMEDOUT signal is scoped to the fan class only: the fan
	 * path (ec_fan_force_duty) enters test mode first and MUST learn when a
	 * duty write didn't land so it can release the fan back to EC-auto.  The
	 * kbd class (backlight brightness / BUS_INIT) has no such safety
	 * dependency and its C4:30 completion clear is unverified on real
	 * silicon, so it keeps the prior best-effort contract (warn + return 0):
	 * a stale/slow poll must not turn a landed backlight write into a
	 * spurious failure.
	 */
	for (i = 0; i < EC_ECCW_POLL_TRIES; i++) {
		msleep(EC_ECCW_POLL_MS);
		ret = ec_sl5b_read(sl5b, EC_ECCW_PAGE, EC_ECCW_REG_TRIGGER,
				   &status);
		if (!ret && status == 0x00)
			return 0;
	}
	if (class == EC_ECCW_CLASS_FAN)
		return -ETIMEDOUT;
	dev_dbg(&sl5b->dev,
		"ECCW class 0x%02x completion poll timed out; best-effort success\n",
		class);
	return 0;
}

/* Fan test-mode actuator (ECCW class 0x01). Caller must hold ec->lock. */
static int ec_fan_set_test_mode(struct asus_ec *ec, bool on)
{
	return ec_eccw(ec->sl5b, EC_ECCW_CLASS_FAN, EC_FAN_FUNC_TESTMODE,
		       on ? EC_FAN_TESTMODE_ON : EC_FAN_TESTMODE_OFF);
}

/*
 * Release the fan back to EC-auto (exit test mode).  The EC's own chassis
 * loop resumes control.  Caller must hold ec->lock.
 */
static int ec_fan_release(struct asus_ec *ec)
{
	int ret;

	if (!ec->sl5b)
		return -ENODEV;
	ret = ec_fan_set_test_mode(ec, false);
	ec->fan_test_mode = false;	/* best-effort: assume released even on err */
	return ret;
}

/*
 * Force the fan to a fixed duty (0–100%) via the ECCW test-mode actuator:
 * enter manual test mode → select fan 1 → apply duty.  On any error path the
 * fan is released back to EC-auto so it can never be stuck in test mode at a
 * low/zero duty.  Caller must hold ec->lock.
 */
static int ec_fan_force_duty(struct asus_ec *ec, u8 duty)
{
	int ret;

	if (!ec->sl5b)
		return -ENODEV;
	if (duty > EC_FAN_DUTY_MAX)
		duty = EC_FAN_DUTY_MAX;

	ret = ec_fan_set_test_mode(ec, true);
	if (ret)
		goto release;
	ret = ec_eccw(ec->sl5b, EC_ECCW_CLASS_FAN, EC_FAN_FUNC_INDEX,
		      EC_FAN_INDEX_1);
	if (ret)
		goto release;
	ret = ec_eccw(ec->sl5b, EC_ECCW_CLASS_FAN, EC_FAN_FUNC_PWMDUTY, duty);
	if (ret)
		goto release;

	ec->fan_test_mode = true;
	ec->fan_duty = duty;
	return 0;

release:
	/* SAFETY: never leave the fan in manual test mode on an error path */
	ec_fan_set_test_mode(ec, false);
	ec->fan_test_mode = false;
	return ret;
}

/*
 * Re-assert the forced fan duty after an EC reset/resume cleared test mode in
 * HW while the driver still holds fan_test_mode.  When the governor is
 * commanding cooling (cooling_state >= 1) the restored duty must be at least
 * the spin-up floor: otherwise a prior manual pwm1=0 would be faithfully
 * restored as a sticky fan-off on every reset/resume, even with a hot CPU.  A
 * genuine user pwm1=0 at cooling_state 0 (fan intentionally at auto's low end)
 * is left as-is.  Caller must hold ec->lock and have checked ec->sl5b.
 */
static void ec_fan_reforce(struct asus_ec *ec)
{
	u8 duty = ec->fan_duty;

	if (ec->cooling_state >= 1 && duty < EC_FAN_DUTY_MIN_SPIN)
		duty = EC_FAN_DUTY_MIN_SPIN;
	ec_fan_force_duty(ec, duty);
}

/*
 * ec_webc — write data + command to C9 fan curve mailbox (DSDT WEBC AML).
 * Sequence: pre-poll C9:6F until 0 → write data to C9:40.. → set bit7 in
 * C9:6F (data ready) → write cmd to C9:6E LAST.
 */
static int ec_webc(struct i2c_client *sl5b, u8 cmd, u8 count, const u8 *data)
{
	u8 status;
	int tries, ret, i;

	for (tries = EC_CURVE_POLL_TRIES; tries > 0; tries--) {
		ret = ec_sl5b_read(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_STATUS, &status);
		if (ret)
			return ret;
		if (!status)
			break;
		usleep_range(100, 200);
	}
	if (!tries) {
		ec_sl5b_read(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_STATUS, &status);
		ec_sl5b_write(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_STATUS,
			      status | EC_CURVE_STATUS_ACK);
		return -ETIMEDOUT;
	}

	for (i = 0; i < count; i++) {
		ret = ec_sl5b_write(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_DATA + i,
				    data[i]);
		if (ret)
			return ret;
	}

	ret = ec_sl5b_write(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_STATUS,
			    EC_CURVE_STATUS_READY);
	if (ret)
		return ret;
	return ec_sl5b_write(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_CMD, cmd);
}

/*
 * ec_rebc — send read command and read data from C9 fan curve mailbox (DSDT REBC AML).
 * Sequence: pre-poll C9:6F until 0 → write 0x20 to C9:6F (host ready) →
 * write cmd to C9:6E → poll bit7 in C9:6F (EC data ready) → read bytes →
 * ack by setting bit6 in C9:6F.
 */
static int ec_rebc(struct i2c_client *sl5b, u8 cmd, u8 *buf, int count)
{
	u8 status;
	int tries, ret, i;

	for (tries = EC_CURVE_POLL_TRIES; tries > 0; tries--) {
		ret = ec_sl5b_read(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_STATUS, &status);
		if (ret)
			return ret;
		if (!status)
			break;
		usleep_range(100, 200);
	}
	if (!tries) {
		ec_sl5b_read(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_STATUS, &status);
		ec_sl5b_write(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_STATUS,
			      status | EC_CURVE_STATUS_ACK);
		return -ETIMEDOUT;
	}

	ret = ec_sl5b_write(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_STATUS,
			    EC_CURVE_STATUS_HOST);
	if (ret)
		return ret;
	ret = ec_sl5b_write(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_CMD, cmd);
	if (ret)
		return ret;

	for (tries = EC_CURVE_DATA_POLL; tries > 0; tries--) {
		ret = ec_sl5b_read(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_STATUS, &status);
		if (ret)
			return ret;
		if (status & EC_CURVE_STATUS_READY)
			break;
		usleep_range(100, 200);
	}
	if (!tries)
		return -ETIMEDOUT;

	for (i = 0; i < count; i++) {
		ret = ec_sl5b_read(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_DATA + i,
				   &buf[i]);
		if (ret)
			return ret;
	}

	ret = ec_sl5b_read(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_STATUS, &status);
	if (ret)
		return ret;
	return ec_sl5b_write(sl5b, EC_CURVE_PAGE, EC_CURVE_REG_STATUS,
			     status | EC_CURVE_STATUS_ACK);
}

/*
 * ec_gdfc_read — read 16-byte fan duty curve for one slot via direct I2C.
 * Implements DSDT \_SB.EC0.GDFC(sel): WEBC(0x20, 1, [sel]) + REBC(0x21, 16).
 * Caller must hold ec->lock.
 */
static int ec_gdfc_read(struct asus_ec *ec, u8 sel, u8 *buf)
{
	int ret;

	ret = ec_webc(ec->sl5b, EC_CURVE_CMD_SELECT, 1, &sel);
	if (ret)
		return ret;
	return ec_rebc(ec->sl5b, EC_CURVE_CMD_READ, buf, EC_CURVE_DATA_LEN);
}

/*
 * ec_sufc_write — write a 16-byte fan duty curve to one write slot via direct I2C.
 * Implements DSDT \_SB.EC0.SUFC: WEBC(0x20, 1, [sel]) + WEBC(0x22, 16, data).
 * sel: 0x40=Fan1, 0x44=Fan2, 0x48=Fan3.
 *
 * ec_webc() pre-polls C9:6F for 0x00 (ready).  C9:6F is host-write-only and
 * always reads 0x00, so the pre-poll passes immediately and writes succeed even
 * though ec_rebc (the read path) is broken for this device.
 * Caller must hold ec->lock.
 */
static int ec_sufc_write(struct asus_ec *ec, u8 sel, const u8 *data)
{
	int ret;

	ret = ec_webc(ec->sl5b, EC_CURVE_CMD_SELECT, 1, &sel);
	if (ret)
		return ret;
	return ec_webc(ec->sl5b, EC_CURVE_CMD_WRITE, EC_CURVE_DATA_LEN, data);
}

static int ec_set_profile(struct asus_ec *ec, u8 fanp)
{
	int ret = ec_direct_write(ec->client, EC_IREG_FANP, fanp);

	if (!ret)
		ec->fanp = fanp;
	return ret;
}

/*
 * Set fan RPM target range via WECB command 0x85 (DSDT FAN0.CRNF).
 * arg0=0, arg1=(urpm<<16)|lrpm — EC uses this as the allowed speed window.
 * RE: asus/re/bios_analysis.txt FAN0.CRNF — ECWM(0, (URPM<<16)|LRPM, 0, 0, 0, 0x85)
 * RE: asus/re/fan_control_analysis.txt — FOPR={3000,3420,3840,6000}; lrpm min=1 (never 0)
 *
 * DSDT ECWM() wraps every WECB write with a WECM trigger at 0x0C7C and
 * polls until the EC clears it.  Without the trigger the EC ignores the
 * WECB payload and fans stay at minimum idle speed.
 */
static int ec_set_fan_rpm_range(struct asus_ec *ec, u16 lrpm, u16 urpm)
{
	u32 arg1 = ((u32)urpm << 16) | lrpm;
	int ret;

	ret = ec_wecb_write(ec->client, EC_WECB_CMD_FAN_RPM_RANGE,
			    0, arg1, 0, 0, 0);
	if (ret)
		return ret;

	ret = ec_wecm_trigger(ec->client, EC_IADDR_CMD_STATUS, 0x01);
	if (ret)
		return ret;

	return ec_wecm_wait(ec->client);
}

/*
 * F28G — read 64-byte thermistor→fan-duty LUT from the EC.
 *
 * Write a 3-byte select command to register 0x28 (profile_id + lut_index),
 * allow the EC ~1 ms to prepare the response, then read 64 bytes back.
 * Analogous to the RECM write-then-read pattern at register 0x52.
 */
static int ec_f28g_read_lut(struct i2c_client *client, u8 profile_id,
			     u8 lut_index, u8 *lut)
{
	u8 cmd[] = { EC_IREG_LUT, profile_id, lut_index };
	u8 rreg = EC_IREG_LUT;
	struct i2c_msg wmsg = {
		.addr = client->addr, .flags = 0,
		.len = sizeof(cmd), .buf = cmd,
	};
	struct i2c_msg rmsgs[2] = {
		{ .addr = client->addr, .flags = 0,        .len = 1,              .buf = &rreg },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = EC_LUT_RD_SIZE, .buf = lut   },
	};
	int ret;

	ret = i2c_transfer(client->adapter, &wmsg, 1);
	if (ret != 1)
		return ret < 0 ? ret : -EIO;
	usleep_range(1000, 2000);
	ret = i2c_transfer(client->adapter, rmsgs, 2);
	return ret == 2 ? 0 : (ret < 0 ? ret : -EIO);
}

/*
 * F28S — write a 65-byte thermistor LUT to the EC.
 *
 * Single 68-byte I2C write: [reg=0x28, profile_id, lut_index, 65 LUT bytes].
 * The 65th byte's purpose is unconfirmed from RE; callers should pass 0x00
 * for the extra byte until it is empirically identified.
 *
 * WARNING: an incorrect payload may corrupt fan ramp response until EC
 * power-cycle.  Always read and verify via F28G before writing.
 */
static int ec_f28s_write_lut(struct i2c_client *client, u8 profile_id,
			      u8 lut_index, const u8 *lut)
{
	/* 1 (reg) + 1 (profile_id) + 1 (lut_index) + EC_LUT_WR_SIZE (65) = 68 */
	u8 payload[1 + 1 + 1 + EC_LUT_WR_SIZE];
	struct i2c_msg msg = {
		.addr = client->addr, .flags = 0,
		.len = sizeof(payload), .buf = payload,
	};

	payload[0] = EC_IREG_LUT;
	payload[1] = profile_id;
	payload[2] = lut_index;
	memcpy(&payload[3], lut, EC_LUT_WR_SIZE);

	return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}


/*
 * Set fan trip point (LRPM or URPM) via F25S / EC reg 0x25.
 * DSDT FF04: payload is [0x25, id, 0x02, rpm_lo, rpm_hi].
 * id = EC_FANTRIP_FAN{1,2}_{L,U}RPM.
 */
static int ec_set_fan_trip(struct i2c_client *client, u8 id, u16 rpm)
{
	u8 data[] = { id, 0x02, rpm & 0xFF, (rpm >> 8) & 0xFF };

	return ec_direct_write_buf(client, EC_IREG_FANTRIP, data, sizeof(data));
}

static int ec_read_fan_rpm(struct asus_ec *ec, int fan, long *rpm)
{
	static const u16 lo_addrs[EC_NUM_FANS] = { EC_IADDR_FAN1_RPM_LO, EC_IADDR_FAN2_RPM_LO };
	static const u16 hi_addrs[EC_NUM_FANS] = { EC_IADDR_FAN1_RPM_HI, EC_IADDR_FAN2_RPM_HI };
	u8 lo, hi;
	int ret;

	ret = ec_recm_read(ec->client, lo_addrs[fan], &lo);
	if (ret)
		return ret;
	ret = ec_recm_read(ec->client, hi_addrs[fan], &hi);
	if (ret)
		return ret;

	*rpm = ((long)hi << 8) | lo;
	return 0;
}

/* Drain + handle any queued EC SCI events (defined in the interrupt section). */
static void ec_drain_events(struct asus_ec *ec);

static void ec_temp_work_fn(struct work_struct *work)
{
	struct asus_ec *ec = container_of(work, struct asus_ec, temp_work.work);
	struct thermal_zone_device *tz;
	int temp_c = EC_DEFAULT_TEMP_C;
	int temp_millic;

	tz = thermal_zone_get_zone_by_name("cpu0-0-top-thermal");
	if (!IS_ERR(tz) && !thermal_zone_get_temp(tz, &temp_millic))
		temp_c = clamp(temp_millic / 1000, 0, 120);

	mutex_lock(&ec->lock);
	ec_temp_push(ec, temp_c);
	mutex_unlock(&ec->lock);

	/*
	 * This board has no decoded EC SCI interrupt, so poll-drain the EC event
	 * FIFO (CMD5) here every cycle — otherwise thermal-trip/battery/hotkey/lid
	 * event codes are queued in the EC and never read.
	 */
	ec_drain_events(ec);

	schedule_delayed_work(&ec->temp_work, msecs_to_jiffies(EC_TEMP_UPDATE_MS));
}

/* hwmon ------------------------------------------------------------------- */

static umode_t asus_ec_hwmon_is_visible(const void *data,
					 enum hwmon_sensor_types type,
					 u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		if (attr == hwmon_fan_input)
			return 0444;
		if (attr == hwmon_fan_min)
			return 0644;
		break;
	case hwmon_pwm:
		if (attr == hwmon_pwm_input || attr == hwmon_pwm_enable)
			return 0644;
		break;
	case hwmon_temp:
		if (attr == hwmon_temp_input ||
		    attr == hwmon_temp_max   ||
		    attr == hwmon_temp_crit  ||
		    attr == hwmon_temp_label)
			return 0444;
		break;
	default:
		break;
	}
	return 0;
}

static int asus_ec_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, long *val)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int ret;

	switch (type) {
	case hwmon_fan:
		if (attr == hwmon_fan_input) {
			mutex_lock(&ec->lock);
			ret = ec_read_fan_rpm(ec, channel, val);
			mutex_unlock(&ec->lock);
			return ret;
		}
		if (attr == hwmon_fan_min) {
			mutex_lock(&ec->lock);
			*val = ec->fan_lrpm[channel];
			mutex_unlock(&ec->lock);
			return 0;
		}
		break;

	case hwmon_pwm:
		if (attr == hwmon_pwm_enable) {
			/* 1 = manual (host forces duty), 2 = EC-auto */
			mutex_lock(&ec->lock);
			*val = ec->fan_test_mode ? 1 : 2;
			mutex_unlock(&ec->lock);
			return 0;
		}
		if (attr == hwmon_pwm_input) {
			mutex_lock(&ec->lock);
			if (ec->fan_test_mode)
				/* map the EC's 0–100 duty back to hwmon 0–255 */
				*val = (long)ec->fan_duty * 255 / EC_FAN_DUTY_MAX;
			else
				*val = 0;	/* EC-auto: duty not host-driven */
			mutex_unlock(&ec->lock);
			return 0;
		}
		break;

	case hwmon_temp:
		if (attr == hwmon_temp_input) {
			u8 raw;
			mutex_lock(&ec->lock);
			ret = ec_recm_read(ec->client,
					   EC_IADDR_THERM_BASE + channel, &raw);
			mutex_unlock(&ec->lock);
			if (!ret)
				*val = (long)raw * 1000; /* milliCelsius */
			return ret;
		}
		if (attr == hwmon_temp_max) {
			*val = EC_THERM_PSV_MC;
			return 0;
		}
		if (attr == hwmon_temp_crit) {
			*val = EC_THERM_HOT_MC;
			return 0;
		}
		break;

	default:
		break;
	}
	return -EOPNOTSUPP;
}

static int asus_ec_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
				u32 attr, int channel, long val)
{
	static const u8 lrpm_ids[EC_NUM_FANS] = {
		EC_FANTRIP_FAN1_LRPM, EC_FANTRIP_FAN2_LRPM
	};
	struct asus_ec *ec = dev_get_drvdata(dev);
	int ret = 0;

	if (type == hwmon_fan && attr == hwmon_fan_min) {
		u16 rpm = (u16)clamp(val, 0L, 6000L);

		mutex_lock(&ec->lock);
		ret = ec_set_fan_trip(ec->client, lrpm_ids[channel], rpm);
		if (!ret)
			ec->fan_lrpm[channel] = rpm;
		mutex_unlock(&ec->lock);
		return ret;
	}

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;

	if (attr == hwmon_pwm_enable) {
		if (val != 1 && val != 2)
			return -EINVAL;
		mutex_lock(&ec->lock);
		if (val == 1)
			/* manual: enter test mode at the last (or a spin) duty */
			ret = ec_fan_force_duty(ec, ec->fan_duty ? ec->fan_duty
							: EC_FAN_DUTY_MIN_SPIN);
		else
			/* auto: release the fan back to the EC's own loop */
			ret = ec_fan_release(ec);
		mutex_unlock(&ec->lock);
		return ret;
	}

	if (attr != hwmon_pwm_input)
		return -EOPNOTSUPP;

	/* Writing pwm1 forces a manual duty via the ECCW actuator (enters test
	 * mode). Map hwmon 0–255 to the EC's 0–100 duty scale. */
	mutex_lock(&ec->lock);
	ret = ec_fan_force_duty(ec,
				(u8)(clamp(val, 0L, 255L) * EC_FAN_DUTY_MAX / 255));
	mutex_unlock(&ec->lock);

	return ret;
}

static const struct hwmon_channel_info * const asus_ec_hwmon_info[] = {
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT | HWMON_F_MIN,
			       HWMON_F_INPUT | HWMON_F_MIN),
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_MAX | HWMON_T_CRIT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_MAX | HWMON_T_CRIT | HWMON_T_LABEL),
	NULL,
};

static int asus_ec_hwmon_read_string(struct device *dev,
				      enum hwmon_sensor_types type,
				      u32 attr, int channel, const char **str)
{
	static const char * const temp_labels[] = { "therm0", "therm1" };

	if (type == hwmon_temp && attr == hwmon_temp_label)
		*str = temp_labels[channel];
	return 0;
}

static const struct hwmon_ops asus_ec_hwmon_ops = {
	.is_visible   = asus_ec_hwmon_is_visible,
	.read         = asus_ec_hwmon_read,
	.write        = asus_ec_hwmon_write,
	.read_string  = asus_ec_hwmon_read_string,
};

static const struct hwmon_chip_info asus_ec_hwmon_chip = {
	.ops  = &asus_ec_hwmon_ops,
	.info = asus_ec_hwmon_info,
};

/* Thermal cooling device -------------------------------------------------- */

static int asus_ec_cooling_get_max_state(struct thermal_cooling_device *cdev,
					  unsigned long *state)
{
	*state = ARRAY_SIZE(fanp_by_state) - 1;
	return 0;
}

static int asus_ec_cooling_get_cur_state(struct thermal_cooling_device *cdev,
					  unsigned long *state)
{
	struct asus_ec *ec = cdev->devdata;

	*state = ec->cooling_state;
	return 0;
}

/*
 * HYBRID cooling policy — the actual fix for "CPU cooks, fans off".  The EC's
 * autonomous loop keys the fan off the CHASSIS thermistor, never CPU Tj, so it
 * leaves the fan off while the CPU is hot.  So:
 *   state 0  (CPU below the first trip)  -> RELEASE to EC-auto: let the EC's
 *            own chassis loop handle the low end (quiet when genuinely cool).
 *   state ≥1 (CPU crossed a passive trip) -> enter test mode + FORCE the duty
 *            mapped from the 60/72/84 °C trips — the lever the EC ignores.
 * Duties are ≥ the ~60% spin-up floor, so escalating CPU temp visibly ramps
 * the fan; on cool-down the governor drops to state 0 and releases to auto.
 */
static int asus_ec_cooling_set_cur_state(struct thermal_cooling_device *cdev,
					  unsigned long state)
{
	struct asus_ec *ec = cdev->devdata;
	int ret;

	state = min_t(unsigned long, state, ARRAY_SIZE(fan_duty_by_state) - 1);

	mutex_lock(&ec->lock);
	if (!ec->sl5b) {
		/* ECCW actuator unavailable: fall back to the (EC-inert) FANP
		 * profile so behaviour is no worse than before. */
		ret = ec_set_profile(ec, fanp_by_state[state]);
		if (!ret)
			ec_set_fan_rpm_range(ec, 1, fan_urpm_by_state[state]);
	} else if (state == 0) {
		ret = ec_fan_release(ec);
	} else {
		ret = ec_fan_force_duty(ec, fan_duty_by_state[state]);
	}
	if (!ret)
		ec->cooling_state = state;
	mutex_unlock(&ec->lock);

	return ret;
}

static const struct thermal_cooling_device_ops asus_ec_cooling_ops = {
	.get_max_state = asus_ec_cooling_get_max_state,
	.get_cur_state = asus_ec_cooling_get_cur_state,
	.set_cur_state = asus_ec_cooling_set_cur_state,
};

/* Hardware reset ---------------------------------------------------------- */

/*
 * Pulse reset-gpios low for EC_RESET_PULSE_MS, then poll FIFO until EC is
 * ready.  Caller must hold ec->lock.
 */
static int ec_do_reset(struct asus_ec *ec)
{
	int i, ret;
	u8 fifo_val;

	if (!ec->reset_gpio)
		return -EOPNOTSUPP;

	gpiod_set_value_cansleep(ec->reset_gpio, 1);
	msleep(EC_RESET_PULSE_MS);
	gpiod_set_value_cansleep(ec->reset_gpio, 0);

	for (i = 0; i < EC_RESET_WAIT_TRIES; i++) {
		msleep(EC_RESET_WAIT_MS);
		ret = ec_direct_read(ec->client, EC_IREG_FIFO, &fifo_val);
		if (!ret && fifo_val == 0x01)
			break;
	}
	if (i == EC_RESET_WAIT_TRIES)
		return -ETIMEDOUT;

	ec_direct_write(ec->client, EC_IREG_SCI, 0x01);
	ec_temp_push(ec, EC_DEFAULT_TEMP_C);
	ec_set_profile(ec, fanp_by_state[ec->cooling_state]);
	ec_set_fan_rpm_range(ec, 1, fan_urpm_by_state[ec->cooling_state]);
	/*
	 * The EC reset cleared the ECCW fan test mode in HW, but the SW state
	 * still reflects the governor-commanded duty.  Only the (inert) FANP
	 * path was re-established above; if we were holding manual cooling
	 * (state ≥1), re-force the duty so a hot CPU at reset time does not
	 * revert to chassis-only fan.  Caller holds ec->lock; ec_fan_reforce
	 * requires (does not take) it, and enters test mode + applies the duty
	 * in one sequence (with a spin floor for the governed case), so there is
	 * no window where the fan is left off.
	 */
	if (ec->sl5b && ec->fan_test_mode)
		ec_fan_reforce(ec);
	return 0;
}

static ssize_t ec_reset_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;
	if (val != 1)
		return -EINVAL;

	mutex_lock(&ec->lock);
	ret = ec_do_reset(ec);
	mutex_unlock(&ec->lock);

	return ret ? ret : count;
}

static DEVICE_ATTR_WO(ec_reset);

/* Battery charge limit sysfs --------------------------------------------- */

/*
 * /sys/bus/i2c/devices/<dev>/charge_control_end_threshold
 *
 * Read/write the battery charge limit in percent (40–100).
 * Writing 100 effectively disables the limit (full charge).
 * The EC enforces the limit; the driver only sends the command.
 *
 * Protocol: FC51 (WECB 21 bytes at reg 0x51) → FC54 trigger (5 bytes at
 * reg 0x54) → poll FC52 RECM at 0x0C7C until EC clears status.
 */
static ssize_t charge_control_end_threshold_show(struct device *dev,
						   struct device_attribute *attr,
						   char *buf)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	u8 limit;

	mutex_lock(&ec->lock);
	limit = ec->batt_limit;
	mutex_unlock(&ec->lock);

	return sysfs_emit(buf, "%u\n", limit);
}

static ssize_t charge_control_end_threshold_store(struct device *dev,
						    struct device_attribute *attr,
						    const char *buf, size_t count)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int limit;
	int ret;

	ret = kstrtouint(buf, 10, &limit);
	if (ret)
		return ret;

	if (limit < EC_BATT_LIMIT_MIN || limit > EC_BATT_LIMIT_MAX)
		return -EINVAL;

	mutex_lock(&ec->lock);
	ret = ec_set_batt_limit(ec, (u8)limit);
	mutex_unlock(&ec->lock);

	return ret ? ret : count;
}

static DEVICE_ATTR_RW(charge_control_end_threshold);

/*
 * fan_lut_dump — read the 64-byte thermistor LUT for the balanced profile.
 *
 * Output: 64 space-separated hex bytes followed by newline.
 * Used to empirically verify the LUT format before attempting F28S writes.
 * Profile EC_LUT_DEFAULT_PROFILE (0x12 = balanced), index 0.
 */
static ssize_t fan_lut_dump_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	u8 lut[EC_LUT_RD_SIZE];
	ssize_t len = 0;
	int ret, i;

	mutex_lock(&ec->lock);
	ret = ec_f28g_read_lut(ec->client, EC_LUT_DEFAULT_PROFILE,
			       EC_LUT_DEFAULT_INDEX, lut);
	mutex_unlock(&ec->lock);
	if (ret)
		return ret;

	for (i = 0; i < EC_LUT_RD_SIZE; i++)
		len += sysfs_emit_at(buf, len, "%02x%c", lut[i],
				     i == EC_LUT_RD_SIZE - 1 ? '\n' : ' ');
	return len;
}
static DEVICE_ATTR_RO(fan_lut_dump);

/*
 * fan_lut_upload — write a modified 65-byte LUT to the balanced profile.
 *
 * Input: exactly EC_LUT_WR_SIZE (65) raw binary bytes.
 * The 65th byte's purpose is unconfirmed from RE; set it to 0x00 until
 * the format is verified via fan_lut_dump analysis.
 *
 * DANGER: an incorrect payload may corrupt fan ramp response.  Always
 * read fan_lut_dump first and verify bytes before uploading.
 */
static ssize_t fan_lut_upload_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	if (count != EC_LUT_WR_SIZE)
		return -EINVAL;

	mutex_lock(&ec->lock);
	ret = ec_f28s_write_lut(ec->client, EC_LUT_DEFAULT_PROFILE,
				EC_LUT_DEFAULT_INDEX, buf);
	mutex_unlock(&ec->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(fan_lut_upload);

/*
 * ec_thermal_dump — read all EC_NUM_THERM thermal sensor values via RECM.
 *
 * Each line: "0xADDR: NN°C" for addresses EC_IADDR_THERM_BASE through
 * EC_IADDR_THERM_BASE + EC_NUM_THERM - 1.  Used to identify which
 * address maps to which physical sensor (chassis thermistors, VRM, etc.)
 * before adding permanent hwmon temp channels.
 */
static ssize_t ec_thermal_dump_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	static const struct { u16 addr; const char *label; } extra_sensors[] = {
		{ EC_IADDR_THERM_EXTRA0, "extra0" },
		{ EC_IADDR_THERM_EXTRA1, "extra1" },
		{ EC_IADDR_THERM_EXTRA2, "extra2" },
		{ EC_IADDR_THERM_EXTRA3, "extra3" },
	};
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;
	int i, ret;
	u8 val;

	mutex_lock(&ec->lock);
	ret = 0;
	for (i = 0; i < EC_NUM_THERM; i++) {
		ret = ec_recm_read(ec->client, EC_IADDR_THERM_BASE + i, &val);
		if (ret)
			break;
		len += sysfs_emit_at(buf, len, "0x%04x: %u\n",
				     EC_IADDR_THERM_BASE + i, val);
	}
	for (i = 0; !ret && i < ARRAY_SIZE(extra_sensors); i++) {
		ret = ec_recm_read(ec->client, extra_sensors[i].addr, &val);
		if (ret)
			break;
		len += sysfs_emit_at(buf, len, "0x%04x (%s): %u\n",
				     extra_sensors[i].addr,
				     extra_sensors[i].label, val);
	}
	mutex_unlock(&ec->lock);
	return ret ? ret : len;
}
static DEVICE_ATTR_RO(ec_thermal_dump);

/*
 * fan_duty_dump — read all 9 GDFC fan duty curve slots via direct I2C.
 * Selector values from DSDT EC0 WMI dispatch (DSDT.dsl:56905-56968).
 * Selectors 0x11/0x12 used previously were wrong (those are FANP profile bytes,
 * not C9 mailbox slot IDs).
 * Output: one hex line per slot, e.g. "0x80: 00 00 ... 00"
 */
static ssize_t fan_duty_dump_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	static const u8 selectors[] = {
		0x80, 0x81, 0x82,	/* WMI command 0x00110024 slots 0-2 */
		0x84, 0x85, 0x86,	/* WMI command 0x00110025 slots 0-2 */
		0x88, 0x89, 0x8A,	/* WMI command 0x00110032 slots 0-2 */
	};
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	u8 curve[EC_CURVE_DATA_LEN];
	ssize_t len = 0;
	int i, j, ret;

	if (!ec->sl5b)
		return -ENODEV;

	mutex_lock(&ec->lock);
	for (i = 0; i < ARRAY_SIZE(selectors); i++) {
		ret = ec_gdfc_read(ec, selectors[i], curve);
		if (ret) {
			len += sysfs_emit_at(buf, len, "0x%02x: error %d\n",
					     selectors[i], ret);
			continue;
		}
		len += sysfs_emit_at(buf, len, "0x%02x:", selectors[i]);
		for (j = 0; j < EC_CURVE_DATA_LEN; j++)
			len += sysfs_emit_at(buf, len, " %02x", curve[j]);
		len += sysfs_emit_at(buf, len, "\n");
	}
	mutex_unlock(&ec->lock);
	return len;
}
static DEVICE_ATTR_RO(fan_duty_dump);

/*
 * fan_duty_write — write a 16-byte SUFC fan duty curve slot via direct I2C.
 *
 * Input: 17 space-separated hex tokens: <selector> <b0> <b1> ... <b15>
 *   selector: write slot — 0x40=Fan1, 0x44=Fan2, 0x48=Fan3
 *   b0..b15:  16-byte payload (semantic format TBD; hypothesis: 8×(temp°C, duty))
 *
 * Example (Fan1, onset at 40°C):
 *   echo "40 00 00 28 20 30 40 38 60 40 80 48 a0 50 c0 60 ff" > fan_duty_write
 */
static ssize_t fan_duty_write_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int vals[1 + EC_CURVE_DATA_LEN];
	u8 curve[EC_CURVE_DATA_LEN];
	int i, ret;

	if (!ec->sl5b)
		return -ENODEV;

	ret = sscanf(buf,
		     "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
		     &vals[0], &vals[1], &vals[2], &vals[3],
		     &vals[4], &vals[5], &vals[6], &vals[7],
		     &vals[8], &vals[9], &vals[10], &vals[11],
		     &vals[12], &vals[13], &vals[14], &vals[15], &vals[16]);
	if (ret != 1 + EC_CURVE_DATA_LEN)
		return -EINVAL;
	for (i = 0; i < 1 + EC_CURVE_DATA_LEN; i++)
		if (vals[i] > 0xff)
			return -EINVAL;

	for (i = 0; i < EC_CURVE_DATA_LEN; i++)
		curve[i] = (u8)vals[i + 1];

	mutex_lock(&ec->lock);
	ret = ec_sufc_write(ec, (u8)vals[0], curve);
	mutex_unlock(&ec->lock);
	return ret ? ret : (ssize_t)count;
}
static DEVICE_ATTR_WO(fan_duty_write);

/*
 * fan_profile — read/write the active fan profile (FC24 FANP byte).
 * Accepts: "silent", "balanced", "balanced-performance", "performance",
 *          "max-power" (matching platform_profile names), or a raw hex
 *          byte (e.g. "0x11").  Reads back the name.
 */
static ssize_t fan_profile_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	const char *name;

	switch (ec->fanp) {
	case EC_FANP_SILENT:      name = "silent";               break;
	case EC_FANP_BALANCED:    name = "balanced";             break;
	case EC_FANP_BALANCED_HI: name = "balanced-performance"; break;
	case EC_FANP_PERF:        name = "performance";          break;
	case EC_FANP_PERF_HI:     name = "performance-max";      break;
	case EC_FANP_TURBO:       name = "max-power";            break;
	case EC_FANP_TURBO_HI:    name = "turbo";                break;
	default:                  name = "unknown";              break;
	}
	return sysfs_emit(buf, "%s\n", name);
}

static ssize_t fan_profile_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));
	u8 fanp;
	int ret;

	if (sysfs_streq(buf, "silent"))               fanp = EC_FANP_SILENT;
	else if (sysfs_streq(buf, "balanced"))         fanp = EC_FANP_BALANCED;
	else if (sysfs_streq(buf, "balanced-performance")) fanp = EC_FANP_BALANCED_HI;
	else if (sysfs_streq(buf, "performance"))      fanp = EC_FANP_PERF;
	else if (sysfs_streq(buf, "performance-max"))  fanp = EC_FANP_PERF_HI;
	else if (sysfs_streq(buf, "max-power"))        fanp = EC_FANP_TURBO;
	else if (sysfs_streq(buf, "turbo"))            fanp = EC_FANP_TURBO_HI;
	else {
		unsigned long v;

		if (kstrtoul(buf, 0, &v) || v > 0xff)
			return -EINVAL;
		fanp = (u8)v;
	}

	mutex_lock(&ec->lock);
	ret = ec_set_profile(ec, fanp);
	if (!ret) {
		u16 urpm;

		switch (fanp) {
		case EC_FANP_SILENT:      urpm = fan_urpm_by_state[0]; break;
		case EC_FANP_BALANCED:
		case EC_FANP_BALANCED_HI: urpm = fan_urpm_by_state[1]; break;
		case EC_FANP_PERF:
		case EC_FANP_PERF_HI:     urpm = fan_urpm_by_state[2]; break;
		default:                  urpm = fan_urpm_by_state[3]; break;
		}
		ec_set_fan_rpm_range(ec, 1, urpm);
	}
	mutex_unlock(&ec->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(fan_profile);

static struct attribute *asus_ec_attrs[] = {
	&dev_attr_charge_control_end_threshold.attr,
	&dev_attr_ec_reset.attr,
	&dev_attr_fan_lut_dump.attr,
	&dev_attr_fan_lut_upload.attr,
	&dev_attr_ec_thermal_dump.attr,
	&dev_attr_fan_duty_dump.attr,
	&dev_attr_fan_duty_write.attr,
	&dev_attr_fan_profile.attr,
	NULL,
};
ATTRIBUTE_GROUPS(asus_ec);

/* Interrupt handler ------------------------------------------------------- */

/*
 * Read CMD5 (reg 0x05, AttribByte) to retrieve the EC event code.
 * On the wire: write [0x05], read 1 data byte = event code.
 * The DSDT INTR() method wraps this in a GSB buffer, but the actual
 * I2C transaction is a plain 1-byte register read.
 */
static u8 ec_read_event(struct i2c_client *client)
{
	u8 event = 0;
	int ret = ec_direct_read(client, 0x05, &event);

	return ret ? 0 : event;
}

/* Act on one decoded EC SCI event code.  Caller must NOT hold ec->lock. */
static void ec_dispatch_event(struct asus_ec *ec, u8 event)
{
	switch (event) {
	case EC_EVENT_FAN1_STAT ... EC_EVENT_FAN2_SPEED:
	case EC_EVENT_LUT_DONE:
	case EC_EVENT_PROFILE_DONE:
	case EC_EVENT_FAN_HIB:
		/* hwmon reads RPM on demand; no further action needed */
		break;
	case EC_EVENT_EC_RESET:
		mutex_lock(&ec->lock);
		ec_direct_write(ec->client, EC_IREG_SCI, 0x01);
		ec_temp_push(ec, EC_DEFAULT_TEMP_C);
		ec_set_profile(ec, fanp_by_state[ec->cooling_state]);
		ec_set_fan_rpm_range(ec, 1, fan_urpm_by_state[ec->cooling_state]);
		/*
		 * The reset cleared the ECCW fan test mode in HW; re-force the
		 * governor-commanded duty if we were holding manual cooling, else
		 * a hot CPU would revert to chassis-only fan (step_wise won't
		 * re-trigger the same target state).  Lock is held; ec_fan_reforce
		 * enters test mode + applies the duty atomically (with a spin floor
		 * for the governed case), so there is no fan-off window.
		 */
		if (ec->sl5b && ec->fan_test_mode)
			ec_fan_reforce(ec);
		mutex_unlock(&ec->lock);
		break;
	default:
		dev_dbg(&ec->client->dev, "EC event 0x%02x\n", event);
		break;
	}
}

/*
 * Drain and handle all pending EC SCI event codes from CMD5 (reg 0x05).
 *
 * The SCI notification GPIO is not decoded for this board (no interrupt in
 * DT), so there is no interrupt to service the EC's event FIFO.  Instead the
 * periodic temperature work drains it here: each CMD5 read returns one queued
 * event code (0 / 0xFF = FIFO empty).  Loop until drained or a sane cap so a
 * burst can't spin forever.  Also reused by the IRQ path if a future DT ever
 * wires the SCI pin.
 */
static void ec_drain_events(struct asus_ec *ec)
{
	int i;

	for (i = 0; i < EC_EVENT_DRAIN_MAX; i++) {
		u8 event;

		mutex_lock(&ec->lock);
		event = ec_read_event(ec->client);
		mutex_unlock(&ec->lock);

		if (!event || event == 0xFF)
			break;

		ec_dispatch_event(ec, event);
	}
}

static irqreturn_t asus_ec_irq_handler(int irq, void *data)
{
	ec_drain_events(data);
	return IRQ_HANDLED;
}

/* Keyboard backlight LED (ECCW path) --------------------------------------- */

struct asus_kbl_led {
	struct led_classdev	cdev;
	struct asus_ec		*ec;
};

static int asus_kbl_set_blocking(struct led_classdev *cdev,
				  enum led_brightness brightness)
{
	struct asus_kbl_led *led = container_of(cdev, struct asus_kbl_led, cdev);
	struct asus_ec *ec = led->ec;
	int ret;

	mutex_lock(&ec->lock);
	ret = ec_eccw(ec->sl5b, EC_ECCW_CLASS_KBD, EC_ECCW_FUNC_KBL,
		      ec_kbl_pwm[brightness]);
	if (!ret)
		ec->kbl_level = (u8)brightness;
	mutex_unlock(&ec->lock);
	return ret;
}

/* Probe / remove ---------------------------------------------------------- */

static int asus_ec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct asus_ec *ec;
	u8 fifo_val;
	int ret;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->client        = client;
	ec->fanp          = EC_FANP_BALANCED;
	/*
	 * Boot HW state: the fan is in EC-auto (not force-driven) and no ECCW
	 * duty has been applied, so start at cooling_state 0 to honour the
	 * "state >= 1 => forced" invariant.  A stale init of 1 would make
	 * get_cur_state report a forced state the driver never actually forced.
	 * The governor sets the real state from CPU temp on its first pass.
	 */
	ec->cooling_state = 0;
	ec->batt_limit    = EC_BATT_LIMIT_DEF;
	mutex_init(&ec->lock);
	i2c_set_clientdata(client, ec);

	ec->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ec->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ec->reset_gpio),
				     "failed to get reset GPIO\n");

	/* Sanity-check: FIFO ctrl register reads 0x01 when EC is up */
	ret = ec_direct_read(client, EC_IREG_FIFO, &fifo_val);
	if (ret)
		return dev_err_probe(dev, ret, "EC not responding\n");
	if (fifo_val != 0x01)
		dev_warn(dev, "unexpected FIFO ctrl value 0x%02x\n", fifo_val);

	/* Pre-init F28G dump: read LUT before any EC writes clear BIOS state.
	 * BIOS initialises the user-override LUT slots; the driver's SCI/temp/
	 * profile writes wipe them, after which F28G returns all-zeros.  This
	 * captures the format on boot for F28S upload work. */
	{
		static const u8 dump_profiles[] = {
			EC_FANP_SILENT, EC_FANP_BALANCED, EC_FANP_BALANCED_HI,
			EC_FANP_PERF,   EC_FANP_TURBO,
		};
		u8 lut[EC_LUT_RD_SIZE];
		int i;

		for (i = 0; i < ARRAY_SIZE(dump_profiles); i++) {
			if (!ec_f28g_read_lut(client, dump_profiles[i], 0, lut))
				dev_info(dev, "F28G 0x%02x: %*ph\n",
					 dump_profiles[i], EC_LUT_RD_SIZE, lut);
		}
	}

	/* Enable EC SCI notifications (fan speed/profile changes, thermal trips) */
	ret = ec_direct_write(client, EC_IREG_SCI, 0x01);
	if (ret)
		dev_warn(dev, "EC SCI enable failed: %d\n", ret);

	/* Temperature push — required before fan profile commands take effect */
	ret = ec_temp_push(ec, EC_DEFAULT_TEMP_C);
	if (ret)
		dev_warn(dev, "EC temp init failed: %d\n", ret);

	/* Apply default profile and set initial RPM range (state=1, balanced) */
	ret = ec_set_profile(ec, EC_FANP_BALANCED);
	if (ret)
		dev_warn(dev, "EC fan profile set failed: %d\n", ret);
	ec_set_fan_rpm_range(ec, 1, fan_urpm_by_state[1]);

	/*
	 * Optional EC SCI interrupt.  The SCI notification GPIO is not decoded
	 * for this board (no interrupts-extended in DT), so client->irq is 0 and
	 * events are drained by the periodic poll instead (ec_drain_events).
	 * Kept guarded so a future DT that wires the pin gets edge-driven service.
	 */
	if (client->irq > 0) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						asus_ec_irq_handler,
						IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
						"asus-ec", ec);
		if (ret)
			dev_warn(dev, "failed to request IRQ %d: %d\n", client->irq, ret);
	}

	/* hwmon: fan1_input, fan2_input, pwm1 */
	hwmon_dev = devm_hwmon_device_register_with_info(dev, "asus_vivobook_s15",
							  ec, &asus_ec_hwmon_chip,
							  NULL);
	if (IS_ERR(hwmon_dev))
		return dev_err_probe(dev, PTR_ERR(hwmon_dev),
				     "failed to register hwmon\n");

	/* Thermal cooling device (links to DT cooling-maps when added) */
	ec->cdev = devm_thermal_of_cooling_device_register(dev, dev->of_node,
							    "asus-ec-fan", ec,
							    &asus_ec_cooling_ops);
	if (IS_ERR(ec->cdev)) {
		dev_warn(dev, "failed to register cooling device: %ld\n",
			 PTR_ERR(ec->cdev));
		ec->cdev = NULL;
	}

	/*
	 * Keyboard backlight via ECCW mailbox on I2C 0x5B.
	 * devm_i2c_new_dummy_device acquires the address on the same bus adapter.
	 * Init sequence: ECCW(BUS_INIT) must precede any KBL write.
	 */
	ec->sl5b = devm_i2c_new_dummy_device(dev, client->adapter, EC_SL5B_ADDR);
	if (IS_ERR(ec->sl5b)) {
		dev_warn(dev, "ECCW: failed to acquire I2C 0x%02X: %ld\n",
			 EC_SL5B_ADDR, PTR_ERR(ec->sl5b));
		ec->sl5b = NULL;
	} else {
		ret = ec_eccw(ec->sl5b, EC_ECCW_CLASS_KBD, EC_ECCW_FUNC_BUS_INIT,
			      EC_ECCW_INIT_VAL);
		if (ret) {
			dev_warn(dev, "ECCW bus init failed: %d\n", ret);
		} else {
			struct asus_kbl_led *kbl;

			ec->kbl_level = EC_KBL_DEFAULT;
			kbl = devm_kzalloc(dev, sizeof(*kbl), GFP_KERNEL);
			if (kbl) {
				kbl->ec = ec;
				kbl->cdev.name = "asus::kbd_backlight_ec";
				kbl->cdev.max_brightness = EC_KBL_MAX_BRIGHTNESS;
				kbl->cdev.brightness = EC_KBL_DEFAULT;
				kbl->cdev.brightness_set_blocking =
					asus_kbl_set_blocking;
				ret = devm_led_classdev_register(dev, &kbl->cdev);
				if (ret)
					dev_warn(dev,
						 "kbd_backlight register failed: %d\n",
						 ret);
			}
		}
	}

	/* Periodic temperature push */
	INIT_DELAYED_WORK(&ec->temp_work, ec_temp_work_fn);
	schedule_delayed_work(&ec->temp_work, msecs_to_jiffies(EC_TEMP_UPDATE_MS));

	dev_info(dev, "ASUS Vivobook S15 EC ready (FIFO=0x%02x)\n", fifo_val);
	return 0;
}

static void asus_ec_remove(struct i2c_client *client)
{
	struct asus_ec *ec = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&ec->temp_work);

	/* SAFETY: never leave the fan stuck in manual test mode on unbind —
	 * hand it back to the EC's own loop. */
	mutex_lock(&ec->lock);
	if (ec->sl5b && ec->fan_test_mode)
		ec_fan_release(ec);
	mutex_unlock(&ec->lock);
}

/*
 * SAFETY: on shutdown/reboot, release the fan to EC-auto so it is never left
 * forced+low across the transition.
 */
static void asus_ec_shutdown(struct i2c_client *client)
{
	struct asus_ec *ec = i2c_get_clientdata(client);

	mutex_lock(&ec->lock);
	if (ec->sl5b && ec->fan_test_mode)
		ec_fan_release(ec);
	mutex_unlock(&ec->lock);
}

/* PM ---------------------------------------------------------------------- */

static int asus_ec_suspend(struct device *dev)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));

	cancel_delayed_work_sync(&ec->temp_work);
	return 0;
}

static int asus_ec_resume(struct device *dev)
{
	struct asus_ec *ec = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&ec->lock);

	/* EC may have power-cycled during deep S3; restore all init state. */
	ec_direct_write(ec->client, EC_IREG_SCI, 0x01);
	ec_temp_push(ec, EC_DEFAULT_TEMP_C);
	ec_set_profile(ec, ec->fanp);
	{
		unsigned int urpm;
		switch (ec->fanp) {
		case EC_FANP_SILENT:      urpm = fan_urpm_by_state[0]; break;
		case EC_FANP_BALANCED:
		case EC_FANP_BALANCED_HI: urpm = fan_urpm_by_state[1]; break;
		case EC_FANP_PERF:
		case EC_FANP_PERF_HI:     urpm = fan_urpm_by_state[2]; break;
		default:                  urpm = fan_urpm_by_state[3]; break;
		}
		ec_set_fan_rpm_range(ec, 1, urpm);
	}
	ec_set_batt_limit(ec, ec->batt_limit);

	if (ec->sl5b) {
		ec_eccw(ec->sl5b, EC_ECCW_CLASS_KBD, EC_ECCW_FUNC_BUS_INIT,
			EC_ECCW_INIT_VAL);
		ec_eccw(ec->sl5b, EC_ECCW_CLASS_KBD, EC_ECCW_FUNC_KBL,
			ec_kbl_pwm[ec->kbl_level]);
		/* EC power-cycles in deep S3: re-force the fan duty if we were
		 * holding manual test mode, else leave it in EC-auto.  Uses the
		 * spin floor for the governed case so a prior pwm1=0 can't come
		 * back as a sticky fan-off. */
		if (ec->fan_test_mode)
			ec_fan_reforce(ec);
	}

	mutex_unlock(&ec->lock);

	schedule_delayed_work(&ec->temp_work, msecs_to_jiffies(5000));
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(asus_ec_pm_ops, asus_ec_suspend, asus_ec_resume);

/* Module ------------------------------------------------------------------ */

static const struct of_device_id asus_ec_of_match[] = {
	{ .compatible = "asus,vivobook-s15-ec" },
	{}
};
MODULE_DEVICE_TABLE(of, asus_ec_of_match);

static const struct i2c_device_id asus_ec_i2c_id[] = {
	{ "asus-vivobook-s15-ec" },
	{}
};
MODULE_DEVICE_TABLE(i2c, asus_ec_i2c_id);

static struct i2c_driver asus_ec_driver = {
	.driver = {
		.name		= "asus-vivobook-s15-ec",
		.of_match_table	= asus_ec_of_match,
		.pm		= pm_sleep_ptr(&asus_ec_pm_ops),
		.dev_groups	= asus_ec_groups,
	},
	.probe    = asus_ec_probe,
	.remove   = asus_ec_remove,
	.shutdown = asus_ec_shutdown,
	.id_table = asus_ec_i2c_id,
};

module_i2c_driver(asus_ec_driver);

MODULE_AUTHOR("dxue <gboworld@yahoo.com>");
MODULE_DESCRIPTION("ASUS Vivobook S15 Embedded Controller");
MODULE_LICENSE("GPL");
