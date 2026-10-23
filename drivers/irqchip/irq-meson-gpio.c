// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 Endless Mobile, Inc.
 * Author: Carlo Caione <carlo@endlessm.com>
 * Copyright (c) 2016 BayLibre, SAS.
 * Author: Jerome Brunet <jbrunet@baylibre.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/irqchip/irq-meson-gpio.h>

#define MAX_NUM_CHANNEL 64
#define MAX_INPUT_MUX 256

#define REG_EDGE_POL	0x00
#define REG_PIN_03_SEL	0x04
#define REG_PIN_47_SEL	0x08
#define REG_FILTER_SEL	0x0c

/* use for A1 like chips */
#define REG_PIN_A1_SEL	0x04

/*
 * Note: The S905X3 datasheet reports that BOTH_EDGE is controlled by
 * bits 24 to 31. Tests on the actual HW show that these bits are
 * stuck at 0. Bits 8 to 15 are responsive and have the expected
 * effect.
 */
#define REG_EDGE_POL_EDGE(params, x)	BIT((params)->edge_single_offset + (x))
#define REG_EDGE_POL_LOW(params, x)	BIT((params)->pol_low_offset + (x))
#define REG_BOTH_EDGE(params, x)	BIT((params)->edge_both_offset + (x))
#define REG_EDGE_POL_MASK(params, x)    (	\
		REG_EDGE_POL_EDGE(params, x) |	\
		REG_EDGE_POL_LOW(params, x)  |	\
		REG_BOTH_EDGE(params, x))
#define REG_PIN_SEL_SHIFT(x)	(((x) % 4) * 8)
#define REG_FILTER_SEL_SHIFT(x)	((x) * 4)

struct meson_gpio_irq_controller;
static void meson8_gpio_irq_sel_pin(struct meson_gpio_irq_controller *ctl,
				    unsigned int channel, unsigned char gpio);
static void meson_gpio_irq_init_dummy(struct meson_gpio_irq_controller *ctl);
static void meson_a1_gpio_irq_sel_pin(struct meson_gpio_irq_controller *ctl,
				      unsigned int channel,
				      unsigned char gpio);
static void meson_a1_gpio_irq_init(struct meson_gpio_irq_controller *ctl);
static int meson8_gpio_irq_set_type(struct meson_gpio_irq_controller *ctl,
				    unsigned int type, u32 *gic_hwirq,
				    unsigned int gpio);
static int meson_s4_gpio_irq_set_type(struct meson_gpio_irq_controller *ctl,
				      unsigned int type, u32 *gic_hwirq,
				      unsigned int gpio);
static void meson8_eeb_gpio_irq_eoi(struct meson_gpio_irq_controller *ctl,
			 u32 *gic_hwirq);

struct irq_ctl_ops {
	void (*gpio_irq_sel_pin)(struct meson_gpio_irq_controller *ctl,
				 unsigned int channel, unsigned char gpio);
	void (*gpio_irq_init)(struct meson_gpio_irq_controller *ctl);
	void (*gpio_irq_eoi)(struct meson_gpio_irq_controller *ctl,
				 u32 *gic_hwirq);
	void (*gpio_irq_unmask)(struct meson_gpio_irq_controller *ctl,
				 u32 *gic_hwirq);
	int (*gpio_irq_set_type)(struct meson_gpio_irq_controller *ctl,
				 unsigned int type, u32 *gic_hwirq,
				 unsigned int gpio);
};

struct meson_gpio_irq_params {
	unsigned int nr_hwirq;
	unsigned int nr_channels;
	bool support_edge_both;
	unsigned int edge_both_offset;
	unsigned int edge_single_offset;
	unsigned int edge_pol_reg;
	unsigned int pol_low_offset;
	unsigned int pin_sel_mask;
	struct irq_ctl_ops ops;
};

#define INIT_MESON_COMMON(irqs, init, sel, type, eoi, unmask)	\
	.nr_hwirq = irqs,					\
	.ops = {						\
		.gpio_irq_init = init,				\
		.gpio_irq_sel_pin = sel,			\
		.gpio_irq_set_type = type,			\
		.gpio_irq_eoi = eoi,				\
		.gpio_irq_unmask = unmask,			\
	},

#define INIT_MESON8_COMMON_DATA(irqs)				\
	INIT_MESON_COMMON(irqs, meson_gpio_irq_init_dummy,	\
			  meson8_gpio_irq_sel_pin,		\
			  meson8_gpio_irq_set_type, NULL, NULL)	\
	.edge_single_offset = 0,				\
	.pol_low_offset = 16,					\
	.pin_sel_mask = 0xff,					\
	.nr_channels = 8,					\

#define INIT_MESON8_EEB_COMMON_DATA(irqs)			\
	INIT_MESON_COMMON(irqs, meson_gpio_irq_init_dummy,	\
			  meson8_gpio_irq_sel_pin,		\
			  meson8_gpio_irq_set_type,		\
			  meson8_eeb_gpio_irq_eoi,		\
			  NULL)					\
	.edge_single_offset = 0,				\
	.pol_low_offset = 16,					\
	.pin_sel_mask = 0xff,					\
	.nr_channels = 8,					\

#define INIT_MESON_A1_COMMON_DATA(irqs)				\
	INIT_MESON_COMMON(irqs, meson_a1_gpio_irq_init,		\
			  meson_a1_gpio_irq_sel_pin,		\
			  meson8_gpio_irq_set_type, NULL, NULL)	\
	.support_edge_both = true,				\
	.edge_both_offset = 16,					\
	.edge_single_offset = 8,				\
	.pol_low_offset = 0,					\
	.pin_sel_mask = 0x7f,					\
	.nr_channels = 8,					\

#define INIT_MESON_A4_AO_COMMON_DATA(irqs)			\
	INIT_MESON_COMMON(irqs, meson_a1_gpio_irq_init,		\
			  meson_a1_gpio_irq_sel_pin,		\
			  meson_s4_gpio_irq_set_type, NULL, NULL)	\
	.support_edge_both = true,				\
	.edge_both_offset = 0,					\
	.edge_single_offset = 12,				\
	.edge_pol_reg = 0x8,					\
	.pol_low_offset = 0,					\
	.pin_sel_mask = 0xff,					\
	.nr_channels = 2,					\

#define INIT_MESON_S4_COMMON_DATA(irqs)				\
	INIT_MESON_COMMON(irqs, meson_a1_gpio_irq_init,		\
			  meson_a1_gpio_irq_sel_pin,		\
			  meson_s4_gpio_irq_set_type, NULL, NULL)	\
	.support_edge_both = true,				\
	.edge_both_offset = 0,					\
	.edge_single_offset = 12,				\
	.edge_pol_reg = 0x1c,					\
	.pol_low_offset = 0,					\
	.pin_sel_mask = 0xff,					\
	.nr_channels = 12,					\

static const struct meson_gpio_irq_params meson8_params = {
	INIT_MESON8_COMMON_DATA(134)
};

static const struct meson_gpio_irq_params meson8b_params = {
	INIT_MESON8_COMMON_DATA(119)
};

static const struct meson_gpio_irq_params gxbb_params = {
	INIT_MESON8_COMMON_DATA(133)
};

static const struct meson_gpio_irq_params gxl_params = {
	INIT_MESON8_EEB_COMMON_DATA(110)
};

static const struct meson_gpio_irq_params axg_params = {
	INIT_MESON8_EEB_COMMON_DATA(100)
};

static const struct meson_gpio_irq_params sm1_params = {
	INIT_MESON8_COMMON_DATA(100)
	.support_edge_both = true,
	.edge_both_offset = 8,
};

static const struct meson_gpio_irq_params a1_params = {
	INIT_MESON_A1_COMMON_DATA(62)
};

static const struct meson_gpio_irq_params a4_params = {
	INIT_MESON_S4_COMMON_DATA(81)
};

static const struct meson_gpio_irq_params a4_ao_params = {
	INIT_MESON_A4_AO_COMMON_DATA(8)
};

static const struct meson_gpio_irq_params a5_params = {
	INIT_MESON_S4_COMMON_DATA(99)
};

static const struct meson_gpio_irq_params s4_params = {
	INIT_MESON_S4_COMMON_DATA(82)
};

static const struct meson_gpio_irq_params c3_params = {
	INIT_MESON_S4_COMMON_DATA(55)
};

static const struct meson_gpio_irq_params t7_params = {
	INIT_MESON_S4_COMMON_DATA(157)
};

static const struct of_device_id meson_irq_gpio_matches[] __maybe_unused = {
	{ .compatible = "amlogic,meson8-gpio-intc", .data = &meson8_params },
	{ .compatible = "amlogic,meson8b-gpio-intc", .data = &meson8b_params },
	{ .compatible = "amlogic,meson-gxbb-gpio-intc", .data = &gxbb_params },
	{ .compatible = "amlogic,meson-gxl-gpio-intc", .data = &gxl_params },
	{ .compatible = "amlogic,meson-axg-gpio-intc", .data = &axg_params },
	{ .compatible = "amlogic,meson-g12a-gpio-intc", .data = &axg_params },
	{ .compatible = "amlogic,meson-sm1-gpio-intc", .data = &sm1_params },
	{ .compatible = "amlogic,meson-a1-gpio-intc", .data = &a1_params },
	{ .compatible = "amlogic,meson-s4-gpio-intc", .data = &s4_params },
	{ .compatible = "amlogic,a4-gpio-ao-intc", .data = &a4_ao_params },
	{ .compatible = "amlogic,a4-gpio-intc", .data = &a4_params },
	{ .compatible = "amlogic,a5-gpio-intc", .data = &a5_params },
	{ .compatible = "amlogic,c3-gpio-intc", .data = &c3_params },
	{ .compatible = "amlogic,t7-gpio-intc", .data = &t7_params },
	{ }
};

/*
 * Controllers without a both-edge mode emulate it. The channel is armed for
 * the edge opposite the pad, and eoi arms again from the pad it reads, so a
 * second edge before eoi does not leave the channel facing the wrong way.
 * set_type samples the live pad. The third fwspec cell from gpiod_to_irq()
 * is only the fallback when that read fails: the pinctrl driver has not
 * published one, or the hwirq matches no bank. A two-cell "interrupts"
 * mapping has no cell and would otherwise start armed for the rising edge
 * whatever the pad is doing.
 *
 * Sampling the pad does not absorb an edge the pad really made. Enabling the
 * output driver on a stale latch is two of those; programming the latch
 * before the driver is what removes them.
 */
struct meson_gpio_irq_info {
	bool is_both_edge;
	bool level_high;	/* pad level this channel is armed against */
	bool replayed;		/* next eoi ends a resend, not a new edge */
	u32 current_state;
};

struct meson_gpio_irq_controller {
	const struct meson_gpio_irq_params *params;
	void __iomem *base;
	struct meson_gpio_irq_info irq_info[MAX_NUM_CHANNEL];
	u32 channel_irqs[MAX_NUM_CHANNEL];
	DECLARE_BITMAP(channel_map, MAX_NUM_CHANNEL);
	/* debugfs accounting: the GPIO input and Linux IRQ on each channel */
	u32 channel_gpio[MAX_NUM_CHANNEL];
	unsigned int channel_virq[MAX_NUM_CHANNEL];
	raw_spinlock_t lock;
};

static int (*meson_gpio_pad_level)(unsigned int hwirq, bool *high);

int meson_gpio_irq_set_pad_level(int (*read_pad)(unsigned int hwirq, bool *high))
{
	if (!read_pad)
		return -EINVAL;

	/*
	 * Pairs with the acquire in meson_gpio_read_pad. The caller links
	 * its controller before this store, so a reader that sees the
	 * function also sees that link.
	 */
	smp_store_release(&meson_gpio_pad_level, read_pad);
	return 0;
}
EXPORT_SYMBOL_GPL(meson_gpio_irq_set_pad_level);

static bool meson_gpio_read_pad(unsigned int hwirq, bool *high)
{
	int (*fn)(unsigned int hwirq, bool *high);

	/* pairs with smp_store_release in meson_gpio_irq_set_pad_level */
	fn = smp_load_acquire(&meson_gpio_pad_level);
	return fn && !fn(hwirq, high);
}

static inline void meson_gpio_irq_update_bits(struct meson_gpio_irq_controller *ctl,
				       unsigned int reg, u32 mask, u32 val)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ctl->lock, flags);

	//pr_debug("%s: reg 0x%x mask 0x%x val %x\n", __func__, reg, mask, val);
	writel_relaxed((readl_relaxed(ctl->base + reg) & ~mask) | val, ctl->base + reg);

	raw_spin_unlock_irqrestore(&ctl->lock, flags);
}

static void meson_gpio_irq_init_dummy(struct meson_gpio_irq_controller *ctl)
{
}

static void meson8_gpio_irq_sel_pin(struct meson_gpio_irq_controller *ctl,
				    unsigned int channel, unsigned char gpio)
{
	unsigned int reg_offset;
	unsigned int bit_offset;

	reg_offset = (channel < 4) ? REG_PIN_03_SEL : REG_PIN_47_SEL;
	bit_offset = REG_PIN_SEL_SHIFT(channel);

	pr_debug("%s: channel %d reg_off %d bit_off %d\n", __func__, channel, reg_offset, bit_offset);

	meson_gpio_irq_update_bits(ctl, reg_offset,
				   ctl->params->pin_sel_mask << bit_offset,
				   gpio << bit_offset);
}

static void meson_a1_gpio_irq_sel_pin(struct meson_gpio_irq_controller *ctl,
				      unsigned int channel,
				      unsigned char gpio)
{
	unsigned int reg_offset;
	unsigned int bit_offset;

	bit_offset = ((channel % 2) == 0) ? 0 : 16;
	reg_offset = REG_PIN_A1_SEL + ((channel / 2) << 2);

	meson_gpio_irq_update_bits(ctl, reg_offset,
				   ctl->params->pin_sel_mask << bit_offset,
				   gpio << bit_offset);
}

/* For a1 or later chips like a1 there is a switch to enable/disable irq */
static void meson_a1_gpio_irq_init(struct meson_gpio_irq_controller *ctl)
{
	meson_gpio_irq_update_bits(ctl, REG_EDGE_POL, BIT(31), BIT(31));
}

static int
meson_gpio_irq_request_channel(struct meson_gpio_irq_controller *ctl,
				struct irq_fwspec *fwspec,
				unsigned int gpio, unsigned int type,
				u32 **gic_hwirq)
{
	unsigned long flags;
	unsigned int idx;

	raw_spin_lock_irqsave(&ctl->lock, flags);

	/* Find a free channel */
	idx = find_first_zero_bit(ctl->channel_map, ctl->params->nr_channels);
	if (idx >= ctl->params->nr_channels) {
		raw_spin_unlock_irqrestore(&ctl->lock, flags);
		pr_err("No channel available\n");
		return -ENOSPC;
	}

	/* Mark the channel as used; nothing of its last user carries over */
	set_bit(idx, ctl->channel_map);
	ctl->channel_gpio[idx] = gpio;
	/* fallback when set_type cannot read the pad */
	ctl->irq_info[idx] = (struct meson_gpio_irq_info) {
		.level_high = fwspec->param_count >= 3 && fwspec->param[2],
	};

	raw_spin_unlock_irqrestore(&ctl->lock, flags);

	/*
	 * Setup the mux of the channel to route the signal of the pad
	 * to the appropriate input of the GIC
	 */
	ctl->params->ops.gpio_irq_sel_pin(ctl, idx, gpio);

	/*
	 * Get the hwirq number assigned to this channel through
	 * a pointer the channel_irq table. The added benefit of this
	 * method is that we can also retrieve the channel index with
	 * it, using the table base.
	 */
	*gic_hwirq = &(ctl->channel_irqs[idx]);

	pr_debug("%s: gpio %u channel %d gic_hwirq %u level_high %d\n",
		 __func__, gpio, idx, **gic_hwirq, ctl->irq_info[idx].level_high);

	return 0;
}

static inline unsigned int
meson_gpio_irq_get_channel_idx(struct meson_gpio_irq_controller *ctl,
			       u32 *gic_hwirq)
{
	return gic_hwirq - ctl->channel_irqs;
}

static void
meson_gpio_irq_release_channel(struct meson_gpio_irq_controller *ctl,
			       u32 *gic_hwirq)
{
	unsigned int idx;

	idx = meson_gpio_irq_get_channel_idx(ctl, gic_hwirq);
	ctl->channel_virq[idx] = 0;
	clear_bit(idx, ctl->channel_map);
}

static void meson8_gpio_irq_arm_both(struct meson_gpio_irq_controller *ctl,
				     struct meson_gpio_irq_info *irq,
				     unsigned int idx)
{
	const struct meson_gpio_irq_params *params = ctl->params;
	u32 val = REG_EDGE_POL_EDGE(params, idx);

	if (irq->level_high)
		val |= REG_EDGE_POL_LOW(params, idx);

	irq->is_both_edge = true;
	irq->current_state = val;
	meson_gpio_irq_update_bits(ctl, REG_EDGE_POL,
				   REG_EDGE_POL_MASK(params, idx), val);
	/*
	 * The update is a relaxed write. Complete it before the caller
	 * samples the pad or the parent's pending bit: those are other
	 * devices, and a sample can otherwise pass the write.
	 */
	readl(ctl->base + REG_EDGE_POL);
}

static int meson8_gpio_irq_set_type(struct meson_gpio_irq_controller *ctl,
				    unsigned int type, u32 *gic_hwirq,
				    unsigned int gpio __always_unused)
{
	unsigned int idx;
	u32 val = 0;
	struct meson_gpio_irq_info *irq;
	const struct meson_gpio_irq_params *params = ctl->params;

	idx = meson_gpio_irq_get_channel_idx(ctl, gic_hwirq);
	irq = &ctl->irq_info[idx];

	/*
	 * The controller has a filter block to operate in either LEVEL or
	 * EDGE mode, then signal is sent to the GIC. To enable LEVEL_LOW and
	 * EDGE_FALLING support (which the GIC does not support), the filter
	 * block is also able to invert the input signal it gets before
	 * providing it to the GIC.
	 */
	type &= IRQ_TYPE_SENSE_MASK;
	irq->replayed = false;

	/*
	 * New controller support EDGE_BOTH trigger. This setting takes
	 * precedence over the other edge/polarity settings
	 */
	if (type == IRQ_TYPE_EDGE_BOTH) {
		if (params->support_edge_both){
			irq->is_both_edge = false;
			val = REG_BOTH_EDGE(params, idx);
		} else if (!params->ops.gpio_irq_eoi) {
			/* no emulation: it would fire on one edge only */
			return -EINVAL;
		} else {
			pr_debug("%s: initial %s trigger\n", __func__,
				 irq->level_high ? "falling" : "rising");
			meson8_gpio_irq_arm_both(ctl, irq, idx);
			return 0;
		}
	} else {
		irq->is_both_edge = false;
		val = (type & IRQ_TYPE_EDGE_BOTH) ? REG_EDGE_POL_EDGE(params, idx) : 0;

		if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_EDGE_FALLING))
			val |= REG_EDGE_POL_LOW(params, idx);
	}

	meson_gpio_irq_update_bits(ctl, REG_EDGE_POL,
				   REG_EDGE_POL_MASK(params, idx), val);

	return 0;
}

/*
 * Every interrupt ends here, whatever its handler (a oneshot one is masked
 * first, a hard one is not). An emulated both-edge channel is armed here for
 * the opposite of the pad read now, which is the other edge after one
 * transition and still the right edge if a second one arrived first.
 */
static void meson8_eeb_gpio_irq_eoi(struct meson_gpio_irq_controller *ctl,
				    u32 *gic_hwirq)
{
	struct meson_gpio_irq_info *irq;
	unsigned int idx;
	bool high;
	u32 val;

	idx = meson_gpio_irq_get_channel_idx(ctl, gic_hwirq);
	irq = &ctl->irq_info[idx];

	if (!irq->is_both_edge)
		return;

	/*
	 * Arm for the opposite of the pad that is there now. A toggle would
	 * be wrong if a second edge arrived before eoi, and a write of the
	 * polarity already programmed only risks a filter glitch.
	 */
	if (meson_gpio_read_pad(ctl->channel_gpio[idx], &high)) {
		val = REG_EDGE_POL_EDGE(ctl->params, idx);
		if (high)
			val |= REG_EDGE_POL_LOW(ctl->params, idx);
		irq->replayed = false;
		if (irq->current_state == val && irq->level_high == high)
			return;
		irq->level_high = high;
		meson8_gpio_irq_arm_both(ctl, irq, idx);
		return;
	}

	/* no pad reader: one flip per real edge, and none for a resend */
	if (irq->replayed) {
		irq->replayed = false;
		return;
	}

	irq->level_high = !irq->level_high;
	irq->current_state ^= REG_EDGE_POL_LOW(ctl->params, idx);
	meson_gpio_irq_update_bits(ctl, REG_EDGE_POL,
				   REG_EDGE_POL_MASK(ctl->params, idx),
				   irq->current_state);
}

/*
 * gpio irq relative registers for s4
 * -PADCTRL_GPIO_IRQ_CTRL0
 * bit[31]:    enable/disable all the irq lines
 * bit[12-23]: single edge trigger
 * bit[0-11]:  polarity trigger
 *
 * -PADCTRL_GPIO_IRQ_CTRL[X]
 * bit[0-16]: 7 bits to choose gpio source for irq line 2*[X] - 2
 * bit[16-22]:7 bits to choose gpio source for irq line 2*[X] - 1
 * where X = 1-6
 *
 * -PADCTRL_GPIO_IRQ_CTRL[7]
 * bit[0-11]: both edge trigger
 */
static int meson_s4_gpio_irq_set_type(struct meson_gpio_irq_controller *ctl,
				      unsigned int type, u32 *gic_hwirq,
				      unsigned int gpio __always_unused)
{
	const struct meson_gpio_irq_params *params = ctl->params;
	unsigned int idx;
	u32 val = 0;

	idx = meson_gpio_irq_get_channel_idx(ctl, gic_hwirq);

	type &= IRQ_TYPE_SENSE_MASK;

	meson_gpio_irq_update_bits(ctl, params->edge_pol_reg, BIT(idx), 0);

	if (type == IRQ_TYPE_EDGE_BOTH) {
		val = BIT(ctl->params->edge_both_offset + idx);
		meson_gpio_irq_update_bits(ctl, params->edge_pol_reg, val, val);
		return 0;
	}

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_EDGE_FALLING))
		val |= BIT(ctl->params->pol_low_offset + idx);

	if (type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING))
		val |= BIT(ctl->params->edge_single_offset + idx);

	meson_gpio_irq_update_bits(ctl, REG_EDGE_POL, BIT(idx) | BIT(12 + idx), val);
	return 0;
};

static unsigned int meson_gic_gpio_irq_type_output(unsigned int type)
{
	unsigned int sense = type & IRQ_TYPE_SENSE_MASK;

	type &= ~IRQ_TYPE_SENSE_MASK;

	/*
	 * The polarity of the signal provided to the GIC should always
	 * be high.
	 */
	if (sense & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW))
		type |= IRQ_TYPE_LEVEL_HIGH;
	else
		type |= IRQ_TYPE_EDGE_RISING;

	return type;
}

static void meson_gpio_irq_mask(struct irq_data *data)
{
	irq_chip_mask_parent(data);
}

static void meson_gpio_irq_eoi(struct irq_data *data)
{
	struct meson_gpio_irq_controller *ctl = data->domain->host_data;
	u32 *gic_hwirq = irq_data_get_irq_chip_data(data);

	if (ctl->params->ops.gpio_irq_eoi)
		ctl->params->ops.gpio_irq_eoi(ctl, gic_hwirq);

	irq_chip_eoi_parent(data);
}

static void meson_gpio_irq_unmask(struct irq_data *data)
{
	struct meson_gpio_irq_controller *ctl = data->domain->host_data;
	u32 *gic_hwirq = irq_data_get_irq_chip_data(data);

	if (ctl->params->ops.gpio_irq_unmask)
		ctl->params->ops.gpio_irq_unmask(ctl, gic_hwirq);

	irq_chip_unmask_parent(data);
}

/*
 * A resend (wakeup from suspend, enable_irq after an edge while disabled)
 * raises the interrupt again with no new edge on the pad; the eoi that ends
 * it must not flip an emulated both-edge channel a second time.
 */
static int meson_gpio_irq_retrigger(struct irq_data *data)
{
	struct meson_gpio_irq_controller *ctl = data->domain->host_data;
	u32 *gic_hwirq = irq_data_get_irq_chip_data(data);
	struct meson_gpio_irq_info *irq;
	bool pending;

	irq = &ctl->irq_info[meson_gpio_irq_get_channel_idx(ctl, gic_hwirq)];
	if (irq->is_both_edge) {
		/*
		 * An edge that came while the interrupt was masked is still
		 * pending in the parent: its delivery is the resend, and a
		 * real edge whose eoi must re-arm from the pad.
		 */
		if (!irq_chip_get_parent_state(data, IRQCHIP_STATE_PENDING,
					       &pending) && pending)
			return 1;
		irq->replayed = true;
	}

	return irq_chip_retrigger_hierarchy(data);
}

/*
 * After the filter is programmed. A pending parent with an unchanged pad is
 * the filter glitching, and is cleared; an edge that lands in that clear is
 * put back, because this arm still matches the pre-edge level. If the clear
 * does not land, or the confirming read fails, the pending bit is left set:
 * that can keep one glitch and cannot hide an edge. A pad that moved is a
 * real edge. Leave it pending when the filter caught it: eoi reads the pad
 * and arms for the opposite of what is there. When nothing is pending the
 * filter was written after the edge, so arm for the level on the pad now
 * and raise the missed edge. That delivery's eoi then arms for the edge
 * after this one.
 */
static void meson_gpio_irq_settle_both(struct irq_data *data,
				       struct meson_gpio_irq_controller *ctl,
				       u32 *gic_hwirq, bool armed_level)
{
	struct meson_gpio_irq_info *irq;
	unsigned int idx, hwirq = (unsigned int)data->hwirq;
	bool now, pending = false;

	if (!meson_gpio_read_pad(hwirq, &now))
		return;

	idx = meson_gpio_irq_get_channel_idx(ctl, gic_hwirq);
	irq = &ctl->irq_info[idx];

	if (now == armed_level) {
		/*
		 * A clear that does not land stays pending: dropping it
		 * would hide an edge. A confirming read that fails is the
		 * same, the pad may have moved and we cannot show it did not.
		 */
		if (irq_chip_set_parent_state(data, IRQCHIP_STATE_PENDING,
					      false))
			return;
		if (!meson_gpio_read_pad(hwirq, &now) || now != armed_level)
			irq_chip_set_parent_state(data, IRQCHIP_STATE_PENDING,
						  true);
		return;
	}

	if (!irq_chip_get_parent_state(data, IRQCHIP_STATE_PENDING, &pending) &&
	    pending)
		return;

	irq->level_high = now;
	meson8_gpio_irq_arm_both(ctl, irq, idx);
	irq_chip_set_parent_state(data, IRQCHIP_STATE_PENDING, true);
}

static int meson_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct meson_gpio_irq_controller *ctl = data->domain->host_data;
	u32 *gic_hwirq = irq_data_get_irq_chip_data(data);
	unsigned int sense = type & IRQ_TYPE_SENSE_MASK;
	bool emulated, level = false, have_level = false;
	struct meson_gpio_irq_info *irq;
	int ret;

	emulated = sense == IRQ_TYPE_EDGE_BOTH &&
		   !ctl->params->support_edge_both &&
		   ctl->params->ops.gpio_irq_eoi;
	irq = &ctl->irq_info[meson_gpio_irq_get_channel_idx(ctl, gic_hwirq)];

	if (emulated &&
	    meson_gpio_read_pad((unsigned int)data->hwirq, &level)) {
		irq->level_high = level;
		have_level = true;
	}

	ret = ctl->params->ops.gpio_irq_set_type(ctl, type, gic_hwirq,
						 data->hwirq);
	if (ret)
		return ret;

	ret = irq_chip_set_type_parent(data,
				       meson_gic_gpio_irq_type_output(type));
	if (ret || !have_level)
		return ret;

	meson_gpio_irq_settle_both(data, ctl, gic_hwirq, level);
	return 0;
}

static struct irq_chip meson_gpio_irq_chip = {
	.name			= "meson-gpio-irqchip",
	.irq_mask		= meson_gpio_irq_mask,
	.irq_unmask		= meson_gpio_irq_unmask,
	.irq_eoi		= meson_gpio_irq_eoi,
	.irq_set_type		= meson_gpio_irq_set_type,
	.irq_retrigger		= meson_gpio_irq_retrigger,
#ifdef CONFIG_SMP
	.irq_set_affinity	= irq_chip_set_affinity_parent,
#endif
	.flags			= IRQCHIP_SET_TYPE_MASKED,
};

static int meson_gpio_irq_domain_translate(struct irq_domain *domain,
					   struct irq_fwspec *fwspec,
					   unsigned long *gpio,
					   unsigned int *type)
{
	struct meson_gpio_irq_controller *ctl = domain->host_data;
	/* a lookup translates too, so this must not change any state */
	if (is_of_node(fwspec->fwnode) && fwspec->param_count >= 2) {
		if (fwspec->param[0] >= ctl->params->nr_hwirq)
			return -EINVAL;
		*gpio = fwspec->param[0];
		*type = fwspec->param[1];

		return 0;
	}

	return -EINVAL;
}

static int meson_gpio_irq_allocate_gic_irq(struct irq_domain *domain,
					   unsigned int virq,
					   u32 gic_hwirq,
					   unsigned int type)
{
	struct irq_fwspec fwspec;

	fwspec.fwnode = domain->parent->fwnode;
	fwspec.param_count = 3;
	fwspec.param[0] = 0;	/* SPI */
	fwspec.param[1] = gic_hwirq;
	fwspec.param[2] = meson_gic_gpio_irq_type_output(type);

	pr_debug("%s: gic_hwirq 0x%x type 0x%x gic_type 0x%x\n", __func__, gic_hwirq, type, fwspec.param[2]);

	return irq_domain_alloc_irqs_parent(domain, virq, 1, &fwspec);
}

static int meson_gpio_irq_domain_alloc(struct irq_domain *domain,
				       unsigned int virq,
				       unsigned int nr_irqs,
				       void *data)
{
	struct irq_fwspec *fwspec = data;
	struct meson_gpio_irq_controller *ctl = domain->host_data;
	unsigned long gpio;
	u32 *gic_hwirq;
	unsigned int type;
	int ret;

	if (WARN_ON(nr_irqs != 1))
		return -EINVAL;

	pr_debug("%s: virq %d nr_irqs %d\n", __func__, virq, nr_irqs);
	
	ret = meson_gpio_irq_domain_translate(domain, fwspec, &gpio, &type);
	if (ret)
		return ret;

	ret = meson_gpio_irq_request_channel(ctl, fwspec, gpio, type, &gic_hwirq);
	if (ret)
		return ret;

	ret = meson_gpio_irq_allocate_gic_irq(domain, virq,
					      *gic_hwirq, type);
	if (ret < 0) {
		pr_err("failed to allocate gic irq %u\n", *gic_hwirq);
		meson_gpio_irq_release_channel(ctl, gic_hwirq);
		return ret;
	}

	irq_domain_set_hwirq_and_chip(domain, virq, gpio,
				      &meson_gpio_irq_chip, gic_hwirq);
	ctl->channel_virq[meson_gpio_irq_get_channel_idx(ctl, gic_hwirq)] = virq;

	return 0;
}

static void meson_gpio_irq_domain_free(struct irq_domain *domain,
				       unsigned int virq,
				       unsigned int nr_irqs)
{
	struct meson_gpio_irq_controller *ctl = domain->host_data;
	struct irq_data *irq_data;
	u32 *gic_hwirq;

	if (WARN_ON(nr_irqs != 1))
		return;

	irq_domain_free_irqs_parent(domain, virq, 1);

	irq_data = irq_domain_get_irq_data(domain, virq);
	gic_hwirq = irq_data_get_irq_chip_data(irq_data);

	meson_gpio_irq_release_channel(ctl, gic_hwirq);
}

static const struct irq_domain_ops meson_gpio_irq_domain_ops = {
	.alloc		= meson_gpio_irq_domain_alloc,
	.free		= meson_gpio_irq_domain_free,
	.translate	= meson_gpio_irq_domain_translate,
};

static int meson_gpio_irq_parse_dt(struct device_node *node, struct meson_gpio_irq_controller *ctl)
{
	const struct of_device_id *match;
	int ret;

	match = of_match_node(meson_irq_gpio_matches, node);
	if (!match)
		return -ENODEV;

	ctl->params = match->data;

	ret = of_property_read_variable_u32_array(node,
						  "amlogic,channel-interrupts",
						  ctl->channel_irqs,
						  ctl->params->nr_channels,
						  ctl->params->nr_channels);
	if (ret < 0) {
		pr_err("can't get %d channel interrupts\n", ctl->params->nr_channels);
		return ret;
	}

	ctl->params->ops.gpio_irq_init(ctl);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
/*
 * /sys/kernel/debug/meson-gpio-intc/<controller>: each channel's GIC line and
 * whether it is free, mapped (holds the channel, no handler) or requested.
 */
static int meson_gpio_irq_channels_show(struct seq_file *s, void *unused)
{
	struct meson_gpio_irq_controller *ctl = s->private;
	unsigned int n = ctl->params->nr_channels;
	unsigned int i, virq, used = 0;

	for (i = 0; i < n; i++)
		used += test_bit(i, ctl->channel_map);

	seq_printf(s, "channels: %u used: %u free: %u\n", n, used, n - used);
	seq_puts(s, "ch  gic-hwirq  gpio-hwirq   irq  state\n");
	for (i = 0; i < n; i++) {
		if (!test_bit(i, ctl->channel_map)) {
			seq_printf(s, "%2u  %9u  %10s  %4s  free\n",
				   i, ctl->channel_irqs[i], "-", "-");
			continue;
		}
		virq = ctl->channel_virq[i];
		seq_printf(s, "%2u  %9u  %10u  %4u  %s\n", i,
			   ctl->channel_irqs[i], ctl->channel_gpio[i], virq,
			   virq && irq_has_action(virq) ? "requested" : "mapped");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(meson_gpio_irq_channels);

static void meson_gpio_irq_debugfs_init(struct meson_gpio_irq_controller *ctl,
					struct device_node *node)
{
	/* one directory for all controllers (A4 and later have two) */
	struct dentry *dir = debugfs_lookup("meson-gpio-intc", NULL);
	bool found = dir;

	if (!found)
		dir = debugfs_create_dir("meson-gpio-intc", NULL);
	debugfs_create_file(of_node_full_name(node), 0444, dir, ctl,
			    &meson_gpio_irq_channels_fops);
	if (found)
		dput(dir);
}
#else
static void meson_gpio_irq_debugfs_init(struct meson_gpio_irq_controller *ctl,
					struct device_node *node)
{
}
#endif

static int meson_gpio_irq_probe(struct platform_device *pdev, struct device_node *parent)
{
	struct device_node *node = pdev->dev.of_node;
	struct irq_domain *domain, *parent_domain;
	struct meson_gpio_irq_controller *ctl;
	int ret;

	if (!parent) {
		pr_err("missing parent interrupt node\n");
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("unable to obtain parent domain\n");
		return -ENXIO;
	}

	ctl = kzalloc(sizeof(*ctl), GFP_KERNEL);
	if (!ctl)
		return -ENOMEM;

	raw_spin_lock_init(&ctl->lock);

	ctl->base = of_iomap(node, 0);
	if (!ctl->base) {
		ret = -ENOMEM;
		goto free_ctl;
	}

	ret = meson_gpio_irq_parse_dt(node, ctl);
	if (ret)
		goto free_channel_irqs;

	domain = irq_domain_create_hierarchy(parent_domain, 0,
					     ctl->params->nr_hwirq,
					     of_fwnode_handle(node),
					     &meson_gpio_irq_domain_ops,
					     ctl);
	if (!domain) {
		pr_err("failed to add domain\n");
		ret = -ENODEV;
		goto free_channel_irqs;
	}

	pr_info("%d to %d gpio interrupt mux initialized\n",
		ctl->params->nr_hwirq, ctl->params->nr_channels);

	meson_gpio_irq_debugfs_init(ctl, node);

	return 0;

free_channel_irqs:
	iounmap(ctl->base);
free_ctl:
	kfree(ctl);

	return ret;
}

IRQCHIP_PLATFORM_DRIVER_BEGIN(meson_gpio_intc)
IRQCHIP_MATCH("amlogic,meson-gpio-intc", meson_gpio_irq_probe)
IRQCHIP_PLATFORM_DRIVER_END(meson_gpio_intc)

MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_DESCRIPTION("Meson GPIO Interrupt Multiplexer driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:meson-gpio-intc");
