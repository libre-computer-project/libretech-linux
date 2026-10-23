/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_IRQCHIP_IRQ_MESON_GPIO_H
#define _LINUX_IRQCHIP_IRQ_MESON_GPIO_H

#include <linux/types.h>

/*
 * Register an atomic reader for the GPIO pad behind an irq-domain hwirq.
 * Called from pinctrl probe. set_type and eoi call it with the irq desc
 * lock held, so the reader must not sleep. A NULL registration leaves the
 * fwspec snapshot, and the toggle, in place.
 */
#if IS_ENABLED(CONFIG_MESON_IRQ_GPIO)
int meson_gpio_irq_set_pad_level(int (*read_pad)(unsigned int hwirq, bool *high));
#else
static inline int
meson_gpio_irq_set_pad_level(int (*read_pad)(unsigned int hwirq, bool *high))
{
	return -ENODEV;
}
#endif

#endif
