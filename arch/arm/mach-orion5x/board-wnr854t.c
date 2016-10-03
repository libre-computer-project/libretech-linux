/*
 * Netgear WNR854T PCI setup
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <asm/mach/pci.h>
#include "common.h"
#include "orion5x.h"

#define WNR854T_PCI_SLOT0_OFFS	7
#define WNR854T_PCI_SLOT0_IRQ_PIN	4

static void __init wnr854t_pci_preinit(void)
{
	int pin;

	/*
	 * Configure PCI GPIO IRQ pins
	 */
	pin = WNR854T_PCI_SLOT0_IRQ_PIN;
	if (gpio_request(pin, "PCI Int") == 0) {
		if (gpio_direction_input(pin) == 0) {
			irq_set_irq_type(gpio_to_irq(pin), IRQ_TYPE_LEVEL_LOW);
		} else {
			pr_err("wnr854t_pci_preinit failed to set_irq_type pin %d\n",
				pin);
			gpio_free(pin);
		}
	} else {
		pr_err("wnr854t_pci_preinit failed to request gpio %d\n", pin);
	}
}

static int __init wnr854t_pci_map_irq(const struct pci_dev *dev, u8 slot,
	u8 pin)
{
	int irq;

	/*
	 * Check for devices with hard-wired IRQs.
	 */
	irq = orion5x_pci_map_irq(dev, slot, pin);
	if (irq != -1)
		return irq;

	/*
	 * PCI IRQs are connected via GPIOs
	 */
	switch (slot - WNR854T_PCI_SLOT0_OFFS) {
	case 0:
		return gpio_to_irq(WNR854T_PCI_SLOT0_IRQ_PIN);
	default:
		return -1;
	}
}

static struct hw_pci wnr854t_pci __initdata = {
	.nr_controllers	= 2,
	.preinit	= wnr854t_pci_preinit,
	.setup		= orion5x_pci_sys_setup,
	.scan		= orion5x_pci_sys_scan_bus,
	.map_irq	= wnr854t_pci_map_irq,
};

static int __init wnr854t_pci_init(void)
{
	if (of_machine_is_compatible("netgear,wnr854t"))
		pci_common_init(&wnr854t_pci);

	return 0;
}
/* NB: Use late_initcall so we can gpio_request() without being deferred */
late_initcall(wnr854t_pci_init);
