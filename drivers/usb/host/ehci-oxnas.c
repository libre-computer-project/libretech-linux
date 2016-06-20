/*
 * EHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 2005 John Larkworthy <john.larkworthy@oxsemi.com>
 * 
 * OXNAS Bus Glue
 *
 * Written by John Larkworthy 
 *
 * This file is licenced under the GPL.
 */

#include <linux/platform_device.h>
#include <asm/hardware.h>

extern spinlock_t oxnas_gpio_spinlock;

int usb_patch = 1;

module_param(usb_patch, int, 1);
MODULE_PARM_DESC (usb_patch, "use usb hw patch");

/* called during probe() after chip reset completes */
static int ehci_oxnas_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	int temp;
	int retval;

	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	retval = ehci_halt(ehci);
	if (retval)
			return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	if (ehci_is_TDI(ehci))
		ehci_reset(ehci);

	/* at least the Genesys GL880S needs fixup here */
	temp = HCS_N_CC(ehci->hcs_params) * HCS_N_PCC(ehci->hcs_params);
	temp &= 0x0f;
	if (temp && HCS_N_PORTS(ehci->hcs_params) > temp) {
		ehci_dbg(ehci, "bogus port configuration: "
			"cc=%d x pcc=%d < ports=%d\n",
			HCS_N_CC(ehci->hcs_params),
			HCS_N_PCC(ehci->hcs_params),
			HCS_N_PORTS(ehci->hcs_params));
	}

	ehci_port_power(ehci, 0);

	return retval;
}

static const struct hc_driver ehci_oxnas_driver = {
	.description =		hcd_name,
	.product_desc =		"OXNAS EHCI Host Controller",
	.hcd_priv_size =	sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =   ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset =   ehci_oxnas_setup,
	.start =   ehci_run,
#ifdef	CONFIG_PM
	.suspend = ehci_suspend,
	.resume =  ehci_resume,
#endif
	.stop =	   ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ehci_urb_enqueue,
	.urb_dequeue =		ehci_urb_dequeue,
	.endpoint_disable =	ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ehci_hub_status_data,
	.hub_control =		ehci_hub_control,
	.bus_suspend =		ehci_bus_suspend,
	.bus_resume =		ehci_bus_resume,
};

static int start_oxnas_usb_ehci(struct platform_device *dev)
{
    unsigned long flags;
    unsigned long input_polarity = 0;
    unsigned long output_polarity = 0;
    unsigned long power_switch_mask = 0;
    unsigned long power_monitor_mask = 0;
    unsigned long power_lines_mask = 0;

	if (usb_disabled())
		return -ENODEV;

	pr_debug("%s: block sizes: qh %Zd qtd %Zd itd %Zd sitd %Zd\n",
		hcd_name,
		sizeof (struct ehci_qh), sizeof (struct ehci_qtd),
		sizeof (struct ehci_itd), sizeof (struct ehci_sitd));

#ifdef CONFIG_OXNAS_USB_PORTA_POWER_CONTROL
    power_switch_mask  |= (1UL << USBA_POWO_GPIO);
    power_monitor_mask |= (1UL << USBA_OVERI_GPIO);
#endif // CONFIG_OXNAS_USB_PORTA_POWER_CONTROL

#ifdef CONFIG_OXNAS_USB_PORTB_POWER_CONTROL
    power_switch_mask  |= (1UL << USBB_POWO_GPIO);
    power_monitor_mask |= (1UL << USBB_OVERI_GPIO);
#endif // CONFIG_OXNAS_USB_PORTB_POWER_CONTROL

#ifdef CONFIG_OXNAS_USB_PORTC_POWER_CONTROL
    power_switch_mask  |= (1UL << USBC_POWO_GPIO);
    power_monitor_mask |= (1UL << USBC_OVERI_GPIO);
#endif // CONFIG_OXNAS_USB_PORTC_POWER_CONTROL

    power_lines_mask = power_switch_mask | power_monitor_mask;

    // Configure USB power monitoring input and switch output GPIOs
#ifdef CONFIG_OXNAS_USB_OVERCURRENT_POLARITY_NEGATIVE
    input_polarity = ((1UL << SYS_CTRL_USBHSMPH_IP_POL_A_BIT) |
                      (1UL << SYS_CTRL_USBHSMPH_IP_POL_B_BIT) |
                      (1UL << SYS_CTRL_USBHSMPH_IP_POL_C_BIT));
#endif // CONFIG_OXNAS_USB_OVERCURRENT_POLARITY_NEGATIVE

#ifdef CONFIG_OXNAS_USB_POWER_SWITCH_POLARITY_NEGATIVE
    output_polarity = ((1UL << SYS_CTRL_USBHSMPH_OP_POL_A_BIT) |
                       (1UL << SYS_CTRL_USBHSMPH_OP_POL_B_BIT) |
                       (1UL << SYS_CTRL_USBHSMPH_OP_POL_C_BIT));
#endif // CONFIG_OXNAS_USB_POWER_SWITCH_POLARITY_NEGATIVE

    // Enable primary function on USB power monitor and switch lines
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_0) |  power_lines_mask, SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_SECSEL_CTRL_0)  & ~power_lines_mask, SYS_CTRL_GPIO_SECSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_TERTSEL_CTRL_0) & ~power_lines_mask, SYS_CTRL_GPIO_TERTSEL_CTRL_0);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

    // Enable GPIO output on USB power switch output GPIOs
    writel(power_switch_mask, GPIO_A_OUTPUT_ENABLE_SET);

    // Enable GPIO input on USB power monitoring input GPIOs
    writel(power_monitor_mask, GPIO_A_OUTPUT_ENABLE_CLEAR);

    // Set the polarity of the USB power switch output and monitoring
    // inputs in system control
    if (usb_patch) {
        writel(input_polarity | output_polarity| (1<<6) , SYS_CTRL_USBHSMPH_CTRL);
    } 
    else {
            writel(input_polarity | output_polarity, SYS_CTRL_USBHSMPH_CTRL);
    }
    
    // Ensure the USB block is properly reset
    writel(1UL << SYS_CTRL_RSTEN_USBHS_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_USBHS_BIT, SYS_CTRL_RSTEN_CLR_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_USBHSPHY_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_USBHSPHY_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

    // Force the high speed clock to be generated all the time, via serial
    // programming of the USB HS PHY
    writel((2UL << SYS_CTRL_USBHSPHY_TEST_ADD) |
           (0xe0UL << SYS_CTRL_USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

    writel((1UL << SYS_CTRL_USBHSPHY_TEST_CLK) |
           (2UL << SYS_CTRL_USBHSPHY_TEST_ADD) |
           (0xe0UL << SYS_CTRL_USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

    writel((0xfUL << SYS_CTRL_USBHSPHY_TEST_ADD) | 
           (0xaaUL << SYS_CTRL_USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

    writel((1UL << SYS_CTRL_USBHSPHY_TEST_CLK) |
           (0xfUL << SYS_CTRL_USBHSPHY_TEST_ADD) |
           (0xaaUL << SYS_CTRL_USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

    // Enable the clock to the USB block
    writel(1UL << SYS_CTRL_CKEN_USBHS_BIT, SYS_CTRL_CKEN_SET_CTRL);

    // Ensure reset and clock operations are complete
    wmb();

	return 0;
}

static void stop_oxnas_usb_ehci(struct platform_device *dev)
{
	// put usb core into reset
    writel(1UL << SYS_CTRL_RSTEN_USBHS_BIT, SYS_CTRL_RSTEN_SET_CTRL);

    // Disable the clock to the USB block
    writel(1UL << SYS_CTRL_CKEN_USBHS_BIT, SYS_CTRL_CKEN_CLR_CTRL);
}

/**
 * usb_hcd_oxnas_probe - initialize OXNAS-based HCD
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 */
static int usb_hcd_oxnas_probe(const struct hc_driver *driver, struct platform_device *dev)
{
	int retval;
	unsigned long ehci_id;
	struct usb_hcd *hcd = 0;
	struct ehci_hcd *ehci;

	if (dev->num_resources != 2) {
		pr_debug("wrong number of resources %d, expected %d", dev->num_resources, 2);
	}

	start_oxnas_usb_ehci(dev);

	if (((ehci_id = readl(USB_BASE)) & 0x2f) != 0x05) {
		pr_debug("wrong chip ID found %lx", ehci_id);
		return -ENODEV;
	}

	hcd = usb_create_hcd(driver, &dev->dev, "usb");
	if (!hcd) {
		pr_debug("usb_create_hcd() failed");
		retval = -ENOMEM;
	}
	hcd->regs = (void *)(USB_BASE + 0x100); /* adjust to point at cap length register */

	printk(DRIVER_INFO "@%p Device ID register %lx\n", (void *)USB_BASE, *(unsigned long *)USB_BASE);

	/* OXNAS device has a transaction translator */
	ehci = hcd_to_ehci(hcd);
	ehci->is_tdi_rh_tt = 1;

	/* Finished initialisation and register */
	if ((retval = usb_add_hcd(hcd, dev->resource[1].start, 0))) {
		pr_debug("usb_add_hcd() failed");
		stop_oxnas_usb_ehci(dev);
		usb_put_hcd(hcd);
		return retval;
	}
	return 0;
}

/**
 * usb_hcd_oxnas_remove - shutdown processing for OXNAS-based HCD
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_oxnas_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_oxnas_remove(struct usb_hcd *hcd, struct platform_device *dev)
{
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	stop_oxnas_usb_ehci(dev);
}

static int ehci_hcd_oxnas_drv_probe(struct platform_device *dev)
{
	if (usb_disabled())
		return -ENODEV;

	return usb_hcd_oxnas_probe(&ehci_oxnas_driver, dev);
}

static int ehci_hcd_oxnas_drv_remove(struct platform_device *dev)
{
	usb_hcd_oxnas_remove(platform_get_drvdata(dev), dev);
	return 0;
}

MODULE_ALIAS("oxnas-ehci");

static struct platform_driver ehci_hcd_oxnas_driver = {
	.probe = ehci_hcd_oxnas_drv_probe,
	.remove = ehci_hcd_oxnas_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		.name = "oxnas-ehci",
	},
};
