// SPDX-License-Identifier: GPL-2.0
/*
 * Amlogic SM1 kernel heartbeat for the open System Controller.
 *
 * The open SC (scp-firmware, SCP_G12A_AP_HEARTBEAT) pets the AO watchdog from
 * its MHU poll loop.  Once it has seen this heartbeat change, it stops that
 * pet when the counter stays still for its grace period, and the AO watchdog
 * resets the chip: a kernel that stops scheduling is reset even when no EE
 * watchdog runs.
 *
 * One 32-bit word in the SC's AP->SC SRAM: [31:28] state, [27:0] a counter
 * this driver advances every second.  States:
 *   RUN     (0x5)  counting
 *   PAUSE   (0x9)  system sleep: the SC keeps petting
 *   RELEASE (0xA)  halt, power-off or kexec: the SC stops watching
 * Anything else reads as "not live" to the SC.  The SC latches only on a
 * change it sees, so the counter starts from the value found + 1: a word an
 * earlier boot left behind is never mistaken for this one.
 *
 * The beat is a delayed_work, not a timer: vpanic() re-enables IRQs before
 * its endless mdelay() loop, so a timer would keep beating inside a panicked
 * kernel (measured on the board, gate G2.1).  Work needs the scheduler, so a
 * panic, a scheduler lockup or a workqueue stall stops the beat; a panic
 * notifier also stops any further write.
 *
 * Restart writes nothing: the reset goes through the EE watchdog, which
 * stops the SC, so a hang on the way down is still reset.  The word is
 * always written whole from the locked copy, and every state write first
 * cancels the work, so a beat on another CPU cannot overwrite PAUSE/RELEASE.
 *
 * Built in only: the kexec test reads kexec_in_progress, which is not
 * exported to modules.  The device tree node opts a board in; without it
 * nothing here runs.
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kexec.h>
#include <linux/of.h>
#include <linux/panic_notifier.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/reboot.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#define SC_HB_STATE		GENMASK(31, 28)
#define SC_HB_COUNTER		GENMASK(27, 0)
#define SC_HB_RUN		0x5
#define SC_HB_PAUSE		0x9
#define SC_HB_RELEASE		0xa
#define SC_HB_PERIOD		HZ

struct sc_hb {
	struct device *dev;
	void __iomem *word;
	spinlock_t lock;		/* state, counter, stopped */
	u32 state;
	u32 counter;
	bool stopped;
	bool panicked;			/* set once by the panic notifier */
	struct workqueue_struct *wq;
	struct delayed_work beat;
	struct notifier_block reboot_nb;
	struct notifier_block panic_nb;
};

static u32 sc_hb_compose(struct sc_hb *hb)
{
	return FIELD_PREP(SC_HB_STATE, hb->state) |
	       FIELD_PREP(SC_HB_COUNTER, hb->counter);
}

static void sc_hb_beat(struct work_struct *work)
{
	struct sc_hb *hb = container_of(to_delayed_work(work), struct sc_hb,
					beat);
	unsigned long flags;
	bool again = false;

	spin_lock_irqsave(&hb->lock, flags);
	if (!hb->stopped && !READ_ONCE(hb->panicked)) {
		hb->counter = (hb->counter + 1) & SC_HB_COUNTER;
		writel(sc_hb_compose(hb), hb->word);
		again = true;
	}
	spin_unlock_irqrestore(&hb->lock, flags);

	if (again)
		queue_delayed_work(hb->wq, &hb->beat, SC_HB_PERIOD);
}

/* Cancel the beat, write the state, read it back; restart the beat for RUN. */
static void sc_hb_set_state(struct sc_hb *hb, u32 state)
{
	unsigned long flags;
	u32 written, readback;
	bool run;

	spin_lock_irqsave(&hb->lock, flags);
	hb->stopped = true;
	spin_unlock_irqrestore(&hb->lock, flags);
	cancel_delayed_work_sync(&hb->beat);

	spin_lock_irqsave(&hb->lock, flags);
	hb->state = state;
	written = sc_hb_compose(hb);
	writel(written, hb->word);
	readback = readl(hb->word);
	run = state == SC_HB_RUN;
	hb->stopped = !run;
	spin_unlock_irqrestore(&hb->lock, flags);

	if (run)
		queue_delayed_work(hb->wq, &hb->beat, SC_HB_PERIOD);

	dev_info(hb->dev, "state %x: wrote %08x, read back %08x\n",
		 state, written, readback);
}

static int sc_hb_panic(struct notifier_block *nb, unsigned long action,
		       void *data)
{
	struct sc_hb *hb = container_of(nb, struct sc_hb, panic_nb);

	/* Atomic context: no lock, no cancel; the beat checks this flag. */
	WRITE_ONCE(hb->panicked, true);
	return NOTIFY_DONE;
}

static int sc_hb_reboot(struct notifier_block *nb, unsigned long action,
			void *data)
{
	struct sc_hb *hb = container_of(nb, struct sc_hb, reboot_nb);

	if (action == SYS_HALT || action == SYS_POWER_OFF ||
	    (action == SYS_RESTART && IS_ENABLED(CONFIG_KEXEC_CORE) &&
	     kexec_in_progress))
		sc_hb_set_state(hb, SC_HB_RELEASE);

	return NOTIFY_DONE;
}

static int sc_hb_suspend_noirq(struct device *dev)
{
	sc_hb_set_state(dev_get_drvdata(dev), SC_HB_PAUSE);
	return 0;
}

static int sc_hb_resume_noirq(struct device *dev)
{
	sc_hb_set_state(dev_get_drvdata(dev), SC_HB_RUN);
	return 0;
}

static const struct dev_pm_ops sc_hb_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(sc_hb_suspend_noirq, sc_hb_resume_noirq)
};

static int sc_hb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct sc_hb *hb;
	u32 found;
	int ret;

	hb = devm_kzalloc(dev, sizeof(*hb), GFP_KERNEL);
	if (!hb)
		return -ENOMEM;
	hb->dev = dev;
	spin_lock_init(&hb->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) < sizeof(u32))
		return dev_err_probe(dev, -EINVAL, "no heartbeat word\n");
	/*
	 * Map WITHOUT requesting: the SRAM window is already claimed by the
	 * mmio-sram node (meson-gxl-ddr-pub-mon.c does the same).
	 */
	hb->word = devm_ioremap(dev, res->start, resource_size(res));
	if (!hb->word)
		return -ENOMEM;

	found = readl(hb->word);
	hb->counter = (FIELD_GET(SC_HB_COUNTER, found) + 1) & SC_HB_COUNTER;
	hb->state = SC_HB_RUN;
	/*
	 * Own workqueue: WQ_MEM_RECLAIM gives the beat a rescuer, so memory
	 * pressure that keeps a pool from creating workers does not stall it
	 * into a false reset; WQ_HIGHPRI keeps it off the nice-0 pool.  Never
	 * destroyed: built in, unbind suppressed.
	 */
	hb->wq = alloc_workqueue("sc-heartbeat", WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	if (!hb->wq)
		return -ENOMEM;
	INIT_DELAYED_WORK(&hb->beat, sc_hb_beat);
	platform_set_drvdata(pdev, hb);

	hb->reboot_nb.notifier_call = sc_hb_reboot;
	hb->reboot_nb.priority = INT_MIN;	/* last: as late as the chain allows */
	ret = devm_register_reboot_notifier(dev, &hb->reboot_nb);
	if (ret)
		return dev_err_probe(dev, ret, "reboot notifier\n");

	/*
	 * Registered last, once probe cannot fail: built in with unbind
	 * suppressed, the device (and so hb) lives as long as the kernel.
	 */
	hb->panic_nb.notifier_call = sc_hb_panic;
	hb->panic_nb.priority = INT_MAX;	/* first: stop the beat at once */
	atomic_notifier_chain_register(&panic_notifier_list, &hb->panic_nb);

	dev_info(dev, "found %08x, starting at counter %07x\n",
		 found, hb->counter);
	sc_hb_set_state(hb, SC_HB_RUN);
	return 0;
}

static const struct of_device_id sc_hb_of_match[] = {
	{ .compatible = "amlogic,sm1-sc-heartbeat" },
	{ }
};

static struct platform_driver sc_hb_driver = {
	.probe = sc_hb_probe,
	.driver = {
		.name = "meson-sc-heartbeat",
		.of_match_table = sc_hb_of_match,
		.pm = pm_sleep_ptr(&sc_hb_pm_ops),
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(sc_hb_driver);
