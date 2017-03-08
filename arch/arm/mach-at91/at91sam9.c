/*
 *  Setup code for AT91SAM9
 *
 *  Copyright (C) 2011 Atmel,
 *                2011 Nicolas Ferre <nicolas.ferre@atmel.com>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/of.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>
#include <asm/system_misc.h>

#include "generic.h"

static void __init at91sam9_common_init(void)
{
	of_platform_default_populate(NULL, NULL, NULL);
}

static void __init at91sam9_dt_device_init(void)
{
	at91sam9_common_init();
	at91sam9260_pm_init();
}

static const char *const at91_dt_board_compat[] __initconst = {
	"atmel,at91sam9",
	NULL
};

DT_MACHINE_START(at91sam_dt, "Atmel AT91SAM9")
	/* Maintainer: Atmel */
	.init_machine	= at91sam9_dt_device_init,
	.dt_compat	= at91_dt_board_compat,
MACHINE_END

static void __init at91sam9g45_dt_device_init(void)
{
	at91sam9_common_init();
	at91sam9g45_pm_init();
}

static const char *const at91sam9g45_board_compat[] __initconst = {
	"atmel,at91sam9g45",
	NULL
};

DT_MACHINE_START(at91sam9g45_dt, "Atmel AT91SAM9G45")
	/* Maintainer: Atmel */
	.init_machine	= at91sam9g45_dt_device_init,
	.dt_compat	= at91sam9g45_board_compat,
MACHINE_END

static void __init at91sam9x5_dt_device_init(void)
{
	at91sam9_common_init();
	at91sam9x5_pm_init();
}

static const char *const at91sam9x5_board_compat[] __initconst = {
	"atmel,at91sam9x5",
	"atmel,at91sam9n12",
	NULL
};

DT_MACHINE_START(at91sam9x5_dt, "Atmel AT91SAM9")
	/* Maintainer: Atmel */
	.init_machine	= at91sam9x5_dt_device_init,
	.dt_compat	= at91sam9x5_board_compat,
MACHINE_END
