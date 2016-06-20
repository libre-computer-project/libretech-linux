/*
 * linux/arch/arm/mach-oxnas/wdc-leds.c
 *
 * Copyright (C) 2006 Western Digital
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <asm/hardware.h>

#define DEBUG

#ifdef DEBUG
   #define DUMP(A,B)  printk(KERN_INFO A,B);
#else
    #define DUMP(A,B)
#endif

/* Number of LEDs */
#define NUM_ACTIVITY_LEDS   4
#define NUM_FUEL_GAUGE_LEDS 6

/* Timer Values and Pulse Width Modulation */
#define PWM_RESOLUTION  255
#define TIMER_LED_MODE  TIMER_MODE_PERIODIC

#define PWM_CLOCK_DATA (

#define LED100  (PWM_RESOLUTION)        /* 100% duty cycle */
#define LED50   (PWM_RESOLUTION / 2)    /* 50%  duty cycle */
#define LED25   (PWM_RESOLUTION / 4)    /* 25%  duty cycle */

#define STEP_RESOLUTION (16)                /* change intensity in 16 steps */

/* Setup Timer2 prescaler, operation mode, and start it */
#define PERIODIC_INTERRUPT                          \
   (                                                \
      (TIMER_PRESCALE_256   << TIMER_PRESCALE_BIT) | \
      (TIMER_LED_MODE      << TIMER_MODE_BIT)     | \
      (TIMER_ENABLE_ENABLE << TIMER_ENABLE_BIT)     \
   )

/*
 * Target frame rate is 60Hz.  Slower frame rates flicker badly.
 * Since each frame has 16 divisions to perform the pulse width
 * modulation that means we need the timer set to 960Hz (i.e. 60 * 16)
 *
 * With a system clock of 25Mhz and a load register value of 1627 prescaled 256 
 * to achieve 60Hz:
 *   25Mhz / 256 / 1627 = ~60
 */
#define FAST_TIMER_INT      (1627)      /* Timer2 count down      */
#define SYS_CLOCK       (25000000)      /* System clock frequency */
#define PRESCALE_VALUE        (256)      /* Value set in prescaler */
#define PWM_PRESCALE       814  /* Value loaded on PWM clock register */
#define MAX_PWM   255
#define SLOW_TPS   ((SYS_CLOCK/PRESCALE_VALUE) / FAST_TIMER_INT)

/* The GPIO assignent to LED Masks need to make sure that the 
 * LED_MASK_GPIO_A and LED_MASK_GPIO_B are set appropriately 
 * based on the GPIOs assigned
 */

/* GPIO bits dedicated to LEDs */
#define LED_MASK_ACT12 (1 << GPIO_6)    /* Activity 12 o'clock   */
#define LED_MASK_ACT3  (1 << GPIO_7)    /* Activity 3 o'clock    */
#define LED_MASK_ACT6  (1 << GPIO_5)    /* Activity 6 o'clock    */
#define LED_MASK_ACT9  (1 << GPIO_34)   /* Activity 9 o'clock    */
#define LED_MASK_FG12  (1 << GPIO_10)   /* Fuel Gauge 10 o'clock */
#define LED_MASK_FG2   (1 << GPIO_9)    /* Fuel Gauge 8 o'clock  */
#define LED_MASK_FG4   (1 << GPIO_25)   /* Fuel Gauge 6 o'clock  */
#define LED_MASK_FG6   (1 << GPIO_26)   /* Fuel Gauge 4 o'clock  */
#define LED_MASK_FG8   (1 << GPIO_27)   /* Fuel Gauge 2 o'clock  */
#define LED_MASK_FG10  (1 << GPIO_33)   /* Fuel Gauge 12 o'clock */

/*
 * Mask for all the LEDs in the Fuel Gauge.  This is in frame buffer format
 * (see LED frame buffer design assumption comments below, near the
 * frame_buffer declaration).
 */
#define LED_MASK_FUEL_GAUGE \
   (                        \
      LED_MASK_FG12 |       \
      LED_MASK_FG2  |       \
      LED_MASK_FG4  |       \
      LED_MASK_FG6  |       \
      LED_MASK_FG8  |       \
      LED_MASK_FG10         \
   )

/* Mask for all the Activity LEDs */
#define LED_MASK_ACTIVITY \
   (                      \
      LED_MASK_ACT12 |    \
      LED_MASK_ACT3  |    \
      LED_MASK_ACT6  |    \
      LED_MASK_ACT9       \
   )

/* The GPIO_A and GPIO_B Masks below need to be set based on the 
 * GPIO numbers that are assigned for the LED MASKS
 */

/* Mask for all the LEDs on GPIO_A */
#define LED_MASK_GPIO_A \
   (                    \
      LED_MASK_ACT12 |  \
      LED_MASK_ACT3  |  \
      LED_MASK_ACT6  |  \
      LED_MASK_FG12  |  \
      LED_MASK_FG2   |  \
      LED_MASK_FG4   |  \
      LED_MASK_FG6   |  \
      LED_MASK_FG8      \
   )

/* Mask for all the LEDs on GPIO_B */
#define LED_MASK_GPIO_B \
   (                    \
      LED_MASK_ACT9  |  \
      LED_MASK_FG10     \
   )

/* I/O register access (FIXME: why not use the standard linux macros?) */
#define ox_writel(data, addr) (*(volatile unsigned long*)addr = (data))
#define ox_readl(addr)        (*(volatile unsigned long*)addr)
#define writel(data, addr)    (*(volatile u32*)addr = (data))
#define readl(addr)           (*(volatile u32*)addr)
#define CLEAR(addr, mask)     writel(readl(addr) & ~mask, addr)


/* Variables to hold the number of LED classes created */
static int leds_created;

/* Variables to save/restore timer register values */
static u32 timer_load;
static u32 timer_control;

/* LED polarity */
static int negative_led_logic = 0;
module_param(negative_led_logic, bool, 0);

/*
 * States for the main LED behavior state machine.  The order of these states
 * is important.  In particular, the TRANS states use this ordering so that
 * incrementing the state variable walks through the ordering shown below.
 * This allows most of the TRANS states to share common code.
 */
enum {
    FULLY_ON__ENTRY,            /* FullyOn    - Initialize                       */
    FULLY_ON__RE_ENTRY,         /* FullyOn    - Initialize (skipping POR)        */
    FULLY_ON__POR_RAMP_UP,      /* FullyOn    - Ramp up full ring                */
    FULLY_ON__POR_HOLD,         /* FullyOn    - Hold a while with full ring      */
    FULLY_ON__RAMP_UP,          /* FullyOn    - Ramp up full ring (skipping POR) */
    FULLY_ON__ACTIVITY,         /* FullyOn    - Read/write activity              */
    FULLY_ON__IDLE_HOLD,        /* FullyOn    - Hold a while for no R/W activity */
    FULLY_ON__RAMP_DOWN,        /* FullyOn    - Ramp down full ring              */
    STANDBY__ENTRY,             /* Standby    - Initialize                       */
    STANDBY__DARK,              /* Standby    - Full ring off                    */
    STANDBY__RAMP_UP,           /* Standby    - Ramp up full ring                */
    STANDBY__RAMP_DOWN,         /* Standby    - Ramp down full ring              */
    POWER_OFF__ENTRY,           /* PowerOff   - Initialize                       */
    POWER_OFF__DARK,            /* PowerOff   - Full ring off                    */
    DEGRADED__ENTRY,            /* Degraded   - Initialize                       */
    DEGRADED__BLINK1,           /* Degraded   - Blink step1                      */
    DEGRADED__BLINK2,           /* Degraded   - Blink step2                      */
    OVERTEMP__ENTRY,            /* OverTemp   - Initialize                       */
    OVERTEMP__BLINK1,           /* OverTemp   - Blink step1                      */
    OVERTEMP__BLINK2,           /* OverTemp   - Blink step2                      */
    TRANS__ENTRY,               /* Transition - Initialize                       */
    TRANS__1_UP,                /* 1st cycle  - up   12 & 6 - down 9 & 3 o'clock */
    TRANS__1_DN,                /* 1st cycle  - down 12 & 6 - up   9 & 3 o'clock */
    TRANS__2_UP,                /* 2nd cycle  - up   12 & 6 - down 9 & 3 o'clock */
    TRANS__2_DN,                /* 2nd cycle  - down 12 & 6 - up   9 & 3 o'clock */
    TRANS__3_UP,                /* 3rd cycle  - up   12 & 6 - down 9 & 3 o'clock */
    TRANS__3_DN,                /* 3rd cycle  - down 12 & 6 - up   9 & 3 o'clock */
    TRANS__4_UP,                /* 4th cycle  - up   12 & 6 - down 9 & 3 o'clock */
    TRANS__4_DN                 /* 4th cycle  - down 12 & 6 - down 9 & 3 o'clock */
};


/* Pattern for the activity behavior */
const u16 activity_pattern[NUM_ACTIVITY_LEDS] = {
    LED100,
    LED25,
    0,
    0
};


/* Various LED state machine constants */
#define TRANS_STEP           (1)
#define SLEEP_HI             (0)
#define SLEEP_LO             (-STEP_RESOLUTION)
#define NUM_POWER_STEPS      (STEP_RESOLUTION)
#define NUM_BREATHE_STEPS    (SLEEP_HI - SLEEP_LO)
#define NUM_TRANS_STEPS      ((SLEEP_HI - SLEEP_LO) / TRANS_STEP)
#define BIT_MASK_FUEL_GAUGE  ((1 << NUM_FUEL_GAUGE_LEDS) - 1)

/*
 * Calculate various speeds of the LED state machine based on the system 
 * clock frequency, the hardware timer prescaler, and the hardware timer
 * load register value.  This results in a calculation of the ticks per
 * second (TPS) of the timer interrupt used to perform the pulse width
 * modulation and the slower TPS of the tasklet that performs the main
 * state machine of the LED behavior.  The remaining speeds are
 * calculated from the slow TPS.
 */
 #define RW_SPEED       (SLOW_TPS / 4)   /* want ~4Hz   */
#define BLINK_SPEED    (SLOW_TPS / 2)   /* want ~2Hz   */
#define HOLD_BREATH    (SLOW_TPS * 4)   /* want ~4sec  */
#define HOLD_ON_ENTER  (SLOW_TPS * 10)  /* want ~10sec */
#define POWER_SPEED    (SLOW_TPS / NUM_POWER_STEPS)
#define BREATHE_SPEED  (SLOW_TPS / NUM_BREATHE_STEPS)
#define TRANS_SPEED    (SLOW_TPS / NUM_TRANS_STEPS)

#if \
   (!SLOW_TPS)      || \
   (!RW_SPEED)      || \
   (!BLINK_SPEED)   || \
   (!POWER_SPEED)   || \
   (!BREATHE_SPEED) || \
   (!TRANS_SPEED)
#   error TPS calculation(s) resulted in zero!
#endif


/* Variables for main LED behavior state machine */
static int state;
static int next_state;
static int active_count;
static int count;
static u32 fuel_gauge_bits;     /* see LED frame buffer design assumption */
static u8 need_to_display_current_fuel_gauge;
static u8 inner_ring_rotate;
static u8 ignore_activity;
static s8 active_tail;
static s8 ramp1;
static s8 ramp2;
static s8 activity_led[NUM_ACTIVITY_LEDS];
static u16 rebuild_percentage;  /* 0=not rebuilding */


/*
 * Declare tasklet for the LED behavior state machine.  The interrupt will
 * only handle the pulse width modulation which can be performed quickly.
 * The slower LED behavior state machine does not need to execute at such
 * a high frequency so it will be executed by a tasklet that is
 * periodically scheduled by the interrupt.
 */
void wdc_leds_behavior(unsigned long);
DECLARE_TASKLET(wdc_leds_behavior_tasklet, wdc_leds_behavior, 0);


/***************************************************************************/
/* FUNCTION: wdc_leds_interrupt                                            */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Interrupt handler for the wdc-leds pulse width modulation             */
/***************************************************************************/
static irqreturn_t wdc_leds_interrupt
    (int irq, void *dev_id) {
    ox_writel(0, TIMER2_CLEAR);


    tasklet_schedule(&wdc_leds_behavior_tasklet);

    return IRQ_HANDLED;
}


/***************************************************************************/
/* FUNCTION: get_inner_ring_bits                                           */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Convert the bit map of inner ring LEDs into the GPIO bit map          */
/***************************************************************************/
static u32 get_inner_ring_bits(u16 value)
{
    u32 pattern = 0;

    // Convert the bit map to the GPIO bit pattern
    if (value & (1 << 0))
        pattern |= LED_MASK_FG2;
    if (value & (1 << 1))
        pattern |= LED_MASK_FG4;
    if (value & (1 << 2))
        pattern |= LED_MASK_FG6;
    if (value & (1 << 3))
        pattern |= LED_MASK_FG8;
    if (value & (1 << 4))
        pattern |= LED_MASK_FG10;
    if (value & (1 << 5))
        pattern |= LED_MASK_FG12;

    return pattern;
}


/***************************************************************************/
/* FUNCTION: get_percentage_pattern                                        */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Convert a percentage to a inner ring bit map.  Note, we never display */
/*   less than 1 LED for a percentage so that something is alway visible.  */
/***************************************************************************/
static u16 get_percentage_pattern(u16 percentage)
{
    if (percentage >= 50) {
        if (percentage >= 67) {
            if (percentage >= 83) {
                if (percentage >= 97) {
                    return 0x3F;        // 6 LEDs is >= 97%
                } else {
                    return 0x1F;        // 5 LEDs is >=  83%
                }
            } else {
                return 0x0F;    // 4 LEDs is >=  67%
            }
        } else {
            return 0x07;        // 3 LEDs is >=  50%
        }
    } else {
        if (percentage >= 33) {
            return 0x03;        // 2 LEDs is >=  33%
        } else {
            return 0x01;        // 1 LED  is >=   0%
        }
    }
}


/***************************************************************************/
/* FUNCTION: set_led                                                       */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set frame buffer for the requested brightness on the requested LED(s) */
/***************************************************************************/
static void set_led(u32 led_bits, s8 value)
{
    u32 bit;
    s8 count = 0;

    if (negative_led_logic) {
        value = MAX_PWM - value;
    }

    for (bit = 1; bit > 0; bit <<= 1, ++count) {
        if (bit & led_bits) writel(value, ((u32 *)PWM_DATA_REGISTER_BASE + count ));
    }
}


/***************************************************************************/
/* FUNCTION: display_inner_ring                                            */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set frame buffer for the requested inner ring bit map                 */
/***************************************************************************/
static void display_inner_ring(u32 inner_ring_bits)
{
    set_led(~inner_ring_bits & LED_MASK_FUEL_GAUGE, 0);
    set_led(inner_ring_bits, LED100);
}


/***************************************************************************/
/* FUNCTION: display_current_fuel_gauge                                    */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Display the current Fuel Gauge if it is not already being displayed   */
/***************************************************************************/
static void display_current_fuel_gauge(void)
{
    if (need_to_display_current_fuel_gauge) {
        display_inner_ring(fuel_gauge_bits);
        need_to_display_current_fuel_gauge = 0;
    }
}


/***************************************************************************/
/* FUNCTION: handle_inner_ring                                             */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform the LED behavior for the inner ring                           */
/***************************************************************************/
static void handle_inner_ring(void)
{
    /* If currently rebuilding then display the rebuild percentage as */
    /* a series of LEDs representing the percentage complete rotating */
    /* around the inner ring.  Note, the percentage is rotated to     */
    /* distinguish it from normal fuel gauge behavior.                */
    if (rebuild_percentage) {
        /* Convert the rebuild percentage into a bit map of LEDs */
        u32 rotated_pattern = get_percentage_pattern(rebuild_percentage);
        /* Now rotate that pattern */
        rotated_pattern <<= inner_ring_rotate;
        rotated_pattern |= (rotated_pattern >> NUM_FUEL_GAUGE_LEDS);
        rotated_pattern &= BIT_MASK_FUEL_GAUGE;
        /* Now display the rotated pattern on the inner ring */
        display_inner_ring(get_inner_ring_bits(rotated_pattern));
        if (++inner_ring_rotate >= NUM_FUEL_GAUGE_LEDS) {
            inner_ring_rotate = 0;
        }

        need_to_display_current_fuel_gauge = 1;
    }

    /* Otherwise not rebuilding so just display normal fuel gauge */
    else {
        display_current_fuel_gauge();
    }
}


/***************************************************************************/
/* FUNCTION: get_next_state_from_fully_on                                  */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Movement to STANDBY or POWER_OFF requires TRANSITION behavior;        */
/*   Otherwise just jump to next_state                                     */
/***************************************************************************/
static int get_next_state_from_fully_on(void)
{
    switch (next_state) {
    case STANDBY__ENTRY:
    case POWER_OFF__ENTRY:
        return TRANS__ENTRY;
    default:
        return next_state;
    }
}

/***************************************************************************/
/* FUNCTION: wdc_leds_behavior_init                                        */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Initialization for LED behavior main state machine                    */
/***************************************************************************/
void wdc_leds_behavior_init(void)
{
    /* State machine variables */
    state = FULLY_ON__ENTRY;
    next_state = FULLY_ON__ENTRY;
    /* Outer ring variables */
    active_count = 0;
    count = 0;
    ignore_activity = 0;
    active_tail = NUM_ACTIVITY_LEDS - 1;
    ramp1 = 0;
    ramp2 = 0;
    activity_led[0] = 0;
    activity_led[1] = 0;
    activity_led[2] = 0;
    activity_led[3] = 0;
    /* Inner ring variables */
    inner_ring_rotate = 0;
    rebuild_percentage = 0;
    fuel_gauge_bits = 0;
    need_to_display_current_fuel_gauge = 1;
}

/***************************************************************************/
/* FUNCTION: wdc_leds_behavior                                             */
/*                                                                         */
/* PURPOSE:                                                                */
/*   LED behavior main state machine                                       */
/***************************************************************************/
void wdc_leds_behavior(unsigned long unused)
{
    static u8 been_there_done_that = 0;
    s8 j, k;
    switch (state) {
    case FULLY_ON__ENTRY:
    case FULLY_ON__RE_ENTRY:
        inner_ring_rotate = 0;
        need_to_display_current_fuel_gauge = 1;
        activity_led[0] = LED100;
        activity_led[1] = LED100;
        activity_led[2] = LED100;
        activity_led[3] = LED100;
        ramp1 = -STEP_RESOLUTION;
        ramp2 = -STEP_RESOLUTION;
        if (state == FULLY_ON__RE_ENTRY) {
            /* Don't go through the POR hold period for a re-entry */
            count = 0;
            state = FULLY_ON__RAMP_UP;
            break;
        }
        count = 0;
        state = FULLY_ON__POR_RAMP_UP;
        /* Fall through */
    case FULLY_ON__POR_RAMP_UP:
        if (--count > 0)
            break;
        count = POWER_SPEED;
        ++ramp2;
        if (++ramp1 < 0)
            break;
        count = HOLD_ON_ENTER;
        state = FULLY_ON__POR_HOLD;
        break;
    case FULLY_ON__POR_HOLD:
        display_current_fuel_gauge();
        if (--count > 0)
            break;
        active_count = 0;
        count = 0;
        state = FULLY_ON__RAMP_UP;
        break;
    case FULLY_ON__RAMP_UP:
        if (next_state != FULLY_ON__ENTRY) {
            state = get_next_state_from_fully_on();
            break;
        }

        display_current_fuel_gauge();
        if (--count > 0)
            break;
        count = POWER_SPEED;
        ++ramp2;
        if (++ramp1 >= 0) {
            ramp1 = 0;
            ramp2 = 0;
        }
        if (active_count == 0) {
            activity_led[0] = LED100;
            activity_led[1] = LED100;
            activity_led[2] = LED100;
            activity_led[3] = LED100;
            break;
        }

        activity_led[0] = 0;
        activity_led[1] = 0;
        activity_led[2] = 0;
        activity_led[3] = 0;
        count = 0;
        ramp1 = 0;
        ramp2 = 0;
        state = FULLY_ON__ACTIVITY;
        /* Fall through */
    case FULLY_ON__ACTIVITY:
        if (next_state != FULLY_ON__ENTRY) {
            state = get_next_state_from_fully_on();
            break;
        }

        if (--count > 0)
            break;
        count = RW_SPEED;
        handle_inner_ring();
        if (active_count) {
            j = active_tail =
                ((active_tail >
                  0) ? (active_tail - 1) : (NUM_ACTIVITY_LEDS - 1)
                );
            k = NUM_ACTIVITY_LEDS;
            while (k--) {
                activity_led[j] = activity_pattern[k];
                j = ((j > 0) ? (j - 1) : (NUM_ACTIVITY_LEDS - 1));
            }
            ramp1 = 0;
            ramp2 = 0;
        }
        if (active_count == 0) {
            count = RW_SPEED;
            state = FULLY_ON__IDLE_HOLD;
        }
        active_count = 0;
        break;
    case FULLY_ON__IDLE_HOLD:
        if (--count > 0)
            break;
        count = 0;
        state = FULLY_ON__RAMP_DOWN;
        break;
    case FULLY_ON__RAMP_DOWN:
        if (--count > 0)
            break;
        count = POWER_SPEED;
        --ramp2;
        if ((--ramp1 <= -STEP_RESOLUTION) || active_count) {
            display_current_fuel_gauge();
            ramp1 = -STEP_RESOLUTION;
            ramp2 = -STEP_RESOLUTION;
            state = FULLY_ON__RAMP_UP;
        }
        break;
    case STANDBY__ENTRY:
        activity_led[0] = LED100;
        activity_led[1] = LED100;
        activity_led[2] = LED100;
        activity_led[3] = LED100;
        ramp1 = -STEP_RESOLUTION;
        ramp2 = -STEP_RESOLUTION;
        count = HOLD_BREATH;
        state = STANDBY__DARK;
        display_current_fuel_gauge();
        /* Fall through */
    case STANDBY__DARK:
        if (next_state != STANDBY__ENTRY) {
            state = next_state;
            break;
        }

        if (--count > 0)
            break;
        state = STANDBY__RAMP_UP;
        break;
    case STANDBY__RAMP_UP:
        ramp2++;
        ramp1++;
        if (ramp1 < SLEEP_HI)
            break;
        state = STANDBY__RAMP_DOWN;
        break;
    case STANDBY__RAMP_DOWN:
        ramp2--;
        if (ramp1-- > -STEP_RESOLUTION)
            break;
        state = STANDBY__ENTRY;
        break;
    case POWER_OFF__ENTRY:
        display_inner_ring(0x00);
        activity_led[0] = 0;
        activity_led[1] = 0;
        activity_led[2] = 0;
        activity_led[3] = 0;
        ramp1 = 0;
        ramp2 = 0;
        state = POWER_OFF__DARK;
        /* Fall through */
    case POWER_OFF__DARK:
        if (next_state != POWER_OFF__ENTRY) {
            state = next_state;
            break;
        }
        break;
    case DEGRADED__ENTRY:
        display_inner_ring(get_inner_ring_bits(BIT_MASK_FUEL_GAUGE));
        ramp1 = 0;
        ramp2 = 0;
        activity_led[0] = 0;
        activity_led[1] = 0;
        activity_led[2] = 0;
        activity_led[3] = 0;
        count = BLINK_SPEED;
        state = DEGRADED__BLINK1;
        /* Fall through */
    case DEGRADED__BLINK1:
        if (next_state != DEGRADED__ENTRY) {
            display_inner_ring(0);
            state = TRANS__ENTRY;
            break;
        }

        if (--count > 0)
            break;
        display_inner_ring(0);
        activity_led[0] = LED100;
        activity_led[1] = LED100;
        activity_led[2] = LED100;
        activity_led[3] = LED100;
        count = BLINK_SPEED;
        state = DEGRADED__BLINK2;
        break;
    case DEGRADED__BLINK2:
        if (--count > 0)
            break;
        state = DEGRADED__ENTRY;
        break;
    case OVERTEMP__ENTRY:
        display_inner_ring(0);
        ramp1 = 0;
        ramp2 = 0;
        activity_led[0] = 0;
        activity_led[1] = 0;
        activity_led[2] = 0;
        activity_led[3] = 0;
        count = BLINK_SPEED;
        state = OVERTEMP__BLINK1;
        /* Fall through */
    case OVERTEMP__BLINK1:
        if (next_state != OVERTEMP__ENTRY) {
            display_inner_ring(0);
            state = TRANS__ENTRY;
            break;
        }

        if (--count > 0)
            break;
        display_inner_ring(get_inner_ring_bits(BIT_MASK_FUEL_GAUGE));
        activity_led[0] = LED100;
        activity_led[1] = LED100;
        activity_led[2] = LED100;
        activity_led[3] = LED100;
        count = BLINK_SPEED;
        state = OVERTEMP__BLINK2;
        break;
    case OVERTEMP__BLINK2:
        if (--count > 0)
            break;
        state = OVERTEMP__ENTRY;
        break;
    case TRANS__ENTRY:
        activity_led[0] = LED100;
        activity_led[1] = LED100;
        activity_led[2] = LED100;
        activity_led[3] = LED100;
        ramp1 = SLEEP_LO;
        ramp2 = SLEEP_HI;
        state = TRANS__1_UP;
        /* Fall through */
    case TRANS__1_UP:
    case TRANS__2_UP:
    case TRANS__3_UP:
    case TRANS__4_UP:
        ramp2 -= TRANS_STEP;
        ramp1 += TRANS_STEP;
        if ((ramp1 - TRANS_STEP) < SLEEP_HI)
            break;
        state++;
        break;
    case TRANS__4_DN:
        if (ramp1 <= -STEP_RESOLUTION) {
            if (next_state == TRANS__ENTRY) {
                /*
                 * If no one told us where to go then just go to FullyOn, but
                 * don't go through the POR hold period because that makes
                 * the next transition display for the next button press wait
                 * too long.
                 *
                 * Note, that next_state needs to be set to FULLY_ON__ENTRY
                 * because it is used for the test whether or not to leave
                 * the FullyOn state even though we are entering FullyOn
                 * through the re-entry path.
                 */
                next_state = FULLY_ON__ENTRY;
                state = FULLY_ON__RE_ENTRY;
            } else {
                state = next_state;
            }
            break;
        }
        ramp2 -= TRANS_STEP;
        /* Fall through */
    case TRANS__1_DN:
    case TRANS__2_DN:
    case TRANS__3_DN:
        ramp2 += TRANS_STEP;
        ramp1 -= TRANS_STEP;
        if ((ramp1 + TRANS_STEP) > -STEP_RESOLUTION)
            break;
        ramp1 = SLEEP_LO;
        ramp2 = SLEEP_HI;
        state++;
        break;
    default:
        if (!been_there_done_that) {
            printk(KERN_ERR "Invalid LED behavior state\n");
            been_there_done_that = 1;
            return;
        }
    }

    /* Set the activity brightness according to value and ramp */
    set_led(LED_MASK_ACT12, activity_led[0] + ramp1);
    set_led(LED_MASK_ACT9, activity_led[1] + ramp2);
    set_led(LED_MASK_ACT6, activity_led[2] + ramp1);
    set_led(LED_MASK_ACT3, activity_led[3] + ramp2);
}


/***************************************************************************/
/* FUNCTION: wdc_leds_set_power                                            */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set the "power" LED to the requested behavior                         */
/***************************************************************************/
static void wdc_leds_set_power
    (struct led_classdev *led_cdev, enum led_brightness value) {
    if (value >= 255) {
        next_state = FULLY_ON__ENTRY;
    } else if (value > 0) {
        next_state = STANDBY__ENTRY;
    } else {
        next_state = POWER_OFF__ENTRY;
    }
}


/***************************************************************************/
/* FUNCTION: wdc_leds_set_activity                                         */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Trigger activity display behavior                                     */
/***************************************************************************/
static void wdc_leds_set_activity
    (struct led_classdev *led_cdev, enum led_brightness value) {
    if (!ignore_activity && (value > 0)) {
        active_count++;
    }
}


/***************************************************************************/
/* FUNCTION: wdc_leds_set_ignore_activity                                  */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set the "ignore activity" setting                                     */
/***************************************************************************/
static void wdc_leds_set_ignore_activity
    (struct led_classdev *led_cdev, enum led_brightness value) {
    ignore_activity = value;
}


/***************************************************************************/
/* FUNCTION: wdc_leds_set_transition                                       */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Trigger "transition" display behavior                                 */
/***************************************************************************/
static void wdc_leds_set_transition
    (struct led_classdev *led_cdev, enum led_brightness value) {
    next_state = ((value > 0) ? TRANS__ENTRY : FULLY_ON__ENTRY);
}


/***************************************************************************/
/* FUNCTION: wdc_leds_set_fuel_gauge                                       */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set the fuel gauge to the requested value (treated as a percentage)   */
/***************************************************************************/
static void wdc_leds_set_fuel_gauge
    (struct led_classdev *led_cdev, enum led_brightness value) {
    fuel_gauge_bits = get_inner_ring_bits(get_percentage_pattern(value));
    need_to_display_current_fuel_gauge = 1;
}


/***************************************************************************/
/* FUNCTION: wdc_leds_set_fg_bitmap                                        */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set the fuel gauge to the requested value (treated as a bitmap)       */
/***************************************************************************/
static void wdc_leds_set_fg_bitmap
    (struct led_classdev *led_cdev, enum led_brightness value) {
    fuel_gauge_bits = get_inner_ring_bits((u16) value);
    need_to_display_current_fuel_gauge = 1;
}


/***************************************************************************/
/* FUNCTION: wdc_leds_set_rebuilding                                       */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set the rebuilding behavior (value = % complete, 0 = not rebuilding)  */
/***************************************************************************/
static void wdc_leds_set_rebuilding
    (struct led_classdev *led_cdev, enum led_brightness value) {
    rebuild_percentage = value;
}


/***************************************************************************/
/* FUNCTION: wdc_leds_set_degraded                                         */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set the degraded mode display behavior                                */
/***************************************************************************/
static void wdc_leds_set_degraded
    (struct led_classdev *led_cdev, enum led_brightness value) {
    next_state = ((value > 0) ? DEGRADED__ENTRY : FULLY_ON__ENTRY);
}


/***************************************************************************/
/* FUNCTION: wdc_leds_set_over_temp                                        */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set the over temperature mode display behavior                        */
/***************************************************************************/
static void wdc_leds_set_over_temp
    (struct led_classdev *led_cdev, enum led_brightness value) {
    next_state = ((value > 0) ? OVERTEMP__ENTRY : FULLY_ON__ENTRY);
}


/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_power                                          */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the wdc-leds "power" pseudo-LED                              */
/***************************************************************************/
static struct led_classdev wdc_leds_power = {
    .name = "wdc-leds:power",.brightness_set = wdc_leds_set_power,
};

/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_activity                                       */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the wdc-leds "activity" pseudo-LED                           */
/***************************************************************************/
static struct led_classdev wdc_leds_activity = {
    .name = "wdc-leds:activity",.brightness_set =
        wdc_leds_set_activity,.default_trigger = "sata-disk"
};

/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_ignore_activity                                */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the wdc-leds "ignore-activity" pseudo-LED                    */
/***************************************************************************/
static struct led_classdev wdc_leds_ignore_activity = {
    .name = "wdc-leds:ignore-act",.brightness_set =
        wdc_leds_set_ignore_activity,
};

/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_transition                                     */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the wdc-leds "transition" pseudo-LED                         */
/***************************************************************************/
static struct led_classdev wdc_leds_transition = {
    .name = "wdc-leds:transition",.brightness_set =
        wdc_leds_set_transition,
};

/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_fuel_gauge                                     */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the wdc-leds "fuel-gauge" LEDs (brightness = % full)         */
/***************************************************************************/
static struct led_classdev wdc_leds_fuel_gauge = {
    .name = "wdc-leds:fuel-gauge",.brightness_set =
        wdc_leds_set_fuel_gauge,
};

/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_fg_bitmap                                      */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the wdc-leds "fuel-gauge" LEDs (brightness = bitmap)         */
/***************************************************************************/
static struct led_classdev wdc_leds_fg_bitmap = {
    .name = "wdc-leds:fg-bitmap",.brightness_set = wdc_leds_set_fg_bitmap,
};

/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_rebuilding                                     */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the wdc-leds RAID1 "rebuilding" mode pseudo-LED              */
/*   (brightness = % complete)                                             */
/***************************************************************************/
static struct led_classdev wdc_leds_rebuilding = {
    .name = "wdc-leds:rebuilding",.brightness_set =
        wdc_leds_set_rebuilding,
};

/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_degraded                                       */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the wdc-leds "degraded" mode pseudo-LED                      */
/***************************************************************************/
static struct led_classdev wdc_leds_degraded = {
    .name = "wdc-leds:degraded",.brightness_set = wdc_leds_set_degraded,
};

/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_over_temp                                      */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the wdc-leds "over-temp" pseudo-LED                          */
/***************************************************************************/
static struct led_classdev wdc_leds_over_temp = {
    .name = "wdc-leds:over-temp",.brightness_set = wdc_leds_set_over_temp,
};

/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_classes[]                                      */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Array of LED classes to create/destroy                                */
/***************************************************************************/
static struct led_classdev *wdc_led_classes[] = {
    &wdc_leds_power,
    &wdc_leds_activity,
    &wdc_leds_ignore_activity,
    &wdc_leds_transition,
    &wdc_leds_fuel_gauge,
    &wdc_leds_fg_bitmap,
    &wdc_leds_rebuilding, 
    &wdc_leds_degraded, 
    &wdc_leds_over_temp
};

#ifdef DEBUG
static ssize_t show_registers (struct device *dev, struct device_attribute *attr, char *buf)
{
    char * out = buf;
    u32 clock_data = readl(PWM_CLOCK_REGISTER);
    u32 data_ptr = PWM_CLOCK_REGISTER;
    u8 no_pwms = (clock_data >> 16);
    u8 i;
    /* report hardware status here */
    out += sprintf(buf,"PWM drive registers\n");
    out += sprintf(out, "clock register:0x%08x @ 0x%08x\n", clock_data, data_ptr); 
    
    for (i = 0; i < no_pwms; ++i)
    {
        data_ptr=(u32)((u32 *)PWM_BASE+i);
        out+= sprintf(out,"%d:%d @ 0x%08x\n", i, (u8)readl(data_ptr),data_ptr);
    }
    
    return out - buf;
}

/* create a register 'file' to enbale reading back the pwm drive register status */
static DEVICE_ATTR (registers, S_IRUGO, show_registers, NULL);

static int create_debug_files(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    return device_create_file(dev, &dev_attr_registers);
}

static void remove_debug_files(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    device_remove_file(dev, &dev_attr_registers);
}
#endif


#ifdef CONFIG_PM
/***************************************************************************/
/* FUNCTION: wdc_leds_suspend                                              */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Suspend all LED class devices created by this driver                  */
/***************************************************************************/
static int wdc_leds_suspend(struct platform_device *pdev,
                            pm_message_t state)
{
    int n = leds_created;
    while (n > 0) {
        if (--n < ARRAY_SIZE(wdc_led_classes)) {
            led_classdev_suspend(wdc_led_classes[n]);
        }
    }

return 0}

/***************************************************************************/
/* FUNCTION: wdc_leds_resume                                               */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Wake up all LED class devices created by this driver                  */
/***************************************************************************/
static int wdc_leds_resume(struct platform_device *pdev,
                           pm_message_t state)
{
    int n = leds_created;
    while (n > 0) {
        if (--n < ARRAY_SIZE(wdc_led_classes)) {
            led_classdev_resume(wdc_led_classes[n]);
        }
    }

return 0}
#endif                          /* CONFIG_PM */

/***************************************************************************/
/* FUNCTION: wdc_leds_probe                                                */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform any necessary probing and initial setup for wdc-leds device   */
/***************************************************************************/
static int wdc_leds_probe(struct platform_device *pdev)
{
    int rc;
    int timer_changed = 0;
    int interrupt_allocated = 0;
    leds_created = 0;
    do {
        /* Enable LED output drivers and disable other uses */
        CLEAR(SYS_CTRL_GPIO_PRIMSEL_CTRL_0, LED_MASK_GPIO_A);
        CLEAR(SYS_CTRL_GPIO_SECSEL_CTRL_0, LED_MASK_GPIO_A);
        CLEAR(SYS_CTRL_GPIO_TERTSEL_CTRL_0, LED_MASK_GPIO_A);
        
        CLEAR(SYS_CTRL_GPIO_PRIMSEL_CTRL_1, LED_MASK_GPIO_B);
        CLEAR(SYS_CTRL_GPIO_SECSEL_CTRL_1, LED_MASK_GPIO_B);
        CLEAR(SYS_CTRL_GPIO_TERTSEL_CTRL_1, LED_MASK_GPIO_B);
        /* Turn off all the LEDs */
        if (negative_led_logic) {
            writel(LED_MASK_GPIO_A, GPIO_A_OUTPUT_SET);
            writel(LED_MASK_GPIO_B, GPIO_B_OUTPUT_SET);
        } else {
            writel(LED_MASK_GPIO_A, GPIO_A_OUTPUT_CLEAR);
            writel(LED_MASK_GPIO_B, GPIO_B_OUTPUT_CLEAR);
        }

        /* bring PWM module out of reset and enable clock */
        writel((1<<SYS_CTRL_RSTEN_MISC_BIT), SYS_CTRL_RSTEN_CLR_CTRL);
        //writel(PWM_CLOCK, SYS_CTRL_CKEN_SET_CTRL);

        /* enable PWM clock */
        writel(PWM_PRESCALE, PWM_CLOCK_REGISTER);

            /* Initialize frame buffer to everything off */
            set_led(LED_MASK_FUEL_GAUGE | LED_MASK_ACTIVITY, 0);
        /* Enable output to the LEDs */
        writel(LED_MASK_GPIO_A, SYS_CTRL_GPIO_PWMSEL_CTRL_0);
        writel(LED_MASK_GPIO_B, SYS_CTRL_GPIO_PWMSEL_CTRL_1);
        /* Initialize the LED behavior state machine */
        wdc_leds_behavior_init();
        /* Save Timer2 state for restoring later */
        timer_load = ox_readl(TIMER2_LOAD);
        timer_control = ox_readl(TIMER2_CONTROL);
        ox_writel(0, TIMER2_CONTROL);
        timer_changed = 1;
        /* Setup Timer2 for LED control */
        rc = request_irq
            (TIMER_2_INTERRUPT,
             wdc_leds_interrupt, 0, "led_pwm", 0);
        if (rc < 0) {
            printk(KERN_ERR "failed to get IRQ\n");
            break;
        }

        interrupt_allocated = 1;
        ox_writel(FAST_TIMER_INT, TIMER2_LOAD);
        ox_writel(PERIODIC_INTERRUPT, TIMER2_CONTROL);
        /* Register each LED class device */
        while (leds_created < ARRAY_SIZE(wdc_led_classes)) {
            rc = led_classdev_register(&pdev->dev,
                                       wdc_led_classes[leds_created]);
            if (rc < 0) {
                printk(KERN_ERR "failed to register led class \"%s\"\n",
                       wdc_led_classes[leds_created]->name);
                break;
            }

            ++leds_created;
        }
    }
    while (0);
    /* If we failed then perform any needed clean up */
    if (rc < 0) {
        /* Unregister any classes we registered */
        while (leds_created > 0) {
            if (--leds_created < ARRAY_SIZE(wdc_led_classes)) {
                led_classdev_unregister(wdc_led_classes[leds_created]);
            }
        }

        /* Free the interrupt if we allocated one */
        if (interrupt_allocated) {
            free_irq(TIMER_2_INTERRUPT, 0);
        }

        /* Restore Timer2 if we changed it */
        if (timer_changed) {
            ox_writel(timer_load, TIMER2_LOAD);
            ox_writel(timer_control, TIMER2_CONTROL);
        }
    }
#ifdef DEBUG
    create_debug_files(pdev);
#endif    
    return rc;
}


/***************************************************************************/
/* FUNCTION: wdc_leds_remove                                               */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform steps to remove the wdc-leds device                           */
/***************************************************************************/
static int wdc_leds_remove(struct platform_device *pdev)
{
    while (leds_created > 0) {
        if (--leds_created < ARRAY_SIZE(wdc_led_classes)) {
            led_classdev_unregister(wdc_led_classes[leds_created]);
        }
    }

    ox_writel(0, TIMER2_CONTROL);
    free_irq(TIMER_2_INTERRUPT, 0);
    ox_writel(timer_load, TIMER2_LOAD);
    ox_writel(timer_control, TIMER2_CONTROL);
    /* Turn off all the LEDs */
    if (negative_led_logic) {
        writel(LED_MASK_GPIO_A, GPIO_A_OUTPUT_SET);
        writel(LED_MASK_GPIO_B, GPIO_B_OUTPUT_SET);
    } else {
        writel(LED_MASK_GPIO_A, GPIO_A_OUTPUT_CLEAR);
        writel(LED_MASK_GPIO_B, GPIO_B_OUTPUT_CLEAR);
    }
    /* put PWM module back into  reset and disable clock */
    writel((1<<SYS_CTRL_RSTEN_MISC_BIT), SYS_CTRL_RSTEN_SET_CTRL);
    // writel(PWM_CLOCK, SYS_CTRL_CKEN_CLR_CTRL);
#ifdef DEBUG
  remove_debug_files(pdev);
#endif
    return 0;
}


/***************************************************************************/
/* DATA STRUCTURE: wdc_leds_driver                                         */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the wdc-leds platform device driver                          */
/***************************************************************************/
static struct platform_driver wdc_leds_driver = {
    .probe = wdc_leds_probe,.remove = wdc_leds_remove,
#ifdef CONFIG_PM
    .suspend = wdc_leds_suspend,.resume = wdc_leds_resume,
#endif                          /* CONFIG_PM */
    .driver = {
               .name = "wdc-leds",},
};

/* Pointer to device returned by platform_device_register_simple */
static struct platform_device *wdc_leds;
/***************************************************************************/
/* FUNCTION: wdc_leds_init                                                 */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform module initialization                                         */
/***************************************************************************/
static int __init wdc_leds_init(void)
{
    int ret;
    printk
        (KERN_INFO "wdc-leds:  SLOW_TPS=%d\n",
         SLOW_TPS);
    ret = platform_driver_register(&wdc_leds_driver);
    if (!ret) {
        wdc_leds =
            platform_device_register_simple("wdc-leds", -1, NULL, 0);
    }

    return ret;
}


/***************************************************************************/
/* FUNCTION: wdc_leds_exit                                                 */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform module unloading and cleanup                                  */
/***************************************************************************/
static void __exit wdc_leds_exit(void)
{
    if (wdc_leds) {
        platform_device_unregister(wdc_leds);
    }
    platform_driver_unregister(&wdc_leds_driver);
}


module_init(wdc_leds_init);
module_exit(wdc_leds_exit);
MODULE_AUTHOR("Michael Webster");
MODULE_DESCRIPTION("WDC 2NC LEDs");
MODULE_LICENSE("GPL");
/******************************* End of File *********************************/
