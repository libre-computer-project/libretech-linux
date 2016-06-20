/*
 * linux/arch/arm/mach-oxnas/oxnas-wd810-leds.c
 *
 * Copyright (c) 2008 Oxford Semiconductor Ltd.
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
#include <asm/io.h>


/* Timer Values and Pulse Width Modulation */
#define PWM_RESOLUTION  255
#define TIMER_LED_MODE  TIMER_MODE_PERIODIC

#define LED100  (PWM_RESOLUTION)	/* 100% duty cycle */
#define LED50   (PWM_RESOLUTION / 2)	/* 50%  duty cycle */
#define LED25   (PWM_RESOLUTION / 4)	/* 25%  duty cycle */

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
 * With a system clock of 25Mhz and a load register value of (1627) prescaled 256 
 * to achieve 2Hz:
 *   25Mhz / 256 / 24414 =~4Hz          //1627 = ~60
 *
 * PWM clock = 183MHz/256/814=~877kHz
 */
#define FAST_TIMER_INT      24414	//48828//(1627)      /* Timer2 count down      */
#define SYS_CLOCK       CONFIG_NOMINAL_RPSCLK_FREQ	/* System clock frequency */
#define PRESCALE_VALUE        (256)	/* Value set in prescaler */
/* Do not reset the below value - check fan tacho modules for more details */
#define PWM_PRESCALE       28	/* Value loaded on PWM clock register > 20  kHz*/
#define MAX_PWM   255
#define SLOW_TPS   ((SYS_CLOCK/PRESCALE_VALUE) / FAST_TIMER_INT)	//=~4Hz, 250ms


																																												/**** if GPIO31~GPIO16 is used, sift left 16 bits ****//* GPIO bits dedicated to LEDs *///Need to modify if different GPIO used
#define LED_MASK_CG5   (1 << GPIO_34)	/* Capacity Gauge TOP LED  */
#define LED_MASK_CG4   (1 << GPIO_7)	/* Capacity Gauge 4th LED  */
#define LED_MASK_CG3   (1 << GPIO_6)	/* Capacity Gauge 3rd LED  */
#define LED_MASK_CG2   (1 << GPIO_5)	/* Capacity Gauge 2nd LED  */
#define LED_MASK_CG1   ((1 << GPIO_26)>>16)	//sift left 16 bits for PWM10 /* Capacity Gauge 1st LED  */
#define LED_MASK_CG0   ((1 << GPIO_25)>>16)	//sift left 16 bits for PWM9  /* Capacity Gauge BOTTOM LED */

/* Mask for all the LEDs in the Fuel Gauge.*/
/* Mask for all the LEDs on GPIO_A */

#define LED_MASK_GPIO_A \
   (                    \
      (LED_MASK_CG0<<16)   |  \
      (LED_MASK_CG1<<16)   |  \
      LED_MASK_CG2   |   \
      LED_MASK_CG3   |   \
      LED_MASK_CG4   \
   )

/* Mask for all the LEDs on GPIO_B */
#define LED_MASK_GPIO_B \
   (                    \
      LED_MASK_CG5  \
   )

#define LED_MASK_GAUGE \
   (                        \
      LED_MASK_CG0 |       \
      LED_MASK_CG1 |       \
      LED_MASK_CG2  |       \
      LED_MASK_CG3  |       \
      LED_MASK_CG4  |       \
      LED_MASK_CG5          \
   )

#define LED_MASK_GAUGE_ODD \
   (                        \
      LED_MASK_CG0 |       \
      LED_MASK_CG2 |       \
      LED_MASK_CG4        \
   )

#define LED_MASK_GAUGE_EVEN \
   (                        \
      LED_MASK_CG1  |       \
      LED_MASK_CG3  |       \
      LED_MASK_CG5          \
   )

#define LED_MASK_GAUGE_CENTER \
   (                        \
      LED_MASK_CG2 |       \
      LED_MASK_CG3         \
   )

#define LED_MASK_GAUGE_MID \
   (                        \
      LED_MASK_CG1  |       \
      LED_MASK_CG4         \
   )

#define LED_MASK_GAUGE_OUTER \
   (                        \
      LED_MASK_CG0  |       \
      LED_MASK_CG5         \
   )

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
 * States for the main LED behavior state machine.  
 */
enum {
	STATE_NOP,
	STANDBY,
	SHOW_CAPACITY,
	ACTIVITY,
	POWER_OFF,
	RESET,
	ATTENTION,
	FAILURE,
	BOOT_OK,
	BOOT_stage1,
	BOOT_stage2,
	BOOT_stage3,
	BOOT_stage4,
	BOOT_stage5,
	BOOT_stage6,
	LEDS_OFF,
	BOOT_init,
};

/*Various LED state timing*/

/* the below are the ramp divider values used to ramp up and down the leds */
/* These values are directly proportional to the PWM frequency used */
#define BOOTOK_RAMP_DIV	330
#define STANBY_RAMP_DIV	270
#define POWEROFF_RAMP_DIV	180
#define FAILURE_RAMP_DIV	180
#define ACTIVITY_RAMP_DIV	30	// 2

#define STANBY_ALT_STEP	(SLOW_TPS*4)	//16//8//250      //~4sec
#define POWEROFF_ALT_STEP	((SLOW_TPS/2)*7)	//14//7//200     //~3.5sec
#define ATTENTION_ALT_STEP	(SLOW_TPS/2)	// 2                 //~2Hz, 0.5sec
#define FAILURE_ALT_STEP	(SLOW_TPS/2)	// 2              //~2Hz, 0.5sec
#define ACTIVITY_ALT_STEP	(SLOW_TPS/4)	// 2             //~2Hz, 0.5sec
#define RESET_ALT_STEP		(SLOW_TPS/2)	// 2             //~2Hz, 0.5sec


/* Variables for main LED behavior state machine */
static int state;
static u8 start;
static u8 act;
static u8 mark;
static u16 act_led[6] = {
	LED_MASK_CG0,
	LED_MASK_CG1,
	LED_MASK_CG2,
	LED_MASK_CG3,
	LED_MASK_CG4,
	LED_MASK_CG5,
};
static u32 alt = 0;
static int count;
static u16 capacity_gauge_bits;	/* see LED frame buffer design assumption */
static u8 leds_switch;
static u8 activity_block = 1;


/*
 * Declare tasklet for the LED behavior state machine.
 */
void oxnas_wd810_leds_behavior(unsigned long);
DECLARE_TASKLET(oxnas_wd810_leds_behavior_tasklet,
				oxnas_wd810_leds_behavior, 0);


/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_interrupt                                            */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Interrupt handler for the oxnas-wd810-leds pulse width modulation             */
/***************************************************************************/
static irqreturn_t oxnas_wd810_leds_interrupt(int irq, void *dev_id)
{
	writel(0, TIMER2_CLEAR);

	tasklet_schedule(&oxnas_wd810_leds_behavior_tasklet);

	return IRQ_HANDLED;
}


/***************************************************************************/
/* FUNCTION: get_vbar_bits                                           */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Convert the bit map of V-bar LEDs into the GPIO bit map          */
/***************************************************************************/
static u16 get_vbar_bits(u16 value)
{
	u16 pattern = 0;

	// Convert the bit map to the GPIO bit pattern
	if (value & (1 << 0))
		pattern |= LED_MASK_CG0;
	if (value & (1 << 1))
		pattern |= LED_MASK_CG1;
	if (value & (1 << 2))
		pattern |= LED_MASK_CG2;
	if (value & (1 << 3))
		pattern |= LED_MASK_CG3;
	if (value & (1 << 4))
		pattern |= LED_MASK_CG4;
	if (value & (1 << 5))
		pattern |= LED_MASK_CG5;

	return pattern;
}


/***************************************************************************/
/* FUNCTION: get_percentage_pattern                                        */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Convert a percentage to a V-bar bit map.  Note, we never display */
/*   less than 1 LED for a percentage so that something is alway visible.  */
/***************************************************************************/
static u16 get_percentage_pattern(u16 percentage)
{
	if (percentage >= 50) {
		if (percentage >= 67) {
			if (percentage >= 83) {
				if (percentage >= 97) {
					return 0x3F;	// 6 LEDs is >= 97%
				} else {
					return 0x1F;	// 5 LEDs is >=  83%
				}
			} else {
				return 0x0F;	// 4 LEDs is >=  67%
			}
		} else {
			return 0x07;		// 3 LEDs is >=  50%
		}
	} else {
		if (percentage >= 33) {
			return 0x03;		// 2 LEDs is >=  33%
		} else {
			return 0x01;		// 1 LED  is >=   0%
		}
	}
}


/***************************************************************************/
/* FUNCTION: set_led                                                       */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set requested brightness on the requested LED(s) */
/***************************************************************************/
static void set_led(u16 led_bits, u8 value, u16 ramp, u16 ramp_div)
{
	u16 bit;
	s8 count = 0;
	u32 reg;
//printk("<1> set led\n");
	if (negative_led_logic) {
		value = MAX_PWM - value;
		if (ramp & 0x100) {
			ramp = ((MAX_PWM - (ramp & 0xFF)) | 0x100);
		} else
			ramp = ((MAX_PWM - (ramp & 0xFF)));
	}

	reg = ((ramp << 16) | value);
//printk(KERN_INFO "set_led:  reg=%x\n", reg);  
	if (ramp & 0x100) {
		writel(ramp_div, (PWM_DATA_REGISTER_BASE + 0x404));
	}
	for (bit = 1; bit > 0; bit <<= 1, ++count) {
		if (bit & led_bits) {
			writel(reg, ((u32 *) PWM_DATA_REGISTER_BASE + count));
		}
	}
}


/***************************************************************************/
/* FUNCTION: display_vbar                                            */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set bits for the requested V-bar map                 */
/***************************************************************************/
static void display_vbar(u16 vbar_bits)
{
	//printk(KERN_INFO "display_vbar:  vbar_bits=%x\n", vbar_bits);

	//set_led(~vbar_bits & LED_MASK_GAUGE, 0, 0, 0);
	set_led(LED_MASK_GAUGE, 0, 0, 0);
	set_led(vbar_bits, LED100, 0, 0);
}


/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_behavior_init                                        */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Initialization for LED behavior main state machine                    */
/***************************************************************************/
void oxnas_wd810_leds_behavior_init(void)
{
	/* State machine variables */
	state = BOOT_init;
	count = 0;
	capacity_gauge_bits = 0;
	leds_switch = 0xFF;
}



/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_behavior                                             */
/*                                                                         */
/* PURPOSE:                                                                */
/*   LED behavior main state machine                                       */
/***************************************************************************/
void oxnas_wd810_leds_behavior(unsigned long unused)
{
//	printk(KERN_INFO "oxnas_wd810_leds_behavior state=%d count=%d\n", state, count);
	switch (state) {
	case STANDBY:
		//All LEDs dim-up and dim-down every 4sec.
		if (leds_switch & (1 << 0)) {	//LEDS display turn on
			if (count-- != 0)
				break;
			alt++;
			if ((alt % 2) == 1) {
				set_led(LED_MASK_GAUGE, 0, 0x1FF, STANBY_RAMP_DIV);
			} else {
				set_led(LED_MASK_GAUGE, 255, 0x100, STANBY_RAMP_DIV);
			}
			count = STANBY_ALT_STEP;
		} else {
			printk(KERN_INFO " state STANDBY LED display off \n");
			state = STATE_NOP;
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}
		break;

	case SHOW_CAPACITY:
		//Each LED represents 1/6 of the total available capacity.
		start = 0;
		mark = 0;
		act = 0;
		if (leds_switch & (1 << 1)) {	//LEDS display turn on
			display_vbar(capacity_gauge_bits);
		} else {
			printk(KERN_INFO " state SHOW_CAPACITY LED display off \n");
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}
		state = STATE_NOP;
		break;


	case ACTIVITY:
		//LEDs illuminate in a up and down "cylon" motion.      
		if (leds_switch & (1 << 2)) {	//LEDS display turn on
			if (count-- != 0)
				break;

			if (start == 0) {
				set_led(LED_MASK_GAUGE, 0, 0x000, 0);
				set_led(act_led[act], 0, 0x1FF, ACTIVITY_RAMP_DIV);
				start = 1;
			} else {
				if ((mark == 0) && (++act < 6)) {
					set_led(act_led[act], 0, 0x1FF, ACTIVITY_RAMP_DIV);
					set_led(act_led[act - 1], 255, 0x100,
							ACTIVITY_RAMP_DIV);
					goto NEXT;
				}
				mark = 1;
				if ((mark == 1) && (act-- > 1)) {
					set_led(act_led[act], 255, 0x100, ACTIVITY_RAMP_DIV);
					set_led(act_led[act - 1], 0, 0x1FF, ACTIVITY_RAMP_DIV);
					goto NEXT;
				}
				mark = 0;
				state = SHOW_CAPACITY;
			}
		  NEXT:
			count = ACTIVITY_ALT_STEP;
		} else {
			printk(KERN_INFO " state ACTIVITY LED display off \n");
			state = STATE_NOP;
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}
		break;

	case POWER_OFF:
		//LEDs in the array dim-up/dim-down in an odd/even alternating pattern.
		if (leds_switch & (1 << 3)) {	//LEDS display turn on
			if (alt == 0){
				set_led(LED_MASK_GAUGE, 0, 0, 0);
				set_led(LED_MASK_GAUGE_ODD, 0, 0x1FF, POWEROFF_RAMP_DIV);
				alt++;
				count = POWEROFF_ALT_STEP;
				break;
			}
			if (count-- != 0)
				break;
			alt++;
			if ((alt % 2) == 1) {
				set_led(LED_MASK_GAUGE_ODD, 0, 0x1FF, POWEROFF_RAMP_DIV);
				set_led(LED_MASK_GAUGE_EVEN, 255, 0x100, POWEROFF_RAMP_DIV);
			} else {
				set_led(LED_MASK_GAUGE_ODD, 255, 0x100, POWEROFF_RAMP_DIV);
				set_led(LED_MASK_GAUGE_EVEN, 0, 0x1FF, POWEROFF_RAMP_DIV);
			}
			count = POWEROFF_ALT_STEP;
		} else {
			printk(KERN_INFO " state POWER_OFF LED display off \n");
			state = STATE_NOP;
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}

		break;

	case RESET:
		//Alternately blink the upper LED and lower LED at 1/2 sec rate.        
		if (leds_switch & (1 << 4)) {	//LEDS display turn on	
			if (alt == 0 )
				set_led(LED_MASK_GAUGE, 0, 0x000, 0);
			if (count-- != 0)
				break;
			alt++;
			if ((alt % 2) == 1) {
				set_led(act_led[0], 255, 0x000, 0);
				set_led(act_led[5], 0, 0x000, 0);
			} else {
				set_led(act_led[0], 0, 0x000, 0);
				set_led(act_led[5], 255, 0x000, 0);
			}
			count = RESET_ALT_STEP;
		} else {
			printk(KERN_INFO " state RESET LED display off \n");
			state = STATE_NOP;
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}

		break;

	case ATTENTION:
		//All LEDs flash simultaneously at a 1/2 sec. rate      
		if (count-- != 0)
			break;
		alt++;
		if ((alt % 2) == 1) {
			set_led(LED_MASK_GAUGE, 255, 0x000, 0);

		} else {
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}
		count = ATTENTION_ALT_STEP;

		break;

	case FAILURE:
		//LEDs illuminate in a continous "center out" sweep pattern.
		if (alt == 0 )
                	set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		
		if (count-- != 0)
			break;
		
		switch ((alt % 4)) {
		case 0:
			set_led(LED_MASK_GAUGE_CENTER, 0, 0x1FF, FAILURE_RAMP_DIV);
			break;
		case 1:
			set_led(LED_MASK_GAUGE_MID, 0, 0x1FF, FAILURE_RAMP_DIV);
			break;
		case 2:
			set_led(LED_MASK_GAUGE_OUTER, 0, 0x1FF, FAILURE_RAMP_DIV);
			break;
		case 3:
			set_led(LED_MASK_GAUGE, 0, 0, 0);
			break;
		}
		alt++;
		count = FAILURE_ALT_STEP;

		break;

	case BOOT_OK:
		//All 6 LEDs ramp up smoothly to full intensity.
		if (leds_switch & (1 << 5)) {	//LEDS display turn on
			set_led(LED_MASK_GAUGE, 0, 0x1FF, BOOTOK_RAMP_DIV);
		} else {
			printk(KERN_INFO " state BOOT_OK LED display off \n");
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}
		state = STATE_NOP;
		break;

	case BOOT_stage1:
		//Bottom LED ramp up smoothly to full intensity.
		if (leds_switch & (1 << 6)) {	//LEDS display turn on
		//	set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		//	set_led(LED_MASK_CG0, 0, 0x1FF, BOOTOK_RAMP_DIV);
			set_led(LED_MASK_CG0, 255, 0x000, 0);
		} else {
			printk(KERN_INFO " state BOOT_stage1~3 LED display off \n");
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}
		state = STATE_NOP;
		break;

	case BOOT_stage2:
		// 2nd LED ramp up smoothly to full intensity.
		if (leds_switch & (1 << 6)) {	//LEDS display turn on
			set_led(LED_MASK_CG1, 0, 0x1FF, BOOTOK_RAMP_DIV);
		} else {
			printk(KERN_INFO " state BOOT_stage1~3 LED display off \n");
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}
		state = STATE_NOP;
		break;

	case BOOT_stage3:
		// 3rd LED ramp up smoothly to full intensity.
		if (leds_switch & (1 << 6)) {	//LEDS display turn on
			set_led(LED_MASK_CG2, 0, 0x1FF, BOOTOK_RAMP_DIV);
		} else {
			printk(KERN_INFO " state BOOT_stage1~3 LED display off \n");
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}
		state = STATE_NOP;
		break;

	case BOOT_stage4:
		// 4th LED ramp up smoothly to full intensity.
		if (leds_switch & (1 << 7)) {	//LEDS display turn on      
			set_led(LED_MASK_CG3, 0, 0x1FF, BOOTOK_RAMP_DIV);
		} else {
			printk(KERN_INFO " state BOOT_stage4~6 LED display off \n");
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}
		state = STATE_NOP;
		break;

	case BOOT_stage5:
		//5th LED ramp up smoothly to full intensity.
		if (leds_switch & (1 << 7)) {	//LEDS display turn on  
			set_led(LED_MASK_CG4, 0, 0x1FF, BOOTOK_RAMP_DIV);
		} else {
			printk(KERN_INFO " state BOOT_stage4~6 LED display off \n");
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}
		state = STATE_NOP;
		break;

	case BOOT_stage6:
		//Top LED ramp up smoothly to full intensity.
		if (leds_switch & (1 << 7)) {	//LEDS display turn on          
			set_led(LED_MASK_CG5, 0, 0x1FF, BOOTOK_RAMP_DIV);
		} else {
			printk(KERN_INFO " state BOOT_stage4~6 LED display off \n");
			set_led(LED_MASK_GAUGE, 0, 0x000, 0);
		}
		state = STATE_NOP;
		break;

	case LEDS_OFF:
		//Turns all leds off
		set_led(LED_MASK_GAUGE, 0, 0x000, 0);

	case BOOT_init:
	case STATE_NOP:
	default:
		return;
	}
}


/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_set_switch                                            */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set the LED display "on/off" of each state by Web GUI                      */
/***************************************************************************/
static void oxnas_wd810_leds_set_switch
	(struct led_classdev *led_cdev, enum led_brightness value) {
	leds_switch = value;
}

/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_set_state                                            */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set the "state" LED to the requested behavior                         */
/***************************************************************************/
static void oxnas_wd810_leds_set_state
	(struct led_classdev *led_cdev, enum led_brightness value) {
	count = 0;
	alt = 0;
	state = value;
	start = 0;
	act = 0;
	mark = 0;
	printk(KERN_INFO "oxnas_wd810_leds_state state=%d\n", state);
//	if (state > 0)
//		set_led(LED_MASK_GAUGE, 0, 0x000, 0);

	if (state > 3)
		activity_block = 1;
	else
		activity_block = 0;
}


/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_set_activity                                         */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Trigger activity display behavior                                     */
/***************************************************************************/ 
static void oxnas_wd810_leds_set_activity
	(struct led_classdev *led_cdev, enum led_brightness value) {
	if (activity_block == 0) {
//printk("<1> oxnas_wd810_leds_set_activity value=%x\n", value);
		state = ACTIVITY;
	}
}

/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_set_capacity_gauge                                       */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Set the fuel gauge to the requested value (treated as a percentage)   */
/***************************************************************************/
static void oxnas_wd810_leds_set_capacity_gauge
	(struct led_classdev *led_cdev, enum led_brightness value) {
	capacity_gauge_bits = get_vbar_bits(get_percentage_pattern(value));
//printk("<1> oxnas_wd810_leds_set_capacity_gauge capacity_gauge_bits=%x\n", capacity_gauge_bits);
//if ( state < 3 )
	if (activity_block == 0 )
	{
//	start = 0;
		state = SHOW_CAPACITY;
//	activity_block = 0;
	}
}

/***************************************************************************/
/* DATA STRUCTURE: oxnas_wd810_leds_switch                                        */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the oxnas-wd810-leds "switch" on/off                            */
/***************************************************************************/
static struct led_classdev oxnas_wd810_leds_switch = {
	.name = "oxnas-wd810-leds:switch",.brightness_set =
		oxnas_wd810_leds_set_switch,
};

/***************************************************************************/
/* DATA STRUCTURE: oxnas_wd810_leds_state                                          */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the oxnas-wd810-leds "power" pseudo-LED                              */
/***************************************************************************/
static struct led_classdev oxnas_wd810_leds_state = {
	.name = "oxnas-wd810-leds:state",
	.brightness_set = oxnas_wd810_leds_set_state,
};

/***************************************************************************/
/* DATA STRUCTURE: oxnas_wd810_leds_activity                                       */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the oxnas-wd810-leds "activity" pseudo-LED                           */
/***************************************************************************/
static struct led_classdev oxnas_wd810_leds_activity = {
	.name = "oxnas-wd810-leds:activity",
	.brightness_set = oxnas_wd810_leds_set_activity,
	.default_trigger = "sata-disk"
};

/***************************************************************************/
/* DATA STRUCTURE: oxnas_wd810_leds_capacity_gauge                                     */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the oxnas-wd810-leds "capacity-gauge" LEDs (brightness = % full)         */
/***************************************************************************/
static struct led_classdev oxnas_wd810_leds_capacity_gauge = {
	.name = "oxnas-wd810-leds:capacity",
	.brightness_set = oxnas_wd810_leds_set_capacity_gauge,
};

/***************************************************************************/
/* DATA STRUCTURE: oxnas_wd810_leds_classes[]                                      */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Array of LED classes to create/destroy                                */
/***************************************************************************/
static struct led_classdev *oxnas_led_classes[] = {
	&oxnas_wd810_leds_switch,
	&oxnas_wd810_leds_state,
	&oxnas_wd810_leds_activity,	
	&oxnas_wd810_leds_capacity_gauge,
};

#ifdef DEBUG
static ssize_t
show_registers(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	char *out = buf;
	u32 clock_data = readl(PWM_CLOCK_REGISTER);
	u32 data_ptr = PWM_CLOCK_REGISTER;
	u8 no_pwms = (clock_data >> 16);
	u8 i;
	/* report hardware status here */
	out += sprintf(buf, "PWM drive registers\n");
	out +=
		sprintf(out, "clock register:0x%08x @ 0x%08x\n", clock_data,
				data_ptr);

	for (i = 0; i < no_pwms; ++i) {
		data_ptr = (u32) ((u32 *) PWM_BASE + i);
		out +=
			sprintf(out, "%d:%d @ 0x%08x\n", i, (u8) readl(data_ptr),
					data_ptr);
	}

	return out - buf;
}

/* create a register 'file' to enbale reading back the pwm drive register status */
static DEVICE_ATTR(registers, S_IRUGO, show_registers, NULL);

static void create_debug_files(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	device_create_file(dev, &dev_attr_registers);
}

static void remove_debug_files(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	device_remove_file(dev, &dev_attr_registers);
}
#endif


#ifdef CONFIG_PM
/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_suspend                                              */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Suspend all LED class devices created by this driver                  */
/***************************************************************************/
static int
oxnas_wd810_leds_suspend(struct platform_device *pdev, pm_message_t state)
{
	int n = leds_created;
	while (n > 0) {
		if (--n < ARRAY_SIZE(oxnas_led_classes)) {
			led_classdev_suspend(oxnas_led_classes[n]);
		}
	}

	return 0;
}

/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_resume                                               */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Wake up all LED class devices created by this driver                  */
/***************************************************************************/
static int
oxnas_wd810_leds_resume(struct platform_device *pdev, pm_message_t state)
{
	int n = leds_created;
	while (n > 0) {
		if (--n < ARRAY_SIZE(oxnas_led_classes)) {
			led_classdev_resume(oxnas_led_classes[n]);
		}
	}

	return 0;
}
#endif							/* CONFIG_PM */

/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_probe                                                */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform any necessary probing and initial setup for oxnas-wd810-leds device   */
/* this module doesnot reset the pwm clock as other modules may be using it */
/***************************************************************************/
static int oxnas_wd810_leds_probe(struct platform_device *pdev)
{
	int rc;
	int timer_changed = 0;
	int interrupt_allocated = 0;
	leds_created = 0;

	/* Reset the LED bit masks */
	if (negative_led_logic) {
		writel(LED_MASK_GPIO_A, GPIO_A_OUTPUT_SET);
		writel(LED_MASK_GPIO_B, GPIO_B_OUTPUT_SET);
	} else {
		writel(LED_MASK_GPIO_A, GPIO_A_OUTPUT_CLEAR);
		writel(LED_MASK_GPIO_B, GPIO_B_OUTPUT_CLEAR);
	}

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

		/* enable PWM clock - beware of the value set as it may affect the fan*/
		writel(PWM_PRESCALE, PWM_CLOCK_REGISTER);

		/* Initialize frame buffer to everything off */
//printk(KERN_INFO "oxnas_wd810_leds_probe:  LED_MASK_GAUGE=%x\n",LED_MASK_GAUGE);

		/* Enable output to the LEDs */
		/* Set our bits in the register - take care not to over write other bits in it */
		*((volatile unsigned long *) SYS_CTRL_GPIO_PWMSEL_CTRL_0) |= (LED_MASK_GPIO_A);
		*((volatile unsigned long *) SYS_CTRL_GPIO_PWMSEL_CTRL_1) |= (LED_MASK_GPIO_B);
		/* Turn Bottom LED to 50% intensity */
		writel(128, (PWM_DATA_REGISTER_BASE+0x24));
		
		/* Initialize the LED behavior state machine */
		oxnas_wd810_leds_behavior_init();
		/* Save Timer2 state for restoring later */
		timer_load = readl(TIMER2_LOAD);
		timer_control = readl(TIMER2_CONTROL);
		writel(0, TIMER2_CONTROL);
		timer_changed = 1;
		/* Setup Timer2 for LED control */
		rc = request_irq(TIMER_2_INTERRUPT, oxnas_wd810_leds_interrupt, 0,
						 "led_pwm", 0);
		if (rc < 0) {
			printk(KERN_ERR "failed to get IRQ\n");
			break;
		}

		interrupt_allocated = 1;
		writel(FAST_TIMER_INT, TIMER2_LOAD);
		writel(PERIODIC_INTERRUPT, TIMER2_CONTROL);

		/* Register each LED class device */
		while (leds_created < ARRAY_SIZE(oxnas_led_classes)) {
			rc = led_classdev_register(&pdev->dev,
									   oxnas_led_classes[leds_created]);
			if (rc < 0) {
				printk(KERN_ERR "failed to register led class \"%s\"\n",
					   oxnas_led_classes[leds_created]->name);
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
			if (--leds_created < ARRAY_SIZE(oxnas_led_classes)) {
				led_classdev_unregister(oxnas_led_classes[leds_created]);
			}
		}

		/* Free the interrupt if we allocated one */
		if (interrupt_allocated) {
			free_irq(TIMER_2_INTERRUPT, 0);
		}

		/* Restore Timer2 if we changed it */
		if (timer_changed) {
			writel(timer_load, TIMER2_LOAD);
			writel(timer_control, TIMER2_CONTROL);
		}
	}
#ifdef DEBUG
	create_debug_files(pdev);
#endif
	return rc;
}

/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_remove                                       */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform steps to remove the oxnas-wd810-leds device                   */
/* donot rese the entire pwm as other modules may be using it 			   */
/***************************************************************************/
static int oxnas_wd810_leds_remove(struct platform_device *pdev)
{
	u16 bit;
	u16 led_pwm_bits = 0;
	s8 count = 0;
	
	while (leds_created > 0) {
		if (--leds_created < ARRAY_SIZE(oxnas_led_classes)) {
			led_classdev_unregister(oxnas_led_classes[leds_created]);
		}
	}

	writel(0, TIMER2_CONTROL);
	free_irq(TIMER_2_INTERRUPT, 0);
	writel(timer_load, TIMER2_LOAD);
	writel(timer_control, TIMER2_CONTROL);
	/* Reset the LED bit masks */
	if (negative_led_logic) {
		writel(LED_MASK_GPIO_A, GPIO_A_OUTPUT_SET);
		writel(LED_MASK_GPIO_B, GPIO_B_OUTPUT_SET);
	} else {
		writel(LED_MASK_GPIO_A, GPIO_A_OUTPUT_CLEAR);
		writel(LED_MASK_GPIO_B, GPIO_B_OUTPUT_CLEAR);
	}
	
	/* Turn off all the leds - do not use SYS_CTRL_RSTEN_SET_CTRL as other 
	 * modules may use pwm as well
	 */
	led_pwm_bits = (LED_MASK_CG5 | LED_MASK_CG4 | LED_MASK_CG3 | LED_MASK_CG2 | LED_MASK_CG1 | LED_MASK_CG0);
	for (bit = 1; bit > 0; bit <<= 1, ++count) {
		if (bit & led_pwm_bits) {
			writel(0, ((u32 *) PWM_DATA_REGISTER_BASE + count));
		}
	}
	
#ifdef DEBUG
	remove_debug_files(pdev);
#endif
	return 0;
}

/***************************************************************************/
/* DATA STRUCTURE: oxnas_wd810_leds_driver                                         */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Describe the oxnas-wd810-leds platform device driver                          */
/***************************************************************************/
static struct platform_driver oxnas_wd810_leds_driver = {
	.probe = oxnas_wd810_leds_probe,
	.remove = oxnas_wd810_leds_remove,
#ifdef CONFIG_PM
	.suspend = oxnas_wd810_leds_suspend,.resume = oxnas_wd810_leds_resume,
#endif							/* CONFIG_PM */
	.driver = {.name = "oxnas-wd810-leds",},
};

/* Pointer to device returned by platform_device_register_simple */
static struct platform_device *oxnas_wd810_leds;
/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_init                                                 */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform module initialization                                         */
/***************************************************************************/
static int __init oxnas_wd810_leds_init(void)
{
	int ret;
//	printk(KERN_INFO "oxnas-wd810-leds:  SLOW_TPS=%d\n", SLOW_TPS);
	ret = platform_driver_register(&oxnas_wd810_leds_driver);
	if (!ret) {
		oxnas_wd810_leds =
			platform_device_register_simple("oxnas-wd810-leds", -1, NULL,
											0);
	}

	return ret;
}


/***************************************************************************/
/* FUNCTION: oxnas_wd810_leds_exit                                                 */
/*                                                                         */
/* PURPOSE:                                                                */
/*   Perform module unloading and cleanup                                  */
/***************************************************************************/
static void __exit oxnas_wd810_leds_exit(void)
{
	if (oxnas_wd810_leds) {
		platform_device_unregister(oxnas_wd810_leds);
	}
	platform_driver_unregister(&oxnas_wd810_leds_driver);
}


module_init(oxnas_wd810_leds_init);
module_exit(oxnas_wd810_leds_exit);
MODULE_DESCRIPTION("oxnas wd810 1NC/2NC LEDs");
MODULE_AUTHOR("Oxford Semiconductor Ltd");
MODULE_LICENSE("GPL");
/******************************* End of File *********************************/
