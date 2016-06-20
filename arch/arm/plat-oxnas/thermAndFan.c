/*
 * Device driver for the i2c thermostat found on the iBook G4, Albook G4
 *
 * Copyright (C) 2003, 2004 Colin Leroy, Rasmus Rohde, Benjamin Herrenschmidt
 *
 * Documentation from
 * http://www.analog.com/UploadedFiles/Data_Sheets/115254175ADT7467_pra.pdf
 * http://www.analog.com/UploadedFiles/Data_Sheets/3686221171167ADT7460_b.pdf
 *
 */
 

#include <linux/types.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/smp_lock.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/capability.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/wait.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>
#include <linux/freezer.h>

#include <asm/io.h>
#include <asm/system.h>
#include <asm/sections.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>

#include "asm/plat-oxnas/taco.h"
#include "thermistorCalibration.h"

//#define DEBUG 1
#undef DEBUG

/* Set this define to simulate different temperature values and test the 
 * working of the fan control module
 */
//#define SIMULATE_TEMPERATURE 1
#undef SIMULATE_TEMPERATURE

/******************************************************************************     
 *                                                                            *
 *  delta t = dutyCycle.Period                                                *
 *                                                                            *
 *  PWM in  >----+    V' =   Vcc - Vt                                         *
 *               |           --------                                         *
 *              _|_             Vcc                                           *
 *            | | |                                                           *
 *             \| |   Rt =    delta t                                         *
 *              | |        -------------                                      *
 *              |\|         -C.ln( V' )                                       *
 *              | |                                                           *
 *              | |\  delta t is discovered by the hardware which performs a  *   
 *              !_! \ varies the PWM duty cycle until one is found that just  *
 *               |    trips the Vt threshold.                                 *
 *               |                                                            *
 *               |----------->  V in (Schmidt trigger @ Vt)                   *
 *               |                                                            *
 *            ___!___                                                         *
 *            _______  C ( eg 100nF )                                         *
 *               |                                                            *
 *               |                                                            *
 *               |                                                            *
 *            ---+---                                                         *
 *                                                                            *
 *  Steinhart Thermistor approximation:                                       *
 *                                                                            *
 *    1/T = A + B.ln(Rt) + C.ln(Rt)^3                                         *
 *                                                                            *
 ******************************************************************************/           

#define MAX_FAN_RATIO_CHANGE 		20
#define OXSEMI_FAN_SPEED_RATIO_MIN 	0
#define OXSEMI_FAN_SPEED_RATIO_MAX 	255
#define FAN_SPEED_RATIO_SET 		(PWM_DATA_8) // Use PWM 8 - quad function on 810
#define MIN_TEMP_COUNT_CHANGE 		2

#define NUMBER_OF_ITERATIONS 		10
#define TEMP_MEASURE_NORMAL 		0
#define TEMP_MEASURE_ERROR 		1
#define FAN_MEASURE_NORMAL 		0
#define FAN_MEASURE_ERROR 		1

/* This is not absolute temperature but counter val - thermistorCalibration*/
static int hot_limit 		= 16; 
static int cold_limit 		= 104;
static int min_fan_speed_ratio  = 64;
static int fan_pulse_per_rev 	= 1;

/* Status reporters */
static int output_flag 		= 0;
static int current_temp 	= 0;
static int current_speed 	= 0;

/* Error Reporters */
static int error_temp 	= 0;
static int error_fan 	= 0;
                                       	
MODULE_AUTHOR(		"Anand Srinivasaraghavan"					);
MODULE_DESCRIPTION(	"Driver for Temperature sense and Fan control of ox810" 		);
MODULE_LICENSE(		"GPL"						);

module_param(hot_limit, int, 0644);
MODULE_PARM_DESC(hot_limit, "Thermistor input for maximum fan drive");

module_param(cold_limit, int, 0644);
MODULE_PARM_DESC(cold_limit,"Thermistor input for minimum fan drive");

module_param(min_fan_speed_ratio, int, 0644);
MODULE_PARM_DESC(min_fan_speed_ratio,"Specify starting( minimum) fan drive (0-255) (default 64)");

module_param(fan_pulse_per_rev, int, 0644);
MODULE_PARM_DESC(fan_pulse_per_rev,"Specify the number of pulses per revolution of fan - 1(default) or 2");

module_param(output_flag, bool, 0644);
MODULE_PARM_DESC(output_flag,"Flag to specify whether temperature and speed output to user is required");

module_param(current_temp, int, S_IRUGO);
MODULE_PARM_DESC(current_temp,"Read only for the current temperature in counts");

module_param(current_speed, int, S_IRUGO);
MODULE_PARM_DESC(current_speed,"Read only for the current speed in rpm");

module_param(error_temp, int, S_IRUGO);
MODULE_PARM_DESC(error_temp,"Read only - System is experiencing problems in temperature measurement");

module_param(error_fan, int, S_IRUGO);
MODULE_PARM_DESC(error_fan,"Read only - System is experiencing problems in fan speed measurement");

struct thermostat {
	int			temps;
	int			cached_temp;
	int			curr_speed;
	int			last_speed;
	int	    		set_speed;
	int			last_var;
	struct semaphore 	sem;
};

static struct thermostat* 	thermostat   = NULL;
static struct task_struct*	thread_therm = NULL;


static int write_reg( int reg, u32 data )
{
	writel( data, reg );
	return 0;
}

static int read_reg( int reg )
{
	int data = 0;
	data = readl( reg );
	return data;
}

/** GetTemperatureCounter is an internal function to the module that reads the
 * temperature as a counter value and returns it to the caller. The counter 
 * value is used in all the internal calculations avoiding the necessity to 
 * convert to Kelvin for interpretation. For corresponding temperature values 
 * in Kelvin look at thermistoCalilbration.h
 */
int GetTemperatureCounter(void)
{
	u32 res;
	int count = 1;
	
	/* Test to ensure we are ready.
	 */
	if ( !thermostat ) {
		printk(KERN_INFO "T&F?::$RERROR - Temperature conv not started\n");
		return -1;
	}

	while ( !(     read_reg( TACHO_THERMISTOR_CONTROL ) & 
		   (1 << TACHO_THERMISTOR_CONTROL_THERM_VALID) ) ) {
/*		printk(KERN_INFO "T&F?::$rWarning - Temperature reading not stabalised\n");
*/		if(count == NUMBER_OF_ITERATIONS) {
			/* Error - temperature not getting stabilised */
			error_temp = TEMP_MEASURE_ERROR;
			/* Force count to be hot_limit - implies max fan speed */
			return hot_limit;
		}
		count ++;
		msleep(100);
	}

	res = read_reg( TACHO_THERMISTOR_RC_COUNTER ) & TACHO_THERMISTOR_RC_COUNTER_MASK;

	/* Executing this code implies that the temperature has stabilised */
	/* Reset temperature measurement error */
	if(error_temp != TEMP_MEASURE_NORMAL)
		error_temp = TEMP_MEASURE_NORMAL;

#ifdef DEBUG
	printk(KERN_INFO "Therm&Fan - Temperature Counter - %d \n",res);
#endif

	return res;
}

/** This function converts the unit of the temperature from counts to
 * temperature in Kelvin
 */
int ConverttoKelvin(int tempCount)
{
	u32 res, arrayIndex;

	/* Convert the Counter Value to Temperature in Kelvin */
	arrayIndex = tempCount/THERM_INTERPOLATION_STEP;
	res = TvsCnt[arrayIndex];
	if ((THERM_ENTRIES_IN_CALIB_TABLE - 2) > arrayIndex)
		res -= (tempCount % THERM_INTERPOLATION_STEP) * (TvsCnt[arrayIndex] - TvsCnt[arrayIndex + 1])  / THERM_INTERPOLATION_STEP;
	else
		res -= (tempCount % THERM_INTERPOLATION_STEP) * (TvsCnt[THERM_ENTRIES_IN_CALIB_TABLE - 2] - TvsCnt[THERM_ENTRIES_IN_CALIB_TABLE - 1]) / THERM_INTERPOLATION_STEP;
			
#ifdef DEBUG
	printk(KERN_INFO "Get Temperature- Temperature in Kelvin = %d\n", res);
#endif
	return res;
}

/**
 * GetTemperature reads the temperature from the thermistor and converts it
 * to the corresponding Kelvin equivalent
 */
int GetTemperature(void)
{
	u32 tempCount;
	tempCount = GetTemperatureCounter();
	return 	ConverttoKelvin(tempCount);
}

/**
 * GetFanRPM will read the fan tacho register and convert the value to 
 * RPM.
 * @return an int that represents the fan speed in RPM, or a
 * negative value in the case of error.
 */
int GetFanRPM(void)
{
	u32 res;
	u32 iCounterValue;
	int count = 1;
	
	if(thermostat->last_speed == OXSEMI_FAN_SPEED_RATIO_MIN)
	{
#ifdef DEBUG
		printk(KERN_INFO "ThermAndFan::GetFanRPM - Fan Speed %d \n", OXSEMI_FAN_SPEED_RATIO_MIN);
#endif
		return OXSEMI_FAN_SPEED_RATIO_MIN;
	}

	write_reg( TACHO_FAN_ONE_SHOT_CONTROL, (1 << TACHO_FAN_ONE_SHOT_CONTROL_START));

	while ( !(read_reg( TACHO_FAN_SPEED_COUNTER ) & 
			   (1 << TACHO_FAN_SPEED_COUNTER_COUNT_VALID) ) ) {
/*		printk(KERN_INFO "ThernAndFan::$rWarning - Fan Counter reading not stabalised\n");
*/		if(count == NUMBER_OF_ITERATIONS) {
			/* Error - fan speed measuring not stabilised */
			error_fan = FAN_MEASURE_ERROR;
			/* Return previous measured fan speed to be safe */
			return (thermostat->curr_speed);
		}
		count ++;
		msleep(100);
	}
	
	iCounterValue = read_reg( TACHO_FAN_SPEED_COUNTER ) 
						& TACHO_FAN_SPEED_COUNTER_MASK;

	/* Reaching here implies fan speed measurement is working good */
	/* Reset any error */
	if(error_fan != FAN_MEASURE_NORMAL)
		error_fan = FAN_MEASURE_NORMAL;

	++iCounterValue;
	
	/* Fan Speed (rpm) = 60 * 2000 / (counter value +1) * pulses per rev */
	res = (60 * 2000 ) / (iCounterValue * fan_pulse_per_rev);

#ifdef SIMULATE_TEMPERATURE
	printk(KERN_INFO "thermAndFan::GetFanRPM == %d\n", res);
#endif /*SIMULATE_TEMPERATURE */

	return res;
}

static void read_sensors(struct thermostat *th)
{
	static int state;
#ifdef SIMULATE_TEMPERATURE
	static int curTemp =  30;
#endif /* SIMULATE_TEMPERATURE */

	if ( !th ) {
		printk(KERN_INFO "thermAndFan::read_sensors $RTH NOT ESTABLISHED YET\n");		
		return;
	}
	switch(state){
	case 0:
		th->temps  = GetTemperatureCounter();

		/* Set to speed measurement */
		write_reg( TACHO_FAN_SPEED_CONTROL, 
				(1 << (TACHO_FAN_SPEED_CONTROL_PWM_ENABLE_BASE 
								+ TACHO_FAN_SPEED_CONTROL_PWM_USED)) 
					| (1 << TACHO_FAN_SPEED_CONTROL_FAN_COUNT_MODE));
		state = 1;

		if(output_flag) /* Set the temperature to user space here */
		{
			current_temp = th->temps;
		}

	#ifdef SIMULATE_TEMPERATURE
		th->temps = curTemp;
		curTemp += 5;
		if(curTemp > cold_limit + 20)
			curTemp = hot_limit - 20;
		printk(KERN_INFO "thermAndFan::read_sensors Temp Set to - %d\n", curTemp);
	#endif /* SIMULATE_TEMPERATURE */
	break;

	case 1:
	default:
		
		th->curr_speed  = GetFanRPM();

		/* Set to Temperature measurement */
		write_reg( TACHO_THERMISTOR_CONTROL, ((1 << TACHO_THERMISTOR_CONTROL_THERM_ENABLE) 
							| (0 << TACHO_THERMISTOR_CONTROL_THERM_VALID)) );
		state = 0;
		
		if(output_flag) /* Set the speed to user space here */
		{
			current_speed = th->curr_speed;
		}
	break;
	}
}

#ifdef DEBUG
/**
 * DumpTachoRegisters is a debug function used to inspect hte tacho registers.
 */
void DumpTachoRegisters(void)
{

	printk(KERN_INFO \
		"\n<Taco Registers> ---------------------------------\n"
		"      TACHO_FAN_SPEED_COUNTER          == 0x%08x\n"
		"      TACHO_THERMISTOR_RC_COUNTER      == 0x%08x\n"
		"      TACHO_THERMISTOR_CONTROL         == 0x%08x\n"
		"      TACHO_CLOCK_DIVIDER              == 0x%08x\n"
		"      PWM_CORE_CLK_DIVIDER_VALUE       == 0x%08x\n"
		"      FAN_SPEED_RATIO_SET              == 0x%08x\n"
		"<\\Taco Registers> --------------------------------\n\n",
		(u32) (read_reg( TACHO_FAN_SPEED_COUNTER) & TACHO_FAN_SPEED_COUNTER_MASK),	
		(u32) (read_reg( TACHO_THERMISTOR_RC_COUNTER) & TACHO_THERMISTOR_RC_COUNTER_MASK),	
		(u32) read_reg( TACHO_THERMISTOR_CONTROL	),	
		(u32) (read_reg( TACHO_CLOCK_DIVIDER) & TACHO_CLOCK_DIVIDER_MASK),	
		(u32) read_reg( PWM_CLOCK_DIVIDER        ),	
		(u32) read_reg( FAN_SPEED_RATIO_SET 			 ) );	
}
#else
void DumpTachoRegisters(void) {}
#endif

static void write_fan_speed(struct thermostat *th, int speed)
{
	/* The fan speed can vary between the max and the min speed ratio or 
	 * it can be min ratio value of 0 
	 */
	if (speed > OXSEMI_FAN_SPEED_RATIO_MAX) 
		speed = OXSEMI_FAN_SPEED_RATIO_MAX;
	else if ((speed < min_fan_speed_ratio) && (speed > OXSEMI_FAN_SPEED_RATIO_MIN))
		speed = min_fan_speed_ratio;
	else if (speed < OXSEMI_FAN_SPEED_RATIO_MIN) 
		speed = OXSEMI_FAN_SPEED_RATIO_MIN;
	
	if (th->last_speed == speed)
		return;
	
	write_reg( FAN_SPEED_RATIO_SET, speed );

#ifdef SIMULATE_TEMPERATURE
	printk(KERN_INFO "Speed Ratio Written - %d\n", speed);
#endif /* SIMULATE_TEMPERATURE */
		
	th->last_speed = speed;			
}

#ifdef DEBUG
static void display_stats(struct thermostat *th)
{
	if ( 1 || th->temps != th->cached_temp) {
		printk(KERN_INFO 
			 "thermAndFan:: Temperature infos:\n"
			 "  * thermostats: %d;\n"
			 "  * pwm: %d;\n"
			 "  * fan speed: %d RPM\n\n",
			 th->temps,
			 min_fan_speed_ratio,
			 GetFanRPM());
		th->cached_temp = th->temps;
	}
}
#endif

/* 
 * Use fuzzy logic type approach to creating the new fan speed. 
 * if count < cold_limit fan should be off.
 * if count > hot_limit fan should be full on.
 * if count between limits set proportionally to base speed + proportional element.
 */
static void update_fan_speed(struct thermostat *th)
{
	int var = th->temps;
	
/* remember that var = 1/T ie smaller var higher temperature and faster fan speed needed */
	if (abs(var - th->last_var) >= MIN_TEMP_COUNT_CHANGE) {
		int new_speed;

		if(var < cold_limit){
			
			if (var < hot_limit)
			{
				th->last_var = var;
				/* too hot for proportional control */
				new_speed = OXSEMI_FAN_SPEED_RATIO_MAX;
			}
			else
			{
				/* fan speed it the user selected starting value for the fan 
				 * so scale operatation from nominal at cold limit to max at hot limit. 
				 */
				new_speed = OXSEMI_FAN_SPEED_RATIO_MAX - 
					(OXSEMI_FAN_SPEED_RATIO_MAX - min_fan_speed_ratio) * (var - hot_limit)/(cold_limit - hot_limit);

				if (th->set_speed == 0 ) th->set_speed = min_fan_speed_ratio;
								
				if ((new_speed - th->set_speed) > MAX_FAN_RATIO_CHANGE)
					new_speed = th->set_speed + MAX_FAN_RATIO_CHANGE; 
				else if ((new_speed - th->set_speed) < -MAX_FAN_RATIO_CHANGE)
					new_speed = th->set_speed - MAX_FAN_RATIO_CHANGE;
				else
					th->last_var = var;
			}
		}
		else {
			
			th->last_var = var;
			/* var greater than low limit - too cold for fan. */
			new_speed = OXSEMI_FAN_SPEED_RATIO_MIN;
		}

		write_fan_speed(th, new_speed);
		th->set_speed = new_speed;
	}
}

static int monitor_task(void *arg)
{
	struct thermostat* th = arg;
	
	while(!kthread_should_stop()) {
		if (unlikely(freezing(current)))
			refrigerator();

		msleep_interruptible(2000);

#ifdef DEBUG
		DumpTachoRegisters();
#endif

		read_sensors(th);

		update_fan_speed(th);

#ifdef DEBUG
		/* be carefule with the stats displayed. The Fan Counter value depends 
		 * on what value is written in the register during the read sensors 
		 * call. If its in temperature read setting, the fan counter and hence 
		 * the rpm will be WRONG
		 */
		display_stats(th);
#endif
	}

	return 0;
}

static int
oxsemi_therm_read(char *buf, char **start, off_t offset,
		  int len, int *eof, void *unused)
{
	len = sprintf(buf, 
		"Thermostat And Fan state ---------\n"
		"      temps_counter          	== %d\n"
		"      speed_ratio_set  	== %d\n"
		"      measured-fan_speed    	== %d\n"
		"      last_temp_counter      	== %d\n\n",
		thermostat->temps,
		thermostat->last_speed,
		thermostat->curr_speed,
		thermostat->last_var );
	return len;
}

static struct proc_dir_entry *proc_oxsemi_therm;


static struct file_operations oxsemi_therm_fops = {
	.owner		= THIS_MODULE,
	.open		= nonseekable_open,
};

static struct miscdevice oxsemi_therm_miscdev = {
	TEMP_MINOR,
	"temp",
	&oxsemi_therm_fops
};

static int __init oxsemi_therm_init(void)
{
	struct thermostat* th;
	int rc, ret;

	if (thermostat)
		return 0;

	read_reg(SYS_CTRL_RSTEN_CTRL);

/* release fan/tacho from system reset */		
	*((volatile unsigned long *) SYS_CTRL_RSTEN_CLR_CTRL) = (1UL << SYS_CTRL_RSTEN_MISC_BIT);
	
/* Pull Down the GPIO 29 from the software */
	*((volatile unsigned long *) SYSCTRL_GPIO_PULLUP_CTRL_0) |= TEMP_TACHO_PULLUP_CTRL_VALUE;
	
	*((volatile unsigned long *) SYS_CTRL_GPIO_PRIMSEL_CTRL_0) &= ~(1UL << QUAD_FUNCTION_ENABLE_FAN_PWM);
	*((volatile unsigned long *) SYS_CTRL_GPIO_PRIMSEL_CTRL_0) |= (1UL << PRIMARY_FUNCTION_ENABLE_FAN_TACHO);
	*((volatile unsigned long *) SYS_CTRL_GPIO_PRIMSEL_CTRL_0) |= (1UL << PRIMARY_FUNCTION_ENABLE_FAN_TEMP);
	
/* disable secondary use */
	*((volatile unsigned long *) SYS_CTRL_GPIO_SECSEL_CTRL_0) &= ~(1UL << QUAD_FUNCTION_ENABLE_FAN_PWM);
	
/* disable tertiary use */
	*((volatile unsigned long *) SYS_CTRL_GPIO_TERTSEL_CTRL_0) &= ~(1UL << QUAD_FUNCTION_ENABLE_FAN_PWM);
	
/* disable quadinary use */
	*((volatile unsigned long *) SYS_CTRL_GPIO_PWMSEL_CTRL_0) |= (1UL << QUAD_FUNCTION_ENABLE_FAN_PWM);
	
	read_reg(SYS_CTRL_RSTEN_CTRL);
	read_reg(SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
	read_reg(SYS_CTRL_GPIO_SECSEL_CTRL_0);
	read_reg(SYS_CTRL_GPIO_TERTSEL_CTRL_0);

	th = (struct thermostat *)
		kmalloc(sizeof(struct thermostat), GFP_KERNEL);
	
	if (!th)
		return -ENOMEM;

	memset(th, 0, sizeof(struct thermostat));
	init_MUTEX( &th->sem );

	rc = read_reg(TACHO_CLOCK_DIVIDER);
	if (rc < 0) {
		printk(KERN_ERR "thermAndFan: Thermostat failed to read config ");
		kfree(th);
		return -ENODEV;
	}
	
	/* Set the Tacho clock divider up */
	write_reg( TACHO_CLOCK_DIVIDER, TACHO_CORE_TACHO_DIVIDER_VALUE );
	
	/* check tacho divider set correctly */	
	rc = read_reg(TACHO_CLOCK_DIVIDER);
	/* Comparing a 10 bit value to a 32 bit return value */
	if ((rc & TACHO_CORE_TACHO_DIVIDER_VALUE) != TACHO_CORE_TACHO_DIVIDER_VALUE) {
		printk(KERN_ERR "thermAndFan: Set Tacho Divider Value Failed readback:%d\n", rc);
		kfree(th);
		return -ENODEV;
	}
	
	write_reg( PWM_CLOCK_DIVIDER, PWM_CORE_CLK_DIVIDER_VALUE );
	
	printk(KERN_INFO "thermAndFan: initializing - ox810\n");
	
#ifdef DEBUG
	DumpTachoRegisters();
#endif
	
	thermostat = th;

	/* Start the thermister measuring */
	write_reg( TACHO_THERMISTOR_CONTROL, (1 << TACHO_THERMISTOR_CONTROL_THERM_ENABLE) );
	
	/* Start Speed measuring */
	write_reg( TACHO_FAN_SPEED_CONTROL, 
			(1 << (TACHO_FAN_SPEED_CONTROL_PWM_ENABLE_BASE 
							+ TACHO_FAN_SPEED_CONTROL_PWM_USED)) 
				| (1 << TACHO_FAN_SPEED_CONTROL_FAN_COUNT_MODE));
	
	/* be sure to really write fan speed the first time */
	th->last_speed    = -2;
	th->last_var	  = -80;

	/* Set fan to initial speed */
	write_fan_speed(th, min_fan_speed_ratio);

	thread_therm = kthread_run(monitor_task, th, "kfand");

	if (thread_therm == ERR_PTR(-ENOMEM)) {
		printk(KERN_INFO "thermAndFan: Kthread creation failed\n");
		thread_therm = NULL;
		return -ENOMEM;
	}

	ret = misc_register(&oxsemi_therm_miscdev);
	if (ret < 0)
		return ret;

	proc_oxsemi_therm = create_proc_entry("therm-fan", 0, NULL);
	if (proc_oxsemi_therm) {
		proc_oxsemi_therm->read_proc = oxsemi_therm_read;
	} else {
		printk(KERN_ERR "therm-fan: unable to register /proc/therm\n");
	}

	return 0;
}


static void __exit oxsemi_therm_exit(void)
{
	if ( thread_therm )
	{
		kthread_stop(thread_therm);
	}
	
	/* Stop the fan so that it doesnot run anymore - dont reset 
	 * SYS_CTRL_RSTEN_MISC_BIT as other modules may use it
	 */
	write_fan_speed(thermostat, 0);
	
	remove_proc_entry("therm-fan", NULL);
	misc_deregister(&oxsemi_therm_miscdev);
	
	kfree(thermostat);
	thermostat = NULL;
	
}


module_init(oxsemi_therm_init);
module_exit(oxsemi_therm_exit);


/* End of File */
