/*
 * linux/include/asm-arm/arch-oxnas/tacho.h
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARM_ARCH_TACHO_H
#define __ASM_ARM_ARCH_TACHO_H

#include "hardware.h"

/* Routines ----------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * DumpTachoRegisters is a debug function used to inspect hte tacho registers.
 */
extern void DumpTachoRegisters(void);


/**
 * GetTemperature will read the thermistor register and convert the value to 
 * kelvin.
 * @return an int that represents the thermister temperature in Kelvin, or a
 * negative value in the case of error.
 */
extern int GetTemperature(void);


/**
 * GetFanRPM will read the fan tacho register and convert the value to 
 * RPM.
 * @return an int that represents the fan speed in RPM, or a
 * negative value in the case of error.
 */
extern int GetFanRPM(void);

#define TACHO_TARGET_CORE_FREQ_HZ       128000
#define TACHO_CORE_TACHO_DIVIDER_VALUE    (((NOMINAL_SYSCLK / TACHO_TARGET_CORE_FREQ_HZ) - 1))

#define QUAD_FUNCTION_ENABLE_FAN_PWM 		8 // this configures PWM 8 to be used for fan
#define PRIMARY_FUNCTION_ENABLE_FAN_TEMP 	29
#define PRIMARY_FUNCTION_ENABLE_FAN_TACHO 	30

#define TEMP_TACHO_PULLUP_CTRL_VALUE 		0x20000000

// 256kHz with 50MHz pclk (reset value)
#define PWM_CORE_CLK_DIVIDER_VALUE      (28)

/* Registers ---------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* FAN Speed Counter ----------------------------------- */

#define TACHO_FAN_SPEED_COUNTER                    (FAN_MON_BASE + 0x00)
    // 31:17 - RO - Unused (0x00)
	// 16 	 - R0 - Fan Count Valid - used in one shot mode
	// 15:10 - R0 - Unused
    //  9:0  - RO - Fan counter value. (See DD for conversion to rpm)
    #define TACHO_FAN_SPEED_COUNTER_FAN_COUNT       0
	#define TACHO_FAN_SPEED_COUNTER_COUNT_VALID 	16

	#define TACHO_FAN_SPEED_COUNTER_MASK 	1023

/* Thermistor RC Counter ------------------------------- */
#define TACHO_THERMISTOR_RC_COUNTER                (FAN_MON_BASE + 0x04)
    // 31:10 - RO - Unused (0x00)
    //  9:0  - RO - Thermistor counter value (See DD for conversion to temperature)
    #define TACHO_THERMISTOR_RC_COUNTER_THERM_COUNT 0

	#define TACHO_THERMISTOR_RC_COUNTER_MASK 	1023


/* Thermistor Control ---------------------------------- */
#define TACHO_THERMISTOR_CONTROL                   (FAN_MON_BASE + 0x08)
    // 31:2  - RO - Unused (0x00)
    //  1:1  - R0 � THERM_COUNT value is valid
    //  0:0  - RW - Set to 1 to enable thermistor PWM output
    #define TACHO_THERMISTOR_CONTROL_THERM_VALID    1
    #define TACHO_THERMISTOR_CONTROL_THERM_ENABLE   0 


/* Clock divider ---------------------------- */
#define TACHO_CLOCK_DIVIDER                        (FAN_MON_BASE + 0x0C)
    // 31:10 - RO - Unused (0x00)
    //  0:9  - RW - set PWM effective clock frequency to a division of pclk (0x030C )
    //         0000 � pclk divided by 1 (=pclk)
    //         0001 - pclk divided by 2
    //         0780 - ~128kHz with 100MHz pclk (reset value)
    //         1023 - pclk divided by 1024
    #define TACHO_CLOCK_DIVIDER_PWM_DIVIDER         0 
	#define TACHO_CLOCK_DIVIDER_MASK 	1023 


/* New hardware registers added for 810 */
/* Fan Speed Control ..........................*/
#define TACHO_FAN_SPEED_CONTROL 					(FAN_MON_BASE + 0x10)
	// 31:N+16 - R0 - Unused (0x0000)
	// N+15:16 - RW - Select PWM which controls FAN speed
	// 15:1 - 		  Unused 0
	// 0 	-	 RW - Fan Count mode 0 - Continuous running mode 1 - One shot mode
	#define TACHO_FAN_SPEED_CONTROL_PWM_ENABLE_BASE 	16
	#define TACHO_FAN_SPEED_CONTROL_PWM_USED 			8
	#define TACHO_FAN_SPEED_CONTROL_FAN_COUNT_MODE 		0


/* Fan One Shot Control .........................*/
#define TACHO_FAN_ONE_SHOT_CONTROL 					(FAN_MON_BASE + 0x14)
	// 31:1 - R - Unused
	// 0 - W - Start One-shot - Tacho - Self Clearing bit
	#define TACHO_FAN_ONE_SHOT_CONTROL_START 			0
   
/* PWM SECTION ------------------------------ */

// 0x00 Channel 0 PWM data 
// 7:0 R/W 0x00
// 31:8 Unused R 0x00000 Unused
#define  PWM_DATA_0 (PWM_BASE+0x00) 
    // value    0  � Output aways lo
    // value    1  � hi for 1 clock, lo for 255
    // . . . 
    // value   127 � 50:50 hi/lo
    // . . .
    // value   255 � hi for 255 clocks, lo for 1
    
// 0x04 Channel 1 PWM data
// 7:0 R/W 0x00
// 31:8 Unused R 0x00000 Unused
#define  PWM_DATA_1 (PWM_BASE+0x04) 
    // value    0  � Output aways lo
    // value    1  � hi for 1 clock, lo for 255
    // . . . 
    // value   127 � 50:50 hi/lo
    // . . .
    // value   255 � hi for 255 clocks, lo for 1
    
// 0x08 Channel 2 PWM data
// 7:0 R/W 0x00
// 31:8 Unused R 0x00000 Unused
#define  PWM_DATA_2 (PWM_BASE+0x08) 
    // value    0  � Output aways lo
    // value    1  � hi for 1 clock, lo for 255
    // . . . 
    // value   127 � 50:50 hi/lo
    // . . .
    // value   255 � hi for 255 clocks, lo for 1
    
// 0x0C Channel 3 PWM data
// 7:0 R/W 0x00
// 31:8 Unused R 0x00000 Unused
#define  PWM_DATA_3 (PWM_BASE+0x0C) 
    // value    0  � Output aways lo
    // value    1  � hi for 1 clock, lo for 255
    // . . . 
    // value   127 � 50:50 hi/lo
    // . . .
    // value   255 � hi for 255 clocks, lo for 1

#define  PWM_DATA_4 (PWM_BASE+0x10) 

#define  PWM_DATA_5 (PWM_BASE+0x14) 

#define  PWM_DATA_6 (PWM_BASE+0x18) 

#define  PWM_DATA_7 (PWM_BASE+0x1C) 

#define  PWM_DATA_8 (PWM_BASE+0x20) 
//0x400 PWM Clock Divider
// 15:0 R/W 0x00C2
// 31:16 Unused R 0x0000 Unused
#define  PWM_CLOCK_DIVIDER (PWM_BASE+0x400)

#endif // __ASM_ARM_ARCH_TACHO_H

/* End oF File */

