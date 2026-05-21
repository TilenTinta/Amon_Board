/*****************************************************************
 * File Name          : Servo.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2023/02/23
 * Description        : Servo motor definitions
*****************************************************************/

#ifndef SERVO_SERVO_H_
#define SERVO_SERVO_H_

#include "stm32h7xx_hal.h"					// For PWM
#include "drone_data.h"
#include "math.h"


/*###########################################################################################################################################################*/
/* Defines */

/*
 * PWM: 50Hz (20ms)
 *  - DutyCycle: 0deg - 0.5ms		(1518)
 *  - DutyCycle: 90deg - 1.5ms		(4555)
 *  - DutyCycle: 180deg - 2.5ms		(7592)
 *
 * 	1deg = 0.011ms
 * 	1deg = 33.4 value
 * 	0.1deg = 3.374 value
 *
 */

// Offsets on motor to make them straight
#define SERVO_XP_OFFSET 	-2		// Servo offset
#define SERVO_XN_OFFSET 	1		// Servo offset
#define SERVO_YP_OFFSET 	-3		// Servo offset
#define SERVO_YN_OFFSET 	0		// Servo offset

#define SERVOS_ZERO			90		// Zero position of servos (straight down)

#define SERVO_MIN_US 		1000	// Minimum duty-cycle of EDF (0%)
#define SERVO_MAX_US 		2000	// Maximum duty-cycle of EDF (100%)
#define SERVO_RANGE_US 		(SERVO_MAX_US - SERVO_MIN_US)

#define RAMPUP_TARGET_PERC	60		// Ramp-up percent target
#define RAMPUP_TIME_MS		3000	// Ramp-up time target
#define RAMPUP_TIME_STEP_MS	20		// Ramp-up timer period of function call



/*###########################################################################################################################################################*/
/* Enums */

typedef enum {
	SERVO_XP,
	SERVO_XN,
	SERVO_YP,
	SERVO_YN,
	PWM_EDF
}e_servo;


/*###########################################################################################################################################################*/
/* Functions */

void DegresToCCR(float degrees, uint8_t Servo);
void PowerToPWMValue(uint8_t power);
void EDFSlowRamp(s_actuators *actuators);


#endif /* SERVO_SERVO_H_ */
