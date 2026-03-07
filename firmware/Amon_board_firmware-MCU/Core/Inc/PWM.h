/*****************************************************************
 * File Name          : Servo.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2023/02/23
 * Description        : Servo motor definitions
*****************************************************************/

#ifndef SERVO_SERVO_H_
#define SERVO_SERVO_H_

#include "stm32f4xx_hal.h"					// For PWM


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
#define SERVO_XP_OFFSET 	-2
#define SERVO_XN_OFFSET 	1
#define SERVO_YP_OFFSET 	-3
#define SERVO_YN_OFFSET 	0

#define SERVOS_ZERO			90

#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_RANGE_US (SERVO_MAX_US - SERVO_MIN_US)



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


#endif /* SERVO_SERVO_H_ */
