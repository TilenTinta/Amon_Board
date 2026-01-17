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
#define SERVO_XP_OFFSET 	-3
#define SERVO_XN_OFFSET 	1
#define SERVO_YP_OFFSET 	-3
#define SERVO_YN_OFFSET 	0


/*###########################################################################################################################################################*/
/* Enums */

typedef enum {
	SERVO_XP = 0x01,
	SERVO_XN = 0x02,
	SERVO_YP = 0x03,
	SERVO_YN = 0x04,
	PWM_EDF  = 0x05
}e_servo;


/*###########################################################################################################################################################*/
/* Functions */

void DegresToCCR(float Degress, uint8_t Servo);


#endif /* SERVO_SERVO_H_ */
