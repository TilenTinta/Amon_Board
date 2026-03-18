/*****************************************************************
 * File Name          : Servo.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2023/02/23
 * Description        : Servo motor definitions
*****************************************************************/


#include <PWM.h>

/*###########################################################################################################################################################*/
/* Private functions */

void SetPWMValue(uint8_t output, uint32_t val);
void PowerToPWMValue(uint8_t power);


/*###########################################################################################################################################################*/
/* Functions */


/*********************************************************************
* @fn     DegresToCCR
*
* @param Degress: degress that you want to set
* @param Servo: servo that you want to set
*
* @brief   Convert degrees to CCR value that can be writen in PWM register
*
* @return  none
*/
void DegresToCCR(float degrees, uint8_t Servo)
{
	// Clamp degrees
	if (degrees < -90.0f)   degrees = -90.0f;
	if (degrees > 90.0f) degrees = 90.0f;

	// Add offset and zero correction
	switch(Servo)
	{
		case SERVO_XN:			// X-
			degrees = degrees + SERVOS_ZERO + SERVO_XN_OFFSET;
			break;

		case SERVO_XP:			// X+
			degrees = degrees + SERVOS_ZERO + SERVO_XP_OFFSET;
			break;

		case SERVO_YN:			// Y-
			degrees = degrees + SERVOS_ZERO + SERVO_YN_OFFSET;
			break;

		case SERVO_YP:			// Y+
			degrees = degrees + SERVOS_ZERO + SERVO_YP_OFFSET;
			break;

		case PWM_EDF:			// EDF
			degrees = degrees + PWM_EDF;
			break;

		default:
			break;
		}

	// Calculate degrees to CCR value (capture and compare register)
	float pulse_us = SERVO_MIN_US + (degrees / 180.0f) * SERVO_RANGE_US;

	uint32_t ccr = (uint32_t)pulse_us;

	SetPWMValue(Servo, ccr);
}



/*********************************************************************
* @fn    PowerToPWMValue
*
* @param power: percent of power you want to set
*
* @brief   Sets procents of power on EDF motor
*
* @return  none
*/
void PowerToPWMValue(uint8_t power)
{
	// 50Hz PWM
	// 0% =0.5ms, 100% = 2.5ms; delta 100% = 2ms
	// Must be set by user (HTIRC HORNET 100A)
	uint32_t value;

	float TimePerPercent = 0.02; // time of PWM for one percent of power

	value = power * TimePerPercent;

	SetPWMValue(PWM_EDF, value);
}



/*********************************************************************
* @fn    SetPWMValue
*
* @param output: servo you want to move
* @param val: CCR value writen in register
*
* @brief   Convert degrees to CCR value that can be writen in PWM register
*
* @return  none
*/
void SetPWMValue(uint8_t output, uint32_t val)
{
	switch(output){
		case SERVO_XN:			// X-
			TIM3->CCR1 = val;
			break;

		case SERVO_XP:			// X+
			TIM3->CCR2 = val;
			break;

		case SERVO_YN:			// Y-
			TIM3->CCR4 = val;
			break;

		case SERVO_YP:			// Y+
			TIM3->CCR3 = val;
			break;

		case PWM_EDF:			// EDF
			TIM2->CCR4 = val;
			break;

		default:
			break;
	}
}





