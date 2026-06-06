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
* @param *actuators: pinter to all actuators parameters
*
* @brief   Sets procents of power on EDF motor
*
* @return  none
*/
void PowerToPWMValue(uint8_t power)
{
    if (power > 100) power = 100;
    if (power < 5) power = 0;

    // 0% =0.870us, 100% = 2.12ms; delta = 1.25ms
	// Must be set by user (HTIRC HORNET 100A)
    // TIM2 tick is ~100us with PSC=8399 at 84MHz, period=20ms (50Hz)
    uint32_t pulse_us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * power) / 100;

    SetPWMValue(PWM_EDF, pulse_us);
}



/*********************************************************************
* @fn    EDFSlowRamp
*
* @param *actuators: pinter to all actuators parameters
*
* @brief   Slow ramp-up of EDF, slow/elegant start
*
* @return  none
*/
void EDFSlowRamp(s_actuators *actuators)
{
	if (!actuators->rampUpEnable)
	{
		actuators->rampUpEnable = 1;	// Enable flag for rampUp

		uint8_t steps = RAMPUP_TIME_MS / RAMPUP_TIME_STEP_MS;
		actuators->rampUpStep = (uint8_t)round((float)RAMPUP_TARGET_PERC / steps);
		actuators->rampUpDone = 0;
	}

	// Increase for calculated step
    actuators->edf_percent = actuators->edf_percent + actuators->rampUpStep;

    // 0% =0.870us, 100% = 2.12ms; delta = 1.25ms
	// Must be set by user (HTIRC HORNET 100A)
    // TIM2 tick is 1us with PSC=167, ARR=19999 at 84MHz, period=20ms (50Hz)
    uint32_t pulse_us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * actuators->edf_percent) / 100;

    if (actuators->edf_percent >= RAMPUP_TARGET_PERC)
    {
    	actuators->rampUpDone = 1;
    	actuators->rampUpEnable = 0;
    }


    SetPWMValue(PWM_EDF, pulse_us);
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
			TIM2->CCR2 = val;
			break;

		case SERVO_XP:			// X+
			TIM2->CCR1 = val;
			break;

		case SERVO_YN:			// Y-
			TIM2->CCR4 = val;
			break;

		case SERVO_YP:			// Y+
			TIM2->CCR3 = val;
			break;

		case PWM_EDF:			// EDF
			TIM3->CCR4 = val;
			break;

		default:
			break;
	}
}





