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
void DegresToCCR(float Degress, uint8_t Servo)
{
	float TimePerDeg = 0.01111; // 0deg = 0.5ms, 90deg = 1.5ms; delta 90deg = 1ms
	//double TimePerDeg = 0.5 / 90; // 0deg = 0.5ms, 90deg = 1.5ms; delta 90deg = 1ms

	float DutyCycle = ((Degress * TimePerDeg + 0.5) * 100) / 20.0f;

	// ARR(AutoReloadRegister) = 2400 (set in GUI editor for timer3)
	uint16_t CCRValue = (uint16_t)(DutyCycle * 2400.0f) / 100.0f;

	SetPWMValue(Servo, CCRValue);
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





