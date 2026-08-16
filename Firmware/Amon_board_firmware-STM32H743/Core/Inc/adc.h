/*****************************************************************
 * File Name          : adc.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/08/04
 * Description        : ADC values filtering and managing
*****************************************************************/

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "drone_data.h"

/*###########################################################################################################################################################*/
/* Structs and enums */
typedef struct
{
	volatile uint8_t 	adc_avg_initialized;
	uint16_t 			main_now;
	uint16_t 			edf_now;
	uint16_t 			buck_5v_now;
	uint16_t 			buck_7v2_now;

} s_adcValues;

/*###########################################################################################################################################################*/
/* Functions */
void ADC_valueParser(s_drone_data *drone, s_adcValues *adc);



#endif /* INC_ADC_H_ */
