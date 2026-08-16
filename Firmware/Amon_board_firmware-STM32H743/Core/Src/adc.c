/*****************************************************************
 * File Name          : adc.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/08/04
 * Description        : ADC values filtering and managing
*****************************************************************/

#include "adc.h"


/*###########################################################################################################################################################*/
/* Functions */

/*********************************************************************
 * @fn      ADC_valueParser
 *
 * @param   *drone: pointer to drone data struct
 *
 * @brief   Managing raw data read from ADCs
 *
 * @return  None
 */
void ADC_valueParser(s_drone_data *drone, s_adcValues *adc)
{
	// Boot ADC check
	if (!adc->adc_avg_initialized)
	{
		for (uint8_t i = 0; i < 10; i++)
		{
			drone->data.bat_main_v[i] = adc->main_now;
			drone->data.bat_edf_v[i] = adc->edf_now;
			drone->data.buck_5V_v[i] = adc->buck_5v_now;
			drone->data.buck_7V2_v[i] = adc->buck_7v2_now;
		}

		adc->adc_avg_initialized = 1;
	}
	else
	{
		// Runtime ADC check
		for (uint8_t i = 1; i < 10; i++)
		{
			drone->data.bat_main_v[i-1] = drone->data.bat_main_v[i];
			drone->data.bat_edf_v[i-1] = drone->data.bat_edf_v[i];
			drone->data.buck_5V_v[i-1] = drone->data.buck_5V_v[i];
			drone->data.buck_7V2_v[i-1] = drone->data.buck_7V2_v[i];
		}

		drone->data.bat_main_v[9] = adc->main_now;
		drone->data.bat_edf_v[9] = adc->edf_now;
		drone->data.buck_5V_v[9] = adc->buck_5v_now;
		drone->data.buck_7V2_v[9] = adc->buck_7v2_now;
	}

	// Average calculation
	uint16_t main_v_temp = 0;
	uint16_t edf_v_temp = 0;
	uint16_t buck_5v_temp = 0;
	uint16_t buck_7v2_temp = 0;
	for (uint8_t i = 0; i < 10; i++)
	{
		main_v_temp += drone->data.bat_main_v[i];
		edf_v_temp += drone->data.bat_edf_v[i];
		buck_5v_temp += drone->data.buck_5V_v[i];
		buck_7v2_temp += drone->data.buck_7V2_v[i];
	}

	drone->data.battery_main_voltage = main_v_temp / (sizeof(drone->data.bat_main_v) / sizeof(uint16_t));
	drone->data.battery_edf_voltage = edf_v_temp / (sizeof(drone->data.bat_edf_v) / sizeof(uint16_t));
	drone->data.buck_5v_voltage = buck_5v_temp / (sizeof(drone->data.buck_5V_v) / sizeof(uint16_t));
	drone->data.buck_7v2_voltage = buck_7v2_temp / (sizeof(drone->data.buck_7V2_v) / sizeof(uint16_t));


	if (drone->data.battery_main_voltage < 1050) // less than 10.5V
	{
		drone->error_code.err_main_bat = 1;
	}
	else
	{
		drone->error_code.err_main_bat = 0;
	}

	if (drone->data.battery_edf_voltage < 2100) // less than 21V
	{
		drone->error_code.err_edf_bat = 1;
	}
	else
	{
		drone->error_code.err_edf_bat = 0;
	}

	if (drone->data.buck_5v_enable)
	{
		if (drone->data.buck_5v_voltage < 400) // less than 4V
		{
			drone->error_code.err_buck_5v = 1;
		}
		else
		{
			drone->error_code.err_buck_5v = 0;
		}
	}
	else
	{
		drone->error_code.err_buck_5v = 0;
	}

	if (drone->data.buck_7v2_enable)
	{
		if (drone->data.buck_7v2_voltage < 500) // less than 5V
		{
			drone->error_code.err_buck_7v2 = 1;
		}
		else
		{
			drone->error_code.err_buck_7v2 = 0;
		}
	}
	else
	{
		drone->error_code.err_buck_7v2 = 0;
	}
}
