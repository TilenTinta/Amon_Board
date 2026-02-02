/*****************************************************************
 * File Name          : filters.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/02/01
 * Description        : Algorithms for filtering data
*****************************************************************/

#include "filters.h"

/*********************************************************************
* @fn     	Complementary_deg
*
* @param	*dev: struct to device data
* @param	*drone: drone data struct
*
* @brief   	Filter raw data from sensor with complementary filter
*
* @return  	None
*/
void Complementary_deg(s_MPU6050 *dev, s_drone_data *drone){

	// Axis orientation on drone are: X+ points down, Z+ points out of sensor and Y+ points right if you watch drone from the board side
	float accel_pitch = 0;
	float accel_roll = 0;

	/* Calculate drone pitch */
	//accel_pitch = atan2f(-dev->ACCEL_Z, sqrtf(dev->ACCEL_Y * dev->ACCEL_Y +  dev->ACCEL_X * dev->ACCEL_X)) * RAD_TO_DEG;
	accel_pitch = atan2f( dev->ACCEL_Z, -dev->ACCEL_X ) * RAD_TO_DEG;

	/* Calculate drone Roll */
	//accel_roll  = atan2f(dev->ACCEL_Y, sqrtf(dev->ACCEL_Z * dev->ACCEL_Z + dev->ACCEL_X * dev->ACCEL_X)) * RAD_TO_DEG;
	accel_roll  = atan2f( dev->ACCEL_Y, -dev->ACCEL_X ) * RAD_TO_DEG;

	/* Complementary Filter */
	drone->position.Pitch = ALPHA * (drone->position.Pitch + dev->GYRO_Y * DT) + (1.0f - ALPHA) * accel_pitch;
	drone->position.Roll = ALPHA * (drone->position.Roll + dev->GYRO_Z * DT) + (1.0f - ALPHA) * accel_roll;
	drone->position.Yaw = drone->position.Yaw + dev->GYRO_X * DT;
}



/*********************************************************************
* @fn     	Kalman_Init
*
* @param 	*k: struct to kalman filter data
*
* @brief   	Kalman filter initialization
*
* @return  	None
*/
void Kalman_Init(s_Kalman *k)
{
	// Default values
    k->angle = 0.0f;
    k->bias  = 0.0f;

    //
    k->P[0][0] = 1;
    k->P[0][1] = 0;
    k->P[1][0] = 0;
    k->P[1][1] = 1;
}



/*********************************************************************
* @fn     	Kalman_Update
*
* @param 	*k: struct to kalman filter data
* @param	accel_angle: angle acceleration for some axis
* @param	gyro_rate:
* @param 	dt: delta time - depends on timer
*
* @brief   	Kalman calculation
*
* @return  	None
*/
float Kalman_Update(s_Kalman *k,
                    float accel_angle,
                    float gyro_rate,
                    float dt)
{
    const float Q_angle = 0.001f;
    const float Q_bias  = 0.003f;
    const float R_measure = 0.03f;

    // 1️⃣ Predict
    k->angle += (gyro_rate - k->bias) * dt;

    k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += Q_bias * dt;

    // 2️⃣ Update
    float S = k->P[0][0] + R_measure;
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    float y = accel_angle - k->angle;

    k->angle += K0 * y;
    k->bias  += K1 * y;

    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];

    k->P[0][0] -= K0 * P00_temp;
    k->P[0][1] -= K0 * P01_temp;
    k->P[1][0] -= K1 * P00_temp;
    k->P[1][1] -= K1 * P01_temp;

    return k->angle;
}







