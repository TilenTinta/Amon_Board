/*****************************************************************
 * File Name          : filters.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/02/01
 * Description        : Algorithms for filtering data
*****************************************************************/

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

#include "MPU6050.h"
#include "drone_data.h"

/*###########################################################################################################################################################*/
/* Defines */

/* --- Drone orientation ---
 * 	- Yaw: X+ points DOWN (along lander body, toward ground)
 * 	- Roll: Z+ points OUT of PCB
 * 	- Pitch: Y+ points RIGHT
 *
 * 	future use: Madgwick or Mahony quaternion filter
 */

#define IMU_X_OFFSET		0			// Offset if IMU from center of drone in X direction
#define IMU_Y_OFFSET		0			// Offset if IMU from center of drone in Y direction
#define IMU_Z_OFFSET		0			// Offset if IMU from center of drone in Z direction

#define RAD_TO_DEG			(180.0f	/ 3.14159265f)	// Radians to degress
#define DEG_TO_RAD  		(3.14159265f / 180.0f)	// Degress to radians
#define ALPHA				0.98		// Alpha value for complementary filter
#define DT					0.005f		// Delta time - 200Hz

#define ALPHA_EQ			0.98f		// Alpha value for Euler - quaternion complementary filter



/*###########################################################################################################################################################*/
/* Structs and enums */

// Gyro Kalman
typedef struct {
    float angle;   // fi - angle
    float bias;    // b - drift / offset
    float P00, P01, P10, P11; // P matrix 2x2 - confidence levels

} s_Kalman;


typedef struct {
    float w, x, y, z;

} s_Quaternion;


/*###########################################################################################################################################################*/
/* Functions */

// Complementary filter
void Complementary_deg(s_MPU6050 *dev, s_drone_data *drone);

// Kalman filter
void Kalman_Init(s_Kalman *k);
void Kalman_rawToAngles(s_MPU6050 *dev, float *roll_angle_accel, float *pitch_angle_accel);
float Kalman_Update(s_Kalman *k, float gyro_meas, float accel_angle, float dt);
float unwrap_to_ref(float meas, float ref);
s_Quaternion eulerToQuaternion(float roll, float pitch, float yaw);
void gyroToQuaternion(s_Quaternion *q, float gyro_x_deg, float gyro_y_deg, float gyro_z_deg, float dt);
void EulerQuaternion_Complementary(s_Quaternion *q, float gyro_x_deg, float gyro_y_deg, float gyro_z_deg, float roll_deg, float pitch_deg, float yaw_deg, float dt, float alpha);


#endif /* INC_FILTERS_H_ */
