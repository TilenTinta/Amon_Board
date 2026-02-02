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

/* Drone orientation:
 * - Yaw: X+ points DOWN (along lander body, toward ground)
 * - Roll: Z+ points OUT of PCB
 * - Pitch: Y+ points RIGHT
 */

#define IMU_X_OFFSET		0			// Offset if IMU from center of drone in X direction
#define IMU_Y_OFFSET		0			// Offset if IMU from center of drone in Y direction
#define IMU_Z_OFFSET		0			// Offset if IMU from center of drone in Z direction

#define RAD_TO_DEG			57.32484076	// Radians to degress: 180deg / 3.14
#define ALPHA				0.98		// Alpha value for complementary filter
#define DT					0.005f		// Delta time - 200Hz



/*###########################################################################################################################################################*/
/* Structs and enums */

// Gyro Kalman
typedef struct {
    float angle;
    float bias;
    float P[2][2];

} s_Kalman;




/*###########################################################################################################################################################*/
/* Functions */

void Complementary_deg(s_MPU6050 *dev, s_drone_data *drone);

#endif /* INC_FILTERS_H_ */
