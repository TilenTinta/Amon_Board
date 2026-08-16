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
 *  - X+ points RIGHT
 *  - Y+ points FORWARD
 *  - Z+ points UP
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

// Low-pass filter for the Kalman vertical velocity estimate. Filtering only vz
// reduces aggressive NMPC reactions without adding delay to the height estimate
#define VZ_LPF_TAU_S		0.20f
#define AZ_LPF_TAU_S        0.08f
#define AZ_PREDICT_LIMIT_M_S2 5.0f
#define AZ_BIAS_LPF_TAU_S   1.0f
#define AZ_BIAS_PROCESS_NOISE 0.002f
#define AZ_BIAS_LIMIT_M_S2   2.0f

// ToF measurement validation (Obsticals or changes in surface)
#define TOF_MEASUREMENT_DT_S             0.20f
#define TOF_MAX_VERTICAL_SPEED_M_S       0.75f
#define TOF_MEASUREMENT_MARGIN_M         0.05f
#define TOF_MAX_HEIGHT_STEP_M            ((TOF_MAX_VERTICAL_SPEED_M_S * TOF_MEASUREMENT_DT_S) + TOF_MEASUREMENT_MARGIN_M)
#define TOF_CANDIDATE_TOLERANCE_M        0.15f
#define TOF_CANDIDATE_CONFIRM_COUNT      3U

#define GYRO_DRIFT_CALIB_TIME 3.0f



/*###########################################################################################################################################################*/
/* Structs and enums */

typedef struct {
	float 		bias_corr_time;			// Current time of correction
	uint8_t		bias_corr_complete;		// Flag to indicate correction completed
	volatile uint8_t flag_bias_corr_meas;	// Flag to take new measurement
	uint16_t	bia_corr_cnt;			// Number of collected samples

	float 		gyro_x_bias;			// Corrected bias value
	float 		gyro_y_bias;			// Corrected bias value
	float 		gyro_z_bias;			// Corrected bias value

} s_gyro_correction;

// Gyro Kalman
typedef struct {
    float 		angle;   				// fi - angle
    float 		bias;    				// b - drift / offset
    float 		P00, P01, P10, P11; 	// P matrix 2x2 - confidence levels

} s_Kalman;

// ToF Kalman
typedef struct {
    float 		z;                       // Estimated height [m]
    float 		vz;                      // Estimated vertical velocity [m/s]
    float 		P[3][3];                 // State error covariance for [z, vz, accel bias]
    float 		q_accel;                 // Continuous acceleration-noise intensity [m^2/s^3]
    float 		R;                       // ToF measurement variance [m^2]
    uint8_t 	measurement_initialized; // First valid ToF measurement received
    float       accel_z_world_filtered;  // Filtered vertical acceleration in world frame [m/s^2]
    float       accel_z_world_bias;      // Bias in vertical acceleration in world frame [m/s^2]
    float       tof_candidate_z;         // Candidate height after a rejected ToF jump [m]
    uint8_t     tof_candidate_count;     // Number of mutually consistent candidate measurements
    uint8_t     tof_candidate_confirmed; // Candidate surface confirmed and rate-limited into filter
    uint32_t    tof_rejected_count;      // Rejected ToF measurement diagnostics
    uint32_t    tof_confirmed_count;     // Confirmed new-surface diagnostics

} s_KalmanZ;

typedef struct {
    float 		w, x, y, z;

} s_Quaternion;


/*###########################################################################################################################################################*/
/* Functions */

// Gyro bias correction //
void gyro_bias_correction(s_gyro_correction *gyro_corr, float gyro_x, float gyro_y, float gyro_z, float dt);

// Complementary filter //
void Complementary_deg(s_MPU6050 *dev, s_drone_data *drone);

// Kalman filter //
// Inits
void Kalman_Init(s_Kalman *k);
void KalmanZ_Init(s_KalmanZ *kf, float q_accel, float R);

// Gyro - accelerometer
void Kalman_rawToAngles(s_drone_data *drone, float *roll_angle_accel, float *pitch_angle_accel);
float Kalman_Update(s_Kalman *k, float gyro_meas, float accel_angle, float dt);

// Gyro - compass
float Kalman_UpdateYaw(s_Kalman *k, float gyro_z, float mag_yaw, uint8_t mag_valid, float dt);

// Height - ToF
void KalmanZ_Predict(s_KalmanZ *kf, float accel_z_world_m_s2, float dt);
void KalmanZ_Update(s_KalmanZ *kf, float z_measured);
uint8_t KalmanZ_ProcessToFMeasurement(s_KalmanZ *kf, float z_measured);

// First-order low-pass filter
float LowPassFilter_Update(float input, float previous_output, float dt);
float LowPassFilter_Accel(float input, float previous_output, float dt);
float AccelBodyToWorldZ(float ax_g, float ay_g, float az_g, const float q[4]);

// Helper functions
float unwrap_to_ref(float meas, float ref);
s_Quaternion eulerToQuaternion(float roll, float pitch, float yaw);
void gyroToQuaternion(s_Quaternion *q, float gyro_x_deg, float gyro_y_deg, float gyro_z_deg, float dt);
void EulerQuaternion_Complementary(s_Quaternion *q, float gyro_x_deg, float gyro_y_deg, float gyro_z_deg, float roll_deg, float pitch_deg, float yaw_deg, float dt, float alpha);


#endif /* INC_FILTERS_H_ */
