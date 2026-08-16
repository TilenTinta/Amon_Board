/*****************************************************************
 * File Name          : filters.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/02/01
 * Description        : Algorithms for filtering data
*****************************************************************/

#include "filters.h"
#include <math.h>
#include <string.h>

/*###########################################################################################################################################################*/
/* Private functions */
static inline void accel_normalize(float *ax, float *ay, float *az);
static inline float wrap180(float angle);
static inline float degToRad(float angle);


/*###########################################################################################################################################################*/
/* Low-pass filter */

/*********************************************************************
* @fn     	LowPassFilter_Update
*
* @param	input: current unfiltered input
* @param	previous_output: filter output from the previous update
* @param	dt: update period [s]
*
* @brief   	Updates a first-order low-pass filter. A larger tau gives
* 			more smoothing but also adds more delay.
*
* @return  	Updated filtered output
*/
float LowPassFilter_Update(float input, float previous_output, float dt)
{
	if (VZ_LPF_TAU_S <= 0.0f)
	{
		return input;
	}

	if (dt <= 0.0f)
	{
		return previous_output;
	}

	const float alpha = dt / (VZ_LPF_TAU_S + dt);
	return previous_output + alpha * (input - previous_output);
}

/*********************************************************************
* @fn     	LowPassFilter_Accel
*
* @param	input: current unfiltered input
* @param	previous_output: filter output from the previous update
* @param	dt: update period [s]
*
* @brief   	Updates a first-order low-pass filter for vertical acceleration
*
* @return  	Updated filtered output
*/
float LowPassFilter_Accel(float input, float previous_output, float dt)
{
    if (AZ_LPF_TAU_S <= 0.0f)
    {
        return input;
    }

    if (dt <= 0.0f)
    {
        return previous_output;
    }

    const float alpha = dt / (AZ_LPF_TAU_S + dt);
    return previous_output + alpha * (input - previous_output);
}


/*********************************************************************
* @fn     	gyro_bias_correction
*
* @param	*drone: pointer to all drone data
* @param	*gyro_corr: pointer to all gyro correction data
*
* @brief   	Checks raw data from gyros, once time is reached it calculates average drift
*
* @return  	gyro bias correction status
*/
void gyro_bias_correction(s_gyro_correction *gyro_corr, float gyro_x, float gyro_y, float gyro_z, float dt)
{
	gyro_corr->gyro_x_bias += gyro_x;
	gyro_corr->gyro_y_bias += gyro_y;
	gyro_corr->gyro_z_bias += gyro_z;

	gyro_corr->bia_corr_cnt++;
	gyro_corr->bias_corr_time += dt;

	if (gyro_corr->bias_corr_time >= GYRO_DRIFT_CALIB_TIME)
	{
		gyro_corr->gyro_x_bias /= gyro_corr->bia_corr_cnt;
		gyro_corr->gyro_y_bias /= gyro_corr->bia_corr_cnt;
		gyro_corr->gyro_z_bias /= gyro_corr->bia_corr_cnt;

		gyro_corr->bias_corr_complete = 1;
	}
	else
	{
		gyro_corr->bias_corr_complete = 0;
	}
}


/*********************************************************************
* @fn     	accel_body_to_world_z_m_s2
*
* @param	ax_g: acceleration in body frame x-axis [g]
* @param	ay_g: acceleration in body frame y-axis [g]
* @param	az_g: acceleration in body frame z-axis [g]
* @param	q: quaternion representing the rotation from body frame to world frame
*
* @brief   	Transforms the acceleration from body frame to world frame and returns the vertical acceleration in m/s^2.
*
* @return  	Vertical acceleration in world frame z-axis [m/s^2]
*/
float AccelBodyToWorldZ(float ax_g, float ay_g, float az_g, const float q[4])
{
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    const float az_world_g = 2.0f * (qx*qz - qw*qy) * ax_g + 2.0f * (qy*qz + qw*qx) * ay_g + (1.0f - 2.0f * (qx*qx + qy*qy)) * az_g;

    return (az_world_g - 1.0f) * 9.80665f;
}

/*###########################################################################################################################################################*/
/* Complementary filter */

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

	// Axis orientation on drone are: X+ points right, Y+ points forward, Z+ points up
	float accel_pitch = 0;
	float accel_roll = 0;
	float mag_yaw = 0;

	/* Calculate drone pitch */
	//accel_pitch = atan2f(-drone->position.accel_x, sqrtf(drone->position.accel_y * drone->position.accel_y +  drone->position.accel_z * drone->position.accel_z)) * RAD_TO_DEG;
	accel_pitch = atan2f( drone->position.accel_x, drone->position.accel_z ) * RAD_TO_DEG;

	/* Calculate drone Roll */
	//accel_roll  = atan2f(-drone->position.accel_y, sqrtf(drone->position.accel_x * drone->position.accel_x + drone->position.accel_z * drone->position.accel_z)) * RAD_TO_DEG;
	accel_roll  = atan2f( -drone->position.accel_y, drone->position.accel_z ) * RAD_TO_DEG;

	/* Calculate drone Yaw */
	mag_yaw = unwrap_to_ref(mag_yaw, drone->position.Yaw);

	/* Complementary Filter */
	drone->position.Pitch = ALPHA * (drone->position.Pitch + drone->position.gyro_y * DT) + (1.0f - ALPHA) * accel_pitch;
	drone->position.Roll = ALPHA * (drone->position.Roll + drone->position.gyro_x * DT) + (1.0f - ALPHA) * accel_roll;
	//drone->position.Yaw = drone->position.Yaw + drone->position.gyro_z * DT;
	drone->position.Yaw = ALPHA * (drone->position.Yaw + drone->position.gyro_z * DT) + (1.0f - ALPHA) * mag_yaw;
}


/*###########################################################################################################################################################*/
/* Kalman filter */
/*
 * Frequency:
 * 	200Hz - current settings -> OK
 * 	400Hz - Better settings
 *
 * State vector:
 *  x = [fi b]^T
 * 		fi - angle (deg), changes over time
 * 		b - gyro bias (deg/s), slowly changes - bias drift
 *
 * 	Input:
 * 	 omega_meas - gyro rate (deg/s)
 * 	 omega_true = omega_meas - b
 *
 * 	Measurement:
 * 	 z = fi_acc (deg) - noisy measurement of fi
 *
 * 	State transition:
 * 	 fi_k = fi_k-1 + (Omega_meas - b_k-1) * dt
 * 	 b_k = b_k-1
 *
 * State Transition Equation / Process model:
 * 	x_k = F*x_k-1 + B*u_k + w_k
 * 		x_k - current state
 * 		F - state transition matrix
 * 		u_k - input (gyro measurement)
 * 		B - control input matrix
 * 		w_k - process noise (model uncertainty)
 *
 * 		State transition matrix F:
 *			F = [1 -dt]
 *	    		[0   1]
 *					fi - dopands on previous fi and b
 *					b - stays the same
 *					-dt - fi decreases if bias is positive
 *
 * 		Input u:
 *	 		u = omega_meas (deg/s)
 *
 * 		Control input matrix B:
 *			B = [dt]
 *	    		[ 0]
 *				omega_meas must contribute to fi over dt
 *
 * Process Noise / Process noise covariance Q (uncertainty):
 * 	Q = [Q_fi   0]
 *		[  0  Q_b]
 *		Q_fi - filter trusts gyro prediction less
 *		Q_b - filter adapts bias faster
 *
 * Covariance Matrix / Estimation error covariance P:
 *	 P = [P_fi_fi  P_fi_b]
 *	 	 [P_b_fi    P_b_b]
 *	 	 P_fi_fi - uncertainty of angle estimate
 *	 	 P_b_b - uncertainty of bias estimate
 *	 	 diagonal values - corelation between error of fi and b
 *
 * State prediction / Predicted State:
 *	 x_k^- = F x_k-1 + B u_k
 *
 * Covariance prediction / Predicted Covariance:
 *  P_k^- = F P_k-1 F^T + Q
 *  	- transform the old uncertainty through the motion model
 *  	- add process noise Q
 *
 * ########### Measurement Update (Correction step) ###########
 * Measurement model / Measurement Equation:
 *  z_k = H x_k + v_k
 *  	z_k - measured accel angle (deg)
 *  	H - measurement matrix
 *  	v_k - measurement noise
 *
 * Measurement matrix H / Observation Matrix:
 * 	H = [1 0]
 * 		we measure only fi
 *
 * Innovation / Measurement Residua:
 * 	difference between reality and prediction
 * 	y = z - Hx^-
 * 		y = 0: prediction is perfect
 *		y > 0: predicted angle too small
 * 		y < 0: predicted angle too big
 *
 * 	Innovation covariance:
 * 	 S = HP^-H^T + R
 * 	 	P^- = predicted covariance
 * 	 	R = measurement noise variance
 *
 * 	 	Large S: don’t trust measurement much
 *		Small S: measurement is reliable
 *
 *		High vibration -> increase R
 * 		Quiet hover -> lower R
 *
 * Kalman Gain / Optimal Gain:
 * 	K = P^- H^T S^-1
 * 	equal as
 * 	K = [P_00^- / S]
 * 		[P_10^- / S]
 *
 * 		K_0: how much we correct the angle
 *		K_1: how much we correct the bias
 *
 * State update / Posterior State Estimate
 *  c = x^- + Ky
 *   this is:
 *   	angle = angle + K0 * innovation	(Angle correction)
 *		bias  = bias  + K1 * innovation	(If accel constantly disagrees -> gyro bias must be wrong -> automatic gyro bias calibration)
 *
 * Covariance update / Posterior Covariance:
 *  P = (I - KH)P^-
 *
 * Known problems:
 * 	- True gimbal lock at +-90deg pitch
 *	- Euler rate coupling
 *
 * */

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
    k->angle = 0.0f; // start with 0 degress
    k->bias  = 0.0f; // no initial bias

    // Start with uncertainty
	k->P00 = 1.0f;	// High uncertainty of angles - trust measurement at first
	k->P01 = 0.0f;  // No init correlation between angles and bias
	k->P10 = 0.0f;	// No init correlation between bias and angles
	k->P11 = 1.0f;	// High uncertainty of bias - trust measurement at first

	// high uncertainty (1.0) means the filter will trust early measurements more than its own predictions
}


/*********************************************************************
* @fn     	KalmanZ_Init
*
* @param 	*kz: struct to kalman filter data
* @param 	 q_accel: continuous acceleration-noise intensity [m^2/s^3]
* @param 	 R: ToF measurement variance [m^2]
*
* @brief   Kalman filter initialization - ToF
*
* @return  None
*/
void KalmanZ_Init(s_KalmanZ *kf, float q_accel, float R)
{
	kf->z = 0.0f;
	kf->vz = 0.0f;
	kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f; kf->P[0][2] = 0.0f;
	kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f; kf->P[1][2] = 0.0f;
	kf->P[2][0] = 0.0f; kf->P[2][1] = 0.0f; kf->P[2][2] = 0.25f;
	kf->q_accel = q_accel;
	kf->R = R;
	kf->measurement_initialized = 0;
	kf->accel_z_world_filtered = 0.0f;
	kf->accel_z_world_bias = 0.0f;
	kf->tof_candidate_z = 0.0f;
	kf->tof_candidate_count = 0;
	kf->tof_candidate_confirmed = 0;
	kf->tof_rejected_count = 0;
	kf->tof_confirmed_count = 0;
}


 /*********************************************************************
 * @fn     	Kalman_Update
 *
 * @param 	*k: struct to kalman filter data
 * @param	accel_angle: acceleration for some axis
 * @param	gyro_meas: gyro angle for some axis
 * @param 	dt: delta time - depends on timer
 *
 * @brief   	Kalman calculation
 *
 * @return  	angle for calculated axis
 */
void Kalman_rawToAngles(s_drone_data *drone, float *roll_angle_accel, float *pitch_angle_accel)
{
	float ax = drone->position.accel_x;
	float ay = drone->position.accel_y;
	float az = drone->position.accel_z;

	accel_normalize(&ax, &ay, &az); // Normalize accel vector

	// Tilt angles (standard stable formulas) - angles from gravitation
	*pitch_angle_accel = -atan2f(-ay, az) * RAD_TO_DEG;  // angle around body X
	*roll_angle_accel = atan2f( ax, az) * RAD_TO_DEG;  // angle around body Y
}



/*********************************************************************
* @fn     	Kalman_Update
*
* @param 	*k: struct to kalman filter data
* @param	accel_angle: acceleration for some axis
* @param	gyro_meas: gyro angle for some axis
* @param 	dt: delta time - depends on timer
*
* @brief   	Kalman calculation
*
* @return  	angle for calculated axis
*/
float Kalman_Update(s_Kalman *k, float gyro_meas, float accel_angle, float dt)
{

	// Q_angle		How much expect the angle to change unpredictably / filter response to gyro, Range: 0.0001 - 0.01
	// Q_bias		How quickly gyro bias can change / how fast filter learns bias, Range: 0.001 - 0.01
	// R_measure	How noisy accelerometer is / how much to trust accelerometer, range: 0.01 - 0.5

	/*
	V Kalmanovem filtru imaš dve ključni matriki, ki ju moraš nastaviti:

	Q – procesni šum
	R – merilni šum

	Q opisuje, kako zelo dvomiš v svoj model. Če veš, da je giroskop slab, povečaš Q. Če veš, da je model zelo natančen, zmanjšaš Q.

	R opisuje, kako šumna je meritev. Če je pospeškomer v vibracijskem okolju, povečaš R.

	To sta tuning parametra. Ne nastavljaš “ojačevalnika”, ampak fizični pomen šuma.

	Če povečaš Q → filter postane bolj odziven na meritve.
	Če povečaš R → filter postane bolj gladek in zaupa modelu.

	Zelo pomembno: če Q ali R nastaviš napačno, filter ne bo nestabilen v klasičnem smislu (kot PID), ampak bo ali preveč počasen ali preveč šumen.
	*/


	// --- Predicted State (x̂⁻) ---
	// θ = θ + (ω - b) dt
	k->angle += (gyro_meas - k->bias) * dt;

	// --- Predicted Covariance (P^- = F P F^T + Q)  ---
	const float Q_angle = 0.001f;	// Process noise for angle - TUNE!
	const float Q_bias  = 0.0001f;	// Process noise for bias - TUNE!

	float P00 = k->P00;
	float P01 = k->P01;
	float P10 = k->P10;
	float P11 = k->P11;

	// This is the expanded matrix math (no matrices at runtime)
	// P⁻ = F P Fᵀ + Q, F = [1 -dt; 0 1], Q = [Q_angle 0, 0 Q_bias]
	k->P00 = P00 + dt * (dt*P11 - P01 - P10 + Q_angle);
	k->P01 = P01 - dt * P11;
	k->P10 = P10 - dt * P11;
	k->P11 = P11 + Q_bias * dt;

	// --- Measurement Noise (R) ---
	const float R_measure = 0.01f;	// Measurement noise - TUNE!

	// --- Innovation / Surprise factor ---
	// y = z - Hx̂⁻
	float innovation = accel_angle - k->angle;

	// --- Innovation Covariance (S) ---
	// S = H P Hᵀ + R, H = [1 0] -> only P00
	float S = k->P00 + R_measure;	// Total uncertainty

	// --- Kalman Gain (K) ---
	// K = P Hᵀ S⁻¹
	float K0 = k->P00 / S;			// Gain for angle correction
	float K1 = k->P10 / S;			// Gain for bias correction

	// --- Posterior State Update ---
	// x = x̂⁻ + K y
	k->angle += K0 * innovation;	// Adjust angle estimate
	k->bias  += K1 * innovation;	// Adjust bias estimate

	// --- Posterior Covariance Update ---
	// P = (I - K H) P
	float P00_temp = k->P00;
	float P01_temp = k->P01;

	k->P00 -= K0 * P00_temp;
	k->P01 -= K0 * P01_temp;
	k->P10 -= K1 * P00_temp;
	k->P11 -= K1 * P01_temp;

	return k->angle;
}



/*********************************************************************
* @fn     	Kalman_Update
*
* @param 	*k: struct to kalman filter data
* @param	gyro_z: gyzo for this axis
* @param	mag_yaw: compas yaw value
* @param	mag_valid: use compass or not
* @param 	dt: delta time - depends on timer
*
* @brief   	Kalman calculation for yaw rotation
*
* @return  	angle for calculated axis
*/
float Kalman_UpdateYaw(s_Kalman *k, float gyro_z, float mag_yaw, uint8_t mag_valid, float dt)
{
    const float Q_angle = 0.001f;
    const float Q_bias  = 0.0005f;
    const float R_mag   = 4.0f;

    // Predict every cycle
    k->angle += (gyro_z - k->bias) * dt;

    float P00 = k->P00;
    float P01 = k->P01;
    float P10 = k->P10;
    float P11 = k->P11;

    k->P00 = P00 + dt * (dt * P11 - P01 - P10) + Q_angle * dt;
    k->P01 = P01 - dt * P11;
    k->P10 = P10 - dt * P11;
    k->P11 = P11 + Q_bias * dt;

    if (mag_valid)
    {
        mag_yaw = unwrap_to_ref(mag_yaw, k->angle);

        float innovation = mag_yaw - k->angle;

        if (innovation > 180.0f) innovation -= 360.0f;
        if (innovation < -180.0f) innovation += 360.0f;

        // Reject magnetic disturbances instead of pulling yaw toward a bad sample.
        if (fabsf(innovation) < 45.0f)
        {
            float S = k->P00 + R_mag;
            float K0 = k->P00 / S;
            float K1 = k->P10 / S;

            k->angle += K0 * innovation;
            k->bias  += K1 * innovation;

            float P00_temp = k->P00;
            float P01_temp = k->P01;

            k->P00 -= K0 * P00_temp;
            k->P01 -= K0 * P01_temp;
            k->P10 -= K1 * P00_temp;
            k->P11 -= K1 * P01_temp;
        }
    }

    if (k->angle > 180.0f)  k->angle -= 360.0f;
    if (k->angle < -180.0f) k->angle += 360.0f;

    return k->angle;
}



/*********************************************************************
* @fn     	KalmanZ_Predict
*
* @param 	*kf: struct to kalman filter data
* @param	accel_z_world_m_s2: vertical acceleration in world frame [m/s^2]
* @param	dt: delta time of filter
*
* @brief   	Kalman prediction step - Call every xxxHz 
*
* @return  	predicted height
*/
void KalmanZ_Predict(s_KalmanZ *kf, float accel_z_world_m_s2, float dt)
{
    if (!kf->measurement_initialized)
    {
        return;
    }

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float accel_corrected = accel_z_world_m_s2 - kf->accel_z_world_bias;

    /*
     * x = [z, vz, accel_bias]
     *
     * z(k+1)  = z(k) + vz(k)*dt + 0.5*az*dt²
     * vz(k+1) = vz(k) + az*dt
     */
    kf->z  += kf->vz * dt + 0.5f * accel_corrected * dt2;
    kf->vz += accel_corrected * dt;

    const float F[3][3] = {
        {1.0f, dt, -0.5f * dt2},
        {0.0f, 1.0f, -dt},
        {0.0f, 0.0f, 1.0f}
    };
    float FP[3][3] = {0};
    float P_new[3][3] = {0};

    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            for (uint8_t k = 0; k < 3; k++)
            {
                FP[i][j] += F[i][k] * kf->P[k][j];
            }
        }
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            for (uint8_t k = 0; k < 3; k++)
            {
                P_new[i][j] += FP[i][k] * F[j][k];
            }
        }
    }

    const float Q00 = kf->q_accel * dt3 / 3.0f;
    const float Q01 = kf->q_accel * dt2 / 2.0f;
    const float Q11 = kf->q_accel * dt;

    P_new[0][0] += Q00;
    P_new[0][1] += Q01;
    P_new[1][0] += Q01;
    P_new[1][1] += Q11;
    P_new[2][2] += AZ_BIAS_PROCESS_NOISE * dt;

    memcpy(kf->P, P_new, sizeof(kf->P));
}



/*********************************************************************
* @fn     	KalmanZ_Update
*
* @param 	*kf: struct to kalman filter data
* @param	z_measured: actual measured height
*
* @brief   	Kalman correction step - Call only
* 			when new meassurement is available
*
* @return  	corrected height
*/
void KalmanZ_Update(s_KalmanZ *kf, float z_measured)
{
    // Initialize height directly from the first valid ToF measurement.
    if (!kf->measurement_initialized)
    {
        kf->z = z_measured;
        kf->vz = 0.0f;
        kf->P[0][0] = kf->R; kf->P[0][1] = 0.0f; kf->P[0][2] = 0.0f;
        kf->P[1][0] = 0.0f;  kf->P[1][1] = 1.0f; kf->P[1][2] = 0.0f;
        kf->P[2][0] = 0.0f;  kf->P[2][1] = 0.0f; kf->P[2][2] = 0.25f;
        kf->measurement_initialized = 1;
        return;
    }

    // Innovation
    float y = z_measured - kf->z;

    // Innovation covariance
    float S = kf->P[0][0] + kf->R;

    // Kalman gain
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;
    float K2 = kf->P[2][0] / S;

    // State update
    kf->z  += K0 * y;
    kf->vz += K1 * y;
    kf->accel_z_world_bias += K2 * y;

    if (kf->accel_z_world_bias > AZ_BIAS_LIMIT_M_S2) kf->accel_z_world_bias = AZ_BIAS_LIMIT_M_S2;
    if (kf->accel_z_world_bias < -AZ_BIAS_LIMIT_M_S2) kf->accel_z_world_bias = -AZ_BIAS_LIMIT_M_S2;

    // Covariance update: P = (I - K*H) * P
    float P_row_0[3] = {kf->P[0][0], kf->P[0][1], kf->P[0][2]};
    float K[3] = {K0, K1, K2};

    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf->P[i][j] -= K[i] * P_row_0[j];
        }
    }
}



/*********************************************************************
* @fn     KalmanZ_ProcessToFMeasurement
*
* @param  *kf: struct to kalman filter data
* @param  z_measured: new ToF height measurement [m]
*
* @brief  Validate a ToF measurement against the physically reachable
*         height change. A larger jump must be confirmed with multiple
*         measurements before it is accepted.
*
* @return 1: measurement accepted, 0: measurement rejected
*/
uint8_t KalmanZ_ProcessToFMeasurement(s_KalmanZ *kf, float z_measured)
{
    if (kf == NULL || !isfinite(z_measured) || z_measured < 0.0f)
    {
        return 0;
    }

    // The first valid ToF measurement initializes the filter directly.
    if (!kf->measurement_initialized)
    {
        KalmanZ_Update(kf, z_measured);
        kf->tof_candidate_count = 0;
        kf->tof_candidate_confirmed = 0;
        return 1;
    }

    float innovation = z_measured - kf->z;

    // A physically reachable measurement is accepted immediately.
    if (fabsf(innovation) <= TOF_MAX_HEIGHT_STEP_M)
    {
        KalmanZ_Update(kf, z_measured);
        kf->tof_candidate_count = 0;
        kf->tof_candidate_confirmed = 0;
        return 1;
    }

    kf->tof_rejected_count++;

    // Start a new candidate if this measurement does not agree with previous surface.
    if (kf->tof_candidate_count == 0 || fabsf(z_measured - kf->tof_candidate_z) > TOF_CANDIDATE_TOLERANCE_M)
    {
        kf->tof_candidate_z = z_measured;
        kf->tof_candidate_count = 1;
        kf->tof_candidate_confirmed = 0;
        return 0;
    }

    // Once confirmed, move toward the persistent surface only by one
    // physically reachable height step per ToF sample.
    if (kf->tof_candidate_confirmed)
    {
        kf->tof_candidate_z += 0.25f * (z_measured - kf->tof_candidate_z);
        float limited_measurement = kf->z;

        if (kf->tof_candidate_z > kf->z)
        {
            limited_measurement += TOF_MAX_HEIGHT_STEP_M;
        }
        else
        {
            limited_measurement -= TOF_MAX_HEIGHT_STEP_M;
        }

        KalmanZ_Update(kf, limited_measurement);
        return 1;
    }

    // Running average prevents one noisy candidate from defining the new surface.
    kf->tof_candidate_count++;
    kf->tof_candidate_z += (z_measured - kf->tof_candidate_z) / (float)kf->tof_candidate_count;

    if (kf->tof_candidate_count < TOF_CANDIDATE_CONFIRM_COUNT)
    {
        return 0;
    }

    // Strange surface can be a real anomaly or landing.
    kf->tof_candidate_confirmed = 1;
    float limited_measurement = kf->z;
    if (kf->tof_candidate_z > kf->z)
    {
        limited_measurement += TOF_MAX_HEIGHT_STEP_M;
    }
    else
    {
        limited_measurement -= TOF_MAX_HEIGHT_STEP_M;
    }
    KalmanZ_Update(kf, limited_measurement);
    kf->tof_confirmed_count++;
    return 1;
}


/*********************************************************************
* @fn     	accel_normalize
*
* @param 	*ax: acceleration value for x axis
* @param 	*ay: acceleration value for y axis
* @param 	*az: acceleration value for z axis
*
* @brief   	normalize acceleration values - stabilization from vibrations
* 			- Small magnitude changes produce angle noise
*			- Normalization removes magnitude from the equation.
*
* @return  	accelerometer values per angle
*/
static inline void accel_normalize(float *ax, float *ay, float *az)
{
    float n = sqrtf((*ax)*(*ax) + (*ay)*(*ay) + (*az)*(*az));

    if (n < 1e-6f) return; // divide by 0 protection

    // Normalization
    *ax /= n;
    *ay /= n;
    *az /= n;
}



/*********************************************************************
* @fn     	wrap180
*
* @param 	angles
*
* @brief   	Wrap angles values if they are at their limit
*
* @return  	angle
*/
static inline float wrap180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}



/*********************************************************************
* @fn     	unwrap_to_ref
*
* @param 	meas: normalized raw values - measured
* @param 	ref: current value of angle
*
* @brief   	Wrap angles values if they are at their limit
* 			- removes Euler angle discontinuity -> Kalman inovation value will not explode
*
* @return  	new measured value
*/
float unwrap_to_ref(float meas, float ref)
{
    meas = wrap180(meas);
    ref  = wrap180(ref);
    float d = meas - ref;
    if (d > 180.0f)  meas -= 360.0f;
    if (d < -180.0f) meas += 360.0f;
    return meas;
}



/*********************************************************************
* @fn     	degToRad
*
* @param 	angles - angle in degrees
*
* @brief   	Convert angle from degress to radians
*
* @return  	angle - [rad]
*/
static inline float degToRad(float angle)
{
    return angle * (M_PI / 180.0f);
}



/*********************************************************************
* @fn     	eulerToQuaternion
*
 * @param 	roll  - rotation around X axis [rad]
 * @param 	pitch - rotation around Y axis [rad]
 * @param 	yaw   - rotation around Z axis [rad]
*
* @brief  	Convert euler angles to quaternion
*
* @return  	quaternion
*/
s_Quaternion eulerToQuaternion(float roll, float pitch, float yaw)
{
//	float roll_rad  = degToRad(roll);
//	float pitch_rad = degToRad(pitch);
//	float yaw_rad   = degToRad(yaw);

	float roll_rad  = roll  * DEG_TO_RAD;
	float pitch_rad = pitch * DEG_TO_RAD;
	float yaw_rad   = yaw   * DEG_TO_RAD;

//	q_roll  = [cos(r/2),  sin(r/2),  0,         0        ]
//	q_pitch = [cos(p/2),  0,         sin(p/2),  0        ]
//	q_yaw   = [cos(y/2),  0,         0,         sin(y/2) ]

    float cr = cosf(roll_rad  * 0.5f);
    float sr = sinf(roll_rad  * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);
    float cy = cosf(yaw_rad   * 0.5f);
    float sy = sinf(yaw_rad   * 0.5f);

    s_Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;  // roll
    q.y = cr * sp * cy + sr * cp * sy;  // pitch
    q.z = cr * cp * sy - sr * sp * cy;  // yaw

//    float sign = q.w * q_prev.w + q.x * q_prev.x + q.y * q_prev.y + q.z * q_prev.z;
//
//    if (sign < 0.0f)
//    {
//        q.w = -q.w;
//        q.x = -q.x;
//        q.y = -q.y;
//        q.z = -q.z;
//    }

    // Normalization
    float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);

    if (norm > 1e-6f)
	{
		q.w = q.w / norm;
		q.x = q.x / norm;
		q.y = q.y / norm;
		q.z = q.z / norm;
	}
    else
    {
    	q.w = 1.0f;
		q.x = 0.0f;
		q.y = 0.0f;
		q.z = 0.0f;
    }

    return q;
}



/*********************************************************************
* @fn     	gyroToQuaternion - OLD
*
* @param 	gyro_x_deg  - rotation around X axis [deg/s]
* @param 	gyro_y_deg  - rotation around Y axis [deg/s]
* @param 	gyro_z_deg  - rotation around Z axis [deg/s]
* @param 	dt			- delta time of filter
*
* @brief  	Convert raw gyro data to quaternion
*
* @return  	None
*/
void gyroToQuaternion(s_Quaternion *q, float gyro_x_deg, float gyro_y_deg, float gyro_z_deg, float dt)
{
    // Convert deg/s -> rad/s
    float wx = gyro_x_deg * DEG_TO_RAD;
    float wy = gyro_y_deg * DEG_TO_RAD;
    float wz = gyro_z_deg * DEG_TO_RAD;

    float qw = q->w;
    float qx = q->x;
    float qy = q->y;
    float qz = q->z;

    // Quaternion derivative
    float dq_w = 0.5f * (-qx*wx - qy*wy - qz*wz);
    float dq_x = 0.5f * ( qw*wx + qy*wz - qz*wy);
    float dq_y = 0.5f * ( qw*wy - qx*wz + qz*wx);
    float dq_z = 0.5f * ( qw*wz + qx*wy - qy*wx);

    // Integrate
    q->w += dq_w * dt;
    q->x += dq_x * dt;
    q->y += dq_y * dt;
    q->z += dq_z * dt;

    // Normalize
    float norm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);

    if(norm > 1e-6f)
    {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
}



/*********************************************************************
* @fn      EulerQuaternion_Complementary
*
* @param   q               - quaternion state
* @param   gyro_x_deg      - gyro X [deg/s]
* @param   gyro_y_deg      - gyro Y [deg/s]
* @param   gyro_z_deg      - gyro Z [deg/s]
* @param   roll_deg        - Kalman roll angle [deg]
* @param   pitch_deg       - Kalman pitch angle [deg]
* @param   yaw_deg         - Kalman yaw angle [deg]
* @param   dt              - delta time [s]
* @param   alpha           - complementary factor
*
* @brief   Quaternion complementary filter
*           - gyro integration -> fast response
*           - Euler quaternion -> drift correction
*
* @return   None
*/
void EulerQuaternion_Complementary(s_Quaternion *q, float gyro_x_deg,  float gyro_y_deg, float gyro_z_deg, float roll_deg, float pitch_deg, float yaw_deg, float dt, float alpha)
{
    // Propagate quaternion using gyro
    //float wx = -gyro_x_deg * DEG_TO_RAD;
    float wx = gyro_x_deg * DEG_TO_RAD;
    float wy = gyro_y_deg * DEG_TO_RAD;
    float wz = gyro_z_deg * DEG_TO_RAD;

    float qw = q->w;
    float qx = q->x;
    float qy = q->y;
    float qz = q->z;

    // Quaternion derivative
    float dq_w = 0.5f * (-qx*wx - qy*wy - qz*wz);
    float dq_x = 0.5f * ( qw*wx + qy*wz - qz*wy);
    float dq_y = 0.5f * ( qw*wy - qx*wz + qz*wx);
    float dq_z = 0.5f * ( qw*wz + qx*wy - qy*wx);

    // Integrate
    s_Quaternion q_gyro;

    q_gyro.w = qw + dq_w * dt;
    q_gyro.x = qx + dq_x * dt;
    q_gyro.y = qy + dq_y * dt;
    q_gyro.z = qz + dq_z * dt;

    // Normalize quaternion
    float norm = sqrtf(q_gyro.w*q_gyro.w + q_gyro.x*q_gyro.x + q_gyro.y*q_gyro.y + q_gyro.z*q_gyro.z);

    if(norm > 1e-6f)
    {
        q_gyro.w /= norm;
        q_gyro.x /= norm;
        q_gyro.y /= norm;
        q_gyro.z /= norm;
    }

    // Create reference quaternion from Kalman Euler
    s_Quaternion q_ref = eulerToQuaternion(pitch_deg, roll_deg, yaw_deg);

    // Quaternion hemisphere correction (prevent interpolation jumps)

    float dot = q_gyro.w * q_ref.w + q_gyro.x * q_ref.x + q_gyro.y * q_ref.y + q_gyro.z * q_ref.z;

    if(dot < 0.0f)
    {
        q_ref.w = -q_ref.w;
        q_ref.x = -q_ref.x;
        q_ref.y = -q_ref.y;
        q_ref.z = -q_ref.z;
    }

    // Complementary filter
    q->w = alpha * q_gyro.w + (1.0f - alpha) * q_ref.w;
    q->x = alpha * q_gyro.x + (1.0f - alpha) * q_ref.x;
    q->y = alpha * q_gyro.y + (1.0f - alpha) * q_ref.y;
    q->z = alpha * q_gyro.z + (1.0f - alpha) * q_ref.z;

    // Normalization
    norm = sqrtf(q->w*q->w + q->x*q->x +  q->y*q->y + q->z*q->z);

    if(norm > 1e-6f)
    {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
    else
    {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
    }
}






