/*****************************************************************
 * File Name          : filters.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/02/01
 * Description        : Algorithms for filtering data
*****************************************************************/

#include "filters.h"

/*###########################################################################################################################################################*/
/* Private functions */
static inline void accel_normalize(float *ax, float *ay, float *az);
static inline float wrap180(float angle);
static inline float degToRad(float angle);

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

	// Axis orientation on drone are: X+ points down, Z+ points out of sensor and Y+ points right if you watch drone from the board side
	float accel_pitch = 0;
	float accel_roll = 0;
	float mag_yaw = 0;

	/* Calculate drone pitch */
	//accel_pitch = atan2f(-dev->ACCEL_Z, sqrtf(dev->ACCEL_Y * dev->ACCEL_Y +  dev->ACCEL_X * dev->ACCEL_X)) * RAD_TO_DEG;
	accel_pitch = atan2f( dev->ACCEL_Z, -dev->ACCEL_X ) * RAD_TO_DEG;

	/* Calculate drone Roll */
	//accel_roll  = atan2f(dev->ACCEL_Y, sqrtf(dev->ACCEL_Z * dev->ACCEL_Z + dev->ACCEL_X * dev->ACCEL_X)) * RAD_TO_DEG;
	accel_roll  = atan2f( dev->ACCEL_Y, -dev->ACCEL_X ) * RAD_TO_DEG;

	/* Calculate drone Yaw */
	mag_yaw = unwrap_to_ref(mag_yaw, drone->position.Yaw);

	/* Complementary Filter */
	drone->position.Pitch = ALPHA * (drone->position.Pitch + dev->GYRO_Y * DT) + (1.0f - ALPHA) * accel_pitch;
	drone->position.Roll = ALPHA * (drone->position.Roll + dev->GYRO_Z * DT) + (1.0f - ALPHA) * accel_roll;
	//drone->position.Yaw = drone->position.Yaw + dev->GYRO_X * DT;
	drone->position.Yaw = ALPHA * (drone->position.Yaw + dev->GYRO_X * DT) + (1.0f - ALPHA) * mag_yaw;
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
void Kalman_rawToAngles(s_MPU6050 *dev, float *roll_angle_accel, float *pitch_angle_accel)
{
	float ax = dev->ACCEL_X;
	float ay = dev->ACCEL_Y;
	float az = dev->ACCEL_Z;

	accel_normalize(&ax, &ay, &az); // Normalize accel vector

	// Tilt angles (standard stable formulas) - angles from gravitation
	*roll_angle_accel  = atan2f(ay, -ax) * RAD_TO_DEG;
	*pitch_angle_accel = atan2f(az, -ax) * RAD_TO_DEG;
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
	float roll_rad  = degToRad(roll);
	float pitch_rad = degToRad(pitch);
	float yaw_rad   = degToRad(yaw);

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

    return q;
}







