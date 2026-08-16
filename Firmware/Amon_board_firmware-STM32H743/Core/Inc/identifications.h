/*
 * identifications.h
 *
 *  Created on: Aug 4, 2026
 *      Author: titit
 */

#ifndef INC_IDENTIFICATIONS_H_
#define INC_IDENTIFICATIONS_H_

#include "drone_data.h"

/*###########################################################################################################################################################*/
/* External accessible definitions */

extern const s_identification test_imu_edf;
extern const s_identification test_edf_offset;
extern const s_identification test_edf_yaw;
extern const s_identification test_fin_pitch;
extern const s_identification test_fin_pitch_validation;
extern const s_identification test_fin_roll;

/*###########################################################################################################################################################*/
/* Functions */
/* Execute one 100 Hz step of the selected identification sequence.
 * Returns 1 while a test is active in STATUS_FLY, otherwise 0. */
uint8_t Identification_RunTest(s_drone_data *drone);

#endif /* INC_IDENTIFICATIONS_H_ */
