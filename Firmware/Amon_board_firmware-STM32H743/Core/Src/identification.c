/*****************************************************************
 * File Name          : identification.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/08/04
 * Description        : Identification definitions for tests
*****************************************************************/

#include "identifications.h"
#include "PWM.h"


/*###########################################################################################################################################################*/
/* Test definitions */

const s_identification test_imu_edf = {
	.identification_start = 0,
	.identification_test_index = 0,
	.identification_time = 0,

	// Based on Python script - similar to a thrust commands
	.identification_sequence_length = 3,
	.event_time_s = {
		 2,  //  0 %: start, 0-2 s
		32,  // 50 %: warm-up,
		34,  //  0 %: stabilizing,

	},

	.edf_thrust_percent = {
		 0,
		50,
		 0
	},
	.servo_xp_deg = {0, 0, 0},
	.servo_xn_deg = {0, 0, 0},
	.servo_yp_deg = {0, 0, 0},
	.servo_yn_deg = {0, 0, 0},
};


const s_identification test_edf_offset = {
	.identification_start = 0,
	.identification_test_index = 0,
	.identification_time = 0,

	// Based on Python script - similar to a thrust commands
	.identification_sequence_length = 13,
	.event_time_s = {
		 2,  //  0 %: start, 0-2 s
		 7,  // 50 %: warm-up,          2-7 s
		12,  //  0 %: stabilizing,     7-12 s
		15,  // 80 %: measuring pulse, 12-15 s
		20,  //  0 %: stabilizing,     15-20 s
		23,  // 70 %: measuring pulse, 20-23 s
		28,  //  0 %: stabilizing,     23-28 s
		31,  // 90 %: measuring pulse, 28-31 s
		36,  //  0 %: stabilizing,     31-36 s
		39,  // 75 %: measuring pulse, 36-39 s
		44,  //  0 %: stabilizing,     39-44 s
		47,  // 85 %: measuring pulse, 44-47 s
		49   //  0 %: end stabilizing, 47-49 s
	},

	.edf_thrust_percent = {
		 0,
		50,
		 0,
		80,
		 0,
		70,
		 0,
		90,
		 0,
		75,
		 0,
		85,
		 0
	},
	.servo_xp_deg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	.servo_xn_deg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	.servo_yp_deg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	.servo_yn_deg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};


const s_identification test_edf_yaw = {
	.identification_start = 0,
	.identification_test_index = 0,
	.identification_time = 0,

	.identification_sequence_length = 13,

	.event_time_s = {
		 2,  //  0 %: start
		 7,  // 50 %: warm-up
		15,  //  0 %: reset & break

		20,  // 40 %: measurement & release
		28,  //  0 %: reset & break

		33,  // 50 %: measurement & release
		41,  //  0 %: reset & break

		46,  // 60 %: measurement & release
		54,  //  0 %: reset & break

		59,  // 75 %: measurement & release
		67,  //  0 %: reset & break

		72,  // 90 %: measurement & release
		80   //  0 %: end
	},

	.edf_thrust_percent = {
		 0,
		50,
		 0,
		40,
		 0,
		50,
		 0,
		60,
		 0,
		75,
		 0,
		90,
		 0
	},

	.servo_xp_deg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	.servo_xn_deg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	.servo_yp_deg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	.servo_yn_deg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};


const s_identification test_fin_pitch = {
	.identification_start = 0,
	.identification_test_index = 0,
	.identification_time = 0,

	.identification_sequence_length = 21,

	.event_time_s = {
		  2.00f,  //  0 %: initial state
		  7.00f,  // 50 %: warm-up
		 15.00f,  //  0 %: manual centering and insert pin

		 18.00f,  // 80 %: EDF stabilization, fins 0 deg
		 23.00f,  // +7.5/-7.5 deg: release pin after about 2 s
		 31.00f,  //  0 %: manual centering

		 34.00f,  // 80 %: stabilization
		 39.00f,  // -7.5/+7.5 deg
		 47.00f,  //  0 %: manual centering

		 50.00f,  // 80 %: stabilization
		 55.00f,  // +10/-10 deg
		 63.00f,  //  0 %: manual centering

		 66.00f,  // 80 %: stabilization
		 71.00f,  // -10/+10 deg
		 79.00f,  //  0 %: manual centering

		 82.00f,  // 80 %: stabilization
		 87.00f,  // +12.5/-12.5 deg
		 95.00f,  //  0 %: manual centering

		 98.00f,  // 80 %: stabilization
		103.00f,  // -12.5/+12.5 deg
		111.00f   //  0 %: final state
	},

	.edf_thrust_percent = {
		 0, 50, 0,

		80, 80, 0,
		80, 80, 0,
		80, 80, 0,
		80, 80, 0,
		80, 80, 0,
		80, 80, 0
	},

	.servo_xp_deg = {
		0, 0, 0,

		0,   7.5f, 0,
		0,  -7.5f, 0,
		0,  10.0f, 0,
		0, -10.0f, 0,
		0,  12.5f, 0,
		0, -12.5f, 0
	},

	.servo_xn_deg = {
		0, 0, 0,

		0,  -7.5f, 0,
		0,   7.5f, 0,
		0, -10.0f, 0,
		0,  10.0f, 0,
		0, -12.5f, 0,
		0,  12.5f, 0
	},

	.servo_yp_deg = {
		0, 0, 0,

		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
	},

	.servo_yn_deg = {
		0, 0, 0,

		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
	},
};


const s_identification test_fin_pitch_validation = {
	.identification_start = 0,
	.identification_test_index = 0,
	.identification_time = 0,

	.identification_sequence_length = 21,

	.event_time_s = {
		  2,  //  0 %: začetno stanje
		  7,  // 50 %: warm-up
		 15,  //  0 %: ročno centriranje

		 18,  // 70 %: stabilizacija, fini 0 deg
		 23,  // 70 %: xp +10, xn -10
		 31,  //  0 %: ročno centriranje

		 34,  // 70 %: stabilizacija, fini 0 deg
		 39,  // 70 %: xp -10, xn +10
		 47,  //  0 %: ročno centriranje

		 50,  // 80 %: stabilizacija, fini 0 deg
		 55,  // 80 %: xp +10, xn -10
		 63,  //  0 %: ročno centriranje

		 66,  // 80 %: stabilizacija, fini 0 deg
		 71,  // 80 %: xp -10, xn +10
		 79,  //  0 %: ročno centriranje

		 82,  // 90 %: stabilizacija, fini 0 deg
		 87,  // 90 %: xp +10, xn -10
		 95,  //  0 %: ročno centriranje

		 98,  // 90 %: stabilizacija, fini 0 deg
		103,  // 90 %: xp -10, xn +10
		111   //  0 %: končno stanje
	},

	.edf_thrust_percent = {
		 0, 50,  0,

		70, 70,  0,
		70, 70,  0,

		80, 80,  0,
		80, 80,  0,

		90, 90,  0,
		90, 90,  0
	},

	.servo_xp_deg = {
		 0,  0, 0,

		 0, 10, 0,
		 0,-10, 0,

		 0, 10, 0,
		 0,-10, 0,

		 0, 10, 0,
		 0,-10, 0
	},

	.servo_xn_deg = {
		 0,  0, 0,

		 0,-10, 0,
		 0, 10, 0,

		 0,-10, 0,
		 0, 10, 0,

		 0,-10, 0,
		 0, 10, 0
	},

	.servo_yp_deg = {
		0, 0, 0,
		0, 0, 0,  0, 0, 0,
		0, 0, 0,  0, 0, 0,
		0, 0, 0,  0, 0, 0
	},

	.servo_yn_deg = {
		0, 0, 0,
		0, 0, 0,  0, 0, 0,
		0, 0, 0,  0, 0, 0,
		0, 0, 0,  0, 0, 0
	},
};


const s_identification test_fin_roll = {
	.identification_start = 0,
	.identification_test_index = 0,
	.identification_time = 0,

	.identification_sequence_length = 27,

	.event_time_s = {
		 2.00f,  // EDF 0%: initial state
		 7.00f,  // EDF 50%: warm-up
		12.00f,  // EDF 0%: manual centering

		14.00f,  // EDF 80%: stabilization before pulse 1
		14.50f,  // pulse 1 end
		19.50f,  // EDF 0%: manual centering

		21.50f,  // stabilization before pulse 2
		22.00f,  // pulse 2 end
		27.00f,  // EDF 0%: manual centering

		29.00f,  // stabilization before pulse 3
		29.50f,  // pulse 3 end
		34.50f,  // EDF 0%: manual centering

		36.50f,  // stabilization before pulse 4
		37.00f,  // pulse 4 end
		42.00f,  // EDF 0%: manual centering

		44.00f,  // stabilization before pulse 5
		44.50f,  // pulse 5 end
		49.50f,  // EDF 0%: manual centering

		51.50f,  // stabilization before pulse 6
		52.00f,  // pulse 6 end
		57.00f,  // EDF 0%: manual centering

		59.00f,  // stabilization before pulse 7
		59.50f,  // pulse 7 end
		64.50f,  // EDF 0%: manual centering

		66.50f,  // stabilization before pulse 8
		67.00f,  // pulse 8 end
		72.00f   // EDF 0%: final state
	},

	.edf_thrust_percent = {
		 0, 50, 0,
		80, 80, 0,
		80, 80, 0,
		80, 80, 0,
		80, 80, 0,
		80, 80, 0,
		80, 80, 0,
		80, 80, 0,
		80, 80, 0
	},

	.servo_yp_deg = {
		0, 0, 0,
		0,  5, 0,
		0, -5, 0,
		0, 10, 0,
		0,-10, 0,
		0,  0, 0,
		0,  0, 0,
		0,  0, 0,
		0,  0, 0
	},

	.servo_yn_deg = {
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0,  5, 0,
		0, -5, 0,
		0, 10, 0,
		0,-10, 0
	},
};



/*###########################################################################################################################################################*/
/* Functions */


/*********************************************************************
 * @fn      Identification_RunTest
 *
 * @param   *drone: pointer to drone data struct
 *
 * @brief   Runs local test for identification use - no cables needed
 *
 * @return  Test in progress / test over
 */
uint8_t Identification_RunTest(s_drone_data *drone)
{
	s_identification *identification = &drone->identifications;

	if (!identification->identification_start || drone->DroneStatus != STATUS_FLY)
	{
		return 0;
	}

	// Check if sequence is okay or is already over
	if (identification->identification_sequence_length == 0 || identification->identification_test_index >= identification->identification_sequence_length)
	{
		identification->identification_start = 0;
		return 0;
	}

	// Test timer
	identification->identification_time += TIM_100HZ_DT;

	if (identification->identification_time >= identification->event_time_s[identification->identification_test_index])
	{
		identification->identification_test_index++;

		if (identification->identification_test_index >= identification->identification_sequence_length)
		{
			// End test with all actuators at their zero command
			identification->identification_start = 0;
			drone->actuators.edf_percent = 0;
			drone->actuators.servo_xp = 0.0f;
			drone->actuators.servo_xn = 0.0f;
			drone->actuators.servo_yp = 0.0f;
			drone->actuators.servo_yn = 0.0f;

			PowerToPWMValue(drone->actuators.edf_percent);
			DegresToCCR(drone->actuators.servo_xp, SERVO_XP);
			DegresToCCR(drone->actuators.servo_xn, SERVO_XN);
			DegresToCCR(drone->actuators.servo_yp, SERVO_YP);
			DegresToCCR(drone->actuators.servo_yn, SERVO_YN);

			drone->DroneStatus = STATUS_FLY_OVER;
			return 0;
		}

		const uint8_t index = identification->identification_test_index;
		drone->actuators.edf_percent = identification->edf_thrust_percent[index];
		drone->actuators.servo_xp = identification->servo_xp_deg[index];
		drone->actuators.servo_xn = identification->servo_xn_deg[index];
		drone->actuators.servo_yp = identification->servo_yp_deg[index];
		drone->actuators.servo_yn = identification->servo_yn_deg[index];

		PowerToPWMValue(drone->actuators.edf_percent);
		DegresToCCR(drone->actuators.servo_xp, SERVO_XP);
		DegresToCCR(drone->actuators.servo_xn, SERVO_XN);
		DegresToCCR(drone->actuators.servo_yp, SERVO_YP);
		DegresToCCR(drone->actuators.servo_yn, SERVO_YN);
	}

	return 1;

}


