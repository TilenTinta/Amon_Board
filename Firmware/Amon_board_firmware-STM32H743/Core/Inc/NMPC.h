/*****************************************************************
 * File Name          : NMPC.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/05/20
 * Description        : NMPC interface for amon_model acados solver
 *
*****************************************************************/

#ifndef INC_NMPC_H_
#define INC_NMPC_H_

#include <stdint.h>
//#include "../acados/acados/acados_solver_amon_model.h"



/*###########################################################################################################################################################*/
/* Defines */

//#define COMPILE_MEM_ANALYZER	// Comment / uncomment to enable

// Dimensions - match acados_solver_amon_model.h !!!
#define NMPC_NX          13 	// AMON_MODEL_NX
#define NMPC_NU          5  	// AMON_MODEL_NU
#define NMPC_N           6	 	// AMON_MODEL_N
#define NMPC_NY          18 	// AMON_MODEL_NY
#define NMPC_NYN         13 	// AMON_MODEL_NYN

/* -----------------------------------------------------------------------
 * Input (control) bounds — from solver setup
 *   u[0]  : total thrust    [0 .. 100]	- percent
 *   u[1]  : servo x+   [-45 .. 45]	- angle [deg]
 *   u[2]  : servo x-	[-45 .. 45]	- angle [deg]
 *   u[3]  : servo y+   [-45 .. 45]	- angle [deg]
 *   u[4]  : servo y- 	[-45 .. 45]	- angle [deg]
 * --------------------------------------------------------------------- */
#define NMPC_U0_MIN   0.0
#define NMPC_U0_MAX   90.0
#define NMPC_UX_MIN  -45.0
#define NMPC_UX_MAX   45.0

/* -----------------------------------------------------------------------
 * Return codes
 * --------------------------------------------------------------------- */
#define NMPC_OK         0
#define NMPC_FAIL       1
#define NMPC_NOT_INIT   2
#define NMPC_SOLVER_ERR 3   // if solver returned non-zero but we still have u


/*###########################################################################################################################################################*/
/* Structs and enums */

typedef struct
{
    // Current state fed into the solver
    double 		x0[NMPC_NX];

    // Last optimal control output u[0]
    double 		u_opt[NMPC_NU];

    // Reference trajectory (same ref applied to all N stages)
    double 		x_ref[NMPC_NX];     		// state  reference (used in yref[0..22])
    double 		u_ref[NMPC_NU];     		// input  reference (used in yref[23..27])

    // Diagnostics
    uint32_t 	solve_time_ms;     			// wall-clock solve time in ms
    int      	last_solver_status;			// raw acados return code
    uint32_t 	solve_count;       			// total successful solves

    // Internal flags
    uint8_t  	initialized;
    float 		nmpc_solve_time_arr[10];	// Array of last 10 times used to solve NMPC
    uint8_t 	nmpc_solve_time_cnt;
    uint8_t 	nmpc_solve_time_samples;
    float 		nmpc_solve_time_sum;
    float 		nmpc_solve_time;

    uint8_t 	warm_start_valid;
    double 		warm_x[NMPC_N + 1][NMPC_NX];
    double 		warm_u[NMPC_N][NMPC_NU];

} s_NMPC;



/*###########################################################################################################################################################*/
/* Functions */

#ifdef __cplusplus
extern "C" {
#endif

int NMPC_Init(s_NMPC *h);
int NMPC_SetState(s_NMPC *h, const double *x);
int NMPC_SetReference(s_NMPC *h, const double *x_ref, const double *u_ref);
int NMPC_Solve(s_NMPC *h);
void NMPC_GetControl(const s_NMPC *h, double *u_out);
void NMPC_DeInit(s_NMPC *h);

#ifdef __cplusplus
}
#endif


#endif /* INC_NMPC_H_ */
