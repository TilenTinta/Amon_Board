/*****************************************************************
 * File Name          : NMPC.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/05/20
 * Description        : NMPC interface for amon_model acados solver
 *
 * Model dimensions (from acados_solver_amon_model.h):
 *   NX = 23  states
 *   NU = 5   inputs:  u[0]=thrust [0..100], u[1..4]=angle_cmds [-45..45]
 *   N  = 10  horizon
 *   NY = 28  cost output: [x(23); u(5)]
 *   NYN= 23  terminal cost output: [x(23)]
 *
*****************************************************************/

#ifndef INC_NMPC_H_
#define INC_NMPC_H_

#include <stdint.h>
#include "../acados/acados/acados_solver_amon_model.h"



/*###########################################################################################################################################################*/
/* Defines */

// Dimensions - match acados_solver_amon_model.h
#define NMPC_NX     AMON_MODEL_NX       // 23
#define NMPC_NU     AMON_MODEL_NU       // 5
#define NMPC_N      AMON_MODEL_N        // 10
#define NMPC_NY     AMON_MODEL_NY       // 28 = NX+NU, stage cost size
#define NMPC_NYN    AMON_MODEL_NYN      // 23 = NX,    terminal cost size

/* -----------------------------------------------------------------------
 * Input (control) bounds — from solver setup
 *   u[0]  : total thrust    [0 .. 100]	- percent
 *   u[1]  : servo x+   [-45 .. 45]	- angle [deg]
 *   u[2]  : servo x-	[-45 .. 45]	- angle [deg]
 *   u[3]  : servo y+   [-45 .. 45]	- angle [deg]
 *   u[4]  : servo y- 	[-45 .. 45]	- angle [deg]
 * --------------------------------------------------------------------- */
#define NMPC_U0_MIN   0.0
#define NMPC_U0_MAX   100.0
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
    double x0[NMPC_NX];

    // Last optimal control output u[0]
    double u_opt[NMPC_NU];

    // Reference trajectory (same ref applied to all N stages)
    double x_ref[NMPC_NX];     // state  reference (used in yref[0..22])
    double u_ref[NMPC_NU];     // input  reference (used in yref[23..27])

    // Diagnostics
    uint32_t solve_time_ms;     // wall-clock solve time in ms
    int      last_solver_status;// raw acados return code
    uint32_t solve_count;       // total successful solves

    // Internal flags
    uint8_t  initialized;

} s_NMPC;



/*###########################################################################################################################################################*/
/* Functions */

#ifdef __cplusplus
extern "C" {
#endif

int NMPC_Init(NMPC_Handle_t *h);
int NMPC_SetState(NMPC_Handle_t *h, const double *x);
int NMPC_SetReference(NMPC_Handle_t *h, const double *x_ref, const double *u_ref);
int NMPC_Solve(NMPC_Handle_t *h);
void NMPC_GetControl(const NMPC_Handle_t *h, double *u_out);
void NMPC_DeInit(NMPC_Handle_t *h);

#ifdef __cplusplus
}
#endif


#endif /* INC_NMPC_H_ */
