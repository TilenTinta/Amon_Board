/*****************************************************************
 * File Name          : NMPC.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/05/20
 * Description        :  NMPC wrapper for amon_model acados solver
 *
 * Solver details (read from acados_solver_amon_model.c):
 *   - SQP with PARTIAL_CONDENSING_HPIPM
 *   - ERK integrator, LINEAR_LS cost
 *   - NBX0 = 0  -> initial state set via ocp_nlp_out_set warm-start
 *   - NBU  = 5  -> input constraints active at every stage
 *   - yref = [x(23); u(5)], yref_e = [x(23)]
*****************************************************************/

#include <NMPC.h>

#include "nmpc_platform.h"
#include "acados_solver_amon_model.h"
#include "acados_c/ocp_nlp_interface.h"
#include <string.h>
#include <stddef.h>

/*###########################################################################################################################################################*/
/* Functions */

// Module-level capsule pointer (one solver instance)
static amon_model_solver_capsule *s_capsule = NULL;



/*********************************************************************
 * @fn      NMPC_Init
 *
 * @param   *h: nmpc struct
 *
 * @brief   Initialize NMPC algorithm and its components
 *
 * @return  0 OK, 1 NOK
 */
int NMPC_Init(s_NMPC *h)
{
    if (h == NULL) return NMPC_FAIL;
    if (h->initialized) return NMPC_OK;   // already done

    // Zero the struct
    memset(h, 0, sizeof(s_NMPC));

    // Allocate capsule
    s_capsule = amon_model_acados_create_capsule();
    if (s_capsule == NULL) return NMPC_FAIL;

    // Create solver — allocates all internal memory and sets defaults
    int ret = amon_model_acados_create(s_capsule);
    if (ret != 0)
    {
        amon_model_acados_free_capsule(s_capsule);
        s_capsule = NULL;
        return NMPC_FAIL;
    }

    h->initialized = 1;
    return NMPC_OK;
}



/*********************************************************************
 * @fn      NMPC_SetState
 *
 * @param   *h: nmpc struct
 * @param   *x: states
 *
 * @brief   Set initial values for solver for next solve - warm-start
 *
 * @return  0 OK, 1 NOK
 */
int NMPC_SetState(s_NMPC *h, const double *x)
{
    if (!h->initialized || x == NULL) return NMPC_FAIL;

    // Store locally
    memcpy(h->x0, x, NMPC_NX * sizeof(double));

    ocp_nlp_constraints_model_set(
					s_capsule->nlp_config,
					s_capsule->nlp_dims,
					s_capsule->nlp_in,
					s_capsule->nlp_out,
					0, "lbx", h->x0
				);

    ocp_nlp_constraints_model_set(
					s_capsule->nlp_config,
					s_capsule->nlp_dims,
					s_capsule->nlp_in,
					s_capsule->nlp_out,
					0, "ubx", h->x0
				);

    // Set initial state as warm-start at node 0
    ocp_nlp_out_set(
    				s_capsule->nlp_config,
                    s_capsule->nlp_dims,
                    s_capsule->nlp_out,
                    s_capsule->nlp_in,
                    0, "x", h->x0
				);

    return NMPC_OK;
}




/*********************************************************************
 * @fn      NMPC_SetReference
 *
 * @param   *h: nmpc struct
 * @param   *x_ref: Pointer to NMPC_NX
 * @param   *u_ref: Pointer to NMPC_NU
 *
 * @brief   Set the reference for all N stages and the terminal stage
 *
 * @return  0 OK, 1 NOK
 */
int NMPC_SetReference(s_NMPC *h, const double *x_ref, const double *u_ref)
{
    if (!h->initialized || x_ref == NULL || u_ref == NULL) return NMPC_FAIL;

    memcpy(h->x_ref, x_ref, NMPC_NX * sizeof(double));
    memcpy(h->u_ref, u_ref, NMPC_NU * sizeof(double));

    // combined yref = [x_ref; u_ref] for stage cost
    double yref[NMPC_NY];                           			// 28
    memcpy(yref, x_ref, NMPC_NX * sizeof(double));   			// [0..22]
    memcpy(yref + NMPC_NX, u_ref, NMPC_NU * sizeof(double));  	// [23..27]

    // Apply to all N stage nodes
    for (int k = 0; k < NMPC_N; k++)
    {
        ocp_nlp_cost_model_set(s_capsule->nlp_config,
                               s_capsule->nlp_dims,
                               s_capsule->nlp_in,
                               k, "yref", yref);
    }

    // Terminal node — state reference only
    ocp_nlp_cost_model_set(s_capsule->nlp_config,
                           s_capsule->nlp_dims,
                           s_capsule->nlp_in,
                           NMPC_N, "yref", h->x_ref);

    return NMPC_OK;
}



/*********************************************************************
 * @fn      NMPC_Solve
 *
 * @param   *h: nmpc struct
 *
 * @brief   Run one NMPC solve step
 * 			Extracts u[0] into h->u_opt
 *
 * @return  0 OK, 1 NOK
 */
int NMPC_Solve(s_NMPC *h)
{
    if (!h->initialized) return NMPC_NOT_INIT;

    uint32_t t0 = NMPC_PlatformGetTickMs();

    int status = amon_model_acados_solve(s_capsule);

    h->solve_time_ms     = NMPC_PlatformGetTickMs() - t0;
    h->last_solver_status = status;

    // Extract u[0] always - best available solution even if solver fails
    ocp_nlp_out_get(s_capsule->nlp_config,
                    s_capsule->nlp_dims,
                    s_capsule->nlp_out,
                    0, "u", h->u_opt);

    if (status == ACADOS_SUCCESS)
    {
        h->solve_count++;
        return NMPC_OK;
    }

    // Non-zero status: solver hit max iterations or failed to converge.
    //	- u_opt still contains the best iteration - use it or not
    return NMPC_SOLVER_ERR;
}



/*********************************************************************
 * @fn      NMPC_GetControl
 *
 * @param   *h: nmpc struct
 * @param   *u_out: Pointer to NMPC_NU
 *
 * @brief   Copy the optimal control output into u_out
 *
 * @return  None
 */
void NMPC_GetControl(const s_NMPC *h, double *u_out)
{
    if (h == NULL || u_out == NULL) return;
    memcpy(u_out, h->u_opt, NMPC_NU * sizeof(double));
}



/*********************************************************************
 * @fn      NMPC_DeInit
 *
 * @param   *h: nmpc struct
 *
 * @brief   Free all acados memory
 * 			Call after drone landing
 *
 * @return  None
 */
void NMPC_DeInit(s_NMPC *h)
{
    if (s_capsule != NULL)
    {
        amon_model_acados_free(s_capsule);
        amon_model_acados_free_capsule(s_capsule);
        s_capsule = NULL;
    }
    if (h != NULL)
    {
        memset(h, 0, sizeof(s_NMPC));
    }
}
