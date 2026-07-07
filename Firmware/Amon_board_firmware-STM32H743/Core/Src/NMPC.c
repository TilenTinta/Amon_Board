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
#include "acados/ocp_nlp/ocp_nlp_common.h"
#include "acados/ocp_nlp/ocp_nlp_sqp_rti.h"
#include <string.h>
#include <stddef.h>

/*###########################################################################################################################################################*/
/* Functions */

// Module-level capsule pointer (one solver instance)
static amon_model_solver_capsule *s_capsule = NULL;
static void NMPC_ApplyCachedWarmStart(s_NMPC *h);
static void NMPC_SaveShiftedWarmStart(s_NMPC *h);



// Helper code
#ifdef COMPILE_MEM_ANALYZER

#define NMPC_MEM_ANALYSIS_MAX_N    64
#define NMPC_MEM_ANALYSIS_MAX_NP1  (NMPC_MEM_ANALYSIS_MAX_N + 1)

typedef struct
{
    uint32_t valid;
    uint32_t error_flags;
    uint32_t progress_marker;
    uint32_t N;
    uint32_t recorded_N;

    uint32_t sizeof_ocp_nlp_solver;
    uint32_t solver_memory_size;
    uint32_t solver_workspace_size;
    uint32_t solver_total_size;

    uint32_t nlp_memory_size;
    uint32_t nlp_workspace_size;
    uint32_t nlp_opts_size;
    uint32_t nlp_in_size;
    uint32_t nlp_out_size;
    uint32_t sens_out_size;

    uint32_t qp_solver_memory_size;
    uint32_t qp_solver_workspace_size;
    uint32_t qp_in_orig_size;
    uint32_t qp_out_orig_size;
    uint32_t qp_seed_orig_size;
    uint32_t qp_res_orig_size;
    uint32_t qp_res_workspace_orig_size;

    uint32_t dynamics_memory_total;
    uint32_t dynamics_workspace_total;
    uint32_t dynamics_workspace_max;
    uint32_t dynamics_ext_workspace_total;
    uint32_t dynamics_ext_workspace_max;

    uint32_t cost_memory_total;
    uint32_t cost_workspace_total;
    uint32_t cost_workspace_max;
    uint32_t cost_ext_workspace_total;
    uint32_t cost_ext_workspace_max;

    uint32_t constraints_memory_total;
    uint32_t constraints_workspace_total;
    uint32_t constraints_workspace_max;
    uint32_t constraints_ext_workspace_total;
    uint32_t constraints_ext_workspace_max;

    uint32_t stage_dynamics_memory[NMPC_MEM_ANALYSIS_MAX_N];
    uint32_t stage_dynamics_workspace[NMPC_MEM_ANALYSIS_MAX_N];
    uint32_t stage_dynamics_ext_workspace[NMPC_MEM_ANALYSIS_MAX_N];

    uint32_t stage_cost_memory[NMPC_MEM_ANALYSIS_MAX_NP1];
    uint32_t stage_cost_workspace[NMPC_MEM_ANALYSIS_MAX_NP1];
    uint32_t stage_cost_ext_workspace[NMPC_MEM_ANALYSIS_MAX_NP1];

    uint32_t stage_constraints_memory[NMPC_MEM_ANALYSIS_MAX_NP1];
    uint32_t stage_constraints_workspace[NMPC_MEM_ANALYSIS_MAX_NP1];
    uint32_t stage_constraints_ext_workspace[NMPC_MEM_ANALYSIS_MAX_NP1];
} s_NMPC_MemAnalysis;

volatile s_NMPC_MemAnalysis g_nmpc_mem_analysis;

static void NMPC_MemAnalysisClear(void)
{
    volatile uint8_t *p = (volatile uint8_t *)&g_nmpc_mem_analysis;
    for (uint32_t i = 0; i < sizeof(g_nmpc_mem_analysis); i++)
    {
        p[i] = 0;
    }
}

static uint32_t NMPC_SizeToU32(acados_size_t value)
{
    if (value > 0xffffffffu)
    {
        return 0xffffffffu;
    }
    return (uint32_t)value;
}

static void NMPC_AddSize(volatile uint32_t *total, volatile uint32_t *maximum, uint32_t value)
{
    *total += value;
    if (value > *maximum)
    {
        *maximum = value;
    }
}


int NMPC_DebugAnalyzeMemory(void)
{
    NMPC_MemAnalysisClear();

    if (s_capsule == NULL)
    {
        g_nmpc_mem_analysis.error_flags = 0x00000001u;
        return NMPC_FAIL;
    }

    ocp_nlp_config *config = s_capsule->nlp_config;
    ocp_nlp_dims *dims = s_capsule->nlp_dims;
    void *solver_opts = s_capsule->nlp_opts;
    ocp_nlp_in *nlp_in = s_capsule->nlp_in;

    if (config == NULL || dims == NULL || solver_opts == NULL || nlp_in == NULL)
    {
        g_nmpc_mem_analysis.error_flags = 0x00000002u;
        return NMPC_FAIL;
    }

    ocp_nlp_sqp_rti_opts *sqp_rti_opts = (ocp_nlp_sqp_rti_opts *)solver_opts;
    ocp_nlp_opts *opts = sqp_rti_opts->nlp_opts;
    if (opts == NULL)
    {
        g_nmpc_mem_analysis.error_flags = 0x00000020u;
        return NMPC_FAIL;
    }

    const int N = dims->N;
    const int recorded_N = (N > NMPC_MEM_ANALYSIS_MAX_N) ? NMPC_MEM_ANALYSIS_MAX_N : N;

    g_nmpc_mem_analysis.N = (uint32_t)N;
    g_nmpc_mem_analysis.recorded_N = (uint32_t)recorded_N;
    if (N > NMPC_MEM_ANALYSIS_MAX_N)
    {
        g_nmpc_mem_analysis.error_flags |= 0x00000004u;
    }

    if (config->opts_update != NULL)
    {
        g_nmpc_mem_analysis.progress_marker = 1u;
        config->opts_update(config, dims, solver_opts);
    }
    else
    {
        g_nmpc_mem_analysis.error_flags |= 0x00000010u;
    }

    g_nmpc_mem_analysis.sizeof_ocp_nlp_solver = (uint32_t)sizeof(ocp_nlp_solver);
    g_nmpc_mem_analysis.progress_marker = 2u;
    g_nmpc_mem_analysis.solver_memory_size =
        NMPC_SizeToU32(config->memory_calculate_size(config, dims, solver_opts, nlp_in));
    g_nmpc_mem_analysis.progress_marker = 3u;
    g_nmpc_mem_analysis.solver_workspace_size =
        NMPC_SizeToU32(config->workspace_calculate_size(config, dims, solver_opts, nlp_in));
    g_nmpc_mem_analysis.solver_total_size =
        g_nmpc_mem_analysis.sizeof_ocp_nlp_solver +
        g_nmpc_mem_analysis.solver_memory_size +
        g_nmpc_mem_analysis.solver_workspace_size;

    g_nmpc_mem_analysis.progress_marker = 4u;
    g_nmpc_mem_analysis.nlp_memory_size =
        NMPC_SizeToU32(ocp_nlp_memory_calculate_size(config, dims, opts, nlp_in));
    g_nmpc_mem_analysis.progress_marker = 5u;
    g_nmpc_mem_analysis.nlp_workspace_size =
        NMPC_SizeToU32(ocp_nlp_workspace_calculate_size(config, dims, opts, nlp_in));
    g_nmpc_mem_analysis.progress_marker = 6u;
    g_nmpc_mem_analysis.nlp_opts_size =
        NMPC_SizeToU32(config->opts_calculate_size(config, dims));
    g_nmpc_mem_analysis.progress_marker = 7u;
    g_nmpc_mem_analysis.nlp_in_size =
        NMPC_SizeToU32(ocp_nlp_in_calculate_size(config, dims));
    g_nmpc_mem_analysis.progress_marker = 8u;
    g_nmpc_mem_analysis.nlp_out_size =
        NMPC_SizeToU32(ocp_nlp_out_calculate_size(config, dims));
    g_nmpc_mem_analysis.sens_out_size = g_nmpc_mem_analysis.nlp_out_size;

    if (config->qp_solver != NULL && dims->qp_solver != NULL && opts->qp_solver_opts != NULL)
    {
        g_nmpc_mem_analysis.progress_marker = 9u;
        g_nmpc_mem_analysis.qp_solver_memory_size =
            NMPC_SizeToU32(config->qp_solver->memory_calculate_size(config->qp_solver,
                                                                    dims->qp_solver,
                                                                    opts->qp_solver_opts));
        g_nmpc_mem_analysis.progress_marker = 10u;
        g_nmpc_mem_analysis.qp_solver_workspace_size =
            NMPC_SizeToU32(config->qp_solver->workspace_calculate_size(config->qp_solver,
                                                                       dims->qp_solver,
                                                                       opts->qp_solver_opts));

        if (dims->qp_solver->orig_dims != NULL)
        {
            g_nmpc_mem_analysis.qp_in_orig_size =
                NMPC_SizeToU32(ocp_qp_in_calculate_size(dims->qp_solver->orig_dims));
            g_nmpc_mem_analysis.qp_out_orig_size =
                NMPC_SizeToU32(ocp_qp_out_calculate_size(dims->qp_solver->orig_dims));
            g_nmpc_mem_analysis.qp_seed_orig_size =
                NMPC_SizeToU32(ocp_qp_seed_calculate_size(dims->qp_solver->orig_dims));
            g_nmpc_mem_analysis.qp_res_orig_size =
                NMPC_SizeToU32(ocp_qp_res_calculate_size(dims->qp_solver->orig_dims));
            g_nmpc_mem_analysis.qp_res_workspace_orig_size =
                NMPC_SizeToU32(ocp_qp_res_workspace_calculate_size(dims->qp_solver->orig_dims));
        }
    }
    else
    {
        g_nmpc_mem_analysis.error_flags |= 0x00000008u;
    }

    for (int i = 0; i < recorded_N; i++)
    {
        uint32_t value;

        g_nmpc_mem_analysis.progress_marker = 100u + (uint32_t)i;
        value = NMPC_SizeToU32(config->dynamics[i]->memory_calculate_size(config->dynamics[i],
                                                                          dims->dynamics[i],
                                                                          opts->dynamics[i]));
        g_nmpc_mem_analysis.stage_dynamics_memory[i] = value;
        g_nmpc_mem_analysis.dynamics_memory_total += value;

        value = NMPC_SizeToU32(config->dynamics[i]->workspace_calculate_size(config->dynamics[i],
                                                                             dims->dynamics[i],
                                                                             opts->dynamics[i]));
        g_nmpc_mem_analysis.stage_dynamics_workspace[i] = value;
        NMPC_AddSize(&g_nmpc_mem_analysis.dynamics_workspace_total,
                     &g_nmpc_mem_analysis.dynamics_workspace_max,
                     value);

        value = NMPC_SizeToU32(config->dynamics[i]->get_external_fun_workspace_requirement(config->dynamics[i],
                                                                                           dims->dynamics[i],
                                                                                           opts->dynamics[i],
                                                                                           nlp_in->dynamics[i]));
        g_nmpc_mem_analysis.stage_dynamics_ext_workspace[i] = value;
        NMPC_AddSize(&g_nmpc_mem_analysis.dynamics_ext_workspace_total,
                     &g_nmpc_mem_analysis.dynamics_ext_workspace_max,
                     value);
    }

    for (int i = 0; i <= recorded_N; i++)
    {
        uint32_t value;

        g_nmpc_mem_analysis.progress_marker = 200u + (uint32_t)i;
        value = NMPC_SizeToU32(config->cost[i]->memory_calculate_size(config->cost[i],
                                                                      dims->cost[i],
                                                                      opts->cost[i]));
        g_nmpc_mem_analysis.stage_cost_memory[i] = value;
        g_nmpc_mem_analysis.cost_memory_total += value;

        value = NMPC_SizeToU32(config->cost[i]->workspace_calculate_size(config->cost[i],
                                                                         dims->cost[i],
                                                                         opts->cost[i]));
        g_nmpc_mem_analysis.stage_cost_workspace[i] = value;
        NMPC_AddSize(&g_nmpc_mem_analysis.cost_workspace_total,
                     &g_nmpc_mem_analysis.cost_workspace_max,
                     value);

        value = NMPC_SizeToU32(config->cost[i]->get_external_fun_workspace_requirement(config->cost[i],
                                                                                       dims->cost[i],
                                                                                       opts->cost[i],
                                                                                       nlp_in->cost[i]));
        g_nmpc_mem_analysis.stage_cost_ext_workspace[i] = value;
        NMPC_AddSize(&g_nmpc_mem_analysis.cost_ext_workspace_total,
                     &g_nmpc_mem_analysis.cost_ext_workspace_max,
                     value);

        g_nmpc_mem_analysis.progress_marker = 300u + (uint32_t)i;
        value = NMPC_SizeToU32(config->constraints[i]->memory_calculate_size(config->constraints[i],
                                                                             dims->constraints[i],
                                                                             opts->constraints[i]));
        g_nmpc_mem_analysis.stage_constraints_memory[i] = value;
        g_nmpc_mem_analysis.constraints_memory_total += value;

        value = NMPC_SizeToU32(config->constraints[i]->workspace_calculate_size(config->constraints[i],
                                                                                dims->constraints[i],
                                                                                opts->constraints[i]));
        g_nmpc_mem_analysis.stage_constraints_workspace[i] = value;
        NMPC_AddSize(&g_nmpc_mem_analysis.constraints_workspace_total,
                     &g_nmpc_mem_analysis.constraints_workspace_max,
                     value);

        value = NMPC_SizeToU32(config->constraints[i]->get_external_fun_workspace_requirement(config->constraints[i],
                                                                                              dims->constraints[i],
                                                                                              opts->constraints[i],
                                                                                              nlp_in->constraints[i]));
        g_nmpc_mem_analysis.stage_constraints_ext_workspace[i] = value;
        NMPC_AddSize(&g_nmpc_mem_analysis.constraints_ext_workspace_total,
                     &g_nmpc_mem_analysis.constraints_ext_workspace_max,
                     value);
    }

    g_nmpc_mem_analysis.progress_marker = 0xffffffffu;
    g_nmpc_mem_analysis.valid = 1u;
    return NMPC_OK;
}

#endif


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
 * @fn      clamp_actuator
 *
 * @param   value: current servo angle
 * @param   min_value: min servo angle
 * @param   max_value: max servo angle
 *
 * @brief   Clamp servo angle
 *
 * @return  servo angle
 */
static double clamp_actuator(double value, double min_value, double max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}



/*********************************************************************
 * @fn      NMPC_SetStage0InputLimits
 *
 * @param   *h: nmpc struct
 *
 * @brief   Run one NMPC solve step
 * 			Extracts u[0] into h->u_opt
 *
 * @return  0 OK, 1 NOK
 */
static void NMPC_SetStage0InputLimits(s_NMPC *h)
{
    double lbu[NMPC_NU] = {
		//NMPC_U0_MIN,
    	clamp_actuator(h->u_opt[0] - NMPC_EDF_MAX_STEP_PERCENT, NMPC_U0_MIN, NMPC_U0_MAX),
		clamp_actuator(h->u_opt[1] - NMPC_SERVO_MAX_STEP_DEG, NMPC_UX_MIN, NMPC_UX_MAX),
		clamp_actuator(h->u_opt[2] - NMPC_SERVO_MAX_STEP_DEG, NMPC_UX_MIN, NMPC_UX_MAX),
		clamp_actuator(h->u_opt[3] - NMPC_SERVO_MAX_STEP_DEG, NMPC_UX_MIN, NMPC_UX_MAX),
		clamp_actuator(h->u_opt[4] - NMPC_SERVO_MAX_STEP_DEG, NMPC_UX_MIN, NMPC_UX_MAX)
    };

    double ubu[NMPC_NU] = {
		//NMPC_U0_MAX,
		clamp_actuator(h->u_opt[0] + NMPC_EDF_MAX_STEP_PERCENT, NMPC_U0_MIN, NMPC_U0_MAX),
		clamp_actuator(h->u_opt[1] + NMPC_SERVO_MAX_STEP_DEG, NMPC_UX_MIN, NMPC_UX_MAX),
		clamp_actuator(h->u_opt[2] + NMPC_SERVO_MAX_STEP_DEG, NMPC_UX_MIN, NMPC_UX_MAX),
		clamp_actuator(h->u_opt[3] + NMPC_SERVO_MAX_STEP_DEG, NMPC_UX_MIN, NMPC_UX_MAX),
		clamp_actuator(h->u_opt[4] + NMPC_SERVO_MAX_STEP_DEG, NMPC_UX_MIN, NMPC_UX_MAX)
    };

    ocp_nlp_constraints_model_set(s_capsule->nlp_config,
                                  s_capsule->nlp_dims,
                                  s_capsule->nlp_in,
                                  s_capsule->nlp_out,
                                  0, "lbu", lbu);

    ocp_nlp_constraints_model_set(s_capsule->nlp_config,
                                  s_capsule->nlp_dims,
                                  s_capsule->nlp_in,
                                  s_capsule->nlp_out,
                                  0, "ubu", ubu);
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

    //uint32_t t0 = NMPC_PlatformGetTickMs();

    //!!! ONLY FOR INSTANT MODEL !!!
    if (h->nmpc_limiter_enable) NMPC_SetStage0InputLimits(h); // Actuator slew-rate (simple instant model actuator limiter)

    int status = amon_model_acados_solve(s_capsule);

    //h->solve_time_ms     = NMPC_PlatformGetTickMs() - t0;
    h->last_solver_status = status;

    if (status == ACADOS_SUCCESS)
    {
        ocp_nlp_out_get(s_capsule->nlp_config,
                        s_capsule->nlp_dims,
                        s_capsule->nlp_out,
                        0, "u", h->u_opt);

        NMPC_SaveShiftedWarmStart(h);

        h->solve_count++;
        return NMPC_OK;
    }

    NMPC_ApplyCachedWarmStart(h);

    // Non-zero status: keep the last successful u_opt and warm-start trajectory.
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




/*********************************************************************
 * @fn      NMPC_ApplyCachedWarmStart
 *
 * @param   *h: nmpc struct
 *
 * @brief   Use previous values from solver
 * 			as start values for a new iteration
 * 			of optimization = warm-start
 *
 * @return  None
 */
static void NMPC_ApplyCachedWarmStart(s_NMPC *h)
{
    if (h == NULL || !h->warm_start_valid || s_capsule == NULL) return;

    for (int k = 0; k <= NMPC_N; k++)
    {
        ocp_nlp_out_set(s_capsule->nlp_config,
                        s_capsule->nlp_dims,
                        s_capsule->nlp_out,
                        s_capsule->nlp_in,
                        k, "x", h->warm_x[k]);
    }

    for (int k = 0; k < NMPC_N; k++)
    {
        ocp_nlp_out_set(s_capsule->nlp_config,
                        s_capsule->nlp_dims,
                        s_capsule->nlp_out,
                        s_capsule->nlp_in,
                        k, "u", h->warm_u[k]);
    }
}




/*********************************************************************
 * @fn      NMPC_SaveShiftedWarmStart
 *
 * @param   *h: nmpc struct
 *
 * @brief   Save current solver values for
 * 			next iteration of optimization
 *
 * @return  None
 */
static void NMPC_SaveShiftedWarmStart(s_NMPC *h)
{
    if (h == NULL || s_capsule == NULL) return;

    for (int k = 0; k < NMPC_N; k++)
    {
        ocp_nlp_out_get(s_capsule->nlp_config,
                        s_capsule->nlp_dims,
                        s_capsule->nlp_out,
                        k + 1, "x", h->warm_x[k]);
    }

    memcpy(h->warm_x[NMPC_N], h->warm_x[NMPC_N - 1], NMPC_NX * sizeof(double));

    for (int k = 0; k < NMPC_N - 1; k++)
    {
        ocp_nlp_out_get(s_capsule->nlp_config,
                        s_capsule->nlp_dims,
                        s_capsule->nlp_out,
                        k + 1, "u", h->warm_u[k]);
    }
    ocp_nlp_out_get(s_capsule->nlp_config,
                    s_capsule->nlp_dims,
                    s_capsule->nlp_out,
                    NMPC_N - 1, "u", h->warm_u[NMPC_N - 1]);

    h->warm_start_valid = 1;
    NMPC_ApplyCachedWarmStart(h);
}

