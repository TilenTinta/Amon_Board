/*****************************************************************
 * File Name          : NMPC.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/05/20
 * Description        :  NMPC wrapper for amon_model acados solver
*****************************************************************/

#include <NMPC.h>

#include "acados_solver_amon_model.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados/ocp_nlp/ocp_nlp_common.h"
#include "acados/ocp_nlp/ocp_nlp_sqp_rti.h"
#include <string.h>
#include <stddef.h>
#include <math.h>

/*###########################################################################################################################################################*/
/* Functions */

// Module-level capsule pointer (one solver instance)
static amon_model_solver_capsule *s_capsule = NULL;
static void NMPC_ApplyCachedWarmStart(s_NMPC *h);
static void NMPC_SaveShiftedWarmStart(s_NMPC *h);

_Static_assert(AMON_MODEL_NP == 1, "NMPC expects eta_T");
_Static_assert(AMON_MODEL_NX == NMPC_NX, "Selected NMPC profile does not match acados NX");
_Static_assert(AMON_MODEL_NU == NMPC_NU, "Selected NMPC profile does not match acados NU");
_Static_assert(AMON_MODEL_NY == NMPC_NY, "Selected NMPC profile does not match acados NY");
_Static_assert(AMON_MODEL_NYN == NMPC_NYN, "Selected NMPC profile does not match acados NYN");
_Static_assert(AMON_MODEL_N == NMPC_N, "Selected NMPC profile does not match acados N");



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
    h->u0_operating_min = 70.0;
    h->u0_operating_max = NMPC_U0_MAX;
    h->eta_T = 1.0;
    h->model_selection = NMPC_MODEL_SELECTION;
    h->model_nx = NMPC_NX;
    h->model_horizon = NMPC_N;
    NMPC_ResetActuatorEstimator(h);
    return NMPC_OK;
}


int NMPC_SetThrustScale(s_NMPC *h, double eta_T)
{
    if (h == NULL) return NMPC_FAIL;

    if (eta_T < 0.85) eta_T = 0.85;
    if (eta_T > 1.15) eta_T = 1.15;
    h->eta_T = eta_T;

    return NMPC_OK;
}



/*********************************************************************
 * @fn      NMPC_ResetActuatorEstimator
 *
 * @param   *h: nmpc struct
 *
 * @brief   Reset first-order actuator estimates and diagnostics
 *
 * @return  None
 */
void NMPC_ResetActuatorEstimator(s_NMPC *h)
{
    if (h == NULL) return;

    h->actuator_estimator_valid = 0;
    h->estimated_thrust_N = 0.0;
    h->estimator_edf_alpha = 0.0;
    h->estimator_servo_alpha = 0.0;

    for (int i = 0; i < 4; i++)
    {
        h->estimated_servo_rad[i] = 0.0;
    }
}



/*********************************************************************
 * @fn      NMPC_UpdateActuatorEstimator
 *
 * @param   *h: nmpc struct
 * @param   *u_applied: last actuator commands [% and deg]
 * @param   commanded_thrust_N: eta_T-scaled EDF LUT thrust [N]
 * @param   dt_s: estimator update period [s]
 *
 * @brief   Exact-discrete first-order estimator for actuator states
 *
 * @return  0 OK, 1 NOK
 */
int NMPC_UpdateActuatorEstimator(s_NMPC *h, const double *u_applied, double commanded_thrust_N, double dt_s)
{
    if (h == NULL || u_applied == NULL || dt_s <= 0.0 ||
        !isfinite(commanded_thrust_N) || commanded_thrust_N < 0.0)
    {
        return NMPC_FAIL;
    }

    double servo_command_rad[4];
    for (int i = 0; i < 4; i++)
    {
        servo_command_rad[i] = u_applied[i + 1] * NMPC_DEG_TO_RAD;
    }

    if (!h->actuator_estimator_valid)
    {
        h->estimated_thrust_N = commanded_thrust_N;
        for (int i = 0; i < 4; i++)
        {
            h->estimated_servo_rad[i] = servo_command_rad[i];
        }
        h->actuator_estimator_valid = 1;
        return NMPC_OK;
    }

    const double edf_tau_s = (commanded_thrust_N >= h->estimated_thrust_N)
                           ? NMPC_EDF_TAU_UP_S
                           : NMPC_EDF_TAU_DOWN_S;
    const double edf_alpha = 1.0 - exp(-dt_s / edf_tau_s);
    const double servo_alpha = 1.0 - exp(-dt_s / NMPC_SERVO_TAU_S);

    h->estimated_thrust_N += edf_alpha * (commanded_thrust_N - h->estimated_thrust_N);
    for (int i = 0; i < 4; i++)
    {
        h->estimated_servo_rad[i] += servo_alpha *
                                    (servo_command_rad[i] - h->estimated_servo_rad[i]);
    }

    h->estimator_edf_alpha = edf_alpha;
    h->estimator_servo_alpha = servo_alpha;
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
 * @fn      NMPC_SetAppliedControl
 *
 * @param   *h: nmpc struct
 * @param   *u_applied: last control command actually sent to actuators
 *
 * @brief   Store the command used as the center of the next stage-0
 * 			actuator slew-rate bounds
 *
 * @return  0 OK, 1 NOK
 */
int NMPC_SetAppliedControl(s_NMPC *h, const double *u_applied)
{
    if (!h->initialized || u_applied == NULL) return NMPC_FAIL;

    memcpy(h->u_applied, u_applied, NMPC_NU * sizeof(double));
    h->u_applied_valid = 1;

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
 * @fn      NMPC_SetEdfOperatingLimits
 *
 * @param   *h: nmpc struct
 * @param   min_percent: minimum edf limits
 * @param   max_percent: maximum edf limits
 *
 * @brief   Set minimum and maximum edf values for complete horizont (envelope)
 *
 * @return  0 OK, 1 NOK
 */
int NMPC_SetEdfOperatingLimits(s_NMPC *h, double min_percent, double max_percent)
{
    if (!h->initialized) return NMPC_NOT_INIT;
    if (min_percent < NMPC_U0_ABS_MIN || max_percent > NMPC_U0_MAX || min_percent > max_percent)
    {
        return NMPC_FAIL;
    }

    h->u0_operating_min = min_percent;
    h->u0_operating_max = max_percent;

    // Stage 0 gets additional slew-rate bounds immediately before solving.
    double lbu[NMPC_NU] = {
        min_percent,
        NMPC_UX_MIN, NMPC_UX_MIN, NMPC_UX_MIN, NMPC_UX_MIN
    };

    double ubu[NMPC_NU] = {
        max_percent,
        NMPC_UX_MAX, NMPC_UX_MAX, NMPC_UX_MAX, NMPC_UX_MAX
    };

    for (int stage = 1; stage < NMPC_N; stage++)
    {
        ocp_nlp_constraints_model_set(s_capsule->nlp_config,
                                      s_capsule->nlp_dims,
                                      s_capsule->nlp_in,
                                      s_capsule->nlp_out,
                                      stage, "lbu", lbu);
        ocp_nlp_constraints_model_set(s_capsule->nlp_config,
                                      s_capsule->nlp_dims,
                                      s_capsule->nlp_in,
                                      s_capsule->nlp_out,
                                      stage, "ubu", ubu);
    }

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
static void NMPC_SetHorizonInputLimits(s_NMPC *h)
{
    // Center the reachable envelope on command that was used in last iteration
    const double *u_previous = h->u_applied_valid ? h->u_applied : h->u_opt;
    double cumulative_time_s = 0.0;

    for (int stage = 0; stage < NMPC_N; stage++)
    {
        double stage_dt_s = 0.0;
        ocp_nlp_in_get(s_capsule->nlp_config,
                       s_capsule->nlp_dims,
                       s_capsule->nlp_in,
                       stage, "Ts", &stage_dt_s);
        cumulative_time_s += stage_dt_s;

        const double edf_reachable = NMPC_EDF_SLEW_PERCENT_PER_S * cumulative_time_s;
#if NMPC_MODEL_SELECTION == NMPC_MODEL_INSTANT
        const double servo_reachable = NMPC_SERVO_SLEW_DEG_PER_S * cumulative_time_s;
#endif

        double lbu[NMPC_NU] = {
            clamp_actuator(u_previous[0] - edf_reachable, h->u0_operating_min, h->u0_operating_max),
#if NMPC_MODEL_SELECTION == NMPC_MODEL_INSTANT
            clamp_actuator(u_previous[1] - servo_reachable, NMPC_UX_MIN, NMPC_UX_MAX),
            clamp_actuator(u_previous[2] - servo_reachable, NMPC_UX_MIN, NMPC_UX_MAX),
            clamp_actuator(u_previous[3] - servo_reachable, NMPC_UX_MIN, NMPC_UX_MAX),
            clamp_actuator(u_previous[4] - servo_reachable, NMPC_UX_MIN, NMPC_UX_MAX)
#else
            NMPC_UX_MIN, NMPC_UX_MIN, NMPC_UX_MIN, NMPC_UX_MIN
#endif
        };

        double ubu[NMPC_NU] = {
            clamp_actuator(u_previous[0] + edf_reachable, h->u0_operating_min, h->u0_operating_max),
#if NMPC_MODEL_SELECTION == NMPC_MODEL_INSTANT
            clamp_actuator(u_previous[1] + servo_reachable, NMPC_UX_MIN, NMPC_UX_MAX),
            clamp_actuator(u_previous[2] + servo_reachable, NMPC_UX_MIN, NMPC_UX_MAX),
            clamp_actuator(u_previous[3] + servo_reachable, NMPC_UX_MIN, NMPC_UX_MAX),
            clamp_actuator(u_previous[4] + servo_reachable, NMPC_UX_MIN, NMPC_UX_MAX)
#else
            NMPC_UX_MAX, NMPC_UX_MAX, NMPC_UX_MAX, NMPC_UX_MAX
#endif
        };

        if (stage == 0)
        {
            h->u0_lbu = lbu[0];
            h->u0_ubu = ubu[0];
        }

        ocp_nlp_constraints_model_set(s_capsule->nlp_config,
                                      s_capsule->nlp_dims,
                                      s_capsule->nlp_in,
                                      s_capsule->nlp_out,
                                      stage, "lbu", lbu);
        ocp_nlp_constraints_model_set(s_capsule->nlp_config,
                                      s_capsule->nlp_dims,
                                      s_capsule->nlp_in,
                                      s_capsule->nlp_out,
                                      stage, "ubu", ubu);
    }
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

#if NMPC_MODEL_SELECTION == NMPC_MODEL_FIRST_ORDER
    // First-order model keeps EDF command slew limits. Servo dynamics are
    // represented by model states, therefore only absolute servo bounds apply.
    NMPC_SetHorizonInputLimits(h);
#else
    // Instant model enables the reachable actuator envelope after spool-up.
    if (h->nmpc_limiter_enable)
    {
        NMPC_SetHorizonInputLimits(h); // Reachable actuator envelope for the instant model
    }
    else
    {
        double lbu[NMPC_NU] = {
            h->u0_operating_min,
            NMPC_UX_MIN, NMPC_UX_MIN, NMPC_UX_MIN, NMPC_UX_MIN
        };
        double ubu[NMPC_NU] = {
            h->u0_operating_max,
            NMPC_UX_MAX, NMPC_UX_MAX, NMPC_UX_MAX, NMPC_UX_MAX
        };

        h->u0_lbu = lbu[0];
        h->u0_ubu = ubu[0];

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
#endif

    // Only eta_T is changed at runtime.
    int parameter_index = 0;
    double eta_T = h->eta_T;
    for (int stage = 0; stage <= NMPC_N; stage++)
    {
        if (amon_model_acados_update_params_sparse(s_capsule, stage,
                                                   &parameter_index, &eta_T, 1) != 0)
        {
            return NMPC_FAIL;
        }
    }

    int status = amon_model_acados_solve(s_capsule);

    int qp_iter = 0;
    double time_tot = 0.0;
    double time_qp = 0.0;
    int sqp_iter = 0;
    ocp_nlp_get(s_capsule->nlp_solver, "qp_iter", &qp_iter);

    // Solver diagnostics supported by FULL_CONDENSING_HPIPM.
    ocp_nlp_get(s_capsule->nlp_solver, "status", &status);
    ocp_nlp_get(s_capsule->nlp_solver, "time_tot", &time_tot);
    ocp_nlp_get(s_capsule->nlp_solver, "time_qp", &time_qp);
    ocp_nlp_get(s_capsule->nlp_solver, "sqp_iter", &sqp_iter);

    h->last_solver_status = status;
    h->nmpc_last_qp_iter = qp_iter;
    h->nmpc_last_qp_status = 0; // Not exposed by FULL_CONDENSING_HPIPM.
    h->nmpc_acados_status = status;
    h->nmpc_time_tot_s = time_tot;
    h->nmpc_time_qp_s = time_qp;
    h->nmpc_sqp_iter = sqp_iter;

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

    // Cache the unshifted solution first.
    for (int k = 0; k <= NMPC_N; k++)
    {
        ocp_nlp_out_get(s_capsule->nlp_config,
                        s_capsule->nlp_dims,
                        s_capsule->nlp_out,
                        k, "x", h->warm_x[k]);
    }

    for (int k = 0; k < NMPC_N; k++)
    {
        ocp_nlp_out_get(s_capsule->nlp_config,
                        s_capsule->nlp_dims,
                        s_capsule->nlp_out,
                        k, "u", h->warm_u[k]);
    }

    // Build node times from the same nonuniform Ts values used by acados.
    double node_time_s[NMPC_N + 1];
    node_time_s[0] = 0.0;
    for (int stage = 0; stage < NMPC_N; stage++)
    {
        double stage_dt_s = 0.0;
        ocp_nlp_in_get(s_capsule->nlp_config,
                       s_capsule->nlp_dims,
                       s_capsule->nlp_in,
                       stage, "Ts", &stage_dt_s);
        node_time_s[stage + 1] = node_time_s[stage] + stage_dt_s;
    }

    // Resample the old state trajectory at t + controller_dt. Processing in
    // ascending order is safe in-place because every source index is >= k.
    for (int k = 0; k <= NMPC_N; k++)
    {
        const double target_time_s = NMPC_DT_S + node_time_s[k];

        if (target_time_s >= node_time_s[NMPC_N])
        {
            memcpy(h->warm_x[k], h->warm_x[NMPC_N], NMPC_NX * sizeof(double));
            continue;
        }

        int lower = 0;
        while (lower < NMPC_N - 1 && target_time_s > node_time_s[lower + 1])
        {
            lower++;
        }

        const double interval_s = node_time_s[lower + 1] - node_time_s[lower];
        const double alpha = (interval_s > 0.0)
                           ? (target_time_s - node_time_s[lower]) / interval_s
                           : 0.0;

        // Save quaternion endpoints before an in-place write can overwrite one.
        double q0[4];
        double q1[4];
        double q_dot = 0.0;
        for (int j = 0; j < 4; j++)
        {
            q0[j] = h->warm_x[lower][6 + j];
            q1[j] = h->warm_x[lower + 1][6 + j];
            q_dot += q0[j] * q1[j];
        }
        if (q_dot < 0.0)
        {
            for (int j = 0; j < 4; j++) q1[j] = -q1[j];
        }

        for (int j = 0; j < NMPC_NX; j++)
        {
            const double x0 = h->warm_x[lower][j];
            const double x1 = h->warm_x[lower + 1][j];
            h->warm_x[k][j] = x0 + alpha * (x1 - x0);
        }

        double q_norm_sq = 0.0;
        for (int j = 0; j < 4; j++)
        {
            h->warm_x[k][6 + j] = q0[j] + alpha * (q1[j] - q0[j]);
            q_norm_sq += h->warm_x[k][6 + j] * h->warm_x[k][6 + j];
        }
        if (q_norm_sq > 1e-12)
        {
            const double q_norm_inv = 1.0 / sqrt(q_norm_sq);
            for (int j = 0; j < 4; j++) h->warm_x[k][6 + j] *= q_norm_inv;
        }
    }

    // Controls live at interval start times. Interpolate where possible and
    // hold the final control when the shifted time exceeds the old grid.
    for (int k = 0; k < NMPC_N; k++)
    {
        const double target_time_s = NMPC_DT_S + node_time_s[k];

        if (target_time_s >= node_time_s[NMPC_N - 1])
        {
            memcpy(h->warm_u[k], h->warm_u[NMPC_N - 1], NMPC_NU * sizeof(double));
            continue;
        }

        int lower = 0;
        while (lower < NMPC_N - 2 && target_time_s > node_time_s[lower + 1])
        {
            lower++;
        }

        const double interval_s = node_time_s[lower + 1] - node_time_s[lower];
        const double alpha = (interval_s > 0.0)
                           ? (target_time_s - node_time_s[lower]) / interval_s
                           : 0.0;

        for (int j = 0; j < NMPC_NU; j++)
        {
            const double u0 = h->warm_u[lower][j];
            const double u1 = h->warm_u[lower + 1][j];
            h->warm_u[k][j] = u0 + alpha * (u1 - u0);
        }
    }

    h->warm_start_valid = 1;
    NMPC_ApplyCachedWarmStart(h);
}






