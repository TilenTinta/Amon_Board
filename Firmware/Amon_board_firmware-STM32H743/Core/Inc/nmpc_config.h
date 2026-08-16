/*****************************************************************
 * File Name          : nmpc_config.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/08/10
 * Description        : Central NMPC model selection and dimensions
*****************************************************************/

#ifndef INC_NMPC_CONFIG_H_
#define INC_NMPC_CONFIG_H_

/*###########################################################################################################################################################*/
/* Model selection - select exactly one generated acados model */

#define NMPC_MODEL_INSTANT       1
#define NMPC_MODEL_FIRST_ORDER   2

#ifndef NMPC_MODEL_SELECTION
#define NMPC_MODEL_SELECTION     NMPC_MODEL_FIRST_ORDER     // NMPC_MODEL_INSTANT
#endif


/*###########################################################################################################################################################*/
/* Dimensions - must match the selected generated acados solver */

#define NMPC_NU                  5
#define NMPC_DT_S                0.02
#define NMPC_DEG_TO_RAD          (3.14159265358979323846 / 180.0)

#if NMPC_MODEL_SELECTION == NMPC_MODEL_INSTANT

    #define MODEL_INSTANT
    #define NMPC_NX              13
    #define NMPC_NY              18
    #define NMPC_NYN             13
    #define NMPC_N               7

#elif NMPC_MODEL_SELECTION == NMPC_MODEL_FIRST_ORDER

    #define MODEL_1ST_ORDER
    #define NMPC_NX              18
    #define NMPC_NY              23
    #define NMPC_NYN             18
    #define NMPC_N               5

#else
    #error "Invalid NMPC_MODEL_SELECTION"
#endif

// Compatibility name used by flight-path reference arrays
#define NMPC_NX_SIZE             NMPC_NX


/*###########################################################################################################################################################*/
/* State indexes */

#define NMPC_X_PX                0
#define NMPC_X_PY                1
#define NMPC_X_PZ                2
#define NMPC_X_VX                3
#define NMPC_X_VY                4
#define NMPC_X_VZ                5
#define NMPC_X_QW                6
#define NMPC_X_QX                7
#define NMPC_X_QY                8
#define NMPC_X_QZ                9
#define NMPC_X_WX                10
#define NMPC_X_WY                11
#define NMPC_X_WZ                12

#if NMPC_MODEL_SELECTION == NMPC_MODEL_FIRST_ORDER
    #define NMPC_X_THRUST_N       13
    #define NMPC_X_DELTA_1_RAD    14
    #define NMPC_X_DELTA_2_RAD    15
    #define NMPC_X_DELTA_3_RAD    16
    #define NMPC_X_DELTA_4_RAD    17
#endif

#endif /* INC_NMPC_CONFIG_H_ */
