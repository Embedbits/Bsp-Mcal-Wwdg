/**
 * \author Mr.Nobody
 * \file Wwdg_Port.h
 * \ingroup Wwdg
 * \brief Wwdg module public functionality
 *
 * This file contains all available public functionality, any other files shall
 * not used outside of the module.
 *
 */

#ifndef WWDG_WWDG_PORT_H
#define WWDG_WWDG_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Wwdg_Types.h"                     /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

wwdg_ModuleVersion_t        Wwdg_Get_ModuleVersion          ( void );

wwdg_RequestState_t         Wwdg_Init                       ( const wwdg_Config_t * const wwdgConfig );
wwdg_RequestState_t         Wwdg_Deinit                     ( void );
void                        Wwdg_Task                       ( void );

wwdg_RequestState_t         Wwdg_Get_DefaultConfig          ( wwdg_Config_t * const wwdgConfig );

/* ------------------------------ Watchdog state ---------------------------- */

wwdg_RequestState_t         Wwdg_Get_State                  ( wwdg_FunctionState_t * const wwdgState );
wwdg_RequestState_t         Wwdg_Get_Timeout                ( wwdg_Time_us_t * const timeout );
wwdg_RequestState_t         Wwdg_Get_Window                 ( wwdg_Time_us_t * const window );
wwdg_RequestState_t         Wwdg_Get_EarlyWakeup            ( wwdg_Time_us_t * const earlyWakeup );
wwdg_RequestState_t         Wwdg_Get_RemainingTime          ( wwdg_Time_us_t * const remainingTime );

/* ----------------------------- Manual refresh ----------------------------- */

wwdg_RequestState_t         Wwdg_Set_Refresh                ( void );

#ifdef __cplusplus
}
#endif

#endif /* WWDG_WWDG_PORT_H */
