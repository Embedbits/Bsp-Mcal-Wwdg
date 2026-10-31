/**
 * \author Mr.Nobody
 * \file Wwdg_Reg.h
 * \ingroup Wwdg
 * \brief Wwdg module Reg component header file.
 *
 * This component contains functionality for register operations. It is private
 * to the module (the register access is not shared with other modules).
 *
 */

#ifndef WWDG_REG_WWDG_REG_H
#define WWDG_REG_WWDG_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Wwdg_Types.h"                     /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/** \brief List of WWDG registers */
typedef enum
{
    WWDG_REG_CR  = 0u, /**< WWDG control register             */
    WWDG_REG_CFR     , /**< WWDG configuration register       */
    WWDG_REG_SR      , /**< WWDG status register              */
    WWDG_REG_CNT       /**< Count of available WWDG registers */
}   wwdg_RegId_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* ---------------------------- Generic accessors --------------------------- */

void                        Wwdg_Set_RegBit             ( wwdg_RegId_t regId, uint32_t bitMask );
void                        Wwdg_Reset_RegBit           ( wwdg_RegId_t regId, uint32_t bitMask );
uint32_t                    Wwdg_Get_RegBit             ( wwdg_RegId_t regId, uint32_t bitMask );

void                        Wwdg_Set_RegVal             ( wwdg_RegId_t regId, uint32_t regMask, uint32_t regValue );
uint32_t                    Wwdg_Get_RegVal             ( wwdg_RegId_t regId, uint32_t regMask );

/* ----------------------------- Register protocol -------------------------- */

wwdg_RequestState_t         Wwdg_Reg_Set_Config         ( uint32_t timebase,
                                                          uint32_t window,
                                                          wwdg_FunctionState_t ewiState );
wwdg_RequestState_t         Wwdg_Reg_Set_Start          ( uint32_t counter );
wwdg_RequestState_t         Wwdg_Reg_Set_Refresh        ( uint32_t counter );

void                        Wwdg_Reg_Clear_EwiFlag      ( void );
wwdg_FlagState_t            Wwdg_Reg_Get_EwiFlag        ( void );

wwdg_FunctionState_t        Wwdg_Reg_Get_RunState       ( void );
wwdg_FunctionState_t        Wwdg_Reg_Get_EwiState       ( void );

#ifdef __cplusplus
}
#endif

#endif /* WWDG_REG_WWDG_REG_H */
