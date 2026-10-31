/**
 * \author Mr.Nobody
 * \file Wwdg.h
 * \ingroup Wwdg
 * \brief Wwdg module common functionality header file.
 *
 * This file contains the common functionality used internally by the module,
 * and shall provide interface between the module and the application.
 *
 */

#ifndef WWDG_WWDG_H
#define WWDG_WWDG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Wwdg_Types.h"            /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/** \brief Prescaler values enumeration */
typedef enum
{
    WWDG_PRESCALER_DIV_1 = 0u, /**< Prescaler divider 1               */
    WWDG_PRESCALER_DIV_2,      /**< Prescaler divider 2               */
    WWDG_PRESCALER_DIV_4,      /**< Prescaler divider 4               */
    WWDG_PRESCALER_DIV_8,      /**< Prescaler divider 8               */
    WWDG_PRESCALER_DIV_16,     /**< Prescaler divider 16              */
    WWDG_PRESCALER_DIV_32,     /**< Prescaler divider 32              */
    WWDG_PRESCALER_DIV_64,     /**< Prescaler divider 64              */
    WWDG_PRESCALER_DIV_128,    /**< Prescaler divider 128             */
    WWDG_PRESCALER_CNT         /**< Number of prescaler divider items */
}   wwdg_Prescaler_t;


/** \brief Type used to store prescaler divider value */
typedef uint32_t wwdg_Divider_t;


/** \brief Type used to store number of watchdog counter ticks */
typedef uint32_t wwdg_Ticks_t;


/** \brief Prescaler configuration item */
typedef struct
{
    uint32_t       RegVal;  /**< Value of the WDGTB field in CFR register (already positioned) */
    wwdg_Divider_t Divider; /**< Prescaler divider value                                       */
}   wwdg_PrescalerConfig_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

#ifdef __cplusplus
}
#endif

#endif /* WWDG_WWDG_H */
