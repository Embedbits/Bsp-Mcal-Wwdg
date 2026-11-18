/**
 * \author Mr.Nobody
 * \file Wwdg_Types.h
 * \ingroup Wwdg
 * \brief Wwdg module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through Port file.
 *
 */

/**
 * \defgroup Wwdg Wwdg
 * \brief Wwdg module
 */

#ifndef WWDG_WWDG_TYPES_H
#define WWDG_WWDG_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define WWDG_NULL_PTR                        ( ( void* ) 0u )

/** Time value used to signal that optional time based function is not used
 *  (window: refresh is allowed during the whole timeout) */
#define WWDG_TIME_UNUSED                     ( 0u )

/* ========================== EXPORTED MACROS =============================== */

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t wwdg_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t wwdg_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t wwdg_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    wwdg_MajorVersion_t Major; /**< Major version */
    wwdg_MinorVersion_t Minor; /**< Minor version */
    wwdg_PatchVersion_t Patch; /**< Patch version */
}   wwdg_ModuleVersion_t;


/** Function status enumeration */
typedef enum
{
    WWDG_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    WWDG_FUNCTION_ACTIVE         /**< Function status is active   */
}   wwdg_FunctionState_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    WWDG_REQUEST_ERROR = 0u, /**< Processing request failed  */
    WWDG_REQUEST_OK          /**< Processing request succeed */
}   wwdg_RequestState_t;


/** Flag states enumeration */
typedef enum
{
    WWDG_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    WWDG_FLAG_ACTIVE         /**< Active flag state   */
}   wwdg_FlagState_t;


/** \brief Type used to store time values in milliseconds [0.001s] (configuration) */
typedef uint32_t wwdg_Time_ms_t;


/** \brief Type used to store time values in microseconds [0.000001s] (read-back
 *         of the achieved values, one counter tick can be shorter than 1 ms) */
typedef uint32_t wwdg_Time_us_t;


/** \brief Interrupt priority type definition */
typedef uint32_t wwdg_IrqPrio_t;


/** \brief Early wakeup interrupt user callback.
 *
 * Called from interrupt context one counter tick before the watchdog reset.
 * The interrupt flag is already acknowledged by the module.
 */
typedef void ( wwdg_EarlyWakeupIsr_t )( void );


/** \brief Type used to store WWDG configuration */
typedef struct
{
    /** Required watchdog timeout in ms (time from refresh to reset), rounded down to whole
     *  counter ticks. Prescaler and counter reload value are calculated by the module, see
     *  \ref Wwdg_Get_Timeout for the achieved value. */
    wwdg_Time_ms_t         Timeout;

    /** Window length in ms before the reset, in which the refresh is allowed, rounded down to
     *  whole counter ticks. Refresh earlier than the window resets the MCU. \ref WWDG_TIME_UNUSED
     *  allows the refresh during the whole timeout. Must not be longer than
     *  \ref wwdg_Config_t::Timeout (window equal to the timeout is allowed). */
    wwdg_Time_ms_t         Window;

    /** Early wakeup interrupt user callback (called one counter tick before the reset),
     *  \ref WWDG_NULL_PTR disables the interrupt */
    wwdg_EarlyWakeupIsr_t *EarlyWakeupIsr;

    /** Early wakeup interrupt priority (0 - highest priority) */
    wwdg_IrqPrio_t         IrqPriority;
}   wwdg_Config_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */


#endif /* WWDG_WWDG_TYPES_H */
