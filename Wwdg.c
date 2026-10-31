/**
 * \author Mr.Nobody
 * \file Wwdg.c
 * \ingroup Wwdg
 * \brief Wwdg module common functionality
 *
 * # Introduction #
 * The system window watchdog (WWDG) is clocked by the APB clock. It is a 7-bit
 * down-counter, which resets the MCU when it is decremented from 0x40 to 0x3F
 * (the watchdog expired) and when it is refreshed while its value is above the
 * window register (the refresh came too early). The early wakeup interrupt (EWI)
 * is raised one counter tick before the reset (counter value 0x40).
 *
 * # Timing #
 * One counter tick lasts <tt>4096 * prescaler / f(PCLK)</tt>, the prescaler is
 * 1 ... 128. The timeout in ticks N is written as the counter value 0x3F + N
 * (N = 1 ... 64), the window of M ticks as the window value 0x3F + M (the refresh
 * is allowed during the last M ticks). The module selects the smallest prescaler
 * with which the timeout fits to the counter. Timeout and window are rounded down
 * to whole ticks (a value below one tick is rejected), the achieved values are
 * reported rounded down to us. The real timeout is up to one tick shorter, because
 * the phase of the prescaler is unknown at the write of the counter.
 *
 * # Watchdog start #
 * The watchdog can not be stopped once started (only a system reset stops it).
 * It is started by \ref Wwdg_Init. A watchdog started by the hardware (option byte
 * WWDG_SW) is not supported by this version: the getters return an error until
 * \ref Wwdg_Init succeeded, \ref Wwdg_Init takes over a running watchdog. The
 * configuration is written with the window opened, the watchdog is started
 * (refreshed) and the final window is written afterwards, so the first write of
 * the counter is never a window violation, even when the watchdog is already running.
 *
 * \note  The counter reload value is not readable from the registers (the counter
 *        register holds the running counter), the module therefore keeps it
 *        and the APB clock frequency read at \ref Wwdg_Init in the runtime data.
 *        They are used to refresh the watchdog by \ref Wwdg_Set_Refresh and by the
 *        getters, which do not use any other module and do not change anything.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Wwdg.h"                           /* Self include                   */
#include "Wwdg_Port.h"                      /* Own port file include          */
#include "Wwdg_Types.h"                     /* Module types definitions       */
#include "Wwdg_Reg.h"                       /* Register access                */
#include "Nvic_Port.h"                      /* NVIC handler functionality     */
#include "Rcc_Port.h"                       /* RCC handler functionality      */
#include "Stm32.h"                          /* WWDG registers definition      */
/* ============================== TYPEDEFS ================================== */

/** \brief Configuration converted to the values of the hardware */
typedef struct
{
    rcc_FreqHz_t         PclkHz;       /**< APB clock frequency the values are calculated with    */
    wwdg_Prescaler_t     PrescId;      /**< Selected prescaler                                    */
    wwdg_Ticks_t         TimeoutTicks; /**< Number of ticks from refresh to reset                 */
    wwdg_Ticks_t         WindowTicks;  /**< Number of ticks of the window, 0 - window not used    */
    wwdg_FunctionState_t EwiState;     /**< Early wakeup interrupt is used                        */
}   wwdg_CalcConfig_t;


/** \brief Runtime data of the module */
typedef struct
{
    wwdg_FunctionState_t   InitState;        /**< Reload value and clock known (set by Wwdg_Init)      */
    wwdg_Ticks_t           TimeoutTicks;     /**< Counter reload value as number of ticks to the reset */
    rcc_FreqHz_t           PclkHz;           /**< APB clock frequency at Wwdg_Init                     */
    wwdg_FunctionState_t   AutoRefreshState; /**< Wwdg_Task refreshes the watchdog                     */
    wwdg_EarlyWakeupIsr_t *EarlyWakeupIsr;   /**< Early wakeup interrupt user callback                 */
}   wwdg_RuntimeData_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static wwdg_RequestState_t  Wwdg_Get_PclkFreq       ( rcc_FreqHz_t * const pclk );
static wwdg_RequestState_t  Wwdg_Get_Timebase       ( wwdg_Divider_t * const divider, rcc_FreqHz_t * const pclk );

static wwdg_RequestState_t  Wwdg_Calc_Ticks         ( wwdg_Time_ms_t time,
                                                      rcc_FreqHz_t pclk,
                                                      wwdg_Divider_t divider,
                                                      wwdg_Ticks_t * const ticks );
static wwdg_RequestState_t  Wwdg_Calc_Prescaler     ( wwdg_Time_ms_t timeout,
                                                      rcc_FreqHz_t pclk,
                                                      wwdg_Prescaler_t * const prescId,
                                                      wwdg_Ticks_t * const ticks );
static wwdg_RequestState_t  Wwdg_Calc_WindowTicks   ( wwdg_Time_ms_t window,
                                                      wwdg_Time_ms_t timeout,
                                                      rcc_FreqHz_t pclk,
                                                      wwdg_Prescaler_t prescId,
                                                      wwdg_Ticks_t timeoutTicks,
                                                      wwdg_Ticks_t * const ticks );
static wwdg_RequestState_t  Wwdg_Calc_Time          ( wwdg_Ticks_t ticks,
                                                      wwdg_Divider_t divider,
                                                      rcc_FreqHz_t pclk,
                                                      wwdg_Time_us_t * const time );
static wwdg_RequestState_t  Wwdg_Calc_Config        ( const wwdg_Config_t * const wwdgConfig,
                                                      wwdg_CalcConfig_t * const calcConfig );

static wwdg_RequestState_t  Wwdg_Set_Watchdog       ( const wwdg_Config_t * const wwdgConfig,
                                                      const wwdg_CalcConfig_t * const calcConfig );
static wwdg_RequestState_t  Wwdg_Set_Registers      ( const wwdg_CalcConfig_t * const calcConfig );
static wwdg_RequestState_t  Wwdg_Set_IsrConfig      ( const wwdg_Config_t * const wwdgConfig );

static void                 Wwdg_IsrHandler         ( void );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define WWDG_MAJOR_VERSION              ( 0u )

/** Value of minor version of SW module */
#define WWDG_MINOR_VERSION              ( 1u )

/** Value of patch version of SW module */
#define WWDG_PATCH_VERSION              ( 0u )

/** Value of the WDGTB field (all bits clear) selecting the prescaler 1 */
#define WWDG_TIMEBASE_DIV_1             ( 0u )

/** Number of milliseconds in one second */
#define WWDG_MS_IN_SECOND               ( 1000u )

/** Number of microseconds in one second */
#define WWDG_US_IN_SECOND               ( 1000000u )

/** Fixed divider of the counter clock (PCLK / 4096), the prescaler follows */
#define WWDG_CLK_DIVIDER                ( 4096u )

/** Frequency value used when the APB clock frequency is not known */
#define WWDG_FREQ_NONE                  ( 0u )

/** Lowest valid APB clock frequency [Hz] */
#define WWDG_FREQ_MIN_HZ                ( 1u )

/** Divider value used when the prescaler is not known */
#define WWDG_DIVIDER_NONE               ( 0u )

/** Lowest valid prescaler divider */
#define WWDG_DIVIDER_MIN                ( 1u )

/** Prescaler divider values of the hardware (WDGTB = 0 ... 7) */
#define WWDG_DIV_1                      ( 1u )
#define WWDG_DIV_2                      ( 2u )
#define WWDG_DIV_4                      ( 4u )
#define WWDG_DIV_8                      ( 8u )
#define WWDG_DIV_16                     ( 16u )
#define WWDG_DIV_32                     ( 32u )
#define WWDG_DIV_64                     ( 64u )
#define WWDG_DIV_128                    ( 128u )

/** Distance between the counter value of the reset (bit T6 cleared) and the bit T6 */
#define WWDG_COUNTER_OFFSET             ( 1u )

/** Counter value at which the reset occurs (bit T6 cleared) - the number of ticks to the reset is
 *  the counter value without this offset */
#define WWDG_RESET_COUNTER              ( WWDG_CR_T_6 - WWDG_COUNTER_OFFSET )

/** Minimal number of watchdog counter ticks (counter value 0x40) */
#define WWDG_TICKS_MIN                  ( 1u )

/** Minimal number of watchdog counter ticks with the early wakeup interrupt. The interrupt is raised at
 *  the counter value 0x40, the shorter timeout would raise it at the start of the watchdog. */
#define WWDG_TICKS_MIN_EWI              ( 2u )

/** Maximal number of watchdog counter ticks (counter value 0x7F) */
#define WWDG_TICKS_MAX                  ( WWDG_CR_T - WWDG_RESET_COUNTER )

/** Number of ticks used to signal that the window is not used (or that no time has elapsed) */
#define WWDG_TICKS_UNUSED               ( 0u )

/** Number of ticks between the early wakeup interrupt and the reset (hardware, fixed) */
#define WWDG_TICKS_EWI                  ( 1u )

/** Window value which allows the refresh during the whole timeout (counter can not be above it) */
#define WWDG_WINDOW_OPEN                ( WWDG_CFR_W >> WWDG_CFR_W_Pos )

/** Default watchdog timeout used by \ref Wwdg_Get_DefaultConfig */
#define WWDG_DEFAULT_TIMEOUT_MS         ( 30u )

/** Default early wakeup interrupt priority used by \ref Wwdg_Get_DefaultConfig */
#define WWDG_DEFAULT_IRQ_PRIO           ( 0u )

/* =============================== MACROS =================================== */

/** Converts the counter value to the value of the T field of CR register */
#define WWDG_COUNTER_TO_REG( counter )  ( ( ( counter ) << WWDG_CR_T_Pos ) & WWDG_CR_T )

/** Converts the value of the T field of CR register to the counter value */
#define WWDG_REG_TO_COUNTER( regVal )   ( ( ( regVal ) & WWDG_CR_T ) >> WWDG_CR_T_Pos )

/** Converts the window value to the value of the W field of CFR register */
#define WWDG_WINDOW_TO_REG( window )    ( ( ( window ) << WWDG_CFR_W_Pos ) & WWDG_CFR_W )

/** Converts the value of the W field of CFR register to the window value */
#define WWDG_REG_TO_WINDOW( regVal )    ( ( ( regVal ) & WWDG_CFR_W ) >> WWDG_CFR_W_Pos )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Prescaler configuration, indexed by \ref wwdg_Prescaler_t */
static const wwdg_PrescalerConfig_t wwdg_PrescalerLut[ ] =
{
    { .RegVal = WWDG_TIMEBASE_DIV_1                    , .Divider = WWDG_DIV_1   }, /**< \ref WWDG_PRESCALER_DIV_1   */
    { .RegVal = WWDG_CFR_WDGTB_0                       , .Divider = WWDG_DIV_2   }, /**< \ref WWDG_PRESCALER_DIV_2   */
    { .RegVal = WWDG_CFR_WDGTB_1                       , .Divider = WWDG_DIV_4   }, /**< \ref WWDG_PRESCALER_DIV_4   */
    { .RegVal = ( WWDG_CFR_WDGTB_1 | WWDG_CFR_WDGTB_0 ), .Divider = WWDG_DIV_8   }, /**< \ref WWDG_PRESCALER_DIV_8   */
    { .RegVal = WWDG_CFR_WDGTB_2                       , .Divider = WWDG_DIV_16  }, /**< \ref WWDG_PRESCALER_DIV_16  */
    { .RegVal = ( WWDG_CFR_WDGTB_2 | WWDG_CFR_WDGTB_0 ), .Divider = WWDG_DIV_32  }, /**< \ref WWDG_PRESCALER_DIV_32  */
    { .RegVal = ( WWDG_CFR_WDGTB_2 | WWDG_CFR_WDGTB_1 ), .Divider = WWDG_DIV_64  }, /**< \ref WWDG_PRESCALER_DIV_64  */
    { .RegVal = WWDG_CFR_WDGTB                         , .Divider = WWDG_DIV_128 }, /**< \ref WWDG_PRESCALER_DIV_128 */
};

_Static_assert( WWDG_PRESCALER_CNT == ( sizeof( wwdg_PrescalerLut ) / sizeof( wwdg_PrescalerConfig_t ) ),
                "WWDG prescaler LUT size mismatch" );


/** \brief Runtime data of the module */
static volatile wwdg_RuntimeData_t wwdg_RuntimeData =
{
    .InitState        = WWDG_FUNCTION_INACTIVE,
    .TimeoutTicks     = WWDG_TICKS_UNUSED,
    .PclkHz           = WWDG_FREQ_NONE,
    .AutoRefreshState = WWDG_FUNCTION_INACTIVE,
    .EarlyWakeupIsr   = WWDG_NULL_PTR,
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
wwdg_ModuleVersion_t Wwdg_Get_ModuleVersion( void )
{
    wwdg_ModuleVersion_t retVersion;

    retVersion.Major = WWDG_MAJOR_VERSION;
    retVersion.Minor = WWDG_MINOR_VERSION;
    retVersion.Patch = WWDG_PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief Initializes and starts the window watchdog.
 *
 * Prescaler, counter reload value and window are calculated from the required
 * timeout and window and the APB clock frequency provided by RCC module (the
 * smallest prescaler with the best resolution is used, the times are rounded down
 * to whole counter ticks). Achieved values can be read by \ref Wwdg_Get_Timeout and
 * \ref Wwdg_Get_Window. Sequence: peripheral clock, interrupt deactivation and
 * configuration (NVIC), prescaler with window opened, start of the watchdog, final
 * window (read-back verification of every register write), interrupt activation.
 *
 * \note  Once started, the watchdog can not be stopped. If the window is not used,
 *        \ref Wwdg_Task refreshes the watchdog automatically. With window,
 *        application has to refresh it by \ref Wwdg_Set_Refresh inside the window.
 * \note  Already running watchdog is taken over: configuration is written and the
 *        counter is reloaded.
 * \note  The early wakeup interrupt enable bit can be set only by software and cleared
 *        only by a system reset. Initialization without callback after an initialization
 *        with callback leaves the bit set, the interrupt is deactivated in NVIC.
 *
 * \pre   Configuration shall be valid: timeout (and window, if used) achievable with
 *        the APB clock (at least one tick), window not longer than the timeout (window
 *        equal to the timeout is allowed), timeout of at least 2 ticks with the early
 *        wakeup callback; otherwise \ref WWDG_REQUEST_ERROR is returned and nothing is
 *        changed. Failure after the first change (interrupt priority refused by NVIC,
 *        mismatch of the read-back) deinitializes the module (\ref Wwdg_Deinit) and
 *        returns \ref WWDG_REQUEST_ERROR - the watchdog may already be running then.
 *
 * \param wwdgConfig [in]: Pointer to WWDG configuration, see \ref wwdg_Config_t. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
wwdg_RequestState_t Wwdg_Init( const wwdg_Config_t * const wwdgConfig )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;
    wwdg_CalcConfig_t   calcConfig;

    if( WWDG_NULL_PTR != wwdgConfig )
    {
        /*------------- Configuration calculation and validation -------------*/
        retState = Wwdg_Calc_Config( wwdgConfig, &calcConfig );

        /*------------------------ Watchdog configuration --------------------*/
        if( WWDG_REQUEST_OK == retState )
        {
            retState = Wwdg_Set_Watchdog( wwdgConfig, &calcConfig );

            if( WWDG_REQUEST_OK != retState )
            {
                /* Self-verification failed - interrupt and automatic refresh are deactivated */
                (void)Wwdg_Deinit();
            }
            else
            {
                /* Watchdog is running with the required configuration */
            }
        }
        else
        {
            /* Invalid configuration, nothing is written */
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes module Wwdg
 *
 * Disables early wakeup interrupt in NVIC, removes user callback and stops
 * automatic refresh from \ref Wwdg_Task.
 *
 * \note  Running watchdog can not be stopped (HW limitation), it keeps running with
 *        its peripheral clock. After Deinit the application has to refresh it by
 *        \ref Wwdg_Set_Refresh, otherwise the MCU is reset after the timeout. The reload
 *        value stays stored, the getters and the refresh keep working.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
wwdg_RequestState_t Wwdg_Deinit( void )
{
    wwdg_RequestState_t       retState  = WWDG_REQUEST_ERROR;
    const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( NVIC_PERIPH_IRQ_WWDG );

    wwdg_RuntimeData.AutoRefreshState = WWDG_FUNCTION_INACTIVE;
    wwdg_RuntimeData.EarlyWakeupIsr   = WWDG_NULL_PTR;

    if( NVIC_REQUEST_OK == nvicState )
    {
        retState = WWDG_REQUEST_OK;
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Main task of module Wwdg
 *
 * Refreshes the watchdog automatically, if the module was initialized without
 * window (see \ref Wwdg_Init). Shall be called periodically with period shorter
 * than configured timeout. With window, or after \ref Wwdg_Deinit, the task does
 * nothing and the application refreshes the watchdog by \ref Wwdg_Set_Refresh.
 */
void Wwdg_Task( void )
{
    if( WWDG_FUNCTION_ACTIVE == wwdg_RuntimeData.AutoRefreshState )
    {
        (void)Wwdg_Set_Refresh();
    }
    else
    {
        /* Automatic refresh is not active */
    }
}


/**
 * \brief Fills configuration structure with default values.
 *
 * Default configuration: timeout 30 ms, window unused, early wakeup interrupt
 * disabled, interrupt priority 0.
 *
 * \param wwdgConfig [out]: Pointer to store default configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
wwdg_RequestState_t Wwdg_Get_DefaultConfig( wwdg_Config_t * const wwdgConfig )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( WWDG_NULL_PTR != wwdgConfig )
    {
        wwdgConfig->Timeout        = WWDG_DEFAULT_TIMEOUT_MS;
        wwdgConfig->Window         = WWDG_TIME_UNUSED;
        wwdgConfig->EarlyWakeupIsr = WWDG_NULL_PTR;
        wwdgConfig->IrqPriority    = WWDG_DEFAULT_IRQ_PRIO;

        retState = WWDG_REQUEST_OK;
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns watchdog running state.
 *
 * The function only reads the activation bit, it does not use RCC and does not
 * change anything.
 *
 * \note  A watchdog started by the hardware (option byte WWDG_SW) before \ref Wwdg_Init
 *        is not read out by this version, the function returns an error until
 *        \ref Wwdg_Init succeeded.
 *
 * \pre   Module shall be initialized by \ref Wwdg_Init (also after \ref Wwdg_Deinit);
 *        otherwise \ref WWDG_REQUEST_ERROR and the state is not changed.
 *
 * \param wwdgState [out]: Pointer to store watchdog state, \ref WWDG_FUNCTION_ACTIVE
 *                         if the watchdog is running. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
wwdg_RequestState_t Wwdg_Get_State( wwdg_FunctionState_t * const wwdgState )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( WWDG_NULL_PTR != wwdgState )
    {
        if( WWDG_FUNCTION_ACTIVE == wwdg_RuntimeData.InitState )
        {
            *wwdgState = Wwdg_Reg_Get_RunState();

            retState = WWDG_REQUEST_OK;
        }
        else
        {
            /* Module is not initialized */
            retState = WWDG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the configured watchdog timeout.
 *
 * Timeout is calculated from the number of ticks stored by \ref Wwdg_Init (the
 * counter reload value is not readable), the prescaler read back from the
 * register and the APB clock frequency read by \ref Wwdg_Init (rounded down to us).
 * The function does not use RCC and does not change anything.
 *
 * \pre   Module shall be initialized by \ref Wwdg_Init (also after \ref Wwdg_Deinit);
 *        otherwise \ref WWDG_REQUEST_ERROR.
 *
 * \param timeout [out]: Pointer to store timeout in us. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
wwdg_RequestState_t Wwdg_Get_Timeout( wwdg_Time_us_t * const timeout )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;
    wwdg_Divider_t      divider  = WWDG_DIVIDER_NONE;
    rcc_FreqHz_t        pclk     = WWDG_FREQ_NONE;

    if( WWDG_NULL_PTR != timeout )
    {
        retState = Wwdg_Get_Timebase( &divider, &pclk );

        if( WWDG_REQUEST_OK == retState )
        {
            retState = Wwdg_Calc_Time( wwdg_RuntimeData.TimeoutTicks, divider, pclk, timeout );
        }
        else
        {
            /* Module is not initialized */
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the configured window.
 *
 * Window (time from the window opening to the reset) is calculated from the window
 * register, the prescaler register and the APB clock frequency read by \ref Wwdg_Init
 * (rounded down to us). The function does not use RCC and does not change anything.
 *
 * \note  Window register at its maximum (counter can be refreshed at any time) is
 *        returned as \ref WWDG_TIME_UNUSED - the whole timeout, also for a window
 *        configured equal to the timeout of 64 ticks.
 *
 * \pre   Module shall be initialized by \ref Wwdg_Init (also after \ref Wwdg_Deinit);
 *        otherwise \ref WWDG_REQUEST_ERROR.
 *
 * \param window [out]: Pointer to store window in us or \ref WWDG_TIME_UNUSED. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
wwdg_RequestState_t Wwdg_Get_Window( wwdg_Time_us_t * const window )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;
    wwdg_Divider_t      divider  = WWDG_DIVIDER_NONE;
    rcc_FreqHz_t        pclk     = WWDG_FREQ_NONE;

    if( WWDG_NULL_PTR != window )
    {
        retState = Wwdg_Get_Timebase( &divider, &pclk );

        if( WWDG_REQUEST_OK == retState )
        {
            const uint32_t windowVal = WWDG_REG_TO_WINDOW( Wwdg_Get_RegVal( WWDG_REG_CFR, WWDG_CFR_W ) );

            if( WWDG_WINDOW_OPEN == windowVal )
            {
                *window = WWDG_TIME_UNUSED;
            }
            else if( WWDG_RESET_COUNTER < windowVal )
            {
                retState = Wwdg_Calc_Time( windowVal - WWDG_RESET_COUNTER, divider, pclk, window );
            }
            else
            {
                /* Window below the bit T6 - the counter could not be refreshed at all */
                retState = WWDG_REQUEST_ERROR;
            }
        }
        else
        {
            /* Module is not initialized */
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the time of the early wakeup interrupt before the reset.
 *
 * The interrupt is raised one counter tick before the reset (hardware), the time
 * is calculated from the prescaler register and the APB clock frequency read by
 * \ref Wwdg_Init (rounded down to us). The function does not use RCC and does not
 * change anything.
 *
 * \note  \ref WWDG_TIME_UNUSED is returned when the interrupt is not used (no callback
 *        configured by \ref Wwdg_Init or removed by \ref Wwdg_Deinit).
 *
 * \pre   Module shall be initialized by \ref Wwdg_Init (also after \ref Wwdg_Deinit);
 *        otherwise \ref WWDG_REQUEST_ERROR.
 *
 * \param earlyWakeup [out]: Pointer to store time in us or \ref WWDG_TIME_UNUSED. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
wwdg_RequestState_t Wwdg_Get_EarlyWakeup( wwdg_Time_us_t * const earlyWakeup )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;
    wwdg_Divider_t      divider  = WWDG_DIVIDER_NONE;
    rcc_FreqHz_t        pclk     = WWDG_FREQ_NONE;

    if( WWDG_NULL_PTR != earlyWakeup )
    {
        retState = Wwdg_Get_Timebase( &divider, &pclk );

        if( WWDG_REQUEST_OK == retState )
        {
            const wwdg_FunctionState_t ewiState = Wwdg_Reg_Get_EwiState();

            if( ( WWDG_NULL_PTR        != wwdg_RuntimeData.EarlyWakeupIsr ) &&
                ( WWDG_FUNCTION_ACTIVE == ewiState                        )    )
            {
                retState = Wwdg_Calc_Time( WWDG_TICKS_EWI, divider, pclk, earlyWakeup );
            }
            else
            {
                *earlyWakeup = WWDG_TIME_UNUSED;
            }
        }
        else
        {
            /* Module is not initialized */
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the time to the reset when the watchdog is not refreshed.
 *
 * The time is calculated from the running counter, the prescaler register and the
 * APB clock frequency read by \ref Wwdg_Init. It is the number of ticks to the reset
 * (the tick in progress counts as whole one, the real time is up to one tick shorter),
 * the refresh is allowed when it is not longer than the window (\ref Wwdg_Get_Window).
 * The function does not use RCC and does not change anything.
 *
 * \pre   Module shall be initialized by \ref Wwdg_Init (also after \ref Wwdg_Deinit);
 *        otherwise \ref WWDG_REQUEST_ERROR.
 *
 * \param remainingTime [out]: Pointer to store the time in us. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
wwdg_RequestState_t Wwdg_Get_RemainingTime( wwdg_Time_us_t * const remainingTime )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;
    wwdg_Divider_t      divider  = WWDG_DIVIDER_NONE;
    rcc_FreqHz_t        pclk     = WWDG_FREQ_NONE;

    if( WWDG_NULL_PTR != remainingTime )
    {
        retState = Wwdg_Get_Timebase( &divider, &pclk );

        if( WWDG_REQUEST_OK == retState )
        {
            const uint32_t counter = WWDG_REG_TO_COUNTER( Wwdg_Get_RegVal( WWDG_REG_CR, WWDG_CR_T ) );
            wwdg_Ticks_t   ticks   = WWDG_TICKS_UNUSED;

            if( WWDG_RESET_COUNTER < counter )
            {
                ticks = counter - WWDG_RESET_COUNTER;
            }
            else
            {
                /* Bit T6 cleared - reset is in progress */
                ticks = WWDG_TICKS_UNUSED;
            }

            retState = Wwdg_Calc_Time( ticks, divider, pclk, remainingTime );
        }
        else
        {
            /* Module is not initialized */
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Refreshes (reloads) the watchdog counter.
 *
 * The counter is reloaded with the value stored by \ref Wwdg_Init. The function does not
 * use other modules and can be called from the early wakeup callback.
 *
 * \note  With window configured, refresh before the window resets the MCU
 *        immediately (HW behavior), it can not be reported by software. Use
 *        \ref Wwdg_Get_RemainingTime to decide whether the window is open.
 *
 * \pre   Module shall be initialized by \ref Wwdg_Init (reload value is known);
 *        otherwise \ref WWDG_REQUEST_ERROR and no register is changed. The refresh
 *        is possible also after \ref Wwdg_Deinit.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
wwdg_RequestState_t Wwdg_Set_Refresh( void )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( WWDG_FUNCTION_ACTIVE == wwdg_RuntimeData.InitState )
    {
        retState = Wwdg_Reg_Set_Refresh( WWDG_COUNTER_TO_REG( WWDG_RESET_COUNTER + wwdg_RuntimeData.TimeoutTicks ) );
    }
    else
    {
        /* Reload value is not known */
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Returns WWDG kernel clock (APB clock) frequency from RCC module.
 *
 * \param pclk [out]: Pointer to store the frequency in Hz. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if the frequency
 *         is available and non-zero. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
static wwdg_RequestState_t Wwdg_Get_PclkFreq( rcc_FreqHz_t * const pclk )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( WWDG_NULL_PTR != pclk )
    {
        const rcc_RequestState_t rccState = Rcc_Get_PeriphClk( RCC_PERIPH_WWDG, pclk );

        if( ( RCC_REQUEST_OK    == rccState ) &&
            ( WWDG_FREQ_MIN_HZ  <= *pclk    )    )
        {
            retState = WWDG_REQUEST_OK;
        }
        else
        {
            retState = WWDG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the prescaler divider of the watchdog and the APB clock frequency stored by Wwdg_Init.
 *
 * Uses no other module. The prescaler divider is read from the register, the frequency
 * is the one of the initialization (the module has to be initialized again after a change
 * of the bus clock).
 *
 * \param divider [out]: Pointer to store the prescaler divider. Must not be NULL.
 * \param pclk    [out]: Pointer to store the APB frequency in Hz. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if the module is
 *         initialized and both values are available. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
static wwdg_RequestState_t Wwdg_Get_Timebase( wwdg_Divider_t * const divider, rcc_FreqHz_t * const pclk )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( ( WWDG_NULL_PTR != divider ) &&
        ( WWDG_NULL_PTR != pclk    )    )
    {
        if( WWDG_FUNCTION_ACTIVE == wwdg_RuntimeData.InitState )
        {
            const uint32_t timebase = Wwdg_Get_RegVal( WWDG_REG_CFR, WWDG_CFR_WDGTB );

            for( wwdg_Prescaler_t searchId = WWDG_PRESCALER_DIV_1; WWDG_PRESCALER_CNT > searchId; searchId ++ )
            {
                if( wwdg_PrescalerLut[ searchId ].RegVal == timebase )
                {
                    *divider = wwdg_PrescalerLut[ searchId ].Divider;
                    *pclk    = wwdg_RuntimeData.PclkHz;
                    retState = WWDG_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Continue with next prescaler */
                }
            }
        }
        else
        {
            /* Module is not initialized */
            retState = WWDG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Calculates number of watchdog counter ticks for required time.
 *
 * \param time    [in]: Required time in ms
 * \param pclk    [in]: APB frequency in Hz, must be non-zero
 * \param divider [in]: Prescaler divider value, must be non-zero
 * \param ticks  [out]: Pointer to store number of ticks (rounded down). Must not be NULL.
 *
 * \return Returns \ref WWDG_REQUEST_OK if the number of ticks is in range
 *         \ref WWDG_TICKS_MIN - \ref WWDG_TICKS_MAX. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
static wwdg_RequestState_t Wwdg_Calc_Ticks( wwdg_Time_ms_t time,
                                            rcc_FreqHz_t pclk,
                                            wwdg_Divider_t divider,
                                            wwdg_Ticks_t * const ticks )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( ( WWDG_NULL_PTR    != ticks   ) &&
        ( WWDG_FREQ_MIN_HZ <= pclk    ) &&
        ( WWDG_DIVIDER_MIN <= divider )    )
    {
        const uint64_t tickPeriod = (uint64_t)divider * WWDG_CLK_DIVIDER * WWDG_MS_IN_SECOND;
        const uint64_t tickCnt    = ( (uint64_t)time * pclk ) / tickPeriod;

        if( ( WWDG_TICKS_MIN <= tickCnt ) &&
            ( WWDG_TICKS_MAX >= tickCnt )    )
        {
            *ticks   = (wwdg_Ticks_t)tickCnt;
            retState = WWDG_REQUEST_OK;
        }
        else
        {
            /* Time is shorter than one tick or not achievable with this prescaler */
            retState = WWDG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the smallest prescaler which can achieve required timeout.
 *
 * \param timeout  [in]: Required timeout in ms, must be non-zero
 * \param pclk     [in]: APB frequency in Hz
 * \param prescId [out]: Pointer to store prescaler identification. Must not be NULL.
 * \param ticks   [out]: Pointer to store number of timeout ticks. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if timeout
 *         is achievable. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
static wwdg_RequestState_t Wwdg_Calc_Prescaler( wwdg_Time_ms_t timeout,
                                                rcc_FreqHz_t pclk,
                                                wwdg_Prescaler_t * const prescId,
                                                wwdg_Ticks_t * const ticks )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( ( WWDG_NULL_PTR    != prescId ) &&
        ( WWDG_NULL_PTR    != ticks   ) &&
        ( WWDG_TIME_UNUSED != timeout )    )
    {
        for( wwdg_Prescaler_t searchId = WWDG_PRESCALER_DIV_1; WWDG_PRESCALER_CNT > searchId; searchId ++ )
        {
            retState = Wwdg_Calc_Ticks( timeout, pclk, wwdg_PrescalerLut[ searchId ].Divider, ticks );

            if( WWDG_REQUEST_OK == retState )
            {
                *prescId = searchId;
                break;
            }
            else
            {
                /* Timeout is too long for this prescaler, try next one */
            }
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Calculates number of ticks of the window.
 *
 * The window is the last part of the timeout, it can be as long as the timeout
 * (refresh allowed immediately), not longer.
 *
 * \param window       [in]: Required window in ms
 * \param timeout      [in]: Required timeout in ms
 * \param pclk         [in]: APB frequency in Hz
 * \param prescId      [in]: Selected prescaler, value from \ref wwdg_Prescaler_t
 * \param timeoutTicks [in]: Number of timeout ticks
 * \param ticks       [out]: Pointer to store number of window ticks. Must not be NULL.
 *
 * \return Returns \ref WWDG_REQUEST_OK if the window is achievable (at least one tick) and
 *         not longer than the timeout. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
static wwdg_RequestState_t Wwdg_Calc_WindowTicks( wwdg_Time_ms_t window,
                                                  wwdg_Time_ms_t timeout,
                                                  rcc_FreqHz_t pclk,
                                                  wwdg_Prescaler_t prescId,
                                                  wwdg_Ticks_t timeoutTicks,
                                                  wwdg_Ticks_t * const ticks )
{
    wwdg_RequestState_t retState  = WWDG_REQUEST_ERROR;
    wwdg_Ticks_t        windowCnt = WWDG_TICKS_UNUSED;

    if( ( WWDG_NULL_PTR      != ticks   ) &&
        ( WWDG_PRESCALER_CNT  > prescId ) &&
        ( timeout            >= window  )    )
    {
        retState = Wwdg_Calc_Ticks( window, pclk, wwdg_PrescalerLut[ prescId ].Divider, &windowCnt );

        if( ( WWDG_REQUEST_OK == retState  ) &&
            ( timeoutTicks    >= windowCnt )    )
        {
            *ticks = windowCnt;
        }
        else
        {
            /* Window is not achievable (shorter than one tick) or not within the timeout */
            retState = WWDG_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter or window longer than the timeout */
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Calculates the time of a number of watchdog counter ticks.
 *
 * \param ticks   [in]: Number of ticks
 * \param divider [in]: Prescaler divider value
 * \param pclk    [in]: APB frequency in Hz, must be non-zero
 * \param time   [out]: Pointer to store the time in us (rounded down). Must not be NULL.
 *
 * \return Returns \ref WWDG_REQUEST_OK if the time fits to the type. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
static wwdg_RequestState_t Wwdg_Calc_Time( wwdg_Ticks_t ticks,
                                           wwdg_Divider_t divider,
                                           rcc_FreqHz_t pclk,
                                           wwdg_Time_us_t * const time )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( ( WWDG_NULL_PTR    != time ) &&
        ( WWDG_FREQ_MIN_HZ <= pclk )    )
    {
        const uint64_t tickTime = (uint64_t)ticks * divider * WWDG_CLK_DIVIDER * WWDG_US_IN_SECOND;
        const uint64_t timeUs   = tickTime / pclk;

        if( UINT32_MAX >= timeUs )
        {
            *time    = (wwdg_Time_us_t)timeUs;
            retState = WWDG_REQUEST_OK;
        }
        else
        {
            /* Time does not fit to the type (APB clock below 8 kHz) */
            retState = WWDG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Converts and validates the configuration, no register is touched.
 *
 * \param wwdgConfig  [in]: Pointer to WWDG configuration. Must not be NULL.
 * \param calcConfig [out]: Pointer to store the values of the hardware. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if the configuration is
 *         valid and achievable. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
static wwdg_RequestState_t Wwdg_Calc_Config( const wwdg_Config_t * const wwdgConfig,
                                             wwdg_CalcConfig_t * const calcConfig )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( ( WWDG_NULL_PTR != wwdgConfig ) &&
        ( WWDG_NULL_PTR != calcConfig )    )
    {
        calcConfig->PclkHz       = WWDG_FREQ_NONE;
        calcConfig->PrescId      = WWDG_PRESCALER_CNT;
        calcConfig->TimeoutTicks = WWDG_TICKS_UNUSED;
        calcConfig->WindowTicks  = WWDG_TICKS_UNUSED;
        calcConfig->EwiState     = WWDG_FUNCTION_INACTIVE;

        retState = Wwdg_Get_PclkFreq( &calcConfig->PclkHz );

        if( WWDG_REQUEST_OK == retState )
        {
            retState = Wwdg_Calc_Prescaler( wwdgConfig->Timeout,
                                            calcConfig->PclkHz,
                                            &calcConfig->PrescId,
                                            &calcConfig->TimeoutTicks );
        }
        else
        {
            /* APB clock is not available, keep error */
        }

        if( ( WWDG_REQUEST_OK  == retState           ) &&
            ( WWDG_TIME_UNUSED != wwdgConfig->Window )    )
        {
            retState = Wwdg_Calc_WindowTicks( wwdgConfig->Window,
                                              wwdgConfig->Timeout,
                                              calcConfig->PclkHz,
                                              calcConfig->PrescId,
                                              calcConfig->TimeoutTicks,
                                              &calcConfig->WindowTicks );
        }
        else
        {
            /* Window is not used or previous step failed */
        }

        if( ( WWDG_REQUEST_OK == retState                   ) &&
            ( WWDG_NULL_PTR   != wwdgConfig->EarlyWakeupIsr )    )
        {
            if( WWDG_TICKS_MIN_EWI <= calcConfig->TimeoutTicks )
            {
                calcConfig->EwiState = WWDG_FUNCTION_ACTIVE;
            }
            else
            {
                /* Interrupt would be raised at the start of the watchdog with a shorter timeout */
                retState = WWDG_REQUEST_ERROR;
            }
        }
        else
        {
            /* Early wakeup interrupt is not used or previous step failed */
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Applies the validated configuration: clock, interrupt, registers.
 *
 * \param wwdgConfig  [in]: Pointer to WWDG configuration. Must not be NULL.
 * \param calcConfig  [in]: Pointer to the values of the hardware. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if the watchdog runs with the
 *         required configuration. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
static wwdg_RequestState_t Wwdg_Set_Watchdog( const wwdg_Config_t * const wwdgConfig,
                                              const wwdg_CalcConfig_t * const calcConfig )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( ( WWDG_NULL_PTR != wwdgConfig ) &&
        ( WWDG_NULL_PTR != calcConfig )    )
    {
        /*------- Automatic refresh and callback of the previous Init ---------*/
        wwdg_RuntimeData.AutoRefreshState = WWDG_FUNCTION_INACTIVE;
        wwdg_RuntimeData.EarlyWakeupIsr   = WWDG_NULL_PTR;

        /*------------------------- Peripheral clock -------------------------*/
        const rcc_RequestState_t rccState = Rcc_Set_PeriphActive( RCC_PERIPH_WWDG );

        if( RCC_REQUEST_OK == rccState )
        {
            /*---------------- Interrupt deactivation (reconfiguration) -------*/
            const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( NVIC_PERIPH_IRQ_WWDG );

            if( NVIC_REQUEST_OK == nvicState )
            {
                retState = WWDG_REQUEST_OK;
            }
            else
            {
                retState = WWDG_REQUEST_ERROR;
            }
        }
        else
        {
            /* Peripheral clock can not be activated */
            retState = WWDG_REQUEST_ERROR;
        }

        /*----------- Interrupt configuration (activated at the end) ----------*/
        if( ( WWDG_REQUEST_OK      == retState             ) &&
            ( WWDG_FUNCTION_ACTIVE == calcConfig->EwiState )    )
        {
            retState = Wwdg_Set_IsrConfig( wwdgConfig );
        }
        else
        {
            /* Early wakeup interrupt is not used or previous step failed */
        }

        /*------------------------ Watchdog registers -------------------------*/
        if( WWDG_REQUEST_OK == retState )
        {
            retState = Wwdg_Set_Registers( calcConfig );
        }
        else
        {
            /* Previous step failed */
        }

        /*----------------------- Interrupt activation ------------------------*/
        if( ( WWDG_REQUEST_OK      == retState             ) &&
            ( WWDG_FUNCTION_ACTIVE == calcConfig->EwiState )    )
        {
            const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Active( NVIC_PERIPH_IRQ_WWDG );

            if( NVIC_REQUEST_OK != nvicState )
            {
                retState = WWDG_REQUEST_ERROR;
            }
            else
            {
                /* Interrupt is active */
            }
        }
        else
        {
            /* Early wakeup interrupt is not used or previous step failed */
        }

        if( ( WWDG_REQUEST_OK   == retState                ) &&
            ( WWDG_TICKS_UNUSED == calcConfig->WindowTicks )    )
        {
            /* Refresh at any time is allowed only without window */
            wwdg_RuntimeData.AutoRefreshState = WWDG_FUNCTION_ACTIVE;
        }
        else
        {
            /* Application has to refresh the watchdog inside the window */
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the prescaler, starts the watchdog and writes the window.
 *
 * The prescaler (and the interrupt enable) is written with the window opened and the
 * counter is written (watchdog started or reloaded) afterwards. The window is written
 * last, so the counter is never written above the window - not even when the watchdog
 * is already running. The reload value and the APB clock frequency are stored from the
 * start on (the refresh needs the reload value).
 *
 * \param calcConfig [in]: Pointer to the values of the hardware. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if all register values were
 *         read back. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
static wwdg_RequestState_t Wwdg_Set_Registers( const wwdg_CalcConfig_t * const calcConfig )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( ( WWDG_NULL_PTR     != calcConfig               ) &&
        ( WWDG_PRESCALER_CNT > calcConfig->PrescId      ) &&
        ( WWDG_TICKS_MIN    <= calcConfig->TimeoutTicks )    )
    {
        const uint32_t timebase = wwdg_PrescalerLut[ calcConfig->PrescId ].RegVal;
        const uint32_t counter  = WWDG_COUNTER_TO_REG( WWDG_RESET_COUNTER + calcConfig->TimeoutTicks );

        /* -1- Prescaler and early wakeup interrupt, window opened */
        retState = Wwdg_Reg_Set_Config( timebase, WWDG_WINDOW_TO_REG( WWDG_WINDOW_OPEN ), calcConfig->EwiState );

        /* -2- Start (reload) of the counter - the reload value is needed from now by the refresh */
        if( WWDG_REQUEST_OK == retState )
        {
            wwdg_RuntimeData.TimeoutTicks = calcConfig->TimeoutTicks;
            wwdg_RuntimeData.PclkHz       = calcConfig->PclkHz;
            wwdg_RuntimeData.InitState    = WWDG_FUNCTION_ACTIVE;

            retState = Wwdg_Reg_Set_Start( counter );
        }
        else
        {
            /* Previous step failed, watchdog is not started */
        }

        /* -3- Window */
        if( ( WWDG_REQUEST_OK   == retState                ) &&
            ( WWDG_TICKS_UNUSED != calcConfig->WindowTicks )    )
        {
            const uint32_t window = WWDG_WINDOW_TO_REG( WWDG_RESET_COUNTER + calcConfig->WindowTicks );

            retState = Wwdg_Reg_Set_Config( timebase, window, calcConfig->EwiState );
        }
        else
        {
            /* Window is not used or previous step failed */
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures early wakeup interrupt in NVIC module (the interrupt is not activated).
 *
 * \param wwdgConfig [in]: Pointer to WWDG configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref WWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
static wwdg_RequestState_t Wwdg_Set_IsrConfig( const wwdg_Config_t * const wwdgConfig )
{
    wwdg_RequestState_t retState  = WWDG_REQUEST_ERROR;
    nvic_RequestState_t nvicState = NVIC_REQUEST_ERROR;

    if( WWDG_NULL_PTR != wwdgConfig )
    {
        wwdg_RuntimeData.EarlyWakeupIsr = wwdgConfig->EarlyWakeupIsr;

        nvicState = Nvic_Set_PeriphIrq_Handler( NVIC_PERIPH_IRQ_WWDG, Wwdg_IsrHandler );

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_WWDG, wwdgConfig->IrqPriority );
        }
        else
        {
            /* Handler configuration failed */
        }

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = WWDG_REQUEST_OK;
        }
        else
        {
            retState = WWDG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/**
 * \brief WWDG early wakeup interrupt handler.
 *
 * Acknowledges early wakeup flag and calls user callback.
 *
 * \note  Early wakeup flag is acknowledged by writing 0, before the callback, so the
 *        callback can refresh the watchdog and the flag is not left pending.
 */
static void Wwdg_IsrHandler( void )
{
    const wwdg_FlagState_t ewiFlag = Wwdg_Reg_Get_EwiFlag();

    if( WWDG_FLAG_ACTIVE == ewiFlag )
    {
        Wwdg_Reg_Clear_EwiFlag();

        if( WWDG_NULL_PTR != wwdg_RuntimeData.EarlyWakeupIsr )
        {
            wwdg_RuntimeData.EarlyWakeupIsr();
        }
        else
        {
            /* User callback is not configured */
        }
    }
    else
    {
        /* Early wakeup interrupt is not pending */
    }
}

/* ================================ TASKS =================================== */
