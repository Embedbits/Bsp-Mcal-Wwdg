/**
 * \author Mr.Nobody
 * \file ItTest_Wwdg.c
 * \ingroup Wwdg
 * \brief Integration tests of System window watchdog (WWDG) module on target.
 *
 * Wwdg module runs on the MCU together with real RCC and NVIC modules and
 * hardware (APB clock). Tests verify behavior which cannot be verified by unit
 * tests (emulated registers): watchdog start, reset of the MCU after timeout,
 * refresh, window violation (refresh too early), achieved timing and early wakeup
 * interrupt.
 *
 * Tests expecting reset of the MCU use stages of the integration testing
 * (\c IntegrationTesting_Set_ResetExpected), stage 1 checks the reset source
 * through RCC module. Running watchdog is stopped by the system reset done by
 * the test runner after every test case.
 *
 * Used resources:
 * - WWDG (APB1 clock)
 * - TIM2 - reference time base 10 kHz (\ref ItWwdg_Get_Time_ms), no external wiring
 *
 * \note The watchdog is frozen while the core is halted by the host (ItTarget_Wwdg.c),
 *       so the time measurements stay consistent.
 * \note The tests need the software watchdog mode (option byte WWDG_SW set, the
 *       default of the devices): a watchdog started by the hardware could not be stopped
 *       by the reset of the test runner.
 * \note Times are chosen for every APB clock of the device (the longest timeout of
 *       the WWDG is 134 ms at 250 MHz). Tolerances cover one counter tick (up to
 *       a few ms with the largest prescaler) and the rounding of the module (down to
 *       whole ticks).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Wwdg_Port.h"                      /* Module under test              */
#include "Rcc_Port.h"                       /* Reset source                   */
#include "Tim_Port.h"                       /* Reference time base            */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void             ItWwdg_Init_RefTime     ( void );
static wwdg_Time_ms_t   ItWwdg_Get_Time_ms      ( void );
static void             ItWwdg_Wait_Until_ms    ( wwdg_Time_ms_t time_ms, wwdg_FunctionState_t taskState );
static void             ItWwdg_Init_Watchdog    ( wwdg_Time_ms_t timeout,
                                                  wwdg_Time_ms_t window,
                                                  wwdg_EarlyWakeupIsr_t *isr );
static void             ItWwdg_Check_WwdgReset  ( void );

static void             ItWwdg_EarlyWakeupCallback( void );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Number of milliseconds in one second */
#define IT_WWDG_MS_IN_SECOND                ( 1000u )

/** Reference timer (32-bit) */
#define IT_WWDG_REF_TIM                     ( TIM_PERIPH_2 )

/** Counter frequency of the reference timer [Hz] */
#define IT_WWDG_REF_FREQ_HZ                 ( 10000u )

/** Reference timer counts per millisecond */
#define IT_WWDG_REF_COUNTS_PER_MS           ( IT_WWDG_REF_FREQ_HZ / IT_WWDG_MS_IN_SECOND )

/** Maximal count of the iterations of a wait loop (the loops end by the reference time) */
#define IT_WWDG_LOOP_MAX                    ( 0xFFFFFFFFu )

/** Number of the stage in which the test starts the watchdog (stage 1 follows the reset) */
#define IT_WWDG_STAGE_START                 ( 0u )

/** Watchdog timeout used by tests [ms] */
#define IT_WWDG_TIMEOUT_MS                  ( 50u )

/** Window used by tests [ms] - refresh allowed in the last 20 ms of the timeout */
#define IT_WWDG_WINDOW_MS                   ( 20u )

/** Time without refresh after which the reset must have occurred [ms] */
#define IT_WWDG_RESET_WAIT_MS               ( 500u )

/** One millisecond (window longer than the timeout by the smallest step) */
#define IT_WWDG_ONE_MS                      ( 1u )

/** Time during which a watchdog which was not started must not reset the MCU [ms] */
#define IT_WWDG_NOT_STARTED_WAIT_MS         ( 200u )

/** Timeout which no APB clock of the device can achieve [ms] */
#define IT_WWDG_TIMEOUT_TOO_LONG_MS         ( 100000u )

/** Relative tolerance of watchdog times (rounding down to counter ticks, tick in progress) [%] */
#define IT_WWDG_TIME_TOL_PCT                ( 20u )

/** Number of percent in one */
#define IT_WWDG_PCT_100                     ( 100u )

/** Number of microseconds in a millisecond */
#define IT_WWDG_US_IN_MS                    ( 1000u )

/** Count of the timeouts of the periodic refresh tests */
#define IT_WWDG_PERIODS_CNT                 ( 5u )

/** Count of the manual refreshes inside the window */
#define IT_WWDG_WINDOW_REFRESH_CNT          ( 3u )

/** Divider of the timeout giving the time between the manual refreshes without window */
#define IT_WWDG_HALF_DIV                    ( 2u )

/** Divider of the timeout giving the upper limit of the early wakeup time (one tick) */
#define IT_WWDG_QUARTER_DIV                 ( 4u )

/** Part of the timeout [%] which the first read of the remaining time has to reach at least */
#define IT_WWDG_REMAINING_MIN_PCT           ( 60u )

/** Decrease of the remaining time expected after the wait (at least) [ms] */
#define IT_WWDG_REMAINING_DECREASE_MS       ( 15u )

/** No call of the early wakeup callback */
#define IT_WWDG_CALLS_NONE                  ( 0u )

/** One call of the early wakeup callback (or one iteration) */
#define IT_WWDG_CALLS_ONCE                  ( 1u )

/** Time of the reference timer after its start / restart [ms] */
#define IT_WWDG_TIME_ZERO_MS                ( 0u )

/** Value of the reference timer after the restart */
#define IT_WWDG_COUNTER_ZERO                ( 0u )

/** Value of the reference timer period (free running 32-bit counter) */
#define IT_WWDG_REF_PERIOD                  ( 0xFFFFFFFFu )

/** Refresh frequency of the reference timer (parameter of the timer configuration) */
#define IT_WWDG_REF_REFRESH_HZ              ( 1u )

/* ============================== MACROS ==================================== */

/** Absolute tolerance of watchdog time */
#define IT_WWDG_TIME_TOL( time )            ( ( ( time ) * IT_WWDG_TIME_TOL_PCT ) / IT_WWDG_PCT_100 )

/* ========================== LOCAL VARIABLES =============================== */

/** Count of early wakeup callback calls */
static volatile uint32_t                itWwdg_EarlyWakeupCnt   = IT_WWDG_CALLS_NONE;

/** Reference time of the first early wakeup callback call [ms] */
static volatile wwdg_Time_ms_t          itWwdg_EarlyWakeupTime  = IT_WWDG_TIME_ZERO_MS;

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    itWwdg_EarlyWakeupCnt  = IT_WWDG_CALLS_NONE;
    itWwdg_EarlyWakeupTime = IT_WWDG_TIME_ZERO_MS;

    ItWwdg_Init_RefTime();
}


void tearDown( void )
{
    /* Running watchdog is stopped by system reset after the test case */
}

/* =============================== TESTS ==================================== */

/*----------------------------- State and init -------------------------------*/

/**
 * \brief   Getters and refresh reject the module before the initialization.
 *
 * \details Calls the getters and the refresh without Wwdg_Init() (system reset done by the test
 *          runner, software watchdog mode).
 *
 * \par Expected results
 * - Wwdg_Get_State(), Wwdg_Get_Timeout() and Wwdg_Set_Refresh() return WWDG_REQUEST_ERROR.
 */
void It_Wwdg_Get_State_BeforeInit_ReturnsError( void )
{
    wwdg_FunctionState_t state   = WWDG_FUNCTION_ACTIVE;
    wwdg_Time_us_t       timeout = WWDG_TIME_UNUSED;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Set_Refresh() );
}


/**
 * \brief   Wwdg_Init() with default configuration starts the watchdog.
 *
 * \details Initializes the watchdog with default configuration, reads the state and
 *          the achieved timeout and refreshes it.
 *
 * \par Expected results
 * - Wwdg_Init() returns WWDG_REQUEST_OK, state reads back WWDG_FUNCTION_ACTIVE.
 * - Timeout reads back the configured value +- 20 % (rounded down to whole ticks).
 * - Wwdg_Set_Refresh() returns WWDG_REQUEST_OK.
 */
void It_Wwdg_Init_DefaultConfig_RunningWithRequestedTimeout( void )
{
    wwdg_Config_t        config;
    wwdg_FunctionState_t state   = WWDG_FUNCTION_INACTIVE;
    wwdg_Time_us_t       timeout = WWDG_TIME_UNUSED;
    wwdg_Time_us_t       expected;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( WWDG_FUNCTION_ACTIVE, state );

    expected = config.Timeout * IT_WWDG_US_IN_MS;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_UINT32_WITHIN( IT_WWDG_TIME_TOL( expected ), expected, timeout );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Set_Refresh() );
}


/**
 * \brief   Wwdg_Init() rejects timeout above the hardware maximum.
 *
 * \details Initializes the watchdog with timeout 100 s (maximum is 64 * 128 * 4096 / APB clock,
 *          approx. 134 ms at 250 MHz, 1 s at 32 MHz), then waits longer than the default
 *          timeout without refresh.
 *
 * \par Expected results
 * - Wwdg_Init() returns WWDG_REQUEST_ERROR, the module stays not initialized (getters and
 *   refresh return WWDG_REQUEST_ERROR).
 * - Watchdog is not started: no reset of the MCU during the wait.
 */
void It_Wwdg_Init_TimeoutNotAchievable_ReturnsErrorNotStarted( void )
{
    wwdg_Config_t        config;
    wwdg_FunctionState_t state = WWDG_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_DefaultConfig( &config ) );
    config.Timeout = IT_WWDG_TIMEOUT_TOO_LONG_MS;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Set_Refresh() );

    ItWwdg_Wait_Until_ms( IT_WWDG_NOT_STARTED_WAIT_MS, WWDG_FUNCTION_INACTIVE );
}


/**
 * \brief   Wwdg_Init() rejects window longer than the timeout.
 *
 * \details Initializes the watchdog with window longer than the timeout, then waits longer than the
 *          timeout without refresh.
 *
 * \par Expected results
 * - Wwdg_Init() returns WWDG_REQUEST_ERROR, the module stays not initialized.
 * - Watchdog is not started: no reset of the MCU during the wait.
 */
void It_Wwdg_Init_WindowLongerThanTimeout_ReturnsErrorNotStarted( void )
{
    wwdg_Config_t        config;
    wwdg_FunctionState_t state = WWDG_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_DefaultConfig( &config ) );
    config.Timeout = IT_WWDG_TIMEOUT_MS;
    config.Window  = IT_WWDG_TIMEOUT_MS + IT_WWDG_ONE_MS;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Set_Refresh() );

    ItWwdg_Wait_Until_ms( IT_WWDG_NOT_STARTED_WAIT_MS, WWDG_FUNCTION_INACTIVE );
}

/*-------------------------------- Timeout -----------------------------------*/

/**
 * \brief   Watchdog without refresh resets the MCU after timeout.
 *
 * \details
 * 1. Stage 0: expects reset, starts the watchdog with 50 ms timeout and waits
 *    without refresh (Wwdg_Task not called) up to 500 ms.
 * 2. Stage 1 (after reset): checks the reset source.
 *
 * \par Expected results
 * 1. No reset before timeout - tolerance (40 ms), MCU is reset before 500 ms.
 * 2. Reset source is WWDG, software reset flag is not set.
 */
void It_Wwdg_Init_NoRefresh_ResetsMcuAfterTimeout( void )
{
    const integrationTesting_Stage_t testStage = IntegrationTesting_Get_Stage();

    if( IT_WWDG_STAGE_START == testStage )
    {
        TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_ResetSourceClear() );
        IntegrationTesting_Set_ResetExpected();
        ItWwdg_Init_Watchdog( IT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, WWDG_NULL_PTR );

        /* No reset before timeout */
        ItWwdg_Wait_Until_ms( IT_WWDG_TIMEOUT_MS - IT_WWDG_TIME_TOL( IT_WWDG_TIMEOUT_MS ), WWDG_FUNCTION_INACTIVE );
        ItWwdg_Wait_Until_ms( IT_WWDG_RESET_WAIT_MS, WWDG_FUNCTION_INACTIVE );

        TEST_FAIL_MESSAGE( "WWDG reset did not occur" );
    }
    else
    {
        ItWwdg_Check_WwdgReset();
    }
}


/**
 * \brief   Periodic Wwdg_Task() keeps the MCU running.
 *
 * \details Starts the watchdog with 50 ms timeout and calls Wwdg_Task() continuously
 *          during 5 timeouts (250 ms).
 *
 * \par Expected results
 * - No reset of the MCU, Wwdg_Set_Refresh() returns WWDG_REQUEST_OK at the end.
 */
void It_Wwdg_Task_PeriodicCall_NoResetAfterTimeout( void )
{
    ItWwdg_Init_Watchdog( IT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, WWDG_NULL_PTR );

    /* Automatic refresh by Wwdg_Task during 5 timeouts */
    ItWwdg_Wait_Until_ms( IT_WWDG_PERIODS_CNT * IT_WWDG_TIMEOUT_MS, WWDG_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Set_Refresh() );
}


/**
 * \brief   Periodic Wwdg_Set_Refresh() keeps the MCU running.
 *
 * \details Starts the watchdog with 50 ms timeout, Wwdg_Task() is not called. The
 *          watchdog is refreshed manually every 25 ms during 5 timeouts (250 ms).
 *
 * \par Expected results
 * - No reset of the MCU, every Wwdg_Set_Refresh() returns WWDG_REQUEST_OK.
 */
void It_Wwdg_Set_Refresh_PeriodicCall_NoResetAfterTimeout( void )
{
    wwdg_Time_ms_t refreshTime = IT_WWDG_TIME_ZERO_MS;

    ItWwdg_Init_Watchdog( IT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, WWDG_NULL_PTR );

    /* Manual refresh every half of timeout during 5 timeouts */
    for( uint32_t refreshIdx = 0u; ( IT_WWDG_PERIODS_CNT * IT_WWDG_HALF_DIV ) > refreshIdx; refreshIdx++ )
    {
        refreshTime += IT_WWDG_TIMEOUT_MS / IT_WWDG_HALF_DIV;
        ItWwdg_Wait_Until_ms( refreshTime, WWDG_FUNCTION_INACTIVE );

        TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Set_Refresh() );
    }
}


/**
 * \brief   Wwdg_Deinit() stops automatic refresh, running watchdog resets the MCU.
 *
 * \details
 * 1. Stage 0: expects reset, starts the watchdog with 50 ms timeout, deinitializes
 *    the module and calls Wwdg_Task() continuously up to 500 ms.
 * 2. Stage 1 (after reset): checks the reset source.
 *
 * \par Expected results
 * 1. Wwdg_Deinit() returns WWDG_REQUEST_OK, Wwdg_Task() does not refresh the watchdog
 *    (the watchdog can not be stopped) - MCU is reset before 500 ms.
 * 2. Reset source is WWDG, software reset flag is not set.
 */
void It_Wwdg_Deinit_AutoRefreshStopped_ResetsMcu( void )
{
    const integrationTesting_Stage_t testStage = IntegrationTesting_Get_Stage();

    if( IT_WWDG_STAGE_START == testStage )
    {
        TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_ResetSourceClear() );
        IntegrationTesting_Set_ResetExpected();
        ItWwdg_Init_Watchdog( IT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, WWDG_NULL_PTR );

        TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Deinit() );

        /* Wwdg_Task does not refresh after Deinit, watchdog keeps running */
        ItWwdg_Wait_Until_ms( IT_WWDG_RESET_WAIT_MS, WWDG_FUNCTION_ACTIVE );

        TEST_FAIL_MESSAGE( "WWDG reset did not occur" );
    }
    else
    {
        ItWwdg_Check_WwdgReset();
    }
}


/**
 * \brief   Remaining time decreases with the time since the refresh.
 *
 * \details Starts the watchdog with 50 ms timeout, refreshes it and reads the remaining time
 *          immediately and after 25 ms (Wwdg_Task() not called).
 *
 * \par Expected results
 * - First read is not longer than the timeout and at least 60 % of it.
 * - Second read is at least 15 ms shorter than the first one.
 */
void It_Wwdg_Get_RemainingTime_AfterRefresh_DecreasesWithTime( void )
{
    wwdg_Time_us_t firstTime  = WWDG_TIME_UNUSED;
    wwdg_Time_us_t secondTime = WWDG_TIME_UNUSED;
    wwdg_Time_us_t timeout    = WWDG_TIME_UNUSED;
    wwdg_Time_ms_t waitTime   = IT_WWDG_TIME_ZERO_MS;

    ItWwdg_Init_Watchdog( IT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, WWDG_NULL_PTR );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Timeout( &timeout ) );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Set_Refresh() );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_RemainingTime( &firstTime ) );

    waitTime = ItWwdg_Get_Time_ms() + ( IT_WWDG_TIMEOUT_MS / IT_WWDG_HALF_DIV );
    ItWwdg_Wait_Until_ms( waitTime, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_RemainingTime( &secondTime ) );

    TEST_ASSERT_LESS_OR_EQUAL_UINT32( timeout, firstTime );
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( ( timeout * IT_WWDG_REMAINING_MIN_PCT ) / IT_WWDG_PCT_100, firstTime );
    TEST_ASSERT_LESS_OR_EQUAL_UINT32( firstTime - ( IT_WWDG_REMAINING_DECREASE_MS * IT_WWDG_US_IN_MS ), secondTime );
}

/*--------------------------------- Window -----------------------------------*/

/**
 * \brief   Wwdg_Get_Window() and Wwdg_Get_EarlyWakeup() return the achieved values.
 *
 * \details Starts the watchdog with 50 ms timeout, 20 ms window and early wakeup callback and reads
 *          the window and the early wakeup time.
 *
 * \par Expected results
 * - Window reads back 20 ms +- 20 % (rounded down to whole ticks).
 * - Early wakeup time is one counter tick (not zero and not longer than a quarter of the timeout).
 */
void It_Wwdg_Get_Window_AfterInit_ReturnsAchievedWindowAndEarlyWakeup( void )
{
    wwdg_Time_us_t window      = WWDG_TIME_UNUSED;
    wwdg_Time_us_t earlyWakeup = WWDG_TIME_UNUSED;
    wwdg_Time_us_t expected    = IT_WWDG_WINDOW_MS * IT_WWDG_US_IN_MS;

    ItWwdg_Init_Watchdog( IT_WWDG_TIMEOUT_MS, IT_WWDG_WINDOW_MS, ItWwdg_EarlyWakeupCallback );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Window( &window ) );
    TEST_ASSERT_UINT32_WITHIN( IT_WWDG_TIME_TOL( expected ), expected, window );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_EarlyWakeup( &earlyWakeup ) );
    TEST_ASSERT_GREATER_THAN_UINT32( WWDG_TIME_UNUSED, earlyWakeup );
    TEST_ASSERT_LESS_OR_EQUAL_UINT32( ( IT_WWDG_TIMEOUT_MS * IT_WWDG_US_IN_MS ) / IT_WWDG_QUARTER_DIV, earlyWakeup );
}


/**
 * \brief   Refresh before the window resets the MCU.
 *
 * \details
 * 1. Stage 0: expects reset, starts the watchdog with 50 ms timeout and 20 ms
 *    window (refresh allowed only in the last 20 ms), refreshes it immediately and
 *    waits 20 ms without refresh.
 * 2. Stage 1 (after reset): checks the reset source.
 *
 * \par Expected results
 * 1. Window violation resets the MCU immediately (before 20 ms).
 * 2. Reset source is WWDG, software reset flag is not set.
 */
void It_Wwdg_Set_Refresh_BeforeWindow_ResetsMcu( void )
{
    const integrationTesting_Stage_t testStage = IntegrationTesting_Get_Stage();

    if( IT_WWDG_STAGE_START == testStage )
    {
        TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_ResetSourceClear() );
        IntegrationTesting_Set_ResetExpected();

        /* Refresh allowed only in the last 20 ms of 50 ms timeout */
        ItWwdg_Init_Watchdog( IT_WWDG_TIMEOUT_MS, IT_WWDG_WINDOW_MS, WWDG_NULL_PTR );

        (void)Wwdg_Set_Refresh();
        ItWwdg_Wait_Until_ms( IT_WWDG_WINDOW_MS, WWDG_FUNCTION_INACTIVE );

        TEST_FAIL_MESSAGE( "WWDG window violation reset did not occur" );
    }
    else
    {
        ItWwdg_Check_WwdgReset();
    }
}


/**
 * \brief   Refresh inside the window keeps the MCU running.
 *
 * \details Starts the watchdog with 50 ms timeout and 20 ms window, Wwdg_Task() is
 *          called continuously (it does not refresh with window). The watchdog is
 *          refreshed manually 40 ms (middle of the window) after the previous refresh,
 *          three times.
 *
 * \par Expected results
 * - No reset of the MCU, every Wwdg_Set_Refresh() returns WWDG_REQUEST_OK.
 */
void It_Wwdg_Set_Refresh_InsideWindow_NoReset( void )
{
    wwdg_Time_ms_t refreshTime = IT_WWDG_TIME_ZERO_MS;

    /* Refresh allowed only in the last 20 ms of 50 ms timeout */
    ItWwdg_Init_Watchdog( IT_WWDG_TIMEOUT_MS, IT_WWDG_WINDOW_MS, WWDG_NULL_PTR );

    for( uint32_t refreshIdx = 0u; IT_WWDG_WINDOW_REFRESH_CNT > refreshIdx; refreshIdx++ )
    {
        /* Refresh in the middle of the window (40 ms after the previous refresh) */
        refreshTime += IT_WWDG_TIMEOUT_MS - ( IT_WWDG_WINDOW_MS / IT_WWDG_HALF_DIV );
        ItWwdg_Wait_Until_ms( refreshTime, WWDG_FUNCTION_ACTIVE );

        TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Set_Refresh() );
    }
}

/*------------------------------ Early wakeup --------------------------------*/

/**
 * \brief   Early wakeup interrupt calls the callback before timeout.
 *
 * \details Starts the watchdog with 50 ms timeout and the early wakeup callback, Wwdg_Task() is not
 *          called. The callback stores the time of the first call and refreshes the watchdog. Waits
 *          for the callback up to 500 ms.
 *
 * \par Expected results
 * - Callback is called at least once, no reset of the MCU.
 * - First call at 50 ms (one tick before the timeout) +- 20 %.
 */
void It_Wwdg_Init_EarlyWakeup_CallbackCalledBeforeReset( void )
{
    const wwdg_Time_ms_t expectedTime = IT_WWDG_TIMEOUT_MS;
    wwdg_Time_ms_t       actualTime   = IT_WWDG_TIME_ZERO_MS;

    ItWwdg_Init_Watchdog( IT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, ItWwdg_EarlyWakeupCallback );

    /* Callback refreshes the watchdog - no reset */
    for( uint32_t loopCnt = 0u;
         ( IT_WWDG_LOOP_MAX > loopCnt ) &&
         ( IT_WWDG_CALLS_NONE == itWwdg_EarlyWakeupCnt ) &&
         ( IT_WWDG_RESET_WAIT_MS > actualTime );
         loopCnt ++ )
    {
        /* Wait for early wakeup interrupt */
        actualTime = ItWwdg_Get_Time_ms();
    }

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32_MESSAGE( IT_WWDG_CALLS_ONCE,
                                                 itWwdg_EarlyWakeupCnt,
                                                 "Early wakeup callback not called" );
    TEST_ASSERT_UINT32_WITHIN( IT_WWDG_TIME_TOL( expectedTime ), expectedTime, itWwdg_EarlyWakeupTime );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Initializes and starts reference time base (TIM2, 10 kHz, free running 32-bit).
 */
static void ItWwdg_Init_RefTime( void )
{
    tim_PeriphConfig_t timConfig;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_DefaultConfig( &timConfig ) );
    timConfig.PeriphId         = IT_WWDG_REF_TIM;
    timConfig.TimerFrequency   = IT_WWDG_REF_FREQ_HZ;
    timConfig.RefreshFrequency = IT_WWDG_REF_REFRESH_HZ;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Init( &timConfig ) );
    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Set_Period( IT_WWDG_REF_TIM, IT_WWDG_REF_PERIOD ) );
    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Start( IT_WWDG_REF_TIM ) );
}


/**
 * \brief Returns reference time since \ref ItWwdg_Init_RefTime or \ref ItWwdg_Init_Watchdog.
 *
 * \return Reference time [ms]
 */
static wwdg_Time_ms_t ItWwdg_Get_Time_ms( void )
{
    tim_Counter_t counter = IT_WWDG_COUNTER_ZERO;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_Counter( IT_WWDG_REF_TIM, &counter ) );

    return ( counter / IT_WWDG_REF_COUNTS_PER_MS );
}


/**
 * \brief Waits until reference time is reached.
 *
 * \param time_ms   [in]: Reference time [ms]
 * \param taskState [in]: \ref WWDG_FUNCTION_ACTIVE - \ref Wwdg_Task is called during waiting
 */
static void ItWwdg_Wait_Until_ms( wwdg_Time_ms_t time_ms, wwdg_FunctionState_t taskState )
{
    wwdg_Time_ms_t actualTime = ItWwdg_Get_Time_ms();

    for( uint32_t loopCnt = 0u;
         ( IT_WWDG_LOOP_MAX > loopCnt ) &&
         ( time_ms > actualTime );
         loopCnt ++ )
    {
        if( WWDG_FUNCTION_ACTIVE == taskState )
        {
            Wwdg_Task();
        }
        else
        {
            /* Watchdog is not refreshed */
        }

        actualTime = ItWwdg_Get_Time_ms();
    }
}


/**
 * \brief Initializes watchdog and restarts reference time (time 0 = watchdog start).
 *
 * \param timeout [in]: Watchdog timeout [ms]
 * \param window  [in]: Window [ms] or \ref WWDG_TIME_UNUSED
 * \param isr     [in]: Early wakeup callback or \ref WWDG_NULL_PTR
 */
static void ItWwdg_Init_Watchdog( wwdg_Time_ms_t timeout, wwdg_Time_ms_t window, wwdg_EarlyWakeupIsr_t *isr )
{
    wwdg_Config_t config;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_DefaultConfig( &config ) );
    config.Timeout        = timeout;
    config.Window         = window;
    config.EarlyWakeupIsr = isr;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Set_Counter( IT_WWDG_REF_TIM, IT_WWDG_COUNTER_ZERO ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );
}


/**
 * \brief Checks that the last reset was caused by the watchdog only.
 */
static void ItWwdg_Check_WwdgReset( void )
{
    rcc_FlagState_t wwdgReset = RCC_FLAG_INACTIVE;
    rcc_FlagState_t swReset   = RCC_FLAG_ACTIVE;

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_ResetSource( RCC_RESET_SRC_WWDG, &wwdgReset ) );
    TEST_ASSERT_EQUAL_MESSAGE( RCC_FLAG_ACTIVE, wwdgReset, "Reset source is not WWDG" );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_ResetSource( RCC_RESET_SRC_SW, &swReset ) );
    TEST_ASSERT_EQUAL( RCC_FLAG_INACTIVE, swReset );
}


/**
 * \brief Early wakeup callback - stores time of the first call and refreshes the watchdog.
 */
static void ItWwdg_EarlyWakeupCallback( void )
{
    if( IT_WWDG_CALLS_NONE == itWwdg_EarlyWakeupCnt )
    {
        itWwdg_EarlyWakeupTime = ItWwdg_Get_Time_ms();
    }
    else
    {
        /* Time of the first call is kept */
    }

    itWwdg_EarlyWakeupCnt++;

    (void)Wwdg_Set_Refresh();
}
