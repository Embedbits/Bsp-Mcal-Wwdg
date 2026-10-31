/**
 * \author Mr.Nobody
 * \file Test_Wwdg.c
 * \ingroup Wwdg
 * \brief Unit tests of System Window Watchdog (WWDG) module.
 *
 * Wwdg.c and Wwdg_Reg.c are compiled unchanged, WWDG registers are emulated by
 * RegMem, RCC (APB clock frequency, peripheral clock) and NVIC modules are mocked
 * by CMock. The getters use no other module: an unexpected RCC or NVIC call in a
 * test of a getter fails the test (CMock).
 *
 * Values used by the tests (APB clock 32 MHz, prescaler 8): one counter tick lasts
 * 4096 * 8 / 32 MHz = 1.024 ms, timeout 40 ms = 39 ticks (39.06 rounded down,
 * counter 0x3F + 39 = 0x66), window 15 ms = 14 ticks (14.65 rounded down, window
 * 0x3F + 14 = 0x4D).
 *
 * \note Limitations of the register emulation (RegMem registers are plain memory):
 *       - The counter does not count down, tests preset the running counter (CR.T).
 *       - WDGA and EWI (set only by software on HW) and EWIF (cleared by writing 0
 *         on HW) are plain bits. The module never clears WDGA and EWI, the tests
 *         check that the bits are not written, not that HW keeps them.
 *       - A register value which HW would not accept (read-back mismatch) is created
 *         by the HW model of RegMem (UtWwdg_HwModel_xxx), which runs after every store.
 *       - Every test case is executed in its own process (CTest), the runtime data
 *         of the module start from the reset state in every test.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Wwdg_Port.h"                      /* Module under test              */
#include "Wwdg_Reg.h"                       /* Register access of the module  */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "Stm32.h"                          /* WWDG registers definition      */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static nvic_RequestState_t  UtWwdg_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId,
                                                          const nvic_IsrCallback_t irqHandler,
                                                          int callCnt );
static rcc_RequestState_t   UtWwdg_RccPclkChangedStub   ( rcc_PeriphId_t periphId,
                                                          rcc_FreqHz_t * const periphClk,
                                                          int callCnt );
static void                 UtWwdg_EarlyWakeupCallback  ( void );
static void                 UtWwdg_RefreshCallback      ( void );
static void                 UtWwdg_Expect_PclkFreq      ( rcc_FreqHz_t pclk );
static void                 UtWwdg_Expect_Init          ( rcc_FreqHz_t pclk, wwdg_FunctionState_t ewiState );
static void                 UtWwdg_Expect_InitFailure   ( void );
static wwdg_Config_t        UtWwdg_Get_Config           ( wwdg_Time_ms_t timeout,
                                                          wwdg_Time_ms_t window,
                                                          wwdg_EarlyWakeupIsr_t *isr );
static void                 UtWwdg_Init_Running         ( wwdg_Time_ms_t timeout,
                                                          wwdg_Time_ms_t window,
                                                          wwdg_EarlyWakeupIsr_t *isr );

static void                 UtWwdg_HwModel_StartWindow  ( void );
static void                 UtWwdg_HwModel_CfgNotKept   ( void );
static void                 UtWwdg_HwModel_WindowNotKept( void );
static void                 UtWwdg_HwModel_StartNotKept ( void );
static void                 UtWwdg_HwModel_CounterNotKept( void );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Expected version of the module */
#define UT_WWDG_VERSION_MAJOR               ( 0u )
#define UT_WWDG_VERSION_MINOR               ( 1u )
#define UT_WWDG_VERSION_PATCH               ( 0u )

/** One millisecond (the window longer than the timeout by the smallest step) */
#define UT_WWDG_ONE_MS                      ( 1u )

/** Offset of the register ID beyond the end of the register list */
#define UT_WWDG_ID_OFFSET                   ( 1u )

/** Prescaler value above the WDGTB field (bit above the field set) */
#define UT_WWDG_TIMEBASE_OUT_OF_FIELD       ( 0x7000u )

/** Window value above the W field (bit above the field set) */
#define UT_WWDG_WINDOW_OUT_OF_FIELD         ( 0x80u )

/** Nominal APB clock frequency */
#define UT_WWDG_PCLK_HZ                     ( 32000000u )

/** APB clock frequency reported by RCC when the clock is not running */
#define UT_WWDG_PCLK_ZERO_HZ                ( 0u )

/** APB clock frequency at which 1 ms is exactly one tick (prescaler 1) */
#define UT_WWDG_PCLK_ONE_TICK_HZ            ( 4096000u )

/** APB clock frequency at which 1 ms is slightly below one tick (prescaler 1) */
#define UT_WWDG_PCLK_BELOW_TICK_HZ          ( 4095999u )

/** APB clock frequency of the reference manual example */
#define UT_WWDG_PCLK_48M_HZ                 ( 48000000u )

/** Fast APB clock frequency (highest of the device) */
#define UT_WWDG_PCLK_250M_HZ                ( 250000000u )

/** Slow APB clock frequency */
#define UT_WWDG_PCLK_1M_HZ                  ( 1000000u )

/** APB clock frequency after a change of the bus clock (half of the nominal) */
#define UT_WWDG_PCLK_CHANGED_HZ             ( 16000000u )

/** Timeout used by tests - 40 ms with APB 32 MHz needs prescaler 8 (39 ticks) */
#define UT_WWDG_TIMEOUT_MS                  ( 40u )

/** Number of ticks of UT_WWDG_TIMEOUT_MS */
#define UT_WWDG_TIMEOUT_TICKS               ( 39u )

/** Counter value written for UT_WWDG_TIMEOUT_MS (0x3F + 39) */
#define UT_WWDG_TIMEOUT_COUNTER             ( 0x66u )

/** Window used by tests - 15 ms = 14.65 ticks, rounded down to 14 ticks with prescaler 8 */
#define UT_WWDG_WINDOW_MS                   ( 15u )

/** Number of ticks of UT_WWDG_WINDOW_MS */
#define UT_WWDG_WINDOW_TICKS                ( 14u )

/** Window register value of UT_WWDG_WINDOW_MS (0x3F + 14) */
#define UT_WWDG_WINDOW_REG                  ( 0x4Du )

/** Window register value below the bit T6 (the counter could never be refreshed) */
#define UT_WWDG_WINDOW_BELOW_T6             ( 0x20u )

/** Prescaler register value of prescaler 8 (WDGTB = 3) */
#define UT_WWDG_TIMEBASE_DIV_8              ( WWDG_CFR_WDGTB_1 | WWDG_CFR_WDGTB_0 )

/** Prescaler register value of prescaler 32 (WDGTB = 5) */
#define UT_WWDG_TIMEBASE_DIV_32             ( WWDG_CFR_WDGTB_2 | WWDG_CFR_WDGTB_0 )

/** Prescaler register value of prescaler 1 (WDGTB = 0) */
#define UT_WWDG_TIMEBASE_DIV_1              ( 0u )

/** Duration of one tick with prescaler 8 and APB 32 MHz [us] */
#define UT_WWDG_TICK_DIV_8_US               ( 1024u )

/** Window register value which opens the window for the whole timeout */
#define UT_WWDG_WINDOW_OPEN                 ( 0x7Fu )

/** Counter value used as the running counter in tests (17 ticks to the reset) */
#define UT_WWDG_RUNNING_COUNTER             ( 0x50u )

/** Number of ticks to the reset of UT_WWDG_RUNNING_COUNTER */
#define UT_WWDG_RUNNING_TICKS               ( 17u )

/** Counter value with the bit T6 cleared (reset is in progress) */
#define UT_WWDG_COUNTER_BELOW_T6            ( 0x3Fu )

/** Counter value above the 7-bit field */
#define UT_WWDG_COUNTER_TOO_WIDE            ( WWDG_CR_T + 1u )

/** Counter value used by the refresh tests of the register layer */
#define UT_WWDG_REFRESH_COUNTER_A           ( 0x60u )

/** Counter value used by the refresh tests of the register layer */
#define UT_WWDG_REFRESH_COUNTER_B           ( 0x70u )

/** Early wakeup interrupt priority used by tests */
#define UT_WWDG_IRQ_PRIO                    ( 2u )

/** Default timeout of Wwdg_Get_DefaultConfig */
#define UT_WWDG_DEFAULT_TIMEOUT_MS          ( 30u )

/** Default interrupt priority of Wwdg_Get_DefaultConfig */
#define UT_WWDG_DEFAULT_IRQ_PRIO            ( 0u )

/** Shortest timeout of the tests [ms] */
#define UT_WWDG_SHORT_TIMEOUT_MS            ( 1u )

/** Number of ticks of the shortest timeout with APB 32 MHz (7.8 ticks rounded down) */
#define UT_WWDG_SHORT_TIMEOUT_TICKS         ( 7u )

/** Counter value of the shortest timeout with APB 32 MHz (0x3F + 7) */
#define UT_WWDG_SHORT_TIMEOUT_COUNTER       ( 0x46u )

/** Longest timeout achievable with APB 32 MHz (prescaler 128, 64.9 ticks rounded down to 64) [ms] */
#define UT_WWDG_TIMEOUT_MAX_MS              ( 1064u )

/** Shortest timeout above the maximum (65.0 ticks with prescaler 128) [ms] */
#define UT_WWDG_TIMEOUT_ABOVE_MAX_MS        ( 1065u )

/** Timeout which is 63.96 ticks with prescaler 128, rounded down to 63 ticks [ms] */
#define UT_WWDG_TIMEOUT_63_TICKS_MS         ( 1048u )

/** Counter value of 63 ticks (0x3F + 63) */
#define UT_WWDG_TIMEOUT_63_TICKS_COUNTER    ( 0x7Eu )

/** Timeout which is 64.02 ticks with prescaler 128, rounded down to 64 ticks [ms] */
#define UT_WWDG_TIMEOUT_64_TICKS_MS         ( 1049u )

/** Longest timeout in us (64 ticks of 16.384 ms) */
#define UT_WWDG_TIMEOUT_MAX_US              ( 1048576u )

/** Long timeout (prescaler 128, 61.03 ticks) and a window longer by 1 ms with the same number of ticks [ms] */
#define UT_WWDG_LONG_TIMEOUT_MS             ( 1000u )

/** Window longer than UT_WWDG_LONG_TIMEOUT_MS by 1 ms (61.09 ticks) */
#define UT_WWDG_LONG_WINDOW_MS              ( 1001u )

/** Timeout of two ticks with APB 4.096 MHz [ms] */
#define UT_WWDG_TWO_TICK_TIMEOUT_MS         ( 2u )

/** Counter value of two ticks (0x3F + 2) */
#define UT_WWDG_TWO_TICK_COUNTER            ( 0x41u )

/** Timeout of the reference manual example [ms] (64.45 ticks with prescaler 8 and APB 48 MHz) */
#define UT_WWDG_RM_TIMEOUT_MS               ( 44u )

/** Timeout of the reference manual example [us] (43.69 ms rounded down) */
#define UT_WWDG_RM_TIMEOUT_US               ( 43690u )

/** Timeout with APB 250 MHz and prescaler 32 (57.2 ticks rounded down) - counter 0x3F + 57 */
#define UT_WWDG_FAST_COUNTER                ( 0x78u )

/** Timeout with APB 1 MHz and prescaler 1 (24.4 ticks rounded down) [ms] */
#define UT_WWDG_SLOW_TIMEOUT_MS             ( 100u )

/** Counter value of UT_WWDG_SLOW_TIMEOUT_MS (0x3F + 24) */
#define UT_WWDG_SLOW_COUNTER                ( 0x57u )

/** Register value without any bit set */
#define UT_WWDG_REG_ZERO                    ( 0u )

/** Value of a time which is not changed by a failing getter */
#define UT_WWDG_TIME_UNTOUCHED              ( 123u )

/** Count of the early wakeup callback calls: none */
#define UT_WWDG_CALLS_NONE                  ( 0u )

/** Count of the early wakeup callback calls: one */
#define UT_WWDG_CALLS_ONCE                  ( 1u )

/** Count of the storage items for the frequency returned by the RCC mock */
#define UT_WWDG_PCLK_SLOT_CNT               ( 4u )

/** Mask of all bits of a register */
#define UT_WWDG_REG_ALL_BITS                ( 0xFFFFFFFFu )

/* ========================== LOCAL VARIABLES =============================== */

/** ISR registered in NVIC */
static nvic_IsrCallback_t   utWwdg_Isr;

/** Count of early wakeup callback calls */
static uint32_t             utWwdg_CallbackCnt;

/** State of the flag EWIF in SR register at the time of the callback */
static uint32_t             utWwdg_FlagAtCallback;

/** Result of the refresh requested from the callback */
static wwdg_RequestState_t  utWwdg_CallbackRefreshState;

/** Value of the window field of CFR register at the time the watchdog was started */
static volatile uint32_t    utWwdg_StartWindow;

/** The start of the watchdog was seen by the HW model */
static volatile uint32_t    utWwdg_StartSeen;

/** Storage of the frequencies returned by the RCC mock (every expectation needs its own) */
static rcc_FreqHz_t         utWwdg_PclkRet[ UT_WWDG_PCLK_SLOT_CNT ];

/** Index of the next free item of utWwdg_PclkRet */
static uint32_t             utWwdg_PclkSlot;

/** Count of the calls of the RCC frequency stub */
static uint32_t             utWwdg_RccCallCnt;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utWwdg_Isr                  = NULL;
    utWwdg_CallbackCnt          = UT_WWDG_CALLS_NONE;
    utWwdg_FlagAtCallback       = UT_WWDG_REG_ZERO;
    utWwdg_CallbackRefreshState = WWDG_REQUEST_ERROR;
    utWwdg_StartWindow          = UT_WWDG_REG_ZERO;
    utWwdg_StartSeen            = UT_WWDG_REG_ZERO;
    utWwdg_PclkSlot             = UT_WWDG_REG_ZERO;
    utWwdg_RccCallCnt           = UT_WWDG_CALLS_NONE;

    Nvic_Set_PeriphIrq_Handler_Stub( UtWwdg_NvicSetHandlerStub );
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Wwdg_Get_ModuleVersion() returns version of the module.
 *
 * \details Reads the module version structure.
 *
 * \par Expected results
 * - Version is 0.1.0 (Major 0, Minor 1, Patch 0).
 */
void Ut_Wwdg_Get_ModuleVersion_ReturnsVersion( void )
{
    wwdg_ModuleVersion_t version = Wwdg_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( UT_WWDG_VERSION_MAJOR, version.Major );
    TEST_ASSERT_EQUAL_UINT8( UT_WWDG_VERSION_MINOR, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( UT_WWDG_VERSION_PATCH, version.Patch );
}

/* =========================== DEFAULT CONFIG =============================== */

/**
 * \brief   Wwdg_Get_DefaultConfig() fills default configuration.
 *
 * \details Reads default configuration, then calls the function with NULL.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, timeout 30 ms, window unused, no callback, priority 0.
 * - NULL pointer: WWDG_REQUEST_ERROR.
 */
void Ut_Wwdg_Get_DefaultConfig_ReturnsDefaults( void )
{
    wwdg_Config_t config;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_DEFAULT_TIMEOUT_MS, config.Timeout );
    TEST_ASSERT_EQUAL_UINT32( WWDG_TIME_UNUSED, config.Window );
    TEST_ASSERT_NULL( config.EarlyWakeupIsr );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_DEFAULT_IRQ_PRIO, config.IrqPriority );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_DefaultConfig( NULL ) );
}

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   Wwdg_Init() with timeout only configures prescaler and counter.
 *
 * \details Initializes 40 ms timeout without window and early wakeup (APB 32 MHz from
 *          RCC mock).
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, peripheral clock activated, NVIC IRQ deactivated, no ISR registered.
 * - CFR: prescaler 8, window opened (0x7F), early wakeup interrupt not enabled.
 * - CR: watchdog activated, counter 0x66 (39 ticks), flag register untouched.
 */
void Ut_Wwdg_Init_TimeoutOnly_ConfiguresPrescalerAndCounter( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_OPEN, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->SR );
    TEST_ASSERT_NULL( utWwdg_Isr );
}


/**
 * \brief   Wwdg_Init() rounds the timeout down to whole ticks and selects the smallest prescaler.
 *
 * \details Initializes 1 ms timeout (APB 32 MHz, 7.8 ticks with prescaler 1).
 *
 * \par Expected results
 * - Prescaler 1 (WDGTB = 0), counter 0x46 (7 ticks of 128 us, not 8).
 */
void Ut_Wwdg_Init_ShortTimeout_RoundsDownToWholeTicks( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_SHORT_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_1 | UT_WWDG_WINDOW_OPEN, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_SHORT_TIMEOUT_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() handles the shortest timeout of one tick.
 *
 * \details Initializes 1 ms timeout with APB 4.096 MHz (one tick lasts exactly 1 ms
 *          with prescaler 1).
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, prescaler 1, counter 0x40 (bit T6 only, 1 tick).
 */
void Ut_Wwdg_Init_ShortestTimeout_OneTick( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_SHORT_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_ONE_TICK_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_1 | UT_WWDG_WINDOW_OPEN, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | WWDG_CR_T_6, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() rejects a timeout shorter than one tick before any register write.
 *
 * \details Initializes 1 ms timeout with APB 4095999 Hz (0.99999 ticks with prescaler 1, rounded
 *          down to zero).
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, no register written, peripheral clock and NVIC not touched.
 */
void Ut_Wwdg_Init_TimeoutBelowOneTick_ReturnsErrorWithoutAccess( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_SHORT_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_BELOW_TICK_HZ );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->SR );
}


/**
 * \brief   Wwdg_Init() handles the longest timeout.
 *
 * \details Initializes 1064 ms timeout (APB 32 MHz, prescaler 128, 64.9 ticks rounded down to 64).
 *
 * \par Expected results
 * - Prescaler 128 (WDGTB = 7), counter 0x7F (64 ticks).
 */
void Ut_Wwdg_Init_LongestTimeout_UsesLargestPrescaler( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MAX_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( WWDG_CFR_WDGTB | UT_WWDG_WINDOW_OPEN, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | WWDG_CR_T, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() rejects timeout above maximum before any register write.
 *
 * \details Initializes timeout of maximum + 1 ms (65.0 ticks with prescaler 128).
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, no register written, peripheral clock and NVIC not touched.
 */
void Ut_Wwdg_Init_TimeoutNotAchievable_ReturnsErrorWithoutAccess( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_ABOVE_MAX_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_HZ );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->SR );
}


/**
 * \brief   Wwdg_Init() rounds a timeout just below a tick boundary down.
 *
 * \details Initializes 1048 ms timeout (APB 32 MHz, prescaler 128, 63.96 ticks).
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, prescaler 128, counter 0x7E (63 ticks, not 64).
 */
void Ut_Wwdg_Init_TimeoutJustBelowTickBoundary_RoundsDown( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_63_TICKS_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( WWDG_CFR_WDGTB | UT_WWDG_WINDOW_OPEN, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_63_TICKS_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() keeps a timeout just above a tick boundary.
 *
 * \details Initializes 1049 ms timeout (APB 32 MHz, prescaler 128, 64.02 ticks).
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, prescaler 128, counter 0x7F (64 ticks).
 */
void Ut_Wwdg_Init_TimeoutJustAboveTickBoundary_KeepsTick( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_64_TICKS_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( WWDG_CFR_WDGTB | UT_WWDG_WINDOW_OPEN, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | WWDG_CR_T, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() rejects zero timeout.
 *
 * \details Initializes timeout WWDG_TIME_UNUSED (0).
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, no register written.
 */
void Ut_Wwdg_Init_ZeroTimeout_ReturnsErrorWithoutAccess( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( WWDG_TIME_UNUSED, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_HZ );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}


/**
 * \brief   Wwdg_Init() reproduces the example of the reference manual.
 *
 * \details Reference manual: APB 48 MHz, WDGTB = 3, T[5:0] = 63 gives tWWDG = 43.69 ms.
 *          Initializes 44 ms timeout with APB 48 MHz.
 *
 * \par Expected results
 * - Prescaler 8 (WDGTB = 3), counter 0x7F (T[5:0] = 63).
 * - Wwdg_Get_Timeout() returns 43690 us (43.69 ms rounded down).
 */
void Ut_Wwdg_Init_ReferenceManualExample_Prescaler8Counter63( void )
{
    wwdg_Config_t  config  = UtWwdg_Get_Config( UT_WWDG_RM_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );
    wwdg_Time_us_t timeout = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Expect_Init( UT_WWDG_PCLK_48M_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_OPEN, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | WWDG_CR_T, WWDG->CR );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_RM_TIMEOUT_US, timeout );
}


/**
 * \brief   Wwdg_Init() selects higher prescaler for a fast bus clock.
 *
 * \details Initializes the default 30 ms timeout with APB 250 MHz (prescaler 16 needs
 *          114 ticks, prescaler 32 needs 57.2 ticks).
 *
 * \par Expected results
 * - Prescaler 32 (WDGTB = 5), counter 0x78 (57 ticks).
 */
void Ut_Wwdg_Init_FastBusClock_SelectsHigherPrescaler( void )
{
    wwdg_Config_t config;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_DefaultConfig( &config ) );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_250M_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_32 | UT_WWDG_WINDOW_OPEN, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_FAST_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() selects the smallest prescaler for a slow bus clock.
 *
 * \details Initializes 100 ms timeout with APB 1 MHz (24.4 ticks with prescaler 1).
 *
 * \par Expected results
 * - Prescaler 1 (WDGTB = 0), counter 0x57 (24 ticks).
 */
void Ut_Wwdg_Init_SlowBusClock_UsesSmallestPrescaler( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_SLOW_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_1M_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_1 | UT_WWDG_WINDOW_OPEN, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_SLOW_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() writes the window after the start of the watchdog.
 *
 * \details Initializes 40 ms timeout and 15 ms window (14.65 ticks rounded down to 14). The HW model
 *          records the window field at the moment the activation bit is set.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, CFR window 0x4D (14 ticks), counter 0x66.
 * - The watchdog was started with the window opened (0x7F), so the first write of the
 *   counter is never a window violation (not even for an already running watchdog).
 */
void Ut_Wwdg_Init_Window_StartsWithOpenWindowAndWritesWindowLast( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, UT_WWDG_WINDOW_MS, NULL );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtWwdg_HwModel_StartWindow ) );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_CALLS_ONCE, utWwdg_StartSeen );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_WINDOW_OPEN, utWwdg_StartWindow );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_REG, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() accepts window equal to the timeout.
 *
 * \details Initializes 40 ms timeout with 40 ms window.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, window register equals the counter (0x66, refresh allowed
 *   immediately).
 */
void Ut_Wwdg_Init_WindowEqualToTimeout_Accepted( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, UT_WWDG_TIMEOUT_MS, NULL );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_TIMEOUT_COUNTER, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() rejects window longer than the timeout before any register write.
 *
 * \details Initializes 40 ms timeout with 41 ms window (39 ticks and 40 ticks with prescaler 8).
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, no register written, peripheral clock and NVIC not touched.
 */
void Ut_Wwdg_Init_WindowLongerThanTimeout_ReturnsErrorWithoutAccess( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, UT_WWDG_TIMEOUT_MS + UT_WWDG_ONE_MS, NULL );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_HZ );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}


/**
 * \brief   Wwdg_Init() rejects window longer than the timeout even when both round down to the same tick.
 *
 * \details Initializes 1000 ms timeout with 1001 ms window (prescaler 128, one tick lasts 16.4 ms, both
 *          times are 61 ticks). The window longer than the timeout in ms is rejected.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, no register written, peripheral clock and NVIC not touched.
 */
void Ut_Wwdg_Init_WindowLongerThanTimeoutSameTicks_ReturnsErrorWithoutAccess( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_LONG_TIMEOUT_MS, UT_WWDG_LONG_WINDOW_MS, NULL );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_HZ );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}


/**
 * \brief   Wwdg_Init() rejects window shorter than one tick.
 *
 * \details Initializes 1000 ms timeout (prescaler 128, 61 ticks of 16.4 ms) with 1 ms window (0.06 ticks,
 *          rounded down to zero).
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, no register written.
 */
void Ut_Wwdg_Init_WindowBelowOneTick_ReturnsErrorWithoutAccess( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_LONG_TIMEOUT_MS, UT_WWDG_SHORT_TIMEOUT_MS, NULL );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_HZ );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}


/**
 * \brief   Wwdg_Init() configures the early wakeup interrupt.
 *
 * \details Initializes 40 ms timeout with callback and priority 2. A stale flag EWIF is
 *          preset. NVIC calls are expected in the order inactive, handler, priority, active.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, ISR registered in NVIC, CFR early wakeup interrupt enable bit set.
 * - Stale flag EWIF cleared before the interrupt was enabled.
 */
void Ut_Wwdg_Init_EarlyWakeup_ConfiguresInterrupt( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_EarlyWakeupCallback );

    WWDG->SR = WWDG_SR_EWIF;    /* Stale flag of a previous run */

    UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_NOT_NULL( utWwdg_Isr );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_OPEN | WWDG_CFR_EWI, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );
    TEST_ASSERT_BITS_LOW( WWDG_SR_EWIF, WWDG->SR );
}


/**
 * \brief   Wwdg_Init() rejects the early wakeup interrupt with a one tick timeout.
 *
 * \details Initializes 1 ms timeout with APB 4.096 MHz (1 tick) and callback. The interrupt
 *          is raised at the counter value 0x40, which is the start value of this timeout.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, no register written, peripheral clock and NVIC not touched.
 */
void Ut_Wwdg_Init_EarlyWakeupWithOneTickTimeout_ReturnsErrorWithoutAccess( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_SHORT_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_EarlyWakeupCallback );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_ONE_TICK_HZ );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}


/**
 * \brief   Wwdg_Init() accepts the early wakeup interrupt with a two tick timeout.
 *
 * \details Initializes 2 ms timeout with APB 4.096 MHz (2 ticks) and callback.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, counter 0x41, interrupt enable bit set.
 */
void Ut_Wwdg_Init_EarlyWakeupWithTwoTickTimeout_Accepted( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TWO_TICK_TIMEOUT_MS,
                                              WWDG_TIME_UNUSED,
                                              UtWwdg_EarlyWakeupCallback );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_ONE_TICK_HZ, WWDG_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_WINDOW_OPEN | WWDG_CFR_EWI, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TWO_TICK_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() reports refused interrupt priority before the watchdog is started.
 *
 * \details NVIC refuses the priority. NVIC deactivation of the failure handling is expected.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, watchdog not started (no WWDG register written), module deinitialized.
 */
void Ut_Wwdg_Init_IrqPriorityRefused_ReturnsErrorNotStarted( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_EarlyWakeupCallback );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_HZ );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_WWDG, RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, UT_WWDG_IRQ_PRIO, NVIC_REQUEST_ERROR );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );    /* Deinit */

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}


/**
 * \brief   Wwdg_Init() reports refused interrupt deactivation.
 *
 * \details First NVIC deactivation fails, the deinitialization repeats it.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, no WWDG register written.
 */
void Ut_Wwdg_Init_IrqDeactivationRefused_ReturnsErrorNotStarted( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_HZ );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_WWDG, RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_ERROR );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );    /* Deinit */

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}


/**
 * \brief   Wwdg_Init() reports refused interrupt activation after the start.
 *
 * \details NVIC refuses the activation after the watchdog was started. Deinitialization
 *          deactivates the interrupt.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, module deinitialized (callback is not called by a pending flag,
 *   automatic refresh is off). The watchdog keeps running (it can not be stopped).
 */
void Ut_Wwdg_Init_IrqActivationRefused_ReturnsErrorAndDeinitializes( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_EarlyWakeupCallback );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_HZ );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_WWDG, RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, UT_WWDG_IRQ_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_ERROR );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );    /* Deinit */

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_BITS_HIGH( WWDG_CR_WDGA, WWDG->CR );    /* Watchdog was started and keeps running */

    WWDG->SR = WWDG_SR_EWIF;
    utWwdg_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_CALLS_NONE, utWwdg_CallbackCnt );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;
    Wwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER, WWDG->CR );    /* Automatic refresh is off */
}


/**
 * \brief   Wwdg_Init() reports error of the APB clock frequency read.
 *
 * \details Rcc_Get_PeriphClk() returns error.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, no register written, peripheral clock and NVIC not touched.
 */
void Ut_Wwdg_Init_RccFrequencyError_ReturnsErrorWithoutAccess( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_WWDG, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}


/**
 * \brief   Wwdg_Init() rejects zero APB clock frequency.
 *
 * \details RCC mock returns frequency 0 Hz.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, no register written.
 */
void Ut_Wwdg_Init_ZeroFrequency_ReturnsErrorWithoutAccess( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_ZERO_HZ );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}


/**
 * \brief   Wwdg_Init() reports refused peripheral clock activation.
 *
 * \details Rcc_Set_PeriphActive() returns error. The deinitialization deactivates the interrupt.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, no WWDG register written.
 */
void Ut_Wwdg_Init_ClockActivationRefused_ReturnsErrorWithoutAccess( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_HZ );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_WWDG, RCC_REQUEST_ERROR );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );    /* Deinit */

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}


/**
 * \brief   Wwdg_Init() rejects NULL configuration.
 *
 * \details Calls Wwdg_Init() with NULL pointer.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, RCC, NVIC and registers not touched.
 */
void Ut_Wwdg_Init_NullConfig_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( NULL ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}


/**
 * \brief   Wwdg_Init() verifies the written configuration by read-back.
 *
 * \details The HW model keeps the window field of CFR at zero (the written value does
 *          not stay).
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, the watchdog is not started (activation bit clear), the
 *   module is deinitialized (NVIC deactivated again).
 */
void Ut_Wwdg_Init_ConfigReadBackMismatch_ReturnsErrorNotStarted( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtWwdg_HwModel_CfgNotKept ) );

    UtWwdg_Expect_InitFailure();

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_BITS_LOW( WWDG_CR_WDGA, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() verifies the start of the watchdog by read-back.
 *
 * \details The HW model clears the activation bit after every store of CR.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, module deinitialized (NVIC deactivated again).
 */
void Ut_Wwdg_Init_StartNotConfirmed_ReturnsError( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtWwdg_HwModel_StartNotKept ) );

    UtWwdg_Expect_InitFailure();

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );
}


/**
 * \brief   Wwdg_Init() verifies the counter value of the start by read-back.
 *
 * \details The HW model sets the counter to 0x7F after every store of CR while the watchdog
 *          is activated (the written value 0x66 does not stay).
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, module deinitialized (NVIC deactivated again).
 */
void Ut_Wwdg_Init_CounterNotLoaded_ReturnsError( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtWwdg_HwModel_CounterNotKept ) );

    UtWwdg_Expect_InitFailure();

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );
}


/**
 * \brief   Wwdg_Init() verifies the window written after the start by read-back.
 *
 * \details The HW model restores the open window (0x7F) after the narrower window is
 *          written.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, module deinitialized (NVIC deactivated again).
 */
void Ut_Wwdg_Init_WindowReadBackMismatch_ReturnsError( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, UT_WWDG_WINDOW_MS, NULL );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtWwdg_HwModel_WindowNotKept ) );

    UtWwdg_Expect_InitFailure();

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_BITS_HIGH( WWDG_CR_WDGA, WWDG->CR );    /* Started before the window was written */
}


/**
 * \brief   Wwdg_Init() takes over a watchdog which is already running.
 *
 * \details The watchdog runs with the reset values (CR = WDGA | 0x7F, CFR = 0x7F), e.g. started by the
 *          hardware. Initializes 40 ms timeout.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, prescaler 8 and counter 0x66 are written, the watchdog keeps running.
 */
void Ut_Wwdg_Init_AlreadyRunning_TakesOver( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    WWDG->CR  = WWDG_CR_WDGA | WWDG_CR_T;
    WWDG->CFR = WWDG_CFR_W;

    UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_OPEN, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() reconfigures an initialized module.
 *
 * \details Initializes 40 ms timeout without window, then 40 ms timeout with 15 ms window and
 *          callback.
 *
 * \par Expected results
 * - Both calls WWDG_REQUEST_OK, CFR holds the second configuration (window 0x4D, interrupt enabled).
 * - Automatic refresh of the first configuration is off (window used).
 */
void Ut_Wwdg_Init_SecondInit_ReplacesConfiguration( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, UT_WWDG_WINDOW_MS, UtWwdg_EarlyWakeupCallback );

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_REG | WWDG_CFR_EWI, WWDG->CFR );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;
    Wwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() with an invalid configuration keeps the running configuration.
 *
 * \details Initializes 40 ms timeout without window, then calls Wwdg_Init() with a timeout above the
 *          maximum.
 *
 * \par Expected results
 * - The second call returns WWDG_REQUEST_ERROR without access to registers, RCC (peripheral clock)
 *   and NVIC.
 * - Wwdg_Task() keeps refreshing the watchdog with the first configuration.
 */
void Ut_Wwdg_Init_InvalidConfigAfterInit_KeepsRunningConfiguration( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_ABOVE_MAX_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_HZ );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_OPEN, WWDG->CFR );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;
    Wwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Init() without callback leaves the early wakeup enable bit as it is.
 *
 * \details The enable bit can be set only by software and cleared only by a system reset.
 *          Initializes with callback, then without callback.
 *
 * \par Expected results
 * - The second initialization is WWDG_REQUEST_OK, the interrupt is deactivated in NVIC (and not
 *   activated again), the enable bit stays set in CFR (the module does not write it).
 * - Wwdg_Get_EarlyWakeup() reports the interrupt as unused.
 */
void Ut_Wwdg_Init_WithoutCallbackAfterCallback_KeepsEnableBitAndDeactivatesInterrupt( void )
{
    wwdg_Time_us_t earlyWakeup = UT_WWDG_TICK_DIV_8_US;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_EarlyWakeupCallback );
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_BITS_HIGH( WWDG_CFR_EWI, WWDG->CFR );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_EarlyWakeup( &earlyWakeup ) );
    TEST_ASSERT_EQUAL_UINT32( WWDG_TIME_UNUSED, earlyWakeup );
}

/* ============================== DEINIT ==================================== */

/**
 * \brief   Wwdg_Deinit() disables the interrupt and the automatic refresh only.
 *
 * \details Initializes 40 ms timeout with callback (no window), then deinitializes. The
 *          peripheral clock is not touched (CMock fails on an unexpected RCC call).
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, NVIC interrupt deactivated, the watchdog keeps running (WDGA, counter, CFR
 *   unchanged).
 * - A pending flag no longer calls the callback, Wwdg_Task() does not refresh the watchdog.
 * - Wwdg_Set_Refresh() still reloads the counter.
 */
void Ut_Wwdg_Deinit_AfterInit_DisablesInterruptAndAutoRefreshOnly( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_EarlyWakeupCallback );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Deinit() );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_OPEN | WWDG_CFR_EWI, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );

    WWDG->SR = WWDG_SR_EWIF;
    utWwdg_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_CALLS_NONE, utWwdg_CallbackCnt );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;
    Wwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER, WWDG->CR );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Set_Refresh() );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Deinit() reports refused interrupt deactivation.
 *
 * \details NVIC returns error. The automatic refresh is stopped anyway.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, Wwdg_Task() does not refresh the watchdog.
 */
void Ut_Wwdg_Deinit_NvicError_ReturnsError( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Deinit() );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;
    Wwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER, WWDG->CR );
}

/* ================================ TASK ==================================== */

/**
 * \brief   Wwdg_Task() refreshes the watchdog when no window is used.
 *
 * \details Initializes 40 ms timeout without window, presets a running counter and calls the task.
 *
 * \par Expected results
 * - Counter reloaded to 0x66, activation bit unchanged.
 */
void Ut_Wwdg_Task_NoWindow_RefreshesCounter( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;

    Wwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Task() does not refresh the watchdog when a window is used.
 *
 * \details Initializes 40 ms timeout with 15 ms window, presets a running counter and calls the task.
 *
 * \par Expected results
 * - Counter not changed (the application refreshes inside the window).
 */
void Ut_Wwdg_Task_WindowUsed_DoesNotRefresh( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, UT_WWDG_WINDOW_MS, NULL );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;

    Wwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Task() does not refresh the watchdog when the window equals the timeout.
 *
 * \details A window equal to the timeout is a window (WWDG_TIME_UNUSED is the only way to not use it).
 *
 * \par Expected results
 * - Counter not changed.
 */
void Ut_Wwdg_Task_WindowEqualToTimeout_DoesNotRefresh( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, UT_WWDG_TIMEOUT_MS, NULL );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;

    Wwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Task() does nothing before the initialization.
 *
 * \details Presets a running counter (e.g. watchdog started by the hardware) and calls the task.
 *
 * \par Expected results
 * - Registers not changed.
 */
void Ut_Wwdg_Task_BeforeInit_DoesNothing( void )
{
    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;

    Wwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
}

/* =============================== REFRESH ================================== */

/**
 * \brief   Wwdg_Set_Refresh() reloads the counter.
 *
 * \details Initializes 40 ms timeout, presets a running counter and refreshes.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, counter 0x66, activation bit unchanged.
 */
void Ut_Wwdg_Set_Refresh_AfterInit_ReloadsCounter( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Set_Refresh() );

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Set_Refresh() reloads the counter of a watchdog with window.
 *
 * \details Initializes 40 ms timeout with 15 ms window and refreshes.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, counter 0x66, window register unchanged.
 */
void Ut_Wwdg_Set_Refresh_WithWindow_ReloadsCounter( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, UT_WWDG_WINDOW_MS, NULL );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Set_Refresh() );

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_REG, WWDG->CFR );
}


/**
 * \brief   Wwdg_Set_Refresh() rejects the refresh before the initialization.
 *
 * \details The reload value is not known. Presets a running counter.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, registers not written.
 */
void Ut_Wwdg_Set_Refresh_BeforeInit_ReturnsErrorWithoutAccess( void )
{
    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Set_Refresh() );

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Set_Refresh() does not start a watchdog which is not running.
 *
 * \details Initialization fails at the start (the HW model clears the activation bit), then the
 *          refresh is requested.
 *
 * \par Expected results
 * - The refresh writes the counter only, the activation bit stays clear.
 */
void Ut_Wwdg_Set_Refresh_AfterFailedStart_DoesNotActivateWatchdog( void )
{
    wwdg_Config_t config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtWwdg_HwModel_StartNotKept ) );

    UtWwdg_Expect_InitFailure();

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Set_Refresh() );

    TEST_ASSERT_BITS_LOW( WWDG_CR_WDGA, WWDG->CR );
}

/* ============================ WATCHDOG STATE ============================== */

/**
 * \brief   Wwdg_Get_State() reports a watchdog started by Wwdg_Init().
 *
 * \details Initializes 40 ms timeout and reads the state. RCC is not called by the getter.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, WWDG_FUNCTION_ACTIVE.
 */
void Ut_Wwdg_Get_State_AfterInit_Active( void )
{
    wwdg_FunctionState_t state = WWDG_FUNCTION_INACTIVE;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( WWDG_FUNCTION_ACTIVE, state );
}


/**
 * \brief   Wwdg_Get_State() keeps reporting the running watchdog after the deinitialization.
 *
 * \details Initializes 40 ms timeout, deinitializes (the watchdog can not be stopped) and reads the state.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, WWDG_FUNCTION_ACTIVE.
 */
void Ut_Wwdg_Get_State_AfterDeinit_Active( void )
{
    wwdg_FunctionState_t state = WWDG_FUNCTION_INACTIVE;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Deinit() );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( WWDG_FUNCTION_ACTIVE, state );
}


/**
 * \brief   Wwdg_Get_State() reports the activation bit read from the register.
 *
 * \details Initialization fails at the start (the HW model clears the activation bit), the state is read.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, WWDG_FUNCTION_INACTIVE (the module reached the start of the watchdog, the
 *   activation bit is clear).
 */
void Ut_Wwdg_Get_State_StartNotConfirmed_ReportsInactive( void )
{
    wwdg_Config_t        config = UtWwdg_Get_Config( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );
    wwdg_FunctionState_t state  = WWDG_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtWwdg_HwModel_StartNotKept ) );

    UtWwdg_Expect_InitFailure();

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Init( &config ) );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( WWDG_FUNCTION_INACTIVE, state );
}


/**
 * \brief   Wwdg_Get_State() rejects a module which is not initialized.
 *
 * \details The activation bit is preset (watchdog started by the hardware), Wwdg_Init() was not called.
 *          The hardware mode is not read out by this version. RCC and registers are not touched.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, state not changed.
 */
void Ut_Wwdg_Get_State_BeforeInit_ReturnsError( void )
{
    wwdg_FunctionState_t state = WWDG_FUNCTION_INACTIVE;

    WWDG->CR = WWDG_CR_WDGA | WWDG_CR_T;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( WWDG_FUNCTION_INACTIVE, state );
}


/**
 * \brief   Wwdg_Get_State() rejects NULL pointer.
 *
 * \details Initializes the module, then calls the function with NULL.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, RCC not called.
 */
void Ut_Wwdg_Get_State_NullPointer_ReturnsError( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_State( NULL ) );
}

/* ============================== TIMEOUT =================================== */

/**
 * \brief   Wwdg_Get_Timeout() returns the achieved timeout.
 *
 * \details Initializes 40 ms timeout (39 ticks of 1.024 ms) and reads the timeout. RCC is not called
 *          by the getter.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, 39936 us.
 */
void Ut_Wwdg_Get_Timeout_AfterInit_ReturnsAchievedTimeout( void )
{
    wwdg_Time_us_t timeout = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_TIMEOUT_TICKS * UT_WWDG_TICK_DIV_8_US, timeout );
}


/**
 * \brief   Wwdg_Get_Timeout() returns the longest timeout.
 *
 * \details Initializes 1064 ms timeout (64 ticks of 16.384 ms) and reads the timeout.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, 1048576 us (the requested time is rounded down to 64 ticks).
 */
void Ut_Wwdg_Get_Timeout_LongestTimeout_ReturnsAchievedTimeout( void )
{
    wwdg_Time_us_t timeout = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MAX_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_TIMEOUT_MAX_US, timeout );
}


/**
 * \brief   Wwdg_Get_Timeout() uses the APB clock frequency of the initialization.
 *
 * \details Initializes 40 ms timeout with APB 32 MHz, the bus clock changes to 16 MHz afterwards. The
 *          getter does not ask RCC (CMock fails on an unexpected call), the module has to be initialized
 *          again after a change of the bus clock.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, 39936 us (frequency of the initialization).
 */
void Ut_Wwdg_Get_Timeout_BusClockChangedAfterInit_ReturnsTimeoutOfInitClock( void )
{
    wwdg_Time_us_t timeout = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    Rcc_Get_PeriphClk_Stub( UtWwdg_RccPclkChangedStub );    /* Bus clock after the change */

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_TIMEOUT_TICKS * UT_WWDG_TICK_DIV_8_US, timeout );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_CALLS_NONE, utWwdg_RccCallCnt );
}


/**
 * \brief   Wwdg_Get_Timeout() keeps working after the deinitialization.
 *
 * \details Initializes 40 ms timeout, deinitializes (the watchdog keeps running) and reads the timeout.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, 39936 us.
 */
void Ut_Wwdg_Get_Timeout_AfterDeinit_ReturnsAchievedTimeout( void )
{
    wwdg_Time_us_t timeout = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Deinit() );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_TIMEOUT_TICKS * UT_WWDG_TICK_DIV_8_US, timeout );
}


/**
 * \brief   Wwdg_Get_Timeout() rejects a module which is not initialized.
 *
 * \details The watchdog runs (hardware mode), the module was not initialized.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, timeout not changed, RCC not called.
 */
void Ut_Wwdg_Get_Timeout_BeforeInit_ReturnsError( void )
{
    wwdg_Time_us_t timeout = UT_WWDG_TIME_UNTOUCHED;

    WWDG->CR  = WWDG_CR_WDGA | WWDG_CR_T;
    WWDG->CFR = UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_OPEN;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_TIME_UNTOUCHED, timeout );
}


/**
 * \brief   Wwdg_Get_Timeout() rejects NULL pointer.
 *
 * \details Initializes the module, then calls the function with NULL.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, RCC not called.
 */
void Ut_Wwdg_Get_Timeout_NullPointer_ReturnsError( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_Timeout( NULL ) );
}

/* =============================== WINDOW =================================== */

/**
 * \brief   Wwdg_Get_Window() returns the achieved window.
 *
 * \details Initializes 40 ms timeout with 15 ms window (14 ticks of 1.024 ms) and reads the window.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, 14336 us (the requested time is rounded down to 14 ticks).
 */
void Ut_Wwdg_Get_Window_AfterInit_ReturnsAchievedWindow( void )
{
    wwdg_Time_us_t window = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, UT_WWDG_WINDOW_MS, NULL );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Window( &window ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_WINDOW_TICKS * UT_WWDG_TICK_DIV_8_US, window );
}


/**
 * \brief   Wwdg_Get_Window() returns the window equal to the timeout.
 *
 * \details Initializes 40 ms timeout with 40 ms window (39 ticks) and reads the window.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, 39936 us (window as long as the timeout).
 */
void Ut_Wwdg_Get_Window_EqualToTimeout_ReturnsTimeout( void )
{
    wwdg_Time_us_t window = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, UT_WWDG_TIMEOUT_MS, NULL );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Window( &window ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_TIMEOUT_TICKS * UT_WWDG_TICK_DIV_8_US, window );
}


/**
 * \brief   Wwdg_Get_Window() reports an unused window.
 *
 * \details Initializes 40 ms timeout without window and reads the window.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, WWDG_TIME_UNUSED.
 */
void Ut_Wwdg_Get_Window_NoWindow_ReturnsUnused( void )
{
    wwdg_Time_us_t window = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Window( &window ) );
    TEST_ASSERT_EQUAL_UINT32( WWDG_TIME_UNUSED, window );
}


/**
 * \brief   Wwdg_Get_Window() rejects a window below the bit T6.
 *
 * \details Initializes the module, then presets the window 0x20 (the counter could never be refreshed).
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, window not changed.
 */
void Ut_Wwdg_Get_Window_BelowT6_ReturnsError( void )
{
    wwdg_Time_us_t window = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    WWDG->CFR = UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_BELOW_T6;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_Window( &window ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_TIME_UNTOUCHED, window );
}


/**
 * \brief   Wwdg_Get_Window() rejects a module which is not initialized.
 *
 * \details The registers hold a window (hardware mode), Wwdg_Init() was not called.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, window not changed, RCC not called.
 */
void Ut_Wwdg_Get_Window_BeforeInit_ReturnsError( void )
{
    wwdg_Time_us_t window = UT_WWDG_TIME_UNTOUCHED;

    WWDG->CR  = WWDG_CR_WDGA | WWDG_CR_T;
    WWDG->CFR = UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_REG;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_Window( &window ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_TIME_UNTOUCHED, window );
}


/**
 * \brief   Wwdg_Get_Window() rejects NULL pointer.
 *
 * \details Initializes the module, then calls the function with NULL.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, RCC not called.
 */
void Ut_Wwdg_Get_Window_NullPointer_ReturnsError( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, UT_WWDG_WINDOW_MS, NULL );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_Window( NULL ) );
}

/* ============================= EARLY WAKEUP =============================== */

/**
 * \brief   Wwdg_Get_EarlyWakeup() returns one counter tick.
 *
 * \details Initializes 40 ms timeout with callback and reads the early wakeup time.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, 1024 us (one tick of prescaler 8 at 32 MHz).
 */
void Ut_Wwdg_Get_EarlyWakeup_WithCallback_ReturnsOneTick( void )
{
    wwdg_Time_us_t earlyWakeup = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_EarlyWakeupCallback );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_EarlyWakeup( &earlyWakeup ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_TICK_DIV_8_US, earlyWakeup );
}


/**
 * \brief   Wwdg_Get_EarlyWakeup() reports an unused interrupt.
 *
 * \details Initializes 40 ms timeout without callback and reads the early wakeup time.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, WWDG_TIME_UNUSED.
 */
void Ut_Wwdg_Get_EarlyWakeup_WithoutCallback_ReturnsUnused( void )
{
    wwdg_Time_us_t earlyWakeup = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_EarlyWakeup( &earlyWakeup ) );
    TEST_ASSERT_EQUAL_UINT32( WWDG_TIME_UNUSED, earlyWakeup );
}


/**
 * \brief   Wwdg_Get_EarlyWakeup() reports an unused interrupt after the deinitialization.
 *
 * \details Initializes with callback, deinitializes and reads the early wakeup time.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, WWDG_TIME_UNUSED (callback removed, the enable bit of CFR is still set).
 */
void Ut_Wwdg_Get_EarlyWakeup_AfterDeinit_ReturnsUnused( void )
{
    wwdg_Time_us_t earlyWakeup = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_EarlyWakeupCallback );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Deinit() );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_EarlyWakeup( &earlyWakeup ) );
    TEST_ASSERT_EQUAL_UINT32( WWDG_TIME_UNUSED, earlyWakeup );
}


/**
 * \brief   Wwdg_Get_EarlyWakeup() rejects a module which is not initialized.
 *
 * \details The interrupt enable bit is preset, Wwdg_Init() was not called.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, time not changed, RCC not called.
 */
void Ut_Wwdg_Get_EarlyWakeup_BeforeInit_ReturnsError( void )
{
    wwdg_Time_us_t earlyWakeup = UT_WWDG_TIME_UNTOUCHED;

    WWDG->CR  = WWDG_CR_WDGA | WWDG_CR_T;
    WWDG->CFR = UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_OPEN | WWDG_CFR_EWI;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_EarlyWakeup( &earlyWakeup ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_TIME_UNTOUCHED, earlyWakeup );
}


/**
 * \brief   Wwdg_Get_EarlyWakeup() rejects NULL pointer.
 *
 * \details Initializes the module, then calls the function with NULL.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, RCC not called.
 */
void Ut_Wwdg_Get_EarlyWakeup_NullPointer_ReturnsError( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_EarlyWakeupCallback );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_EarlyWakeup( NULL ) );
}

/* ============================ REMAINING TIME ============================== */

/**
 * \brief   Wwdg_Get_RemainingTime() returns the time to the reset.
 *
 * \details Initializes 40 ms timeout, presets the running counter 0x50 and reads the remaining time.
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, 17 ticks (0x50 - 0x3F) of 1.024 ms = 17408 us.
 */
void Ut_Wwdg_Get_RemainingTime_CounterRunning_ReturnsTimeToReset( void )
{
    wwdg_Time_us_t remainingTime = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_RemainingTime( &remainingTime ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_RUNNING_TICKS * UT_WWDG_TICK_DIV_8_US, remainingTime );
}


/**
 * \brief   Wwdg_Get_RemainingTime() equals the window when the window opens.
 *
 * \details Initializes 40 ms timeout with 15 ms window, presets the counter at the window value (0x4D, the
 *          first value at which the refresh is allowed) and reads the remaining time and the window.
 *
 * \par Expected results
 * - Remaining time equals the window (14336 us): the window is open when the remaining time is not
 *   longer than the window.
 */
void Ut_Wwdg_Get_RemainingTime_CounterAtWindow_EqualsWindow( void )
{
    wwdg_Time_us_t remainingTime = UT_WWDG_TIME_UNTOUCHED;
    wwdg_Time_us_t window        = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, UT_WWDG_WINDOW_MS, NULL );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_WINDOW_REG;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_RemainingTime( &remainingTime ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Window( &window ) );

    TEST_ASSERT_EQUAL_UINT32( window, remainingTime );
}


/**
 * \brief   Wwdg_Get_RemainingTime() returns zero when the bit T6 is cleared.
 *
 * \details Initializes the module, then presets the counter 0x3F (the reset is in progress).
 *
 * \par Expected results
 * - WWDG_REQUEST_OK, 0 us.
 */
void Ut_Wwdg_Get_RemainingTime_CounterBelowT6_ReturnsZero( void )
{
    wwdg_Time_us_t remainingTime = UT_WWDG_TIME_UNTOUCHED;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_COUNTER_BELOW_T6;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_RemainingTime( &remainingTime ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_REG_ZERO, remainingTime );
}


/**
 * \brief   Wwdg_Get_RemainingTime() rejects a module which is not initialized.
 *
 * \details The watchdog runs (hardware mode), Wwdg_Init() was not called.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, time not changed, RCC not called.
 */
void Ut_Wwdg_Get_RemainingTime_BeforeInit_ReturnsError( void )
{
    wwdg_Time_us_t remainingTime = UT_WWDG_TIME_UNTOUCHED;

    WWDG->CR  = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;
    WWDG->CFR = UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_OPEN;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_RemainingTime( &remainingTime ) );
    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_TIME_UNTOUCHED, remainingTime );
}


/**
 * \brief   Wwdg_Get_RemainingTime() rejects NULL pointer.
 *
 * \details Initializes the module, then calls the function with NULL.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, RCC not called.
 */
void Ut_Wwdg_Get_RemainingTime_NullPointer_ReturnsError( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, NULL );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Get_RemainingTime( NULL ) );
}

/* ============================ GETTER SIDE EFFECTS ========================= */

/**
 * \brief   The getters do not change any register and do not call other modules.
 *
 * \details Initializes the module (callback and window), presets a running counter and a pending flag,
 *          calls all getters. CMock fails on any RCC or NVIC call.
 *
 * \par Expected results
 * - All getters WWDG_REQUEST_OK, CR, CFR and SR are exactly as before the calls.
 */
void Ut_Wwdg_Get_AllGetters_AfterInit_NoRegisterChange( void )
{
    wwdg_FunctionState_t state       = WWDG_FUNCTION_INACTIVE;
    wwdg_Time_us_t       timeValue   = UT_WWDG_TIME_UNTOUCHED;
    const uint32_t       crBefore    = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;
    const uint32_t       cfrBefore   = UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_REG | WWDG_CFR_EWI;

    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, UT_WWDG_WINDOW_MS, UtWwdg_EarlyWakeupCallback );

    WWDG->CR = crBefore;
    WWDG->SR = WWDG_SR_EWIF;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Timeout( &timeValue ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_Window( &timeValue ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_EarlyWakeup( &timeValue ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Get_RemainingTime( &timeValue ) );

    TEST_ASSERT_EQUAL_HEX32( crBefore, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( cfrBefore, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_SR_EWIF, WWDG->SR );
}

/* ========================= EARLY WAKEUP INTERRUPT ========================= */

/**
 * \brief   Early wakeup interrupt acknowledges the flag and calls the callback.
 *
 * \details Initializes 40 ms timeout with callback, sets the flag EWIF and calls the ISR registered in NVIC.
 *          The callback stores the state of the flag at the time of the call.
 *
 * \par Expected results
 * - Callback called once, the flag EWIF was already cleared when it was called, the flag is clear after the ISR.
 */
void Ut_Wwdg_IsrHandler_FlagSet_ClearsFlagAndCallsCallback( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_EarlyWakeupCallback );

    WWDG->SR = WWDG_SR_EWIF;

    utWwdg_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_CALLS_ONCE, utWwdg_CallbackCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, utWwdg_FlagAtCallback );
    TEST_ASSERT_BITS_LOW( WWDG_SR_EWIF, WWDG->SR );
}


/**
 * \brief   Early wakeup interrupt without the flag does not call the callback.
 *
 * \details Initializes 40 ms timeout with callback and calls the ISR with the flag EWIF clear.
 *
 * \par Expected results
 * - Callback not called.
 */
void Ut_Wwdg_IsrHandler_FlagNotSet_DoesNotCallCallback( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_EarlyWakeupCallback );

    utWwdg_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_WWDG_CALLS_NONE, utWwdg_CallbackCnt );
}


/**
 * \brief   The callback of the early wakeup interrupt can refresh the watchdog.
 *
 * \details Initializes 40 ms timeout with a callback which calls Wwdg_Set_Refresh(), presets a running
 *          counter, sets the flag EWIF and calls the ISR.
 *
 * \par Expected results
 * - Wwdg_Set_Refresh() in the callback returns WWDG_REQUEST_OK, counter reloaded to 0x66.
 */
void Ut_Wwdg_IsrHandler_CallbackRefreshes_ReloadsCounter( void )
{
    UtWwdg_Init_Running( UT_WWDG_TIMEOUT_MS, WWDG_TIME_UNUSED, UtWwdg_RefreshCallback );

    WWDG->CR = WWDG_CR_WDGA | WWDG_CR_T_6;
    WWDG->SR = WWDG_SR_EWIF;

    utWwdg_Isr();

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, utWwdg_CallbackRefreshState );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_TIMEOUT_COUNTER, WWDG->CR );
}

/* ============================ REGISTER ACCESS ============================= */

/**
 * \brief   Generic accessors modify only the selected bits of the selected register.
 *
 * \details Uses Wwdg_Set_RegBit(), Wwdg_Reset_RegBit(), Wwdg_Get_RegBit(), Wwdg_Set_RegVal() and
 *          Wwdg_Get_RegVal() on the registers CR, CFR and SR.
 *
 * \par Expected results
 * - Bits are set / cleared in the selected register only, masks of the getters and of the value setter
 *   are applied.
 */
void Ut_Wwdg_Reg_Accessors_ValidId_ModifySelectedBitsOnly( void )
{
    Wwdg_Set_RegBit( WWDG_REG_CR, WWDG_CR_WDGA );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA, WWDG->CR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->SR );

    Wwdg_Set_RegBit( WWDG_REG_CFR, WWDG_CFR_EWI );
    Wwdg_Set_RegBit( WWDG_REG_SR, WWDG_SR_EWIF );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CFR_EWI, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_SR_EWIF, WWDG->SR );

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA, Wwdg_Get_RegBit( WWDG_REG_CR, WWDG_CR_WDGA | WWDG_CR_T ) );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, Wwdg_Get_RegBit( WWDG_REG_CR, WWDG_CR_T ) );

    Wwdg_Reset_RegBit( WWDG_REG_CFR, WWDG_CFR_EWI );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA, WWDG->CR );

    Wwdg_Set_RegVal( WWDG_REG_CFR,
                     WWDG_CFR_WDGTB | WWDG_CFR_W,
                     UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_REG | WWDG_CFR_EWI );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_REG, WWDG->CFR );    /* EWI is out of the mask */

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_WINDOW_REG, Wwdg_Get_RegVal( WWDG_REG_CFR, WWDG_CFR_W ) );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8, Wwdg_Get_RegVal( WWDG_REG_CFR, WWDG_CFR_WDGTB ) );
}


/**
 * \brief   Generic accessors check the register ID before the access.
 *
 * \details Calls all generic accessors with the register ID WWDG_REG_CNT and with a far invalid ID, with
 *          the registers preset to a known value.
 *
 * \par Expected results
 * - Getters return no bit set, setters do not change any register (no access through the table beyond
 *   its size).
 */
void Ut_Wwdg_Reg_Accessors_InvalidId_NoAccess( void )
{
    const wwdg_RegId_t invalidId[] =
    {
        WWDG_REG_CNT,
        (wwdg_RegId_t)( WWDG_REG_CNT + UT_WWDG_ID_OFFSET ),
        (wwdg_RegId_t)UT_WWDG_REG_ALL_BITS
    };
    const uint32_t     idCnt       = sizeof( invalidId ) / sizeof( invalidId[ 0u ] );

    for( uint32_t idIdx = 0u; idCnt > idIdx; idIdx ++ )
    {
        WWDG->CR  = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;
        WWDG->CFR = UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_REG;
        WWDG->SR  = WWDG_SR_EWIF;

        Wwdg_Set_RegBit( invalidId[ idIdx ], UT_WWDG_REG_ALL_BITS );
        Wwdg_Reset_RegBit( invalidId[ idIdx ], UT_WWDG_REG_ALL_BITS );
        Wwdg_Set_RegVal( invalidId[ idIdx ], UT_WWDG_REG_ALL_BITS, UT_WWDG_REG_ZERO );

        TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, Wwdg_Get_RegBit( invalidId[ idIdx ], UT_WWDG_REG_ALL_BITS ) );
        TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, Wwdg_Get_RegVal( invalidId[ idIdx ], UT_WWDG_REG_ALL_BITS ) );

        TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER, WWDG->CR );
        TEST_ASSERT_EQUAL_HEX32( UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_REG, WWDG->CFR );
        TEST_ASSERT_EQUAL_HEX32( WWDG_SR_EWIF, WWDG->SR );
    }
}


/**
 * \brief   Wwdg_Reg_Set_Config() rejects values out of their register fields.
 *
 * \details Calls the function with a prescaler value above the WDGTB field and with a window value above the
 *          W field.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR in both cases, CFR not written.
 */
void Ut_Wwdg_Reg_Set_Config_ValueOutOfField_ReturnsErrorWithoutWrite( void )
{
    WWDG->CFR = UT_WWDG_WINDOW_OPEN;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR,
                       Wwdg_Reg_Set_Config( UT_WWDG_TIMEBASE_OUT_OF_FIELD,
                                            UT_WWDG_WINDOW_REG,
                                            WWDG_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR,
                       Wwdg_Reg_Set_Config( UT_WWDG_TIMEBASE_DIV_8,
                                            UT_WWDG_WINDOW_OUT_OF_FIELD,
                                            WWDG_FUNCTION_INACTIVE ) );

    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_WINDOW_OPEN, WWDG->CFR );
}


/**
 * \brief   Wwdg_Reg_Set_Config() does not write the early wakeup enable bit when it is not requested.
 *
 * \details The enable bit is preset (it can not be cleared by software on hardware). Writes the
 *          configuration without the interrupt, then with the interrupt.
 *
 * \par Expected results
 * - Without the interrupt the enable bit is left as it is (set) and the flag EWIF is not touched.
 * - With the interrupt the enable bit stays set and the flag EWIF is not cleared (the interrupt was enabled
 *   already).
 */
void Ut_Wwdg_Reg_Set_Config_EarlyWakeupBit_NotWrittenWhenNotRequested( void )
{
    WWDG->CFR = WWDG_CFR_EWI | UT_WWDG_WINDOW_OPEN;
    WWDG->SR  = WWDG_SR_EWIF;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK,
                       Wwdg_Reg_Set_Config( UT_WWDG_TIMEBASE_DIV_8, UT_WWDG_WINDOW_REG, WWDG_FUNCTION_INACTIVE ) );

    TEST_ASSERT_EQUAL_HEX32( WWDG_CFR_EWI | UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_REG, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_SR_EWIF, WWDG->SR );

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK,
                       Wwdg_Reg_Set_Config( UT_WWDG_TIMEBASE_DIV_8, UT_WWDG_WINDOW_REG, WWDG_FUNCTION_ACTIVE ) );

    TEST_ASSERT_EQUAL_HEX32( WWDG_CFR_EWI | UT_WWDG_TIMEBASE_DIV_8 | UT_WWDG_WINDOW_REG, WWDG->CFR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_SR_EWIF, WWDG->SR );
}


/**
 * \brief   Wwdg_Reg_Set_Start() and Wwdg_Reg_Set_Refresh() reject a counter without the bit T6.
 *
 * \details Calls both functions with counters which would reset the MCU (bit T6 clear) and with a counter
 *          above the 7-bit field.
 *
 * \par Expected results
 * - WWDG_REQUEST_ERROR, CR not written.
 */
void Ut_Wwdg_Reg_SetStartAndRefresh_CounterWithoutT6_ReturnsErrorWithoutWrite( void )
{
    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Reg_Set_Start( UT_WWDG_COUNTER_BELOW_T6 ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Reg_Set_Start( UT_WWDG_COUNTER_TOO_WIDE ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Reg_Set_Refresh( UT_WWDG_COUNTER_BELOW_T6 ) );
    TEST_ASSERT_EQUAL( WWDG_REQUEST_ERROR, Wwdg_Reg_Set_Refresh( WWDG_CR_WDGA | WWDG_CR_T_6 ) );

    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER, WWDG->CR );
}


/**
 * \brief   Wwdg_Reg_Set_Refresh() writes the counter only.
 *
 * \details Refreshes a watchdog which is not activated, then an activated one.
 *
 * \par Expected results
 * - Counter written, the activation bit is not set by the refresh and not changed when set.
 */
void Ut_Wwdg_Reg_Set_Refresh_ValidCounter_WritesCounterOnly( void )
{
    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Reg_Set_Refresh( UT_WWDG_REFRESH_COUNTER_A ) );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REFRESH_COUNTER_A, WWDG->CR );

    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Reg_Set_Refresh( UT_WWDG_REFRESH_COUNTER_B ) );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_REFRESH_COUNTER_B, WWDG->CR );
}


/**
 * \brief   Early wakeup flag functions read and clear the flag EWIF.
 *
 * \details Reads the flag, sets it, reads it again, clears it and reads it for the last time.
 *
 * \par Expected results
 * - WWDG_FLAG_INACTIVE, WWDG_FLAG_ACTIVE, then WWDG_FLAG_INACTIVE after the clear, other registers untouched.
 */
void Ut_Wwdg_Reg_EwiFlag_SetAndClear_ReportsFlag( void )
{
    WWDG->CR = WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER;

    TEST_ASSERT_EQUAL( WWDG_FLAG_INACTIVE, Wwdg_Reg_Get_EwiFlag() );

    WWDG->SR = WWDG_SR_EWIF;

    TEST_ASSERT_EQUAL( WWDG_FLAG_ACTIVE, Wwdg_Reg_Get_EwiFlag() );

    Wwdg_Reg_Clear_EwiFlag();

    TEST_ASSERT_EQUAL( WWDG_FLAG_INACTIVE, Wwdg_Reg_Get_EwiFlag() );
    TEST_ASSERT_EQUAL_HEX32( UT_WWDG_REG_ZERO, WWDG->SR );
    TEST_ASSERT_EQUAL_HEX32( WWDG_CR_WDGA | UT_WWDG_RUNNING_COUNTER, WWDG->CR );
}


/**
 * \brief   Register state functions report the activation bit and the interrupt enable bit.
 *
 * \details Reads both states with the bits clear, with only the activation bit set and with only the
 *          interrupt enable bit set.
 *
 * \par Expected results
 * - Each state is WWDG_FUNCTION_ACTIVE only while its own bit is set.
 */
void Ut_Wwdg_Reg_GetStates_BitsSetAndClear_ReportsStates( void )
{
    TEST_ASSERT_EQUAL( WWDG_FUNCTION_INACTIVE, Wwdg_Reg_Get_RunState() );
    TEST_ASSERT_EQUAL( WWDG_FUNCTION_INACTIVE, Wwdg_Reg_Get_EwiState() );

    WWDG->CR = WWDG_CR_WDGA;

    TEST_ASSERT_EQUAL( WWDG_FUNCTION_ACTIVE, Wwdg_Reg_Get_RunState() );
    TEST_ASSERT_EQUAL( WWDG_FUNCTION_INACTIVE, Wwdg_Reg_Get_EwiState() );

    WWDG->CR  = UT_WWDG_REG_ZERO;
    WWDG->CFR = WWDG_CFR_EWI;

    TEST_ASSERT_EQUAL( WWDG_FUNCTION_INACTIVE, Wwdg_Reg_Get_RunState() );
    TEST_ASSERT_EQUAL( WWDG_FUNCTION_ACTIVE, Wwdg_Reg_Get_EwiState() );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/** Stub of Nvic_Set_PeriphIrq_Handler - stores registered WWDG ISR */
static nvic_RequestState_t UtWwdg_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId,
                                                      const nvic_IsrCallback_t irqHandler,
                                                      int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_PERIPH_IRQ_WWDG, irqId );

    utWwdg_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/** Stub of Rcc_Get_PeriphClk - counts the calls and returns the bus clock after a change */
static rcc_RequestState_t UtWwdg_RccPclkChangedStub( rcc_PeriphId_t periphId,
                                                     rcc_FreqHz_t * const periphClk,
                                                     int callCnt )
{
    (void)callCnt;
    (void)periphId;

    utWwdg_RccCallCnt++;

    *periphClk = UT_WWDG_PCLK_CHANGED_HZ;

    return ( RCC_REQUEST_OK );
}


/** Early wakeup user callback - counts calls and stores the state of the flag */
static void UtWwdg_EarlyWakeupCallback( void )
{
    utWwdg_CallbackCnt++;
    utWwdg_FlagAtCallback = ( WWDG->SR & WWDG_SR_EWIF );
}


/** Early wakeup user callback - refreshes the watchdog */
static void UtWwdg_RefreshCallback( void )
{
    utWwdg_CallbackCnt++;
    utWwdg_CallbackRefreshState = Wwdg_Set_Refresh();
}


/** Expects the read of the APB clock frequency (every expectation has its own storage of the returned value) */
static void UtWwdg_Expect_PclkFreq( rcc_FreqHz_t pclk )
{
    if( UT_WWDG_PCLK_SLOT_CNT > utWwdg_PclkSlot )
    {
        utWwdg_PclkRet[ utWwdg_PclkSlot ] = pclk;

        Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_WWDG, NULL, RCC_REQUEST_OK );
        Rcc_Get_PeriphClk_IgnoreArg_periphClk();
        Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &utWwdg_PclkRet[ utWwdg_PclkSlot ] );

        utWwdg_PclkSlot++;
    }
    else
    {
        TEST_FAIL_MESSAGE( "Not enough storage for the frequencies of the RCC mock" );
    }
}


/** Expects the RCC and NVIC calls of successful Wwdg_Init() */
static void UtWwdg_Expect_Init( rcc_FreqHz_t pclk, wwdg_FunctionState_t ewiState )
{
    UtWwdg_Expect_PclkFreq( pclk );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_WWDG, RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );

    if( WWDG_FUNCTION_ACTIVE == ewiState )
    {
        Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, UT_WWDG_IRQ_PRIO, NVIC_REQUEST_OK );
        Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );
    }
    else
    {
        /* Interrupt is not configured */
    }
}


/** Expects the RCC and NVIC calls of Wwdg_Init() which fails at a register (APB 32 MHz, no interrupt):
 *  the configuration calls and the deactivation of the interrupt by Wwdg_Deinit() */
static void UtWwdg_Expect_InitFailure( void )
{
    UtWwdg_Expect_PclkFreq( UT_WWDG_PCLK_HZ );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_WWDG, RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_WWDG, NVIC_REQUEST_OK );    /* Deinit */
}


/** Returns watchdog configuration */
static wwdg_Config_t UtWwdg_Get_Config( wwdg_Time_ms_t timeout, wwdg_Time_ms_t window, wwdg_EarlyWakeupIsr_t *isr )
{
    wwdg_Config_t config;

    config.Timeout        = timeout;
    config.Window         = window;
    config.EarlyWakeupIsr = isr;
    config.IrqPriority    = UT_WWDG_IRQ_PRIO;

    return ( config );
}


/** Initializes the watchdog (APB 32 MHz) and expects success */
static void UtWwdg_Init_Running( wwdg_Time_ms_t timeout, wwdg_Time_ms_t window, wwdg_EarlyWakeupIsr_t *isr )
{
    wwdg_Config_t config = UtWwdg_Get_Config( timeout, window, isr );

    if( NULL != isr )
    {
        UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_ACTIVE );
    }
    else
    {
        UtWwdg_Expect_Init( UT_WWDG_PCLK_HZ, WWDG_FUNCTION_INACTIVE );
    }

    TEST_ASSERT_EQUAL( WWDG_REQUEST_OK, Wwdg_Init( &config ) );
}

/* ============================== HW MODELS ================================= */

/** HW model: records the window field when the activation bit is seen set for the first time */
static void UtWwdg_HwModel_StartWindow( void )
{
    if( ( UT_WWDG_REG_ZERO != ( WWDG->CR & WWDG_CR_WDGA ) ) &&
        ( UT_WWDG_REG_ZERO == utWwdg_StartSeen           )    )
    {
        utWwdg_StartWindow = ( WWDG->CFR & WWDG_CFR_W );
        utWwdg_StartSeen   = UT_WWDG_CALLS_ONCE;
    }
    else
    {
        /* Watchdog not started yet or the start was already recorded */
    }
}


/** HW model: the window field of CFR stays zero (written configuration does not stay) */
static void UtWwdg_HwModel_CfgNotKept( void )
{
    (void)__atomic_and_fetch( &WWDG->CFR, (uint32_t)~WWDG_CFR_W, __ATOMIC_SEQ_CST );
}


/** HW model: a window narrower than the open window is replaced by the open window */
static void UtWwdg_HwModel_WindowNotKept( void )
{
    if( WWDG_CFR_W != ( WWDG->CFR & WWDG_CFR_W ) )
    {
        (void)__atomic_or_fetch( &WWDG->CFR, (uint32_t)WWDG_CFR_W, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* Window is open, it stays as written */
    }
}


/** HW model: the activation bit is cleared after every store (start not confirmed) */
static void UtWwdg_HwModel_StartNotKept( void )
{
    (void)__atomic_and_fetch( &WWDG->CR, (uint32_t)~WWDG_CR_WDGA, __ATOMIC_SEQ_CST );
}


/** HW model: the counter is 0x7F while the watchdog is activated (written counter does not stay) */
static void UtWwdg_HwModel_CounterNotKept( void )
{
    if( UT_WWDG_REG_ZERO != ( WWDG->CR & WWDG_CR_WDGA ) )
    {
        (void)__atomic_or_fetch( &WWDG->CR, (uint32_t)WWDG_CR_T, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* Watchdog is not activated, the counter stays as written */
    }
}
