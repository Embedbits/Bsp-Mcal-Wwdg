/**
 * \author Mr.Nobody
 * \file Wwdg_Reg.c
 * \ingroup Wwdg
 * \brief Wwdg module Reg component functionality.
 *
 * Register access of the module: generic accessors (bounds checked register ID)
 * and the register protocol of the window watchdog. The registers are written
 * only here, the masks come from the CMSIS device header.
 *
 * Properties of the registers, which the protocol respects (RM0481, RM0492,
 * RM0517 - WWDG chapter):
 * - WWDG_CR.WDGA is set only by software and cleared only by a system reset.
 * - WWDG_CFR.EWI is set only by software and cleared only by a system reset.
 * - WWDG_CR is written with the bit T6 always set, otherwise the MCU is reset.
 * - The refresh (write of WWDG_CR) above the window (WWDG_CFR.W) resets the MCU.
 * - WWDG_SR.EWIF is cleared by writing 0, writing 1 has no effect.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Wwdg_Reg.h"                       /* Self include                   */
#include "Wwdg_Types.h"                     /* Module types definitions       */
#include "Stm32.h"                          /* WWDG registers definition      */
/* ============================== TYPEDEFS ================================== */

/** \brief Structure type used to store physical address of WWDG registers.
 * This array is used to reduce size of configuration array. With this style
 * the registers are referenced through enumeration.
 *
 */
typedef struct
{
    wwdg_RegId_t        RegId;   /**< Peripheral register ID      */
    volatile uint32_t * RegAddr; /**< Peripheral register address */
}   wwdg_RegList_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static wwdg_RequestState_t  Wwdg_Reg_Wait_RegVal    ( wwdg_RegId_t regId, uint32_t regMask, uint32_t regValue );
static wwdg_FunctionState_t Wwdg_Reg_Get_BitState   ( wwdg_RegId_t regId, uint32_t bitMask );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Count of read-back iterations of a written register value */
#define WWDG_REG_READBACK_TIMEOUT_RAW   ( 0x1000u )

/** \brief Count of read-back iterations of the watchdog start (counter and activation bit) */
#define WWDG_REG_START_TIMEOUT_RAW      ( 0x1000u )

/** \brief Register value without any bit set (masked register value of an inactive bit field) */
#define WWDG_REG_BITS_CLEAR             ( 0u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief WWDG registers configuration structure
 *
 * This structure is used to store addresses of WWDG registers.
 * It is used to reduce code size while referring to WWDG registers.
 */
static const wwdg_RegList_t             wwdg_RegList[] =
{
    { .RegId = WWDG_REG_CR  , .RegAddr = &(WWDG->CR ) }, /**< WWDG control register       */
    { .RegId = WWDG_REG_CFR , .RegAddr = &(WWDG->CFR) }, /**< WWDG configuration register */
    { .RegId = WWDG_REG_SR  , .RegAddr = &(WWDG->SR ) }, /**< WWDG status register        */
};

_Static_assert( (sizeof(wwdg_RegList) / sizeof(wwdg_RegList_t)) == WWDG_REG_CNT,
                "Wwdg_Reg: wwdg_RegList has incorrect size." );

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Sets the register bit with given mask.
 *
 * \param regId   [in]: WWDG register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void Wwdg_Set_RegBit( wwdg_RegId_t regId, uint32_t bitMask )
{
    if( WWDG_REG_CNT > regId )
    {
        *wwdg_RegList[regId].RegAddr |= bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Resets the register bit with given mask.
 *
 * \param regId   [in]: WWDG register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void Wwdg_Reset_RegBit( wwdg_RegId_t regId, uint32_t bitMask )
{
    if( WWDG_REG_CNT > regId )
    {
        *wwdg_RegList[regId].RegAddr &= ~bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register bit value with given mask.
 *
 * \param regId   [in]: WWDG register ID
 * \param bitMask [in]: Mask to apply to the register
 * \return Register bit value with applied mask, no bit set for invalid register ID
 */
uint32_t Wwdg_Get_RegBit( wwdg_RegId_t regId, uint32_t bitMask )
{
    uint32_t retValue = WWDG_REG_BITS_CLEAR;

    if( WWDG_REG_CNT > regId )
    {
        retValue = ( *wwdg_RegList[regId].RegAddr & bitMask );
    }
    else
    {
        /* Invalid register ID, no bit is set */
        retValue = WWDG_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Updates the register value with given mask and value.
 *
 * \param regId    [in]: WWDG register ID
 * \param regMask  [in]: Mask to apply to the register
 * \param regValue [in]: Value to set in the register
 */
void Wwdg_Set_RegVal( wwdg_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    if( WWDG_REG_CNT > regId )
    {
        /* Set the register value with mask */
        *wwdg_RegList[regId].RegAddr = ( ( *wwdg_RegList[regId].RegAddr & ~regMask ) |
                                         ( regValue & regMask ) );
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register value with given mask.
 *
 * \param regId   [in]: WWDG register ID.
 * \param regMask [in]: Mask to apply to the register
 * \return Register value with applied mask, no bit set for invalid register ID
 */
uint32_t Wwdg_Get_RegVal( wwdg_RegId_t regId, uint32_t regMask )
{
    uint32_t retValue = WWDG_REG_BITS_CLEAR;

    if( WWDG_REG_CNT > regId )
    {
        /* Get the register value with mask */
        retValue = ( *wwdg_RegList[regId].RegAddr & regMask );
    }
    else
    {
        /* Invalid register ID, no bit is set */
        retValue = WWDG_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Writes the timebase, the window and the early wakeup interrupt enable
 *        to the configuration register and verifies them by read-back.
 *
 * The early wakeup interrupt enable bit is set only by software and cleared only
 * by a system reset, it is therefore written (and verified) only when the interrupt
 * is requested, otherwise the bit is left as it is. The pending flag is cleared
 * before the enable bit is set for the first time (reference manual).
 *
 * \param timebase [in]: Value of the WDGTB field (already positioned)
 * \param window   [in]: Value of the W field (already positioned)
 * \param ewiState [in]: \ref WWDG_FUNCTION_ACTIVE sets the early wakeup interrupt enable bit
 *
 * \return Returns \ref WWDG_REQUEST_OK if the values read back within the timeout.
 *         Returns \ref WWDG_REQUEST_ERROR for a value out of its field or when the
 *         read-back does not match.
 */
wwdg_RequestState_t Wwdg_Reg_Set_Config( uint32_t timebase, uint32_t window, wwdg_FunctionState_t ewiState )
{
    wwdg_RequestState_t retState   = WWDG_REQUEST_ERROR;
    uint32_t            writeMask  = ( WWDG_CFR_WDGTB | WWDG_CFR_W );
    uint32_t            writeValue = ( timebase | window );

    if( ( WWDG_REG_BITS_CLEAR == ( timebase & ~WWDG_CFR_WDGTB ) ) &&
        ( WWDG_REG_BITS_CLEAR == ( window   & ~WWDG_CFR_W     ) )    )
    {
        if( WWDG_FUNCTION_ACTIVE == ewiState )
        {
            const wwdg_FunctionState_t ewiEnabled = Wwdg_Reg_Get_EwiState();

            if( WWDG_FUNCTION_INACTIVE == ewiEnabled )
            {
                /* Unwanted pending interrupt is cleared before the enable bit is set */
                Wwdg_Reg_Clear_EwiFlag();
            }
            else
            {
                /* Interrupt is enabled already, flag is not touched */
            }

            writeMask  |= WWDG_CFR_EWI;
            writeValue |= WWDG_CFR_EWI;
        }
        else
        {
            /* Enable bit can not be cleared by software, it is not written */
        }

        Wwdg_Set_RegVal( WWDG_REG_CFR, writeMask, writeValue );

        retState = Wwdg_Reg_Wait_RegVal( WWDG_REG_CFR, writeMask, writeValue );
    }
    else
    {
        /* Value is out of its register field, nothing is written */
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts the watchdog (activation bit and counter) and verifies the start.
 *
 * The counter is a running down-counter, the value read back is therefore checked
 * for the range between the bit T6 and the written value, the activation bit
 * has to be set. The bit T6 of the written counter is set (reset of the MCU otherwise).
 *
 * \param counter [in]: Counter value (already positioned) with bit T6 set
 *
 * \return Returns \ref WWDG_REQUEST_OK if the watchdog is running within the timeout.
 *         Returns \ref WWDG_REQUEST_ERROR for an invalid counter (nothing is written)
 *         or when the start was not confirmed.
 */
wwdg_RequestState_t Wwdg_Reg_Set_Start( uint32_t counter )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( ( WWDG_CR_T_6          == ( counter & WWDG_CR_T_6 ) ) &&
        ( WWDG_REG_BITS_CLEAR  == ( counter & ~WWDG_CR_T  ) )    )
    {
        Wwdg_Set_RegVal( WWDG_REG_CR, ( WWDG_CR_WDGA | WWDG_CR_T ), ( WWDG_CR_WDGA | counter ) );

        for( uint32_t iterationCnt = 0u; WWDG_REG_START_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = Wwdg_Get_RegVal( WWDG_REG_CR, ( WWDG_CR_WDGA | WWDG_CR_T ) );
            const uint32_t cntValue = ( regValue & WWDG_CR_T );

            if( ( WWDG_CR_WDGA == ( regValue & WWDG_CR_WDGA ) ) &&
                ( WWDG_CR_T_6  == ( cntValue & WWDG_CR_T_6  ) ) &&
                ( counter      >= cntValue                    )    )
            {
                retState = WWDG_REQUEST_OK;
                break;
            }
            else
            {
                /* Start is not confirmed yet */
                retState = WWDG_REQUEST_ERROR;
            }
        }
    }
    else
    {
        /* Counter would reset the MCU, nothing is written */
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Refreshes (reloads) the watchdog counter.
 *
 * The activation bit is not written (it is set only by software), the refresh
 * therefore never starts a stopped watchdog. A refresh above the window resets
 * the MCU (hardware behavior).
 *
 * \param counter [in]: Counter value (already positioned) with bit T6 set
 *
 * \return Returns \ref WWDG_REQUEST_OK if the counter was written. Returns
 *         \ref WWDG_REQUEST_ERROR for an invalid counter (nothing is written).
 */
wwdg_RequestState_t Wwdg_Reg_Set_Refresh( uint32_t counter )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    if( ( WWDG_CR_T_6          == ( counter & WWDG_CR_T_6 ) ) &&
        ( WWDG_REG_BITS_CLEAR  == ( counter & ~WWDG_CR_T  ) )    )
    {
        Wwdg_Set_RegVal( WWDG_REG_CR, WWDG_CR_T, counter );

        retState = WWDG_REQUEST_OK;
    }
    else
    {
        /* Counter would reset the MCU, nothing is written */
        retState = WWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Clears the early wakeup interrupt flag.
 *
 * The flag is cleared by writing 0 (writing 1 has no effect).
 */
void Wwdg_Reg_Clear_EwiFlag( void )
{
    Wwdg_Reset_RegBit( WWDG_REG_SR, WWDG_SR_EWIF );
}


/**
 * \brief Returns the early wakeup interrupt flag.
 *
 * \note  The flag is set by hardware also when the interrupt is not enabled.
 *
 * \return \ref WWDG_FLAG_ACTIVE if the counter reached the value 0x40 since the
 *         flag was cleared, otherwise \ref WWDG_FLAG_INACTIVE.
 */
wwdg_FlagState_t Wwdg_Reg_Get_EwiFlag( void )
{
    wwdg_FlagState_t           retState = WWDG_FLAG_INACTIVE;
    const wwdg_FunctionState_t bitState = Wwdg_Reg_Get_BitState( WWDG_REG_SR, WWDG_SR_EWIF );

    if( WWDG_FUNCTION_ACTIVE == bitState )
    {
        retState = WWDG_FLAG_ACTIVE;
    }
    else
    {
        retState = WWDG_FLAG_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Returns the activation bit of the watchdog.
 *
 * \return \ref WWDG_FUNCTION_ACTIVE if the watchdog is activated (it can not be
 *         deactivated again by software), otherwise \ref WWDG_FUNCTION_INACTIVE.
 */
wwdg_FunctionState_t Wwdg_Reg_Get_RunState( void )
{
    return ( Wwdg_Reg_Get_BitState( WWDG_REG_CR, WWDG_CR_WDGA ) );
}


/**
 * \brief Returns the early wakeup interrupt enable bit.
 *
 * \return \ref WWDG_FUNCTION_ACTIVE if the enable bit is set (it can not be
 *         cleared again by software), otherwise \ref WWDG_FUNCTION_INACTIVE.
 */
wwdg_FunctionState_t Wwdg_Reg_Get_EwiState( void )
{
    return ( Wwdg_Reg_Get_BitState( WWDG_REG_CFR, WWDG_CFR_EWI ) );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Waits until the register (bit field) reads back the required value.
 *
 * \param regId    [in]: WWDG register ID
 * \param regMask  [in]: Mask of the checked bits
 * \param regValue [in]: Required value of the masked bits
 *
 * \return Returns \ref WWDG_REQUEST_OK if the value was read back within the
 *         timeout. Otherwise returns \ref WWDG_REQUEST_ERROR.
 */
static wwdg_RequestState_t Wwdg_Reg_Wait_RegVal( wwdg_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    wwdg_RequestState_t retState = WWDG_REQUEST_ERROR;

    for( uint32_t iterationCnt = 0u; WWDG_REG_READBACK_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const uint32_t actualValue = Wwdg_Get_RegVal( regId, regMask );

        if( ( regValue & regMask ) == actualValue )
        {
            retState = WWDG_REQUEST_OK;
            break;
        }
        else
        {
            /* Register has not been updated yet */
            retState = WWDG_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Returns the state of a register bit field as module enumeration.
 *
 * \param regId   [in]: WWDG register ID
 * \param bitMask [in]: Mask of the checked bits
 *
 * \return \ref WWDG_FUNCTION_ACTIVE if any bit of the mask is set, otherwise
 *         \ref WWDG_FUNCTION_INACTIVE (also for an invalid register ID).
 */
static wwdg_FunctionState_t Wwdg_Reg_Get_BitState( wwdg_RegId_t regId, uint32_t bitMask )
{
    wwdg_FunctionState_t retState = WWDG_FUNCTION_INACTIVE;
    const uint32_t       bitValue = Wwdg_Get_RegBit( regId, bitMask );

    if( WWDG_REG_BITS_CLEAR != bitValue )
    {
        retState = WWDG_FUNCTION_ACTIVE;
    }
    else
    {
        retState = WWDG_FUNCTION_INACTIVE;
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
