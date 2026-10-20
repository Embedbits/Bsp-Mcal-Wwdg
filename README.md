# WWDG Peripheral Driver

This module provides an abstraction layer for the **system window watchdog** (`WWDG`) of STM32 MCUs: start, configuration
and refresh of the watchdog in **milliseconds**, the early wakeup interrupt and the readout of the state. It is a
low-level MCAL module: it owns the watchdog hardware, the prescaler, the counter and window calculation, the interrupt
and the read-back verification of the configuration; the application and the other modules never touch the registers.

The public interface (`Wwdg_Port.h`, `Wwdg_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: design.** No implementation exists yet, the interface below is a proposal. Stories and branches follow.

---

## Intent and purpose

- The independent watchdog (IWDG module) detects a program that **stopped**. The window watchdog detects a program that
  **runs at the wrong time**: it resets the MCU when the refresh comes too late **and** when it comes too early. It is
  the right tool for a cyclic task whose period is known (a control loop, a communication cycle) - a runaway loop that
  refreshes the watchdog on every pass is caught, which the independent watchdog does not do.
- The WWDG is clocked by the APB clock (not by an independent oscillator), so its timing is accurate and follows the bus
  clock, but it dies together with that clock. It complements the IWDG, it does not replace it: a safety-relevant
  application runs both.
- The user thinks in time, not in registers: the module converts the required timeout and window in ms to the prescaler,
  the counter reload value and the window value of the family, and reports the **achieved** values back.
- The module hides the differences between the families: the width of the prescaler, the position of the counter and
  window fields, the write protection, the interrupt line and the option bytes that start the watchdog in hardware
  mode differ, the interface stays the same.

What the module is **not**: it is not a general purpose timer and it does not stop the watchdog (the hardware does not
allow it). It does not configure the freezing of the watchdog in the debug mode (debug MCU module, future) and it does
not decide what the application does in the early wakeup callback.

---

## Concept

The watchdog is a **7-bit down counter** that counts from the reload value down to `0x40` (the "T6" bit). Three events
are defined by the hardware:

| Event | Result |
|---|---|
| The counter passes from `0x40` to `0x3F` (the bit T6 is cleared) | **Reset** of the MCU (the watchdog expired) |
| The counter is written (refreshed) while its value is **above** the window value | **Reset** of the MCU (too early) |
| The counter reaches `0x40` | **Early wakeup interrupt** (EWI), if enabled - one counter tick before the reset |

One counter tick lasts `4096 x prescaler / f(PCLK)`. The module works with these user visible quantities:

| Item | Meaning |
|---|---|
| **Timeout** | Time from a refresh to the reset when the watchdog is not refreshed again. |
| **Window** | Length of the last part of the timeout, in which the refresh is allowed. A refresh earlier than `Window` before the end of the timeout resets the MCU. `WWDG_TIME_UNUSED` opens the window for the whole timeout (the counter can be refreshed any time). |
| **Early wakeup** | Fixed by the hardware: one counter tick before the reset. Only the callback is configured, the time is read back from the module. |

```
 refresh                                  window opens                         reset
   |<--------------------------- Timeout ------------------------------------->|
   |                                   |<------------- Window ---------------->|
   |   refresh here = reset (early)    |   refresh here = OK    | EWI -> reset |
```

The module chooses the **smallest prescaler** with which the timeout fits to the counter (the best resolution), rounds the
timeout and the window to the counter ticks and verifies the written values by a read-back. `Wwdg_Get_Timeout`,
`Wwdg_Get_Window` and `Wwdg_Get_EarlyWakeup` return the real values.

Rules:

1. `Wwdg_Init` starts the watchdog. From this moment the application has to refresh it in the window; **the watchdog
   cannot be stopped** (a hardware limitation) except by a system reset.
2. `Wwdg_Deinit` disables the early wakeup interrupt and the automatic refresh only. The watchdog keeps running.
3. An invalid configuration (timeout out of range, window longer than the timeout) is rejected **before** any register is
   written.
4. `Wwdg_Task` refreshes the watchdog automatically, but **only when the window is not used** - with a window the
   application has to call `Wwdg_Set_Refresh` inside it.
5. If the watchdog was started by the option bytes (hardware watchdog mode of the families that have it),
   `Wwdg_Get_State` returns active before `Wwdg_Init` and `Wwdg_Init` only takes over the refresh and the interrupt.

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Watchdog | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `WWDG`, APB1 clock, 7-bit counter, 3-bit prescaler (1 ... 128), early wakeup interrupt, hardware mode by the option byte `WWDG_SW` | planned (first) |
| STM32U5 | `Dev/STM32U5` | `WWDG`, APB1 clock | planned |
| STM32G4, STM32L4 | `Dev/STM32G4`, `Dev/STM32L4` | `WWDG`, APB1 clock, 3-bit prescaler | planned |
| STM32F4, STM32F7 | `Dev/STM32F4`, `Dev/STM32F7` | `WWDG`, APB1 clock, 2-bit prescaler (1 ... 8), no hardware mode | planned |
| STM32H7 | `Dev/STM32H7` | `WWDG1` (and `WWDG2` on the dual core devices) | planned |
| STM32U3 | `Dev/STM32U3` | `WWDG` | planned |

A function without a meaning in a family stays in the interface, returns `WWDG_REQUEST_ERROR` and suppresses the unused
parameters (for example the readout of the hardware mode on a family without option bytes for the WWDG).

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `Wwdg_Get_ModuleVersion`, `Wwdg_Init`, `Wwdg_Deinit`, `Wwdg_Task`, `Wwdg_Get_DefaultConfig` |
| State | `Wwdg_Get_State`, `Wwdg_Get_Timeout`, `Wwdg_Get_Window`, `Wwdg_Get_EarlyWakeup`, `Wwdg_Get_RemainingTime` |
| Refresh | `Wwdg_Set_Refresh` |

```c
wwdg_ModuleVersion_t Wwdg_Get_ModuleVersion  ( void );

wwdg_RequestState_t  Wwdg_Init               ( const wwdg_Config_t * const wwdgConfig );
wwdg_RequestState_t  Wwdg_Deinit             ( void );
void                 Wwdg_Task               ( void );
wwdg_RequestState_t  Wwdg_Get_DefaultConfig  ( wwdg_Config_t * const wwdgConfig );

wwdg_RequestState_t  Wwdg_Get_State          ( wwdg_FunctionState_t * const wwdgState );
wwdg_RequestState_t  Wwdg_Get_Timeout        ( wwdg_Time_us_t * const timeout );
wwdg_RequestState_t  Wwdg_Get_Window         ( wwdg_Time_us_t * const window );
wwdg_RequestState_t  Wwdg_Get_EarlyWakeup    ( wwdg_Time_us_t * const earlyWakeup );
wwdg_RequestState_t  Wwdg_Get_RemainingTime  ( wwdg_Time_us_t * const remainingTime );

wwdg_RequestState_t  Wwdg_Set_Refresh        ( void );
```

Every function returns `WWDG_REQUEST_OK` / `WWDG_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `wwdg_Config_t` | `Timeout` (ms), `Window` (ms or `WWDG_TIME_UNUSED`), `EarlyWakeupIsr` (callback, `WWDG_NULL_PTR` = interrupt disabled), `IrqPriority` |
| `wwdg_Time_ms_t` | Time in ms, the type of the configuration (`WWDG_TIME_UNUSED` = no window) |
| `wwdg_Time_us_t` | Time in us, the type of the read-back functions (one tick can be shorter than 1 ms) |
| `wwdg_FunctionState_t` | `WWDG_FUNCTION_INACTIVE` / `WWDG_FUNCTION_ACTIVE` (is the watchdog running) |
| `wwdg_EarlyWakeupIsr_t` | `void ( void )` callback called from the interrupt |
| `wwdg_RequestState_t` | `WWDG_REQUEST_OK` / `WWDG_REQUEST_ERROR` |

---

## Usage

### Watchdog with a window and a warning

```c
#include "Wwdg_Port.h"

static void App_WatchdogWarning( void )
{
    /* One tick before the reset: store the diagnostic data, no waiting, no heavy work */
}

wwdg_Config_t wwdgConfig;

(void)Wwdg_Get_DefaultConfig( &wwdgConfig );        /* timeout 30 ms, no window, no early wakeup interrupt  */

wwdgConfig.Timeout        = 40u;                    /* reset 40 ms after the last refresh                    */
wwdgConfig.Window         = 15u;                    /* refresh allowed only in the last 15 ms               */
wwdgConfig.EarlyWakeupIsr = App_WatchdogWarning;
wwdgConfig.IrqPriority    = 0u;

(void)Wwdg_Init( &wwdgConfig );

/* in the cyclic task, 25 ms ... 40 ms after the previous refresh */
(void)Wwdg_Set_Refresh();
```

### Watchdog refreshed by the module

```c
wwdg_Config_t wwdgConfig;

(void)Wwdg_Get_DefaultConfig( &wwdgConfig );
wwdgConfig.Timeout = 25u;                           /* no window */
(void)Wwdg_Init( &wwdgConfig );

for( ;; )
{
    Wwdg_Task();                                    /* refreshes the watchdog (no window configured) */
}
```

### Reading the achieved values

```c
wwdg_Time_us_t achievedTimeout = 0u;

(void)Wwdg_Get_Timeout( &achievedTimeout );         /* e.g. 40 ms is rounded to the counter ticks */
```

The reset caused by the window watchdog is detected after the start-up by the RCC module:
`Rcc_Get_ResetSource( RCC_RESET_SRC_WWDG, &flag )`, and cleared by `Rcc_Set_ResetSourceClear()`.

---

## Behaviour and limits

- **No stop.** The watchdog cannot be stopped once started. `Wwdg_Deinit` does not stop it, the application has to keep
  refreshing it.
- **Refresh outside of the window** resets the MCU at once (hardware behaviour), it cannot be reported by the software.
  Use `Wwdg_Get_RemainingTime` to decide whether the window is already open.
- **Range.** The timeout is limited by the 6 usable bits of the counter (64 ticks) and by the prescaler. With a bus clock
  of 32 MHz and a prescaler of 1 ... 128 (STM32H5) it is approx. 128 us ... 1.05 s; with the 4 prescaler steps of STM32F4
  the upper limit is lower. The range of the family is in the README of the family branch.
- **Clock dependency.** The timing is derived from the APB clock read from the RCC module at `Wwdg_Init`. When the bus
  clock changes after the initialization, the module has to be initialized again.
- **Early wakeup callback** runs in the interrupt one tick before the reset (the tick can be shorter than a millisecond):
  keep it short and do not call functions that wait. The interrupt flag is cleared by the module.
- **Low-power and debug.** The behaviour in the low-power modes and the freezing of the counter when the core is
  stopped in the debugger differ between the families; the README of the family describes it.
- **Not part of the first version:** the freezing of the counter in the debug mode (debug module), the selection of the
  hardware watchdog mode (written by the flash / option bytes module).

---

## Dependencies

- **Rcc** - peripheral clock and the frequency of the APB clock (`RCC_PERIPH_WWDG`), reset source
- **Nvic** - early wakeup interrupt
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC and NVIC configurations are handled internally.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the calculation of the prescaler / counter / window from the timeout
  for the bus clocks and the limits (smallest and longest timeout, window equal to the timeout), the rejection of an invalid
  configuration before any register write, the read-back verification, the `Wwdg_Task` rules, the early wakeup interrupt.
- **Integration tests** (target, probe-rs): refresh inside and outside of the window (the reset cause read from RCC), the
  achieved timeout measured by a timer, the early wakeup callback.

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `WWDG_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `Wwdg_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `Wwdg_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`wwdg_FunctionState_t`, `wwdg_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `WWDG_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `Wwdg_Port.h`, `Wwdg_Types.h` | Public interface (identical in all families) |
| `Wwdg*.c`, `Wwdg.h` | Implementation of the family |
| `CMakeLists.txt` | Library `Wwdg_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE Wwdg_Lib )
```

1. Link `Wwdg_Lib` to your CMake library.
2. Include `Wwdg_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
