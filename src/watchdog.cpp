// ---------------------------------------------------------------------------
// Rally CAN Gauge - hardware watchdog implementation (i.MX RT1062 WDOG1)
// ---------------------------------------------------------------------------
// How the peripheral works: a down-counter that resets the chip when it
// reaches zero. Writing the magic sequence 0x5555, 0xAAAA to the service
// register (WSR) reloads it - that's "feeding". The magic numbers exist so
// runaway code scribbling over memory-mapped registers can't feed it by
// accident; it would have to write exactly those two values in order.
//
// To test the whole chain: comment out the watchdogFeed() call in main.cpp,
// flash, and watch the board reboot once a second with "WATCHDOG RESET" in
// the boot banner.
// ---------------------------------------------------------------------------

#include "watchdog.h"
#include "config.h"

#include <Arduino.h> // imxrt.h register definitions come in through here

void watchdogPrintResetCause()
    {
    // SRC_SRSR = System Reset Controller / Reset Status Register. Each bit
    // remembers one reset cause until software clears it (write-1-to-clear).
    const uint32_t srsr = SRC_SRSR;
    Serial.printf("Reset cause: SRC_SRSR=0x%03lX%s%s%s\n",
                  (unsigned long)srsr,
                  (srsr & SRC_SRSR_IPP_RESET_B) ? " [power-on]" : "",
                  (srsr & SRC_SRSR_WDOG_RST_B) ? " [WATCHDOG RESET - firmware hung!]" : "",
                  (srsr & SRC_SRSR_LOCKUP_SYSRESETREQ) ? " [CPU lockup/software reset]" : "");
    SRC_SRSR = srsr; // clear, so the next boot reports only its own cause
    }

void watchdogInit()
    {
    // Timeout field WT: timeout = 0.5 s * (WT + 1), so 1000 ms -> WT = 1.
    static_assert(WATCHDOG_TIMEOUT_MS % 500 == 0 && WATCHDOG_TIMEOUT_MS >= 500,
                  "watchdog timeout must be a multiple of 500 ms");
    const uint16_t wt = (uint16_t)(WATCHDOG_TIMEOUT_MS / 500 - 1);

    WDOG1_WMCR = 0; // switch off the separate 16 s "power-down counter"
                    // (a second, always-armed watchdog that must be
                    // disabled once shortly after every reset)

    // One deliberate write assembles the whole control register. SRS and WDA
    // are ACTIVE-LOW action bits: writing 0 to them would trigger a reset
    // right now, so they must be written as 1 while setting everything else.
    WDOG1_WCR = WDOG_WCR_WT(wt)   // the timeout
              | WDOG_WCR_SRS      // (active low) do NOT software-reset now
              | WDOG_WCR_WDA      // (active low) do NOT assert WDOG_B now
              | WDOG_WCR_WDT      // on timeout: assert WDOG_B -> chip reset
              | WDOG_WCR_WDBG     // pause while a debugger has the core halted
              | WDOG_WCR_WDZST    // pause in low-power modes (we never sleep,
                                  // but it makes bench debugging forgiving)
              | WDOG_WCR_WDE;     // enable. From here on this bit CANNOT be
                                  // cleared - only a reset disarms the dog.

    watchdogFeed(); // start from a full timeout

    Serial.printf("Watchdog armed: %lu ms timeout, fed by the 20 ms tick\n",
                  (unsigned long)WATCHDOG_TIMEOUT_MS);
    }

void watchdogFeed()
    {
    WDOG1_WSR = 0x5555; // the magic service sequence
    WDOG1_WSR = 0xAAAA;
    }
