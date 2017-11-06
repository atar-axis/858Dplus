#include "watchdog.h"

#include <stdbool.h>

#include <avr/wdt.h>
#include <util/delay.h>


uint8_t _mcusr __attribute__ ((section(".noinit")));

void watchdogOff(bool early)
{
	wdt_reset();
	if(early) _mcusr = MCUSR;
	MCUSR &= ~_BV(WDRF);	// clear WDRF, as it overrides WDE-bit in WDTCSR-reg and leads to endless reset-loop (15ms timeout after wd-reset)
	wdt_disable();
}

void watchdogOn(void)
{
#ifdef USE_WATCHDOG
	wdt_reset();
	MCUSR &= ~_BV(WDRF);	// clear WDRF, as it overrides WDE-bit in WDTCSR-reg and leads to endless reset-loop (15ms timeout after wd-reset)
	wdt_enable(WDTO_120MS);
#endif
}

void watchdogTestCpuFreq(void)
{
#ifdef USE_WATCHDOG
	/*
	 * Hopefully cause a watchdog reset if the CKDIV8 FUSE is set (F_CPU 1MHz instead of 8MHz)
	 *
	 */
	watchdogReset();
	MCUSR &= ~_BV(WDRF);	// clear WDRF, as it overrides WDE-bit in WDTCSR-reg and leads to endless reset-loop (15ms timeout after wd-reset)
	wdt_enable(WDTO_120MS);
	_delay_ms(40);		// IF "CKDIV8" fuse is erroneously set, this should delay by 8x40 = 320ms & cause the dog to bite!

	watchdogOff(false);		// IF we got to here, F_CPU is OK.
#endif
}

inline bool watchdogCheck()
{
#ifdef USE_WATCHDOG
	return ((_mcusr & _BV(WDRF)) != 0 ? true : false);
#endif
}

inline void watchdogReset()
{
#ifdef USE_WATCHDOG
	wdt_reset();
#endif
}
