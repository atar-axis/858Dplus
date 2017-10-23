#ifndef INC_WATCHDOG_H
#define INC_WATCHDOG_H

#include "settings.h"

#include <stdint.h>
#include <stdbool.h>


extern uint8_t _mcusr;

extern void watchdogOn();
extern void watchdogOff(bool early);
extern void watchdogTestCpuFreq();
extern bool watchdogCheck();
extern void watchdogReset();

#endif /* INC_WATCHDOG_H */