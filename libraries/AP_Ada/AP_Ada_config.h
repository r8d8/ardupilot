#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// enabled by ./waf configure --enable-ada, which also builds the library
#ifndef AP_ADA_ENABLED
#define AP_ADA_ENABLED 0
#endif

// per-call time budget of each shadow (HZ-19, HZ-20): a call over it is
// counted and reported; after AP_ADA_TRIP_OVERRUNS of them since boot that
// shadow is switched off. Set well above the expected few microseconds so
// preemption by interrupts and higher-priority threads does not trip it
// (the H7 times are still to be measured on the bench).
#ifndef AP_ADA_UBX_BUDGET_US
#define AP_ADA_UBX_BUDGET_US 100
#endif

#ifndef AP_ADA_SWASH_BUDGET_US
#define AP_ADA_SWASH_BUDGET_US 100
#endif

#ifndef AP_ADA_TRIP_OVERRUNS
#define AP_ADA_TRIP_OVERRUNS 10
#endif
