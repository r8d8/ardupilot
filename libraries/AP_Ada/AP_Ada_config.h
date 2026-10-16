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

#ifndef AP_ADA_CRSF_BUDGET_US
#define AP_ADA_CRSF_BUDGET_US 100
#endif

// the CRSF shadow runs in the RC input thread, which higher-priority
// threads preempt: it is switched off once the 99th percentile of its
// per-byte times is over budget this many seconds in a row
#ifndef AP_ADA_CRSF_TRIP_INTERVALS
#define AP_ADA_CRSF_TRIP_INTERVALS 3
#endif

// the navigation filter: one predict (21-state covariance) and the
// fusions that have new data, at AP_ADA_NAV_RATE_HZ
#ifndef AP_ADA_NAV_BUDGET_US
#define AP_ADA_NAV_BUDGET_US 1000
#endif

// predict rate: the IMU is added up over the interval (100 Hz matches EKF3
// on the Goblin logs as well as 400 Hz does, fcs/README.md)
#ifndef AP_ADA_NAV_RATE_HZ
#define AP_ADA_NAV_RATE_HZ 100
#endif

#ifndef AP_ADA_TRIP_OVERRUNS
#define AP_ADA_TRIP_OVERRUNS 10
#endif
