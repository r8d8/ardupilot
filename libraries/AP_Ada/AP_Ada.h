#pragma once

#include "AP_Ada_config.h"

#if AP_ADA_ENABLED

#include <AP_Common/AP_Common.h>
#include <stdint.h>
#include <cw_fcs.h>

/*
  Shim to the Clearwater Ada/SPARK flight software, libcw_fcs.a, built
  from the fcs/ project by Tools/ardupilotwaf/ada.py.

  The Ada code runs in shadow: it computes what the C++ code already
  computes, the two results are compared and logged, and nothing in flight
  uses the Ada result.
  - ADA:  quaternion rotation against AP_Math
  - ADAU: the UBX parser against AP_GPS_UBLOX, NAV-PVT by NAV-PVT
 */
class AP_Ada {
public:
    AP_Ada();

    CLASS_NO_COPY(AP_Ada);

    static AP_Ada *get_singleton() { return _singleton; }

    // initialise the Ada library and run its self-test
    void init();

    // compare the Ada code with AP_Math, called at 10 Hz
    void update();

    bool healthy() const { return _healthy; }

    // UBX shadow, called by AP_GPS_UBLOX: every byte it reads, before it
    // parses it; a reset of its parser; every NAV-PVT it decodes
    void ubx_feed(uint8_t instance, uint8_t byte);
    void ubx_reset(uint8_t instance);
    void ubx_check_nav_pvt(uint8_t instance, const cw_ubx_nav_pvt_t &ap);

private:
    static AP_Ada *_singleton;

    bool _healthy;

    // since the last ADA log message
    uint32_t _checks;
    uint32_t _rejects;   // inputs the Ada code refused
    float _max_err;      // largest difference from AP_Math, m/s/s

    // since the last ADAU log message
    struct {
        uint32_t ap_pvt;      // NAV-PVTs decoded by AP_GPS_UBLOX
        uint32_t ada_pvt;     // NAV-PVTs completed by the Ada parser
        uint32_t match;       // AP NAV-PVTs the Ada decode equals
        uint32_t mismatch;    // AP NAV-PVTs the Ada decode differs from, or misses
        uint32_t ck_errors;   // frames the Ada parser dropped on checksum
        uint32_t bytes;       // bytes fed to the Ada parser
        uint32_t feed_us;     // time in ubx_feed, sum of 1 us tick differences
        uint32_t feed_max_us; // longest single ubx_feed
    } _ubx;
    bool _ubx_mismatch_reported;

    uint32_t _last_log_ms;
};

#endif  // AP_ADA_ENABLED
