#pragma once

#include "AP_Ada_config.h"

#if AP_ADA_ENABLED

#include "AP_Ada_CallTimer.h"

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
  - ADAS: the swashplate mixer against AP_MotorsHeli_Swash, servo by servo
  The UBX and swashplate calls are timed (AP_Ada_CallTimer); a shadow whose
  calls overrun their budget too often is switched off (HZ-19, HZ-20).
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

    // swashplate shadow, called by AP_MotorsHeli_Swash (instance 0 or 1):
    // its configuration whenever it is configured, and every mix with the
    // inputs before collective reversal and its four outputs
    void swash_configure(uint8_t instance, const cw_swash_config_t &cfg);
    void swash_check(uint8_t instance, float roll, float pitch, float collective,
                     const bool enabled[4], const float output[4]);

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
    } _ubx;
    bool _ubx_mismatch_reported;
    // calls of ubx_feed, one per byte
    AP_Ada_CallTimer _ubx_time{AP_ADA_UBX_BUDGET_US, AP_ADA_TRIP_OVERRUNS};

    // swashplate mixers the Ada side accepted (by instance)
    bool _swash_ok[CW_SWASH_MAX_INSTANCES];
    // since the last ADAS log message
    struct {
        uint32_t checks;      // mixes compared
        uint32_t rejects;     // inputs the Ada mixer refused
        uint32_t unsupported; // mixes not compared: configuration not supported
        uint32_t mismatch;    // mixes with a servo or enable flag differing
        float max_err;        // largest servo output difference
        float max_cyclic;     // largest |roll| or |pitch| input, to show coverage
    } _swash;
    bool _swash_mismatch_reported;
    // calls of the Ada mixer
    AP_Ada_CallTimer _swash_time{AP_ADA_SWASH_BUDGET_US, AP_ADA_TRIP_OVERRUNS};

    // count one timed call; report the first overrun and the trip
    void timed(AP_Ada_CallTimer &timer, const char *name, uint32_t dt_us);

    uint32_t _last_log_ms;
};

#endif  // AP_ADA_ENABLED
