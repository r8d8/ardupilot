#pragma once

#include "AP_Ada_config.h"

#if AP_ADA_ENABLED

#include <AP_Common/AP_Common.h>
#include <stdint.h>

/*
  Shim to the Clearwater Ada/SPARK flight software, libcw_fcs.a, built
  from the fcs/ project by Tools/ardupilotwaf/ada.py.

  The Ada code runs in shadow: it computes what the C++ code already
  computes, the two results are compared and logged (ADA message), and
  nothing in flight uses the Ada result.
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

private:
    static AP_Ada *_singleton;

    bool _healthy;

    // since the last ADA log message
    uint32_t _checks;
    uint32_t _rejects;   // inputs the Ada code refused
    float _max_err;      // largest difference from AP_Math, m/s/s

    uint32_t _last_log_ms;
};

#endif  // AP_ADA_ENABLED
