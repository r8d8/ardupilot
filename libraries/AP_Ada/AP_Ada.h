#pragma once

#include "AP_Ada_config.h"

#if AP_ADA_ENABLED

#include "AP_Ada_CallTimer.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
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
  - ADAN, ADNT: the navigation filter (CW_Nav.INS) against the AHRS (EKF3)
  - ADAC: the CRSF parser against AP_RCProtocol_CRSF, frame by frame
  - ADAR: the heli rate controllers against AC_PID / AC_HELI_PID, call by
    call
  The UBX, swashplate, navigation and CRSF calls are timed
  (AP_Ada_CallTimer); a
  shadow whose calls overrun their budget too often is switched off (HZ-19,
  HZ-20; the same for the navigation shadow).
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

    // navigation shadow: nav_sample, every loop (a fast task), adds up the
    // IMU's delta angles and velocities; nav_update (AP_ADA_NAV_RATE_HZ)
    // predicts with them, fuses the sensors that have new data and
    // compares with the AHRS
    void nav_sample();
    void nav_update();

    // CRSF shadow, called by AP_RCProtocol_CRSF: every byte it processes,
    // with the time it uses, before it does; every frame it decodes, with
    // its type and length byte, the channels and the link quality after.
    // On ChibiOS these run in the RC input thread (1 KB stack), not the
    // main loop: they count and time there, publish a snapshot once a
    // second, and leave logging and messages to update()
    void crsf_feed(uint8_t byte, uint32_t now_us);
    void crsf_check(uint8_t type, uint8_t length, const uint16_t *channels,
                    int16_t link_quality);

    // rate controller shadow, called by the AC_PIDs set_ada_shadow marks
    // (the heli roll, pitch and yaw rate PIDs): every update with its
    // inputs, configuration and results, and every other state change
    void rate_check(uint8_t axis, const cw_rate_config_t &config, bool supported,
                    float target, float measurement, float dt, bool limit,
                    float pd_scale, float i_scale, float output, float ff,
                    float integrator);
    void rate_leak(uint8_t axis, float rate);
    void rate_reset_i(uint8_t axis);
    void rate_set_integrator(uint8_t axis, float integrator);
    void rate_relax_integrator(uint8_t axis, float integrator, float dt, float tc);
    void rate_reset_filter(uint8_t axis);

    // called by AP_VisualOdom_MAV with every pose, rotated and corrected
    // as the EKF gets it; consumed: the EKF was given it (quality)
    void nav_vio(const Vector3f &pos, float pos_err, uint8_t reset_counter, bool consumed);

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

    // CRSF shadow counts, over one interval of about a second
    struct CrsfCounts {
        uint32_t ap_frames;   // frames AP_RCProtocol_CRSF decoded
        uint32_t ada_frames;  // frames the Ada parser completed
        uint32_t rc;          // RC channel frames compared
        uint32_t match;       // AP frames the Ada decode equals
        uint32_t mismatch;    // AP frames the Ada decode differs from, or misses
        uint32_t subset;      // CRSF v3 subset RC frames, not compared
        uint32_t crc_errors;  // frames the Ada parser dropped on CRC
    };
    // what the RC input thread publishes for the main thread
    struct CrsfSnapshot {
        CrsfCounts counts;
        uint32_t calls, sum_us, max_us, p99_us, overruns;
        bool tripped;
        uint32_t seq;         // advances with every new snapshot
        // events since the main thread last looked
        const char *mismatch; // the first mismatch (reported once)
        uint8_t mismatch_type;
        bool trip;            // the shadow has just switched off
    };
    // RC input thread only
    CrsfCounts _crsf;
    uint32_t _crsf_roll_ms;
    bool _crsf_mismatch_seen;
    // calls of crsf_feed, one per byte; judged by the interval's 99th
    // percentile, as the RC input thread is preempted by higher ones
    AP_Ada_CallTimer _crsf_time{AP_ADA_CRSF_BUDGET_US, AP_ADA_CRSF_TRIP_INTERVALS,
                                AP_Ada_CallTimer::Judge::INTERVAL_P99};
    // shared, under _crsf_sem
    CrsfSnapshot _crsf_pub;
    HAL_Semaphore _crsf_sem;
    // main thread only
    uint32_t _crsf_logged_seq;

    void crsf_publish();
    void crsf_report();

    // rate controller shadow, per axis
    static const uint8_t RATE_AXES = 3;
    struct RateAxis {
        cw_rate_config_t config;   // last sent to the Ada controller
        bool configured;
        bool in_sync;              // false after a refused or unsupported call
        uint16_t settle;           // calls not compared after a resync
    } _rate[RATE_AXES];
    // since the last ADAR log message
    struct {
        uint32_t checks;           // updates compared
        uint32_t rejects;          // inputs the Ada controllers refused
        uint32_t unsupported;      // updates not compared: SMAX or a notch set
        uint32_t settling;         // updates not compared after a resync
        uint32_t mismatch;         // outputs differing beyond the tolerance
        float max_err[RATE_AXES];  // largest P + D + I difference per axis
        float max_ff_err;          // largest feed-forward difference
    } _rate_stats;
    bool _rate_mismatch_reported;
    AP_Ada_CallTimer _rate_time{AP_ADA_RATE_BUDGET_US, AP_ADA_TRIP_OVERRUNS};
    void rate_resync(uint8_t axis, float integrator);

    // navigation shadow
    struct {
        Vector3f dangle;    // rad, since the last predict
        Vector3f dvel;      // m/s
        float dt_angle;     // s
        float dt_vel;
    } _nav_imu;
    struct {
        Vector2f ne;        // m, NED from the origin
        float err;          // m
        uint32_t ms;        // when it came, 0: none yet
        uint8_t reset_counter;
        bool consumed;
        bool fresh;         // not yet fused
    } _nav_vio;
    bool _nav_vio_reset_known;
    uint8_t _nav_vio_reset;
    uint32_t _nav_mag_us;   // last compass sample fused
    uint32_t _nav_baro_ms;  // last baro sample fused
    uint32_t _nav_gps_ms;   // last GPS fix fused
    uint32_t _nav_gaps;     // IMU intervals dropped (longer than a predict step)
    uint32_t _nav_rejects;  // inputs the Ada filter refused
    bool _nav_started;
    // EK3_MAG_CAL, found once: how EKF3 fuses the compass, followed by
    // the filter (heading only or three axes)
    AP_Int8 *_nav_mag_cal;
    bool _nav_mag_cal_looked_up;
    void nav_mag_cal();
    uint32_t _nav_log_ms;
    uint32_t _nav_time_log_ms;
    bool _nav_final_logged;
    // a GPS fix held until fused (one measurement per nav_update)
    struct {
        Vector2f ne;          // m, NED from the origin
        float pos_sigma;      // m
        Vector3f vel;         // m/s, NED
        float vel_sigma;      // m/s
        bool position_pending;
        bool velocity_pending;
    } _nav_gps;
    // the measurement whose turn comes first in the next nav_update:
    // 0 position, 1 velocity, 2 magnetometer, 3 barometer
    static const uint8_t NAV_MEASUREMENTS = 4;
    uint8_t _nav_turn;
    // the parts of nav_update, for ADNP: longest call and calls this interval
    enum NavPart : uint8_t {
        NAV_PREDICT, NAV_POSITION, NAV_VELOCITY, NAV_MAG, NAV_HEIGHT, NAV_START, NAV_PARTS
    };
    struct {
        uint32_t max_us[NAV_PARTS];
        uint16_t n[NAV_PARTS];
        uint8_t max_steps;
    } _nav_parts;
    // calls of nav_update that reach the Ada filter
    AP_Ada_CallTimer _nav_time{AP_ADA_NAV_BUDGET_US, AP_ADA_TRIP_OVERRUNS};

    void nav_start();
    void nav_fuse();
    void nav_take_gps();
    void nav_part(NavPart part, uint32_t t0_us);
    void nav_log();
    void nav_log_stats();

    // count one timed call; report the first overrun and the trip
    void timed(AP_Ada_CallTimer &timer, const char *name, uint32_t dt_us);

    uint32_t _last_log_ms;
};

#endif  // AP_ADA_ENABLED
