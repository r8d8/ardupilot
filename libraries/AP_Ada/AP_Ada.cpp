#include "AP_Ada.h"

#if AP_ADA_ENABLED

#include <cw_fcs.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Common/Location.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Ada *AP_Ada::_singleton;

AP_Ada::AP_Ada()
{
    _singleton = this;
}

// rotate v from body to earth frame by q with the Ada code
static bool ada_rotate(const Quaternion &q, const Vector3f &v, Vector3f &result)
{
    const cw_quat_t cq { double(q.q1), double(q.q2), double(q.q3), double(q.q4) };
    const cw_vec3_t cv { double(v.x), double(v.y), double(v.z) };
    cw_vec3_t r;
    bool ok = false;
    cw_quat_rotate(&cq, &cv, &r, &ok);
    result = Vector3f(float(r.x), float(r.y), float(r.z));
    return ok;
}

void AP_Ada::init()
{
    cw_fcsinit();

    if (cw_fcs_version() != CW_FCS_API_VERSION) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Ada: library API %d, expected %d",
                      cw_fcs_version(), CW_FCS_API_VERSION);
        return;
    }

    // yaw 90 degrees right turns body forward into east
    Quaternion q;
    q.from_euler(0, 0, radians(90));
    Vector3f r;
    if (!ada_rotate(q, Vector3f(1, 0, 0), r) || (r - Vector3f(0, 1, 0)).length() > 1e-5f) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Ada: self-test failed");
        return;
    }

    _healthy = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ada: fcs API %d ok", cw_fcs_version());
}

void AP_Ada::update()
{
    if (!_healthy) {
        return;
    }

    // body acceleration rotated to NED, by AP_Math and by the Ada code
    Quaternion q;
    AP::ahrs().get_quat_body_to_ned(q);
    const Vector3f &accel = AP::ins().get_accel();
    Matrix3f m;
    q.rotation_matrix(m);
    const Vector3f ref = m * accel;

    Vector3f ada;
    if (ada_rotate(q, accel, ada)) {
        _max_err = MAX(_max_err, (ada - ref).length());
    } else {
        _rejects++;
    }
    _checks++;

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_log_ms < 1000) {
        return;
    }
    _last_log_ms = now_ms;

#if HAL_LOGGING_ENABLED
// @LoggerMessage: ADA
// @Description: Ada/SPARK flight software in shadow, compared with the C++ code
// @Field: TimeUS: Time since system startup
// @Field: N: comparisons in this interval
// @Field: Rej: inputs the Ada code rejected
// @Field: Err: largest difference from AP_Math in this interval
    AP::logger().Write("ADA", "TimeUS,N,Rej,Err", "s--o", "F--0", "QIIf",
                       AP_HAL::micros64(), _checks, _rejects, _max_err);

// @LoggerMessage: ADAU
// @Description: Ada/SPARK UBX parser in shadow, compared with AP_GPS_UBLOX
// @Field: TimeUS: Time since system startup
// @Field: PAP: NAV-PVT messages decoded by AP_GPS_UBLOX in this interval
// @Field: PAda: NAV-PVT messages completed by the Ada parser in this interval
// @Field: Mat: AP_GPS_UBLOX NAV-PVTs the Ada decode equals
// @Field: Mis: AP_GPS_UBLOX NAV-PVTs the Ada decode differs from or misses
// @Field: CkE: frames the Ada parser dropped on a checksum error
// @Field: Byt: bytes fed to the Ada parser in this interval
// @Field: FT: time spent in the Ada parser in this interval (sum of 1 us tick differences, so an unbiased estimate even below 1 us per byte)
// @Field: FMx: longest single call of the Ada parser in this interval
// @Field: FP: 99th percentile of the Ada parser's call times in this interval (upper edge of its histogram bin, at most FMx)
// @Field: Ovr: Ada parser calls over the time budget (AP_ADA_UBX_BUDGET_US) in this interval
// @Field: Off: 1 once the UBX shadow is switched off after AP_ADA_TRIP_OVERRUNS overruns since boot
    AP::logger().Write("ADAU", "TimeUS,PAP,PAda,Mat,Mis,CkE,Byt,FT,FMx,FP,Ovr,Off",
                       "s------sss--", "F------FFF--", "QIIIIIIIIIIB",
                       AP_HAL::micros64(), _ubx.ap_pvt, _ubx.ada_pvt,
                       _ubx.match, _ubx.mismatch, _ubx.ck_errors,
                       _ubx_time.calls(), _ubx_time.sum_us(), _ubx_time.max_us(),
                       _ubx_time.percentile_us(99), _ubx_time.overruns(),
                       uint8_t(_ubx_time.tripped()));

// @LoggerMessage: ADAS
// @Description: Ada/SPARK swashplate mixer in shadow, compared with AP_MotorsHeli_Swash
// @Field: TimeUS: Time since system startup
// @Field: N: mixes compared in this interval
// @Field: Rej: inputs the Ada mixer refused
// @Field: Uns: mixes not compared because the configuration is not supported (linearized servos)
// @Field: Mis: mixes where a servo output or enable flag differs
// @Field: Err: largest servo output difference in this interval
// @Field: Cyc: largest roll or pitch input compared in this interval
// @Field: TT: time spent in the Ada mixer in this interval (sum of 1 us tick differences)
// @Field: TMx: longest single call of the Ada mixer in this interval
// @Field: TP: 99th percentile of the Ada mixer's call times in this interval (upper edge of its histogram bin, at most TMx)
// @Field: Ovr: Ada mixer calls over the time budget (AP_ADA_SWASH_BUDGET_US) in this interval
// @Field: Off: 1 once the swashplate shadow is switched off after AP_ADA_TRIP_OVERRUNS overruns since boot
    AP::logger().Write("ADAS", "TimeUS,N,Rej,Uns,Mis,Err,Cyc,TT,TMx,TP,Ovr,Off",
                       "s------sss--", "F------FFF--", "QIIIIffIIIIB",
                       AP_HAL::micros64(), _swash.checks, _swash.rejects,
                       _swash.unsupported, _swash.mismatch, _swash.max_err,
                       _swash.max_cyclic, _swash_time.sum_us(), _swash_time.max_us(),
                       _swash_time.percentile_us(99), _swash_time.overruns(),
                       uint8_t(_swash_time.tripped()));

// @LoggerMessage: ADAR
// @Description: Ada/SPARK heli rate controllers in shadow, compared with AC_PID / AC_HELI_PID update by update
// @Field: TimeUS: Time since system startup
// @Field: N: updates compared in this interval (all three axes)
// @Field: Rej: inputs the Ada controllers refused
// @Field: Uns: updates not compared: SMAX or a notch filter set
// @Field: Set: updates not compared while the Ada controller settles after being brought back in step
// @Field: Mis: updates whose P + D + I or feed-forward differs by more than 1e-4
// @Field: ER: largest roll P + D + I difference in this interval
// @Field: EP: largest pitch P + D + I difference in this interval
// @Field: EY: largest yaw P + D + I difference in this interval
// @Field: EF: largest feed-forward difference in this interval
// @Field: TT: time spent in the Ada controllers in this interval
// @Field: TMx: longest single update in this interval
// @Field: TP: 99th percentile of the update times in this interval
// @Field: Ovr: updates over the time budget (AP_ADA_RATE_BUDGET_US) in this interval
// @Field: Off: 1 once the rate shadow is switched off after AP_ADA_TRIP_OVERRUNS overruns since boot
    AP::logger().Write("ADAR", "TimeUS,N,Rej,Uns,Set,Mis,ER,EP,EY,EF,TT,TMx,TP,Ovr,Off",
                       "s---------sss--", "F---------FFF--", "QIIIIIffffIIIIB",
                       AP_HAL::micros64(), _rate_stats.checks, _rate_stats.rejects,
                       _rate_stats.unsupported, _rate_stats.settling, _rate_stats.mismatch,
                       _rate_stats.max_err[0], _rate_stats.max_err[1], _rate_stats.max_err[2],
                       _rate_stats.max_ff_err, _rate_time.sum_us(), _rate_time.max_us(),
                       _rate_time.percentile_us(99), _rate_time.overruns(),
                       uint8_t(_rate_time.tripped()));
#endif

    crsf_report();

    _checks = 0;
    _rejects = 0;
    _max_err = 0;
    _ubx = {};
    _swash = {};
    _ubx_time.new_interval();
    _swash_time.new_interval();
    _rate_stats = {};
    _rate_time.new_interval();
}

// the CRSF snapshot from the RC input thread: log it if new, send what it
// reports (main thread)
void AP_Ada::crsf_report()
{
    CrsfSnapshot s;
    {
        WITH_SEMAPHORE(_crsf_sem);
        s = _crsf_pub;
        _crsf_pub.mismatch = nullptr;
        _crsf_pub.trip = false;
    }
    if (s.mismatch != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Ada: CRSF frame 0x%02x differs: %s",
                      unsigned(s.mismatch_type), s.mismatch);
    }
    if (s.trip) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Ada: CRSF shadow off after %u s with p99 over %u us",
                      unsigned(AP_ADA_CRSF_TRIP_INTERVALS), unsigned(AP_ADA_CRSF_BUDGET_US));
    }
    if (s.seq == _crsf_logged_seq) {
        return;
    }
    _crsf_logged_seq = s.seq;

#if HAL_LOGGING_ENABLED
// @LoggerMessage: ADAC
// @Description: Ada/SPARK CRSF parser in shadow, compared with AP_RCProtocol_CRSF (about once a second while CRSF bytes come)
// @Field: TimeUS: Time since system startup
// @Field: FAP: frames decoded by AP_RCProtocol_CRSF in this interval
// @Field: FAda: frames completed by the Ada parser in this interval
// @Field: RC: RC channel frames compared, channel by channel
// @Field: Mat: AP_RCProtocol_CRSF frames the Ada decode equals
// @Field: Mis: AP_RCProtocol_CRSF frames the Ada decode differs from or misses
// @Field: Sub: CRSF v3 subset RC frames (not compared)
// @Field: CRC: frames the Ada parser dropped on a CRC error
// @Field: Byt: bytes fed to the Ada parser in this interval
// @Field: FT: time spent in the Ada parser in this interval (sum of 1 us tick differences)
// @Field: FMx: longest single call of the Ada parser in this interval
// @Field: FP: 99th percentile of the Ada parser's call times in this interval (upper edge of its histogram bin, at most FMx)
// @Field: Ovr: Ada parser calls over the time budget (AP_ADA_CRSF_BUDGET_US) in this interval, counted only: the RC input thread is preempted in the middle of calls
// @Field: Off: 1 once the CRSF shadow is switched off: FP over the budget AP_ADA_CRSF_TRIP_INTERVALS intervals in a row
    const CrsfCounts &c = s.counts;
    AP::logger().Write("ADAC", "TimeUS,FAP,FAda,RC,Mat,Mis,Sub,CRC,Byt,FT,FMx,FP,Ovr,Off",
                       "s--------sss--", "F--------FFF--", "QIIIIIIIIIIIIB",
                       AP_HAL::micros64(), c.ap_frames, c.ada_frames, c.rc,
                       c.match, c.mismatch, c.subset, c.crc_errors,
                       s.calls, s.sum_us, s.max_us, s.p99_us, s.overruns,
                       uint8_t(s.tripped));
#endif
}

void AP_Ada::timed(AP_Ada_CallTimer &timer, const char *name, uint32_t dt_us)
{
    const bool was_tripped = timer.tripped();
    if (!timer.add(dt_us)) {
        return;
    }
    if (timer.overruns_total() == 1) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Ada: %s call took %u us, budget %u us",
                      name, unsigned(dt_us), unsigned(timer.budget_us()));
    }
    if (timer.tripped() && !was_tripped) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Ada: %s shadow off after %u calls over %u us",
                      name, unsigned(timer.overruns_total()), unsigned(timer.budget_us()));
    }
}

void AP_Ada::ubx_feed(uint8_t instance, uint8_t byte)
{
    if (!_healthy || _ubx_time.tripped()) {
        return;
    }
    // timed for HZ-19: what the shadow adds to every GPS byte
    const uint32_t t0 = AP_HAL::micros();
    uint8_t status = CW_UBX_STATUS_NONE;
    cw_ubx_feed(instance, byte, &status);
    if (status == CW_UBX_STATUS_FRAME) {
        cw_ubx_nav_pvt_t pvt;
        bool ok = false;
        cw_ubx_nav_pvt(instance, &pvt, &ok);
        if (ok) {
            _ubx.ada_pvt++;
        }
    } else if (status == CW_UBX_STATUS_CHECKSUM) {
        _ubx.ck_errors++;
    }
    timed(_ubx_time, "UBX", AP_HAL::micros() - t0);
}

void AP_Ada::swash_configure(uint8_t instance, const cw_swash_config_t &cfg)
{
    if (!_healthy || instance >= CW_SWASH_MAX_INSTANCES) {
        return;
    }
    bool ok = false;
    cw_swash_configure(instance, &cfg, &ok);
    _swash_ok[instance] = ok;
}

// ArduPilot's outputs are floats, the Ada mixer computes in double
static const float SWASH_TOLERANCE = 1.0e-5f;

void AP_Ada::swash_check(uint8_t instance, float roll, float pitch, float collective,
                         const bool enabled[4], const float output[4])
{
    if (!_healthy || instance >= CW_SWASH_MAX_INSTANCES) {
        return;
    }
    if (_swash_time.tripped()) {
        return;
    }
    if (!_swash_ok[instance]) {
        _swash.unsupported++;
        return;
    }
    cw_swash_out_t ada;
    bool ok = false;
    // timed for HZ-20: what the shadow adds to every mix
    const uint32_t t0 = AP_HAL::micros();
    cw_swash_calculate(instance, roll, pitch, collective, &ada, &ok);
    timed(_swash_time, "swash", AP_HAL::micros() - t0);
    if (!ok) {
        _swash.rejects++;
        return;
    }
    _swash.checks++;
    _swash.max_cyclic = MAX(_swash.max_cyclic, MAX(fabsf(roll), fabsf(pitch)));

    bool differs = false;
    uint8_t first = 0;
    float err = 0;
    for (uint8_t i = 0; i < 4; i++) {
        const bool ada_enabled = (ada.enabled & (1U << i)) != 0;
        if (ada_enabled != enabled[i]) {
            differs = true;
            first = i;
            break;
        }
        if (enabled[i]) {
            const float e = fabsf(float(ada.servo[i]) - output[i]);
            if (e > err) {
                err = e;
                first = i;
            }
        }
    }
    _swash.max_err = MAX(_swash.max_err, err);
    differs = differs || err > SWASH_TOLERANCE;
    if (!differs) {
        return;
    }
    _swash.mismatch++;
    if (!_swash_mismatch_reported) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Ada: swash %u servo %u differs: %.6f vs %.6f",
                      unsigned(instance), unsigned(first + 1),
                      double(ada.servo[first]), double(output[first]));
        _swash_mismatch_reported = true;
    }
}

void AP_Ada::ubx_reset(uint8_t instance)
{
    if (_healthy && !_ubx_time.tripped()) {
        cw_ubx_reset(instance);
    }
}

// the Ada parser saw the same bytes first, so its frame must be this NAV-PVT
void AP_Ada::ubx_check_nav_pvt(uint8_t instance, const cw_ubx_nav_pvt_t &ap)
{
    if (!_healthy || _ubx_time.tripped()) {
        return;
    }
    _ubx.ap_pvt++;

    cw_ubx_nav_pvt_t ada;
    bool ok = false;
    cw_ubx_nav_pvt(instance, &ada, &ok);

    const char *field = nullptr;
    if (!ok) {
        field = "missing";
    }
#define CHECK_FIELD(f) else if (ada.f != ap.f) { field = #f; }
    CHECK_FIELD(itow)
    CHECK_FIELD(lon)
    CHECK_FIELD(lat)
    CHECK_FIELD(height)
    CHECK_FIELD(h_msl)
    CHECK_FIELD(h_acc)
    CHECK_FIELD(v_acc)
    CHECK_FIELD(vel_n)
    CHECK_FIELD(vel_e)
    CHECK_FIELD(vel_d)
    CHECK_FIELD(g_speed)
    CHECK_FIELD(head_mot)
    CHECK_FIELD(s_acc)
    CHECK_FIELD(head_acc)
    CHECK_FIELD(p_dop)
    CHECK_FIELD(valid)
    CHECK_FIELD(fix_type)
    CHECK_FIELD(flags)
    CHECK_FIELD(num_sv)
#undef CHECK_FIELD

    if (field == nullptr) {
        _ubx.match++;
        return;
    }
    _ubx.mismatch++;
    if (!_ubx_mismatch_reported) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Ada: UBX %u NAV-PVT differs: %s",
                      unsigned(instance), field);
        _ubx_mismatch_reported = true;
    }
}

// ---------------------------------------------------------------------------
// CRSF shadow: the Ada parser sees every byte first, with the driver's own
// time, so when the driver decodes a frame the Ada parser has just
// completed the same one

void AP_Ada::crsf_feed(uint8_t byte, uint32_t now_us)
{
    if (!_healthy) {
        return;
    }
    if (!_crsf_time.tripped()) {
        // timed: what the shadow adds to every RC byte
        const uint32_t t0 = AP_HAL::micros();
        uint8_t status = CW_CRSF_STATUS_NONE;
        cw_crsf_feed(byte, now_us, &status);
        if (status == CW_CRSF_STATUS_FRAME) {
            _crsf.ada_frames++;
        } else if (status == CW_CRSF_STATUS_CRC) {
            _crsf.crc_errors++;
        }
        // a call over the budget is only counted: this thread is preempted
        // in the middle of calls, the interval decides (crsf_publish)
        UNUSED_RESULT(_crsf_time.add(AP_HAL::micros() - t0));
    }
    crsf_publish();
}

// once a second, in the RC input thread: the interval's counts and times
// to the snapshot, then the interval judged (INTERVAL_P99) and a new one
void AP_Ada::crsf_publish()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _crsf_roll_ms < 1000) {
        return;
    }
    _crsf_roll_ms = now_ms;
    {
        WITH_SEMAPHORE(_crsf_sem);
        _crsf_pub.counts = _crsf;
        _crsf_pub.calls = _crsf_time.calls();
        _crsf_pub.sum_us = _crsf_time.sum_us();
        _crsf_pub.max_us = _crsf_time.max_us();
        _crsf_pub.p99_us = _crsf_time.percentile_us(99);
        _crsf_pub.overruns = _crsf_time.overruns();
        _crsf_pub.tripped = _crsf_time.tripped();
        _crsf_pub.seq++;
    }
    _crsf = {};
    const bool was_tripped = _crsf_time.tripped();
    _crsf_time.new_interval();
    if (_crsf_time.tripped() && !was_tripped) {
        WITH_SEMAPHORE(_crsf_sem);
        _crsf_pub.trip = true;
    }
}

// CRSF frame types (AP_CRSF_Protocol)
static const uint8_t CRSF_LINK_STATISTICS = 0x14;
static const uint8_t CRSF_RC_CHANNELS = 0x16;
static const uint8_t CRSF_RC_SUBSET = 0x17;

void AP_Ada::crsf_check(uint8_t type, uint8_t length, const uint16_t *channels,
                        int16_t link_quality)
{
    // the Ada parser saw the same bytes first (crsf_feed, same thread)
    if (!_healthy || _crsf_time.tripped()) {
        return;
    }
    _crsf.ap_frames++;

    uint8_t ada_type = 0;
    uint8_t ada_payload = 0;
    bool ok = false;
    cw_crsf_frame(&ada_type, &ada_payload, &ok);
    const char *what = nullptr;
    if (!ok) {
        what = "missing";
    } else if (ada_type != type || ada_payload + 2U != length) {
        what = "type or length";
    } else if (type == CRSF_RC_CHANNELS) {
        _crsf.rc++;
        cw_crsf_rc_t rc;
        cw_crsf_rc(&rc, &ok);
        if (!ok) {
            what = "RC frame length";
        } else {
            for (uint8_t i = 0; i < ARRAY_SIZE(rc.channels); i++) {
                if (rc.channels[i] != channels[i]) {
                    what = "RC channel";
                    break;
                }
            }
        }
    } else if (type == CRSF_LINK_STATISTICS) {
        cw_crsf_link_t link;
        cw_crsf_link(&link, &ok);
        if (!ok || link.uplink_lq != link_quality) {
            what = "link quality";
        }
    } else if (type == CRSF_RC_SUBSET) {
        // framing compared; its channels are not decoded on the Ada side
        _crsf.subset++;
        return;
    }

    if (what == nullptr) {
        _crsf.match++;
        return;
    }
    _crsf.mismatch++;
    if (!_crsf_mismatch_seen) {
        // reported by the main thread: no GCS text on this stack
        WITH_SEMAPHORE(_crsf_sem);
        _crsf_pub.mismatch = what;
        _crsf_pub.mismatch_type = type;
        _crsf_mismatch_seen = true;
    }
}

// ---------------------------------------------------------------------------
// rate controller shadow: the heli roll, pitch and yaw rate PIDs (AC_PID with
// AC_HELI_PID) mirrored, state change for state change, by CW_PID

// the outputs are of order 1; AC_PID computes in float, as CW_PID does,
// but the compilers may fuse multiply-adds differently
static const float RATE_TOLERANCE = 1.0e-4f;
// calls not compared after the Ada controller was brought back in step,
// while its filters settle (2 s at 400 Hz)
static const uint16_t RATE_SETTLE_CALLS = 800;

// bring the Ada controller back in step: its filters restart from the next
// input and its integrator is ArduPilot's; compared again once settled
void AP_Ada::rate_resync(uint8_t axis, float integrator)
{
    bool ok = false;
    cw_rate_reset_filter(axis);
    cw_rate_set_integrator(axis, integrator, &ok);
    _rate[axis].in_sync = ok;
    _rate[axis].settle = RATE_SETTLE_CALLS;
}

void AP_Ada::rate_check(uint8_t axis, const cw_rate_config_t &config, bool supported,
                        float target, float measurement, float dt, bool limit,
                        float pd_scale, float i_scale, float output, float ff,
                        float integrator)
{
    if (!_healthy || axis >= RATE_AXES || _rate_time.tripped()) {
        return;
    }
    RateAxis &r = _rate[axis];
    if (!supported) {
        // no Ada counterpart for SMAX or a notch: out of step until it goes
        _rate_stats.unsupported++;
        r.in_sync = false;
        return;
    }
    if (!r.configured || memcmp(&config, &r.config, sizeof(config)) != 0) {
        bool ok = false;
        cw_rate_configure(axis, &config, &ok);
        r.configured = ok;
        r.config = config;
        if (!ok) {
            _rate_stats.rejects++;
            r.in_sync = false;
            return;
        }
    }
    if (!r.in_sync) {
        // ArduPilot has just updated: start the Ada controller from its
        // integrator, its filters from the next call
        rate_resync(axis, integrator);
        return;
    }

    // timed: what the shadow adds to every rate controller update
    const uint32_t t0 = AP_HAL::micros();
    float ada_out = 0;
    float ada_ff = 0;
    bool ok = false;
    cw_rate_update(axis, target, measurement, dt, limit ? 1 : 0, pd_scale, i_scale,
                   &ada_out, &ada_ff, &ok);
    timed(_rate_time, "rate", AP_HAL::micros() - t0);
    if (!ok) {
        _rate_stats.rejects++;
        r.in_sync = false;
        return;
    }
    if (r.settle > 0) {
        r.settle--;
        _rate_stats.settling++;
        return;
    }
    _rate_stats.checks++;
    const float err = fabsf(ada_out - output);
    const float ff_err = fabsf(ada_ff - ff);
    _rate_stats.max_err[axis] = MAX(_rate_stats.max_err[axis], err);
    _rate_stats.max_ff_err = MAX(_rate_stats.max_ff_err, ff_err);
    if (err <= RATE_TOLERANCE && ff_err <= RATE_TOLERANCE) {
        return;
    }
    _rate_stats.mismatch++;
    if (!_rate_mismatch_reported) {
        static const char *names[RATE_AXES] { "roll", "pitch", "yaw" };
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Ada: rate %s differs: %.5f vs %.5f (ff %.5f vs %.5f)",
                      names[axis], double(ada_out), double(output), double(ada_ff), double(ff));
        _rate_mismatch_reported = true;
    }
}

void AP_Ada::rate_leak(uint8_t axis, float rate)
{
    if (!_healthy || axis >= RATE_AXES || _rate_time.tripped() || !_rate[axis].in_sync) {
        return;
    }
    bool ok = false;
    cw_rate_leak(axis, rate, &ok);
    if (!ok) {
        _rate_stats.rejects++;
        _rate[axis].in_sync = false;
    }
}

void AP_Ada::rate_reset_i(uint8_t axis)
{
    if (_healthy && axis < RATE_AXES && !_rate_time.tripped()) {
        cw_rate_reset_i(axis);
    }
}

void AP_Ada::rate_set_integrator(uint8_t axis, float integrator)
{
    if (!_healthy || axis >= RATE_AXES || _rate_time.tripped()) {
        return;
    }
    bool ok = false;
    cw_rate_set_integrator(axis, integrator, &ok);
    if (!ok) {
        _rate_stats.rejects++;
        _rate[axis].in_sync = false;
    }
}

void AP_Ada::rate_relax_integrator(uint8_t axis, float integrator, float dt, float tc)
{
    if (!_healthy || axis >= RATE_AXES || _rate_time.tripped()) {
        return;
    }
    bool ok = false;
    cw_rate_relax_integrator(axis, integrator, dt, tc, &ok);
    if (!ok) {
        _rate_stats.rejects++;
        _rate[axis].in_sync = false;
    }
}

void AP_Ada::rate_reset_filter(uint8_t axis)
{
    if (_healthy && axis < RATE_AXES && !_rate_time.tripped()) {
        cw_rate_reset_filter(axis);
    }
}

// ---------------------------------------------------------------------------
// navigation shadow: CW_Nav.INS against the AHRS (EKF3)

static cw_vec3_t c_vec(const Vector3f &v)
{
    return cw_vec3_t { double(v.x), double(v.y), double(v.z) };
}

// a VIO pose this recent makes VIO the position source, otherwise GPS
static const uint32_t NAV_VIO_TIMEOUT_MS = 2000;
// the magnetometer and barometer at most this often: the Goblin logs the
// filter was validated on have them at 10 Hz
static const uint32_t NAV_SLOW_SENSOR_MS = 50;
// the Ada filter's longest predict step (CW_Nav.Step), s; a longer IMU
// interval is split into up to NAV_MAX_STEPS steps, beyond that dropped
static const float NAV_MAX_STEP_S = 0.05f;
static const uint8_t NAV_MAX_STEPS = 4;
// noise floors as EKF3's defaults: EK3_POSNE_M_NSE, EK3_VELNE_M_NSE
static const float NAV_GPS_POS_MIN_M = 0.5f;
static const float NAV_GPS_VEL_MIN_MS = 0.3f;

void AP_Ada::nav_sample()
{
    if (!_healthy || _nav_time.tripped()) {
        return;
    }
    const AP_AHRS &ahrs = AP::ahrs();
    const AP_InertialSensor &ins = AP::ins();
    Vector3f da, dv;
    float dta, dtv;
    if (!ins.get_delta_angle(ahrs.get_primary_gyro_index(), da, dta) ||
        !ins.get_delta_velocity(ahrs.get_primary_accel_index(), dv, dtv)) {
        return;
    }
    _nav_imu.dangle += da;
    _nav_imu.dvel += dv;
    _nav_imu.dt_angle += dta;
    _nav_imu.dt_vel += dtv;
}

void AP_Ada::nav_vio(const Vector3f &pos, float pos_err, uint8_t reset_counter, bool consumed)
{
    _nav_vio.ne = pos.xy();
    _nav_vio.err = pos_err;
    _nav_vio.reset_counter = reset_counter;
    _nav_vio.consumed = consumed;
    _nav_vio.ms = AP_HAL::millis();
    _nav_vio.fresh = true;
}

void AP_Ada::nav_update()
{
    if (!_healthy) {
        return;
    }
    if (_nav_time.tripped()) {
        // the last interval's records, with Off set, once
        if (!_nav_final_logged) {
            _nav_final_logged = true;
            nav_log_stats();
        }
        return;
    }
    // timed for the main loop: what the shadow adds at AP_ADA_NAV_RATE_HZ
    const uint32_t t0 = AP_HAL::micros();
    // disarmed is on the ground, as EKF3 takes it (AP_DAL): the filter
    // then holds the accelerometer bias of the axes not near vertical
    cw_nav_set_armed(hal.util->get_soft_armed() ? 1 : 0);
    if (_nav_started) {
        nav_fuse();
    } else {
        nav_start();
    }
    timed(_nav_time, "nav", AP_HAL::micros() - t0);
    nav_log();
}

// one Ada call of a part of nav_update, for ADNP
void AP_Ada::nav_part(NavPart part, uint32_t t0_us)
{
    const uint32_t dt = AP_HAL::micros() - t0_us;
    _nav_parts.max_us[part] = MAX(_nav_parts.max_us[part], dt);
    _nav_parts.n[part]++;
}

// start at rest once the EKF has an origin and there is a position
void AP_Ada::nav_start()
{
    const auto imu = _nav_imu;
    _nav_imu = {};
    if (imu.dt_vel <= 0) {
        return;
    }
    const AP_AHRS &ahrs = AP::ahrs();
    const Compass &compass = AP::compass();
    const AP_Baro &baro = AP::baro();
    const AP_GPS &gps = AP::gps();
    Location origin;
    if (!ahrs.get_origin(origin) || !compass.healthy() || !baro.healthy()) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    const bool vio = _nav_vio.ms != 0 && now_ms - _nav_vio.ms < NAV_VIO_TIMEOUT_MS;
    Vector2f ne;
    if (vio) {
        if (!_nav_vio.consumed) {
            return;
        }
        ne = _nav_vio.ne;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        ne = origin.get_distance_NE(gps.location());
    } else {
        return;
    }

    const cw_vec3_t accel = c_vec(imu.dvel / imu.dt_vel);
    const cw_vec3_t mag = c_vec(compass.get_field());
    const cw_vec3_t pos { double(ne.x), double(ne.y), -double(baro.get_altitude()) };
    bool ok = false;
    const uint32_t t0 = AP_HAL::micros();
    cw_nav_start(&accel, &mag, double(compass.get_declination()), &pos, &ok);
    nav_part(NAV_START, t0);
    if (!ok) {
        _nav_rejects++;
        return;
    }
    _nav_started = true;
    _nav_mag_us = compass.last_update_usec();
    _nav_baro_ms = baro.get_last_update();
    _nav_gps_ms = gps.last_fix_time_ms();
    _nav_gps = {};
    _nav_vio.fresh = false;
    _nav_vio_reset_known = vio;
    _nav_vio_reset = _nav_vio.reset_counter;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ada: nav started (%s)", vio ? "VIO" : "GPS");
}

void AP_Ada::nav_fuse()
{
    bool ok = false;

    // predict over the IMU's interval since the last call, at its mean
    // rate and specific force
    const auto imu = _nav_imu;
    _nav_imu = {};
    if (imu.dt_angle > 0 && imu.dt_vel > 0) {
        const uint8_t steps = uint8_t(ceilf(imu.dt_angle / NAV_MAX_STEP_S));
        if (steps <= NAV_MAX_STEPS) {
            const cw_vec3_t gyro = c_vec(imu.dangle / imu.dt_angle);
            const cw_vec3_t accel = c_vec(imu.dvel / imu.dt_vel);
            const double dt = double(imu.dt_angle) / steps;
            _nav_parts.max_steps = MAX(_nav_parts.max_steps, steps);
            const uint32_t t0 = AP_HAL::micros();
            for (uint8_t i = 0; i < steps; i++) {
                cw_nav_predict(&gyro, &accel, dt, &ok);
                if (!ok) {
                    _nav_rejects++;
                    break;
                }
            }
            nav_part(NAV_PREDICT, t0);
        } else {
            _nav_gaps++;
        }
    }

    // what has new data
    const uint32_t now_ms = AP_HAL::millis();
    const bool vio = _nav_vio.ms != 0 && now_ms - _nav_vio.ms < NAV_VIO_TIMEOUT_MS;
    if (!vio) {
        nav_take_gps();
    }
    const Compass &compass = AP::compass();
    const AP_Baro &baro = AP::baro();
    const bool ready[NAV_MEASUREMENTS] = {
        vio ? _nav_vio.fresh : _nav_gps.position_pending,
        !vio && _nav_gps.velocity_pending,
        compass.healthy() && compass.last_update_usec() - _nav_mag_us >= NAV_SLOW_SENSOR_MS * 1000U,
        baro.healthy() && baro.get_last_update() - _nav_baro_ms >= NAV_SLOW_SENSOR_MS,
    };

    // then at most one measurement, so that a call is one predict and one
    // update; the others wait for the next calls (10 ms each), in turns
    for (uint8_t k = 0; k < NAV_MEASUREMENTS; k++) {
        const uint8_t m = (_nav_turn + k) % NAV_MEASUREMENTS;
        if (!ready[m]) {
            continue;
        }
        _nav_turn = (m + 1) % NAV_MEASUREMENTS;
        const uint32_t t0 = AP_HAL::micros();
        switch (m) {
        case 0:
            if (vio) {
                // as the EKF: only the poses it was given; a new reset
                // counter is the VIO's reset, taken as is
                _nav_vio.fresh = false;
                if (_nav_vio.consumed) {
                    const bool reset = _nav_vio_reset_known && _nav_vio.reset_counter != _nav_vio_reset;
                    cw_nav_fuse_position(double(_nav_vio.ne.x), double(_nav_vio.ne.y),
                                         double(_nav_vio.err), reset ? 1 : 0, &ok);
                    _nav_rejects += ok ? 0 : 1;
                }
                _nav_vio_reset = _nav_vio.reset_counter;
                _nav_vio_reset_known = true;
            } else {
                _nav_gps.position_pending = false;
                cw_nav_fuse_position(double(_nav_gps.ne.x), double(_nav_gps.ne.y),
                                     double(_nav_gps.pos_sigma), 0, &ok);
                _nav_rejects += ok ? 0 : 1;
            }
            nav_part(NAV_POSITION, t0);
            break;
        case 1: {
            _nav_gps.velocity_pending = false;
            const cw_vec3_t vel = c_vec(_nav_gps.vel);
            cw_nav_fuse_velocity(&vel, double(_nav_gps.vel_sigma), &ok);
            _nav_rejects += ok ? 0 : 1;
            nav_part(NAV_VELOCITY, t0);
            break;
        }
        case 2: {
            _nav_mag_us = compass.last_update_usec();
            const cw_vec3_t mag = c_vec(compass.get_field());
            cw_nav_fuse_mag(&mag, double(compass.get_declination()), &ok);
            _nav_rejects += ok ? 0 : 1;
            nav_part(NAV_MAG, t0);
            break;
        }
        default:
            _nav_baro_ms = baro.get_last_update();
            cw_nav_fuse_height(double(baro.get_altitude()), &ok);
            _nav_rejects += ok ? 0 : 1;
            nav_part(NAV_HEIGHT, t0);
            break;
        }
        break;
    }
}

// a new GPS fix: its position and velocity, held until fused
void AP_Ada::nav_take_gps()
{
    const AP_GPS &gps = AP::gps();
    const uint32_t fix_ms = gps.last_fix_time_ms();
    Location origin;
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D || fix_ms == _nav_gps_ms ||
        !AP::ahrs().get_origin(origin)) {
        return;
    }
    _nav_gps_ms = fix_ms;
    float hacc = NAV_GPS_POS_MIN_M;
    float sacc = NAV_GPS_VEL_MIN_MS;
    UNUSED_RESULT(gps.horizontal_accuracy(hacc));
    UNUSED_RESULT(gps.speed_accuracy(sacc));
    _nav_gps.ne = origin.get_distance_NE(gps.location());
    _nav_gps.pos_sigma = MAX(hacc, NAV_GPS_POS_MIN_M);
    _nav_gps.vel = gps.velocity();
    _nav_gps.vel_sigma = MAX(sacc, NAV_GPS_VEL_MIN_MS);
    _nav_gps.position_pending = true;
    _nav_gps.velocity_pending = true;
}

void AP_Ada::nav_log()
{
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (_nav_started && now_ms - _nav_log_ms >= 100) {
        _nav_log_ms = now_ms;
        cw_nav_state_t st;
        cw_nav_state(&st);
        const AP_AHRS &ahrs = AP::ahrs();

        // in the AHRS's (vehicle body) frame: AP_AHRS rotates EKF3's
        // attitude by -AHRS_TRIM
        Quaternion q(float(st.q.w), float(st.q.x), float(st.q.y), float(st.q.z));
        q.rotate(-ahrs.get_trim());
        float roll, pitch, yaw;
        q.to_euler(roll, pitch, yaw);
        const Vector3f vel(float(st.vel.x), float(st.vel.y), float(st.vel.z));
        const Vector3f pos(float(st.pos.x), float(st.pos.y), float(st.pos.z));

        // differences from the AHRS, NaN where it has none
        float d_roll = NAN, d_pitch = NAN, d_yaw = NAN;
        float d_vel = NAN, d_horiz = NAN, d_down = NAN;
        Quaternion qa;
        if (ahrs.get_quaternion(qa)) {
            float r, p, y;
            qa.to_euler(r, p, y);
            d_roll = degrees(wrap_PI(roll - r));
            d_pitch = degrees(wrap_PI(pitch - p));
            d_yaw = degrees(wrap_PI(yaw - y));
        }
        Vector3f va;
        if (ahrs.get_velocity_NED(va)) {
            d_vel = (vel - va).length();
        }
        Vector3p pa;
        if (ahrs.get_relative_position_NED_origin(pa)) {
            const Vector3f d = pos - pa.tofloat();
            d_horiz = d.xy().length();
            d_down = d.z;
        }

// @LoggerMessage: ADAN
// @Description: Ada/SPARK navigation filter (CW_Nav.INS) in shadow, compared with the AHRS (EKF3)
// @Field: TimeUS: Time since system startup
// @Field: Roll: Ada filter roll, vehicle body (AHRS_TRIM applied as the AHRS does)
// @Field: Pitch: Ada filter pitch
// @Field: Yaw: Ada filter yaw
// @Field: VN: Ada filter velocity north
// @Field: VE: Ada filter velocity east
// @Field: VD: Ada filter velocity down
// @Field: PN: Ada filter position north of the EKF origin
// @Field: PE: Ada filter position east of the EKF origin
// @Field: PD: Ada filter position down from the EKF origin
// @Field: dR: Ada roll minus AHRS roll
// @Field: dP: Ada pitch minus AHRS pitch
// @Field: dY: Ada yaw minus AHRS yaw
// @Field: dV: length of the velocity difference from the AHRS
// @Field: dH: horizontal distance from the AHRS position
// @Field: dD: Ada down position minus AHRS down position
        AP::logger().Write("ADAN", "TimeUS,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,dR,dP,dY,dV,dH,dD",
                           "sdddnnnmmmdddnmm", "F000000000000000", "Qfffffffffffffff",
                           AP_HAL::micros64(), degrees(roll), degrees(pitch), wrap_360(degrees(yaw)),
                           vel.x, vel.y, vel.z, pos.x, pos.y, pos.z,
                           d_roll, d_pitch, d_yaw, d_vel, d_horiz, d_down);
    }

    if (now_ms - _nav_time_log_ms >= 1000) {
        _nav_time_log_ms = now_ms;
        nav_log_stats();
    }
#endif
}

// ADNT and ADNP for the interval just ended, and a new interval
void AP_Ada::nav_log_stats()
{
#if HAL_LOGGING_ENABLED
    cw_nav_state_t st;
    cw_nav_state(&st);
    const uint32_t *c = st.counters;

// @LoggerMessage: ADNT
// @Description: Ada/SPARK navigation filter in shadow: measurements used and rejected (since it started), time spent
// @Field: TimeUS: Time since system startup
// @Field: PU: positions fused
// @Field: PR: positions rejected by the innovation gate
// @Field: VU: velocities fused (GPS)
// @Field: VR: velocities rejected
// @Field: HU: barometer heights fused
// @Field: HR: barometer heights rejected
// @Field: MU: magnetometer samples fused
// @Field: MR: magnetometer samples rejected
// @Field: Rst: position source resets taken
// @Field: Gap: IMU intervals dropped as too long to predict over
// @Field: Rej: inputs the Ada filter refused (NaN, out of range, not started)
// @Field: TT: time spent in the Ada filter in this interval
// @Field: TMx: longest single nav_update in this interval
// @Field: Ovr: nav_update calls over the time budget (AP_ADA_NAV_BUDGET_US) in this interval
// @Field: Off: 1 once the navigation shadow is switched off after AP_ADA_TRIP_OVERRUNS overruns since boot
    AP::logger().Write("ADNT", "TimeUS,PU,PR,VU,VR,HU,HR,MU,MR,Rst,Gap,Rej,TT,TMx,Ovr,Off",
                       "s-----------ss--", "F-----------FF--", "QIIIIIIIIIIIIIIB",
                       AP_HAL::micros64(),
                       c[CW_NAV_POSITION_USED], c[CW_NAV_POSITION_REJECTED],
                       c[CW_NAV_VELOCITY_USED], c[CW_NAV_VELOCITY_REJECTED],
                       c[CW_NAV_HEIGHT_USED], c[CW_NAV_HEIGHT_REJECTED],
                       c[CW_NAV_MAG_USED], c[CW_NAV_MAG_REJECTED],
                       c[CW_NAV_POSITION_RESETS], _nav_gaps, _nav_rejects,
                       _nav_time.sum_us(), _nav_time.max_us(), _nav_time.overruns(),
                       uint8_t(_nav_time.tripped()));

// @LoggerMessage: ADNP
// @Description: Ada/SPARK navigation filter in shadow: the parts of nav_update in this interval, longest single call and number of calls each
// @Field: TimeUS: Time since system startup
// @Field: Pr: longest predict (all its steps)
// @Field: PrN: predicts
// @Field: St: most predict steps in one call (an IMU interval over 50 ms is split)
// @Field: Ps: longest position update
// @Field: PsN: position updates
// @Field: Vl: longest velocity update
// @Field: VlN: velocity updates
// @Field: Mg: longest magnetometer update (three axes and the declination)
// @Field: MgN: magnetometer updates
// @Field: Hg: longest barometer height update
// @Field: HgN: barometer height updates
// @Field: Sr: the start (once)
    const auto &np = _nav_parts;
    AP::logger().Write("ADNP", "TimeUS,Pr,PrN,St,Ps,PsN,Vl,VlN,Mg,MgN,Hg,HgN,Sr",
                       "ss--s-s-s-s-s", "FF--F-F-F-F-F", "QIHBIHIHIHIHI",
                       AP_HAL::micros64(),
                       np.max_us[NAV_PREDICT], np.n[NAV_PREDICT], np.max_steps,
                       np.max_us[NAV_POSITION], np.n[NAV_POSITION],
                       np.max_us[NAV_VELOCITY], np.n[NAV_VELOCITY],
                       np.max_us[NAV_MAG], np.n[NAV_MAG],
                       np.max_us[NAV_HEIGHT], np.n[NAV_HEIGHT],
                       np.max_us[NAV_START]);

    if (st.started) {
        const AP_AHRS &ahrs = AP::ahrs();
        // the AHRS keeps EKF3's gyro bias negated, as a drift to add
        const Vector3f egb = -ahrs.get_gyro_drift();
        const Vector3f &eab = ahrs.get_accel_bias();
// @LoggerMessage: ADNB
// @Description: Ada/SPARK navigation filter in shadow: its IMU bias estimates next to the AHRS's (EKF3)
// @Field: TimeUS: Time since system startup
// @Field: GX: Ada gyro bias, body X
// @Field: GY: Ada gyro bias, body Y
// @Field: GZ: Ada gyro bias, body Z
// @Field: AX: Ada accelerometer bias, body X
// @Field: AY: Ada accelerometer bias, body Y
// @Field: AZ: Ada accelerometer bias, body Z
// @Field: EGX: AHRS gyro bias, body X
// @Field: EGY: AHRS gyro bias, body Y
// @Field: EGZ: AHRS gyro bias, body Z
// @Field: EAX: AHRS accelerometer bias, body X
// @Field: EAY: AHRS accelerometer bias, body Y
// @Field: EAZ: AHRS accelerometer bias, body Z
// @Field: Arm: 1 if the filter was told the vehicle is armed
// @Field: Hld: accelerometer bias axes held on the ground, bit k for body axis k (0 X, 1 Y, 2 Z)
        AP::logger().Write("ADNB", "TimeUS,GX,GY,GZ,AX,AY,AZ,EGX,EGY,EGZ,EAX,EAY,EAZ,Arm,Hld",
                           "sEEEoooEEEooo--", "F000000000000--", "QffffffffffffBB",
                           AP_HAL::micros64(),
                           float(st.gyro_bias.x), float(st.gyro_bias.y), float(st.gyro_bias.z),
                           float(st.accel_bias.x), float(st.accel_bias.y), float(st.accel_bias.z),
                           egb.x, egb.y, egb.z, eab.x, eab.y, eab.z,
                           st.armed, st.accel_bias_held);
    }
#endif
    _nav_time.new_interval();
    _nav_parts = {};
}

#endif  // AP_ADA_ENABLED
