#include "AP_Ada.h"

#if AP_ADA_ENABLED

#include <cw_fcs.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

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
#endif

    _checks = 0;
    _rejects = 0;
    _max_err = 0;
    _ubx = {};
    _swash = {};
    _ubx_time.new_interval();
    _swash_time.new_interval();
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

#endif  // AP_ADA_ENABLED
