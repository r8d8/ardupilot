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
    AP::logger().Write("ADAU", "TimeUS,PAP,PAda,Mat,Mis,CkE", "s-----", "F-----",
                       "QIIIII", AP_HAL::micros64(), _ubx.ap_pvt, _ubx.ada_pvt,
                       _ubx.match, _ubx.mismatch, _ubx.ck_errors);
#endif

    _checks = 0;
    _rejects = 0;
    _max_err = 0;
    _ubx = {};
}

void AP_Ada::ubx_feed(uint8_t instance, uint8_t byte)
{
    if (!_healthy) {
        return;
    }
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
}

void AP_Ada::ubx_reset(uint8_t instance)
{
    if (_healthy) {
        cw_ubx_reset(instance);
    }
}

// the Ada parser saw the same bytes first, so its frame must be this NAV-PVT
void AP_Ada::ubx_check_nav_pvt(uint8_t instance, const cw_ubx_nav_pvt_t &ap)
{
    if (!_healthy) {
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
