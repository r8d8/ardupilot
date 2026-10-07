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
#endif

    _checks = 0;
    _rejects = 0;
    _max_err = 0;
}

#endif  // AP_ADA_ENABLED
