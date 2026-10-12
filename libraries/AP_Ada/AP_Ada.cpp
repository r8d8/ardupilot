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
    if (!_healthy || _nav_time.tripped()) {
        return;
    }
    // timed for the main loop: what the shadow adds at AP_ADA_NAV_RATE_HZ
    const uint32_t t0 = AP_HAL::micros();
    if (_nav_started) {
        nav_fuse();
    } else {
        nav_start();
    }
    timed(_nav_time, "nav", AP_HAL::micros() - t0);
    nav_log();
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
    cw_nav_start(&accel, &mag, double(compass.get_declination()), &pos, &ok);
    if (!ok) {
        _nav_rejects++;
        return;
    }
    _nav_started = true;
    _nav_mag_us = compass.last_update_usec();
    _nav_baro_ms = baro.get_last_update();
    _nav_gps_ms = gps.last_fix_time_ms();
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
            for (uint8_t i = 0; i < steps; i++) {
                cw_nav_predict(&gyro, &accel, dt, &ok);
                if (!ok) {
                    _nav_rejects++;
                    break;
                }
            }
        } else {
            _nav_gaps++;
        }
    }

    const Compass &compass = AP::compass();
    const uint32_t mag_us = compass.last_update_usec();
    if (compass.healthy() && mag_us - _nav_mag_us >= NAV_SLOW_SENSOR_MS * 1000U) {
        _nav_mag_us = mag_us;
        const cw_vec3_t mag = c_vec(compass.get_field());
        cw_nav_fuse_mag(&mag, double(compass.get_declination()), &ok);
        _nav_rejects += ok ? 0 : 1;
    }

    const AP_Baro &baro = AP::baro();
    const uint32_t baro_ms = baro.get_last_update();
    if (baro.healthy() && baro_ms - _nav_baro_ms >= NAV_SLOW_SENSOR_MS) {
        _nav_baro_ms = baro_ms;
        cw_nav_fuse_height(double(baro.get_altitude()), &ok);
        _nav_rejects += ok ? 0 : 1;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (_nav_vio.ms != 0 && now_ms - _nav_vio.ms < NAV_VIO_TIMEOUT_MS) {
        // as the EKF: only the poses it was given; a new reset counter is
        // the VIO's reset, taken as is
        if (_nav_vio.fresh) {
            _nav_vio.fresh = false;
            if (_nav_vio.consumed) {
                const bool reset = _nav_vio_reset_known && _nav_vio.reset_counter != _nav_vio_reset;
                cw_nav_fuse_position(double(_nav_vio.ne.x), double(_nav_vio.ne.y),
                                     double(_nav_vio.err), reset ? 1 : 0, &ok);
                _nav_rejects += ok ? 0 : 1;
            }
            _nav_vio_reset = _nav_vio.reset_counter;
            _nav_vio_reset_known = true;
        }
        return;
    }

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
    const Vector2f ne = origin.get_distance_NE(gps.location());
    cw_nav_fuse_position(double(ne.x), double(ne.y), double(MAX(hacc, NAV_GPS_POS_MIN_M)), 0, &ok);
    _nav_rejects += ok ? 0 : 1;
    const cw_vec3_t vel = c_vec(gps.velocity());
    cw_nav_fuse_velocity(&vel, double(MAX(sacc, NAV_GPS_VEL_MIN_MS)), &ok);
    _nav_rejects += ok ? 0 : 1;
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

    if (now_ms - _nav_time_log_ms < 1000) {
        return;
    }
    _nav_time_log_ms = now_ms;
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
    _nav_time.new_interval();
#endif
}

#endif  // AP_ADA_ENABLED
