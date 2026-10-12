#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_LightWareSerial : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_LightWareSerial(_state, _params);
    }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    AP_RangeFinder_LightWareSerial(RangeFinder::RangeFinder_State &_state,
                                   AP_RangeFinder_Params &_params);

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    int8_t get_signal_quality_pct() const override {
        if (text_mode()) {
            return text_strength_pct;
        }
        return no_signal ? RangeFinder::SIGNAL_QUALITY_MIN : RangeFinder::SIGNAL_QUALITY_MAX;
    }

private:
    // get a reading
    bool get_reading(float &reading_m) override;
    bool is_lost_signal_distance(int16_t distance_cm, int16_t distance_cm_max);

    char linebuf[10];           // legacy protocol buffer
    uint8_t linebuf_len;        // legacy protocol buffer length
    uint32_t last_init_ms;      // init time used to switch lw20 to serial mode
    uint8_t high_byte;          // binary protocol high byte
    bool high_byte_received;    // true if high byte has been received

    // automatic protocol decision variables
    enum class ProtocolState {
        UNKNOWN,    // the protocol used is not yet known
        LEGACY,     // legacy protocol, distances are sent as strings
        BINARY      // binary protocol, distances are sent using two bytes
    } protocol_state;
    uint8_t legacy_valid_count;
    uint8_t binary_valid_count;

    bool no_signal = false;

    /*
      Clearwater (hazard HZ-22): with RNGFNDx_LW_ST above 0 the driver uses
      the LW20 text commands instead of the legacy protocol: each update
      asks for the raw distance to the last return (?LDL,1) and its signal
      strength (?LHL), and a distance whose strength is below LW_ST % is
      not a reading.
     */
    AP_Int8 min_signal_pct;
    bool text_mode() const { return min_signal_pct > 0; }
    bool get_reading_text(float &reading_m);
    char textbuf[24];           // text protocol reply line
    uint8_t textbuf_len;
    float text_distance_m;      // last ?LDL,1 reply waiting for its strength
    bool text_distance_pending;
    bool text_lost_signal;      // that reply was the lost-signal value
    int8_t text_strength_pct = RangeFinder::SIGNAL_QUALITY_UNKNOWN;
};

#endif  // AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED
