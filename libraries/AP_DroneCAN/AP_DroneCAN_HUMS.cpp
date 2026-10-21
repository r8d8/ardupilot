/*
  Clearwater HUMS node on DroneCAN: receive, log and forward its messages
  (see AP_DroneCAN_HUMS.h).
 */
#include "AP_DroneCAN_HUMS.h"

#if AP_DRONECAN_HUMS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void AP_DroneCAN_HUMS::handle_indicators(const CanardRxTransfer& transfer, const clearwater_hums_Indicators& msg)
{
#if HAL_LOGGING_ENABLED
    const uint64_t now_us = AP_HAL::micros64();
#endif
    const uint8_t n = MIN(msg.indicators.len, ARRAY_SIZE(msg.indicators.data));
    for (uint8_t i = 0; i < n; i++) {
        const auto &ind = msg.indicators.data[i];
#if HAL_LOGGING_ENABLED
// @LoggerMessage: HUMI
// @Description: Clearwater HUMS node indicator value, one row per indicator (advisory)
// @Field: TimeUS: Time since system startup
// @Field: Drv: DroneCAN driver the message came on
// @Field: Node: sending node ID
// @Field: Ver: indicator table version (hums/dronecan/indicators.toml)
// @Field: Seq: message sequence number; a gap is a lost message
// @Field: Id: indicator id
// @Field: St: status bits: 1 valid, 2 learning, 4 caution, 8 warning, 16 stale
// @Field: Val: value in the table's unit; NaN if none
        AP::logger().Write("HUMI", "TimeUS,Drv,Node,Ver,Seq,Id,St,Val", "s#------", "F-------", "QBBBBHBf",
                           now_us, _driver_index, uint8_t(transfer.source_node_id), msg.table_version,
                           msg.sequence, ind.id, ind.status, ind.value);
#endif
#if HAL_GCS_ENABLED
        char name[11];
        hal.util->snprintf(name, sizeof(name), "HU%04X", unsigned(ind.id));
        gcs().send_named_float(name, ind.value);
#endif
    }
}

void AP_DroneCAN_HUMS::handle_status(const CanardRxTransfer& transfer, const clearwater_hums_Status& msg)
{
    _last_status_ms = AP_HAL::millis();
    _node_id = transfer.source_node_id;
    _seen = true;
    if (_silent) {
        _silent = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HUMS: status back");
    }
    log_status(_node_id, msg.node_time_ms, msg.mode, msg.alert_level, msg.alerts_active,
               msg.sensors_present, msg.sensors_healthy, msg.recording, msg.sd_free_mb,
               msg.rotor_index_rpm, msg.table_version);
#if HAL_GCS_ENABLED
    gcs().send_named_float("HUMSMODE", msg.mode);
    gcs().send_named_float("HUMSALRT", msg.alert_level);
#endif
}

void AP_DroneCAN_HUMS::update(void)
{
    if (!_seen || _silent) {
        return;
    }
    if (AP_HAL::millis() - _last_status_ms > STATUS_TIMEOUT_MS) {
        _silent = true;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "HUMS: no status");
        log_status(_node_id, 0, MODE_SILENT, 0, 0, 0, 0, 0, 0, NaNf, 0);
    }
}

void AP_DroneCAN_HUMS::log_status(uint8_t node, uint32_t node_time_ms, uint8_t mode, uint8_t alert_level,
                                  uint8_t alerts_active, uint16_t present, uint16_t healthy, uint8_t recording,
                                  uint16_t sd_free_mb, float rotor_index_rpm, uint8_t table_version)
{
#if HAL_LOGGING_ENABLED
// @LoggerMessage: HUMS
// @Description: Clearwater HUMS node status, 1 Hz (advisory)
// @Field: TimeUS: Time since system startup
// @Field: Drv: DroneCAN driver the message came on
// @Field: Node: sending node ID
// @Field: NdMs: the node's own clock, ms since its boot, for aligning its SD snapshots with this log
// @Field: Mode: 0 starting, 1 learning, 2 monitoring, 3 degraded; 255 no status for 5 s
// @Field: Alrt: highest active alert: 0 none, 1 info, 2 caution, 3 warning
// @Field: NAl: number of indicators over a limit
// @Field: SPr: sensor channels present (bits)
// @Field: SHl: sensor channels healthy (bits)
// @Field: Rec: recording bits: 1 SD present, 2 recording, 4 snapshot, 8 SD full, 16 SD error
// @Field: SDMB: SD free space
// @Field: IRPM: rotor speed from the node's own 1/rev index
// @Field: Ver: indicator table version
    AP::logger().Write("HUMS", "TimeUS,Drv,Node,NdMs,Mode,Alrt,NAl,SPr,SHl,Rec,SDMB,IRPM,Ver",
                       "s#-s-------q-", "F--C---------", "QBBIBBBHHBHfB",
                       AP_HAL::micros64(), _driver_index, node, node_time_ms, mode, alert_level,
                       alerts_active, present, healthy, recording, sd_free_mb, rotor_index_rpm,
                       table_version);
#endif
}

#endif // AP_DRONECAN_HUMS_ENABLED
