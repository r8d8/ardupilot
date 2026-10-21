/*
  Clearwater HUMS node on DroneCAN: receive, log and forward its messages.

  The HUMS node (hums/ in the clearwater repo; node 60) sends
  clearwater.hums.Indicators (20600) and clearwater.hums.Status (20601),
  whose DSDL is copied unchanged into dsdl/clearwater/hums. Advisory only:
  nothing here acts on the values. Each value is logged (HUMI) and sent to
  the GCS as NAMED_VALUE_FLOAT "HU<id in hex>"; the status is logged (HUMS,
  with the node's own clock beside TimeUS) and its mode and alert level are
  sent as NAMED_VALUE_FLOAT. The node's NodeStatus is always OK, so a dead
  node shows only here: once a Status has been seen, 5 s without one gives
  "HUMS: no status" and a HUMS row with Mode 255. The node's alert texts
  come as debug.LogMessage through AP_DroneCAN::handle_debug.
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_DRONECAN_HUMS_ENABLED
#define AP_DRONECAN_HUMS_ENABLED HAL_ENABLE_DRONECAN_DRIVERS
#endif

#if AP_DRONECAN_HUMS_ENABLED

#include "AP_Canard_iface.h"   // Canard::allocate, before the canard C++ headers
#include <canard/subscriber.h>
#include <dronecan_msgs.h>

class AP_DroneCAN_HUMS
{
public:
    AP_DroneCAN_HUMS(const uint8_t driver_index) : _driver_index(driver_index) {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_DroneCAN_HUMS);

    // called from the DroneCAN thread's loop: the silence check
    void update(void);

    // a status that has stopped for this long is reported
    static constexpr uint32_t STATUS_TIMEOUT_MS = 5000;
    // HUMS row Mode when the status has stopped
    static constexpr uint8_t MODE_SILENT = 255;

private:
    void handle_indicators(const CanardRxTransfer& transfer, const clearwater_hums_Indicators& msg);
    void handle_status(const CanardRxTransfer& transfer, const clearwater_hums_Status& msg);
    void log_status(uint8_t node, uint32_t node_time_ms, uint8_t mode, uint8_t alert_level,
                    uint8_t alerts_active, uint16_t present, uint16_t healthy, uint8_t recording,
                    uint16_t sd_free_mb, float rotor_index_rpm, uint8_t table_version);

    const uint8_t _driver_index;

    Canard::ObjCallback<AP_DroneCAN_HUMS, clearwater_hums_Indicators> indicators_cb{this, &AP_DroneCAN_HUMS::handle_indicators};
    Canard::Subscriber<clearwater_hums_Indicators> indicators_listener{indicators_cb, _driver_index};
    Canard::ObjCallback<AP_DroneCAN_HUMS, clearwater_hums_Status> status_cb{this, &AP_DroneCAN_HUMS::handle_status};
    Canard::Subscriber<clearwater_hums_Status> status_listener{status_cb, _driver_index};

    // all touched from the DroneCAN thread only (handlers run in its loop)
    uint32_t _last_status_ms;
    uint8_t _node_id;
    bool _seen;
    bool _silent;
};

#endif // AP_DRONECAN_HUMS_ENABLED
