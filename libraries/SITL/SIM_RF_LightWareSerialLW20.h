/*
  Simulator for a serial LightWare LW20/C answering its text commands
  (?LDL,1 last-return raw distance, ?LHL last-return strength in %), as
  seen on the Clearwater Goblin (hazard HZ-22): up to 60 m the true range
  with a strength falling with range, above 60 m a weak phantom of 5.5 m
  as in the Goblin's flight logs. The phantom strength is assumed, not
  measured. Lost signal (beyond 100 m) is "-1.00".

  SITL: --serial5=sim:lw20
param set SERIAL5_PROTOCOL 9
param set RNGFND1_TYPE 8
param set RNGFND1_LW_ST 10
*/

#pragma once

#include "SIM_SerialRangeFinder.h"

namespace SITL {

class RF_LightWareSerialLW20 : public SerialRangeFinder {
public:

    static SerialRangeFinder *create() { return NEW_NOTHROW RF_LightWareSerialLW20(); }

    void update(float range) override;

    // unused: replies are sent per query
    uint32_t packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen) override { return 0; }

    static constexpr float PHANTOM_ABOVE_M = 60.0f;
    static constexpr float PHANTOM_M = 5.5f;
    static constexpr uint8_t PHANTOM_STRENGTH_PCT = 4;

private:

    void reply(const char *cmd, float range);

    char line[24];
    uint8_t line_len;
};

}
