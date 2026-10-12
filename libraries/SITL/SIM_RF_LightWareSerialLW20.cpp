/*
  Simulator for a serial LightWare LW20/C answering its text commands;
  see SIM_RF_LightWareSerialLW20.h.
*/

#include "SIM_RF_LightWareSerialLW20.h"

#include <stdio.h>
#include <string.h>

using namespace SITL;

void RF_LightWareSerialLW20::reply(const char *cmd, float range)
{
    float distance_m;
    unsigned strength_pct;
    if (!isfinite(range) || range > 100) {
        distance_m = -1;
        strength_pct = 0;
    } else if (range > PHANTOM_ABOVE_M) {
        distance_m = PHANTOM_M;
        strength_pct = PHANTOM_STRENGTH_PCT;
    } else {
        distance_m = range;
        strength_pct = unsigned(MAX(20.0f, 90.0f - range));
    }

    char buf[32];
    int n = 0;
    if (strcmp(cmd, "?LDL,1") == 0) {
        n = snprintf(buf, sizeof(buf), "ldl,1:%.2f\r\n", double(distance_m));
    } else if (strcmp(cmd, "?LHL") == 0) {
        n = snprintf(buf, sizeof(buf), "lhl:%u\r\n", strength_pct);
    }
    // "www" and anything else: no reply
    if (n > 0) {
        write_to_autopilot(buf, MIN(size_t(n), sizeof(buf) - 1));
    }
}

void RF_LightWareSerialLW20::update(float range)
{
    char c;
    while (read_from_autopilot(&c, 1) == 1) {
        if (c == '\r' || c == '\n') {
            line[line_len] = 0;
            if (line_len > 0) {
                reply(line, range);
            }
            line_len = 0;
        } else if (line_len < sizeof(line) - 1) {
            line[line_len++] = c;
        } else {
            line_len = 0;
        }
    }
}
