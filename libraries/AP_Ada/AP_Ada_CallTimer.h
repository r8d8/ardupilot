#pragma once

#include <stdint.h>

/*
  Durations of the calls into one Ada shadow (hazards HZ-19, HZ-20).

  Per log interval: the number of calls, their total, the longest and the
  99th percentile, from a histogram with 1 us bins below 16 us and octaves
  above (16-31, 32-63, ... 2048 and over). A percentile is reported as the
  upper edge of its bin, capped at the longest call, so it is never below
  the true value and never above the longest call.

  Since boot: the calls over the budget. Once trip_after of them have
  happened the timer is tripped and AP_Ada stops calling that shadow, so a
  slow Ada path can delay the vehicle's loop at most trip_after times. A
  call that never returns is caught by the hardware watchdog, not here.

  Plain C++ without HAL, so tests/call_timer_check.cpp runs it on the host.
 */
class AP_Ada_CallTimer {
public:
    AP_Ada_CallTimer(uint32_t budget_us, uint16_t trip_after) :
        _budget_us(budget_us),
        _trip_after(trip_after),
        _overruns_total(0)
    {
        new_interval();
    }

    // record one call; true if it took longer than the budget
    bool add(uint32_t dt_us)
    {
        _bins[bin(dt_us)]++;
        _calls++;
        _sum_us += dt_us;
        if (dt_us > _max_us) {
            _max_us = dt_us;
        }
        if (dt_us <= _budget_us) {
            return false;
        }
        _overruns++;
        if (_overruns_total < UINT16_MAX) {
            _overruns_total++;
        }
        return true;
    }

    bool tripped() const { return _overruns_total >= _trip_after; }

    uint32_t budget_us() const { return _budget_us; }
    uint16_t overruns_total() const { return _overruns_total; }

    // this interval
    uint32_t calls() const { return _calls; }
    uint32_t sum_us() const { return _sum_us; }
    uint32_t max_us() const { return _max_us; }
    uint32_t overruns() const { return _overruns; }

    // smallest bin edge at or below which at least pct % of this
    // interval's calls lie, capped at the longest call; 0 without calls
    uint32_t percentile_us(uint8_t pct) const
    {
        if (_calls == 0) {
            return 0;
        }
        // calls ranked from 1: the percentile is call number ceil (calls pct / 100)
        const uint64_t rank = (uint64_t(_calls) * pct + 99) / 100;
        uint64_t seen = 0;
        for (uint8_t i = 0; i < BINS; i++) {
            seen += _bins[i];
            if (seen >= rank) {
                const uint32_t edge = upper_edge(i);
                return edge < _max_us ? edge : _max_us;
            }
        }
        return _max_us;
    }

    void new_interval()
    {
        for (uint8_t i = 0; i < BINS; i++) {
            _bins[i] = 0;
        }
        _calls = 0;
        _sum_us = 0;
        _max_us = 0;
        _overruns = 0;
    }

private:
    static const uint8_t FINE = 16;          // 1 us bins: 0 .. 15 us
    static const uint8_t OCTAVES = 8;        // 16-31 .. 1024-2047, 2048 and over
    static const uint8_t BINS = FINE + OCTAVES;

    static uint8_t bin(uint32_t dt_us)
    {
        if (dt_us < FINE) {
            return uint8_t(dt_us);
        }
        uint8_t i = FINE;
        for (uint32_t edge = 2 * FINE; dt_us >= edge && i < BINS - 1; edge *= 2) {
            i++;
        }
        return i;
    }

    static uint32_t upper_edge(uint8_t i)
    {
        if (i < FINE) {
            return i;
        }
        if (i == BINS - 1) {
            return UINT32_MAX;
        }
        return (uint32_t(2 * FINE) << (i - FINE)) - 1;
    }

    const uint32_t _budget_us;
    const uint16_t _trip_after;
    uint16_t _overruns_total;

    uint32_t _bins[BINS];
    uint32_t _calls;
    uint32_t _sum_us;
    uint32_t _max_us;
    uint32_t _overruns;
};
