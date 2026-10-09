/*
  Host check of AP_Ada_CallTimer, no HAL or gtest needed:

    g++ -std=c++17 -Wall -Werror -I libraries/AP_Ada \
        libraries/AP_Ada/tests/call_timer_check.cpp -o /tmp/call_timer_check
    /tmp/call_timer_check

  Known result: all checks pass, among them a p99 of 3 us for 990 calls of
  2 or 3 us plus 10 of 500 us, and a trip on the 10th call over budget.
 */
#include "AP_Ada_CallTimer.h"

#include <cstdio>

static int failures;

static void check(const char *name, bool cond)
{
    std::printf("%s %s\n", cond ? "ok  " : "FAIL", name);
    if (!cond) {
        failures++;
    }
}

int main()
{
    AP_Ada_CallTimer t(100, 10);
    check("empty interval: no calls, percentile 0",
          t.calls() == 0 && t.percentile_us(99) == 0 && t.max_us() == 0);

    // 990 fast calls, 10 slow ones (1 %): p99 stays on the fast calls
    for (int i = 0; i < 990; i++) {
        t.add(i % 2 ? 3 : 2);
    }
    for (int i = 0; i < 10; i++) {
        t.add(500);
    }
    check("counts, sum and max", t.calls() == 1000 && t.sum_us() == 495 * 2 + 495 * 3 + 5000 &&
          t.max_us() == 500);
    check("p99 of 990 x 2-3 us and 10 x 500 us is 3 us", t.percentile_us(99) == 3);
    check("495 of 1000 calls take 2 us: p49 is 2 us, p50 is 3 us",
          t.percentile_us(49) == 2 && t.percentile_us(50) == 3);
    check("p100 is the longest call, not its bin edge", t.percentile_us(100) == 500);
    check("10 calls over 100 us: overruns and tripped",
          t.overruns() == 10 && t.overruns_total() == 10 && t.tripped());

    // one more slow call ranks into the octave bin: its edge, capped at max
    AP_Ada_CallTimer u(100, 10);
    for (int i = 0; i < 98; i++) {
        u.add(1);
    }
    u.add(40);
    u.add(45);
    check("p99 in the 32-63 us bin is capped at the longest call (45 us)",
          u.percentile_us(99) == 45);
    u.add(20);
    u.add(70);
    check("p98 in the 32-63 us bin with a longer max is the bin edge 63 us",
          u.percentile_us(98) == 63);
    check("exactly the budget is not an overrun", !u.add(100) && u.add(101) && !u.tripped());

    // octave boundaries
    AP_Ada_CallTimer b(1000000, 1);
    b.add(15);
    check("15 us is a fine bin", b.percentile_us(100) == 15);
    b.new_interval();
    b.add(16);
    b.add(5000);
    check("16 us falls in the 16-31 us bin, reported as its edge 31 us",
          b.percentile_us(50) == 31);
    check("5000 us falls in the open top bin, reported as the max",
          b.percentile_us(100) == 5000);

    // a new interval clears the statistics, not the trip
    t.new_interval();
    check("new interval: statistics cleared, trip kept",
          t.calls() == 0 && t.max_us() == 0 && t.overruns() == 0 && t.tripped());

    std::printf(failures ? "%d failure(s)\n" : "all AP_Ada_CallTimer checks passed\n", failures);
    return failures ? 1 : 0;
}
