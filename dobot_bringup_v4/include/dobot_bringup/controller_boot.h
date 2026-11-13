/**
 * Which controller boot the arm was last enabled over TCP in.
 *
 * After a power-on with the arm enabled on the pendant or in the app, a ServoJ
 * stream can move every joint toward 0°, even behind a MovJ to the current
 * joints (case 9, 2026-11-11). DisableRobot() + EnableRobot() on port 29999
 * prevented it (2 of 2, 2026-10-27), so the driver does that once per boot,
 * before the boot's first stream.
 *
 * A boot is identified by its start on the driver's steady clock: now minus the
 * controller's run time from the realtime feed (whole seconds on the CR5).
 */

#ifndef CONTROLLER_BOOT_H
#define CONTROLLER_BOOT_H

#include <cmath>
#include <cstdint>

class ControllerBoot
{
public:
    // Two estimates of one boot's start differ by the run time's 1 s
    // resolution plus the feed's age; a reboot moves it by at least the
    // controller's start-up time (~20 s).
    static constexpr double kSlackS = 5.0;

    // True until mark() has been called in the boot that run_time_s belongs to.
    bool isNew(uint64_t run_time_s, double now_s) const
    {
        return !known_ || std::fabs(bootStart(run_time_s, now_s) - boot_s_) > kSlackS;
    }

    void mark(uint64_t run_time_s, double now_s)
    {
        boot_s_ = bootStart(run_time_s, now_s);
        known_ = true;
    }

private:
    static double bootStart(uint64_t run_time_s, double now_s)
    {
        return now_s - static_cast<double>(run_time_s);
    }

    bool known_ = false;
    double boot_s_ = 0.0;
};

#endif  // CONTROLLER_BOOT_H
