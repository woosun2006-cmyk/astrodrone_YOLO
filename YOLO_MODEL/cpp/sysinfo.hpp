#pragma once

// Port of yolo_live.py's sysinfo()/_cpu_thermal_zone(): board-level
// CPU/RAM/clock/temperature, read straight from /proc and /sys. Fields are
// std::optional the same way the Python version only sets a dict key on
// success - a transient read failure for one field must not blank out the
// others.

#include <optional>
#include <string>

struct SysSample {
    std::optional<double> cpu_pct;
    std::optional<double> mem_pct;
    std::optional<double> mem_used_mb;
    std::optional<double> mem_total_mb;
    std::optional<double> clock_mhz;
    std::optional<double> temp_c;
};

class SysInfoReader {
public:
    SysSample sample();

private:
    bool have_cpu_prev_ = false;
    double cpu_prev_total_ = 0.0, cpu_prev_idle_ = 0.0;
    std::string thermal_zone_path_;  // resolved once, cached
    std::string cpu_thermal_zone();
};
