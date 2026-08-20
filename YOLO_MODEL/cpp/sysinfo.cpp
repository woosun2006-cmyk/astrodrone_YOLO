#include "sysinfo.hpp"

#include <dirent.h>

#include <cmath>
#include <fstream>
#include <sstream>

namespace {

double round_to(double v, int decimals) {
    double scale = std::pow(10.0, decimals);
    return std::round(v * scale) / scale;
}

}  // namespace

std::string SysInfoReader::cpu_thermal_zone() {
    if (!thermal_zone_path_.empty()) return thermal_zone_path_;

    const std::string base = "/sys/class/thermal";
    DIR* dir = opendir(base.c_str());
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name(entry->d_name);
            if (name.rfind("thermal_zone", 0) != 0) continue;
            std::ifstream type_f(base + "/" + name + "/type");
            std::string type;
            if (type_f && std::getline(type_f, type) && type == "CPU-therm") {
                thermal_zone_path_ = base + "/" + name + "/temp";
                closedir(dir);
                return thermal_zone_path_;
            }
        }
        closedir(dir);
    }
    thermal_zone_path_ = "/sys/class/thermal/thermal_zone0/temp";
    return thermal_zone_path_;
}

SysSample SysInfoReader::sample() {
    SysSample out;

    // CPU%: delta of (total - idle) over delta total, between this call and
    // the last one - matches yolo_live.py's _cpu_prev bookkeeping.
    {
        std::ifstream f("/proc/stat");
        std::string line;
        if (f && std::getline(f, line)) {
            std::istringstream iss(line);
            std::string cpu_label;
            iss >> cpu_label;
            double vals[7] = {0, 0, 0, 0, 0, 0, 0};
            bool ok = true;
            for (double& v : vals) {
                if (!(iss >> v)) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                double idle = vals[3] + vals[4];
                double total = 0;
                for (double v : vals) total += v;
                if (have_cpu_prev_) {
                    double dt = total - cpu_prev_total_;
                    double di = idle - cpu_prev_idle_;
                    out.cpu_pct = dt > 0 ? round_to(100.0 * (dt - di) / dt, 1) : 0.0;
                }
                cpu_prev_total_ = total;
                cpu_prev_idle_ = idle;
                have_cpu_prev_ = true;
            }
        }
    }

    // RAM
    {
        std::ifstream f("/proc/meminfo");
        std::string line;
        double mem_total_kb = 0, mem_avail_kb = 0;
        bool have_total = false, have_avail = false;
        while (f && std::getline(f, line)) {
            size_t colon = line.find(':');
            if (colon == std::string::npos) continue;
            std::string key = line.substr(0, colon);
            std::istringstream iss(line.substr(colon + 1));
            double kb;
            if (!(iss >> kb)) continue;
            if (key == "MemTotal") {
                mem_total_kb = kb;
                have_total = true;
            } else if (key == "MemAvailable") {
                mem_avail_kb = kb;
                have_avail = true;
            }
        }
        if (have_total && mem_total_kb > 0) {
            double avail = have_avail ? mem_avail_kb : 0.0;
            out.mem_pct = round_to(100.0 * (mem_total_kb - avail) / mem_total_kb, 1);
            out.mem_used_mb = std::round((mem_total_kb - avail) / 1024.0);
            out.mem_total_mb = std::round(mem_total_kb / 1024.0);
        }
    }

    // Clock
    {
        std::ifstream f("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
        long khz;
        if (f && (f >> khz)) {
            out.clock_mhz = std::round(khz / 1000.0);
        }
    }

    // Temp
    {
        std::ifstream f(cpu_thermal_zone());
        long milli_c;
        if (f && (f >> milli_c)) {
            out.temp_c = round_to(milli_c / 1000.0, 1);
        }
    }

    return out;
}
