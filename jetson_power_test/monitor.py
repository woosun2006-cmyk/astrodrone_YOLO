#!/usr/bin/env python3
import argparse
import csv
import glob
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

GPU_LOAD = Path("/sys/devices/57000000.gpu/load")
GPU_FREQ = Path("/sys/devices/57000000.gpu/devfreq/57000000.gpu/cur_freq")
THERMAL = Path("/sys/class/thermal")
INA = Path("/sys/devices/50000000.host1x/546c0000.i2c/i2c-6/6-0040/iio:device0")

def read_text(path):
    try:
        return Path(path).read_text().strip()
    except (OSError, ValueError):
        return ""

def read_float(path, scale=1.0):
    try:
        return float(read_text(path)) / scale
    except ValueError:
        return None

def thermal_values():
    result = {}
    for zone in sorted(THERMAL.glob("thermal_zone*")):
        name = read_text(zone / "type")
        value = read_float(zone / "temp", 1000.0)
        if name and value is not None:
            result[name] = value
    return result

def cpu_sample(previous):
    fields = [int(v) for v in read_text("/proc/stat").splitlines()[0].split()[1:]]
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
    total = sum(fields)
    if previous is None or total == previous[0]:
        return None, (total, idle)
    usage = 100.0 * (1.0 - (idle - previous[1]) / (total - previous[0]))
    return usage, (total, idle)

def power_values():
    values = {}
    for i in range(3):
        rail = read_text(INA / ("rail_name_%d" % i))
        if not rail:
            continue
        values[rail + "_voltage_mV"] = read_float(INA / ("in_voltage%d_input" % i))
        values[rail + "_current_mA"] = read_float(INA / ("in_current%d_input" % i))
        values[rail + "_power_mW"] = read_float(INA / ("in_power%d_input" % i))
    return values

def main():
    parser = argparse.ArgumentParser(description="Jetson Nano combined CPU/GPU power test")
    parser.add_argument("--seconds", type=int, default=60)
    parser.add_argument("--max-temp", type=float, default=85.0)
    parser.add_argument("--target", type=int, default=85, choices=range(1, 101), metavar="1-100")
    parser.add_argument("--gpu-target", type=int, default=85, choices=range(1, 101), metavar="1-100")
    parser.add_argument("--log", default="jetson_power_test.csv")
    args = parser.parse_args()
    if args.seconds < 5:
        parser.error("--seconds must be at least 5")

    here = Path(__file__).resolve().parent
    stress = here / "jetson_stress"
    if not stress.exists():
        sys.exit("jetson_stress is missing; run ./build.sh first")

    initial_power = power_values()
    if not initial_power:
        print("NOTE: INA3221 power rails are unreadable. Run with sudo for voltage/current/power logging.")

    _, baseline_state = cpu_sample(None)
    time.sleep(0.5)
    baseline_cpu, _ = cpu_sample(baseline_state)
    baseline_cpu = baseline_cpu or 0.0
    cores = os.cpu_count() or 4
    cpu_threads = cores
    cpu_duty = max(1, min(100, int(round(args.target - baseline_cpu))))
    cpu_control_ema = baseline_cpu
    cpu_control_path = Path("/tmp/jetson_power_test_cpu_%d" % os.getpid())
    gpu_control_ema = args.gpu_target
    # On Nano, CUDA launch/driver gaps make measured GR3D utilization a few
    # points lower than the raw duty.  This small compensation was calibrated
    # with CPU load active (80% requested -> about 76-80% measured).
    gpu_duty = max(1, min(100, args.gpu_target - 5))
    cpu_control_path.write_text("%d %d" % (cpu_duty, gpu_duty))
    print("Baseline CPU=%.1f%%; stress CPU threads=%d at %d%% duty; GPU target=%d%% (duty=%d%%)" %
          (baseline_cpu, cpu_threads, cpu_duty, args.gpu_target, gpu_duty))
    proc = subprocess.Popen([str(stress), str(args.seconds + 5), str(cpu_threads),
                             str(cpu_duty), str(gpu_duty), str(cpu_control_path)])
    rows = []
    previous_cpu = None
    stopped_for_heat = False
    fieldnames = ["elapsed_s", "cpu_pct", "gpu_pct", "gpu_mhz", "CPU_C", "GPU_C", "PMIC_C"]
    fieldnames += sorted(initial_power)
    start = time.monotonic()
    try:
        while proc.poll() is None and time.monotonic() - start < args.seconds:
            cpu, previous_cpu = cpu_sample(previous_cpu)
            gpu = read_float(GPU_LOAD, 10.0)
            if cpu is not None:
                cpu_control_ema = 0.7 * cpu_control_ema + 0.3 * cpu
                change = max(-8.0, min(8.0, 0.6 * (args.target - cpu_control_ema)))
                cpu_duty = max(1, min(100, int(round(cpu_duty + change))))
            elapsed = time.monotonic() - start
            if gpu is not None and elapsed >= 3:
                gpu_control_ema = 0.8 * gpu_control_ema + 0.2 * gpu
                change = max(-3.0, min(3.0, 0.25 * (args.gpu_target - gpu_control_ema)))
                gpu_duty = max(1, min(100, int(round(gpu_duty + change))))
            cpu_control_path.write_text("%d %d" % (cpu_duty, gpu_duty))
            freq = read_float(GPU_FREQ, 1000000.0)
            therm = thermal_values()
            power = power_values()
            row = {
                "elapsed_s": round(time.monotonic() - start, 2),
                "cpu_pct": None if cpu is None else round(cpu, 1),
                "gpu_pct": gpu,
                "gpu_mhz": freq,
                "CPU_C": therm.get("CPU-therm"),
                "GPU_C": therm.get("GPU-therm"),
                "PMIC_C": therm.get("PMIC-Die"),
            }
            row.update(power)
            rows.append(row)
            shown = "CPU={cpu} GPU={gpu} GPUclk={freq}MHz CPUtemp={ct}C GPUtemp={gt}C".format(
                cpu="--" if cpu is None else "%.1f%%" % cpu,
                gpu="--" if gpu is None else "%.1f%%" % gpu,
                freq="--" if freq is None else "%.0f" % freq,
                ct=row["CPU_C"], gt=row["GPU_C"])
            if power:
                shown += " " + " ".join("%s=%.0f" % (k, v) for k, v in sorted(power.items()) if v is not None)
            print(shown, flush=True)
            hottest = max((v for v in therm.values() if v is not None), default=0)
            if hottest >= args.max_temp:
                stopped_for_heat = True
                print("SAFETY STOP: temperature %.1fC reached limit %.1fC" % (hottest, args.max_temp), file=sys.stderr)
                break
            time.sleep(1)
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        try:
            cpu_control_path.unlink()
        except OSError:
            pass

    with open(args.log, "w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    valid = rows[2:] if len(rows) > 2 else rows
    avg_cpu = sum(r["cpu_pct"] for r in valid if r["cpu_pct"] is not None) / max(1, sum(r["cpu_pct"] is not None for r in valid))
    avg_gpu = sum(r["gpu_pct"] for r in valid if r["gpu_pct"] is not None) / max(1, sum(r["gpu_pct"] is not None for r in valid))
    max_cpu = max((r["cpu_pct"] for r in valid if r["cpu_pct"] is not None), default=0)
    max_gpu = max((r["gpu_pct"] for r in valid if r["gpu_pct"] is not None), default=0)
    max_temp = max((v for r in rows for k, v in r.items() if k.endswith("_C") and v is not None), default=0)
    print("\nSUMMARY avg CPU=%.1f%%, avg GPU=%.1f%%, max temp=%.1fC, log=%s" % (avg_cpu, avg_gpu, max_temp, args.log))
    input_voltage_fields = [k for k in fieldnames if k.endswith("_voltage_mV") and ("VDD_IN" in k or "5V" in k)]
    low_voltage = False
    for key in input_voltage_fields:
        samples = [r.get(key) for r in rows if r.get(key) is not None]
        if samples:
            minimum = min(samples)
            print("SUMMARY %s minimum=%.0fmV" % (key, minimum))
            low_voltage = low_voltage or minimum < 4750
    if stopped_for_heat:
        return 3
    if low_voltage:
        print("RESULT: FAIL/WARNING (5V input dropped below 4.75V)")
        return 4
    if max_cpu > args.target + 5 or max_gpu > args.gpu_target + 10:
        print("RESULT: CAP WARNING (whole-system utilization exceeded target; other applications are included)")
        return 2
    if avg_cpu < args.target - 10 or avg_gpu < args.gpu_target - 10:
        print("RESULT: CAP RESPECTED, BUT LOAD BELOW TARGET (power stress result is inconclusive)")
        return 2
    print("RESULT: TARGET LOAD PASSED. Check the log for voltage droop; a longer run is recommended.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
