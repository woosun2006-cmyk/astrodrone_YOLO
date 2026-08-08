#!/usr/bin/env python3
import csv
import math
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import monitor
try:
    from jtop import jtop
except ImportError:
    jtop = None

BASE = Path(__file__).resolve().parent
LOG_DIR = BASE / "logs"
BG = "#111827"
PANEL = "#1f2937"
TEXT = "#f9fafb"
MUTED = "#9ca3af"
GREEN = "#22c55e"
YELLOW = "#f59e0b"
RED = "#ef4444"
BLUE = "#38bdf8"

LOG_FIELDS = [
    "timestamp", "event", "elapsed_s", "duration_s", "target_pct",
    "gpu_target_pct", "gpu_stress_duty",
    "cpu_stress_threads", "cpu_stress_duty", "stress_exit_code",
    "cpu_pct", "gpu_pct", "gpu_mhz", "CPU_C", "GPU_C", "PMIC_C",
    "POM_5V_IN_voltage_mV", "POM_5V_IN_current_mA", "POM_5V_IN_power_mW",
    "POM_5V_GPU_voltage_mV", "POM_5V_GPU_current_mA", "POM_5V_GPU_power_mW",
    "POM_5V_CPU_voltage_mV", "POM_5V_CPU_current_mA", "POM_5V_CPU_power_mW",
]


class MetricCard(tk.Frame):
    def __init__(self, parent, title, unit="", show_bar=False):
        super().__init__(parent, bg=PANEL, highlightthickness=1, highlightbackground="#374151")
        self.unit = unit
        tk.Label(self, text=title, bg=PANEL, fg=MUTED, font=("Sans", 12)).pack(anchor="w", padx=18, pady=(14, 0))
        self.value = tk.Label(self, text="--", bg=PANEL, fg=TEXT, font=("Sans", 28, "bold"))
        self.value.pack(anchor="w", padx=18, pady=(3, 8))
        self.bar = None
        if show_bar:
            self.bar = tk.Canvas(self, height=8, bg="#374151", highlightthickness=0)
            self.bar.pack(fill="x", padx=18, pady=(0, 15))
            self.bar.bind("<Configure>", lambda _e: self._draw_bar())
        self.percent = 0
        self.color = BLUE

    def set(self, value, color=TEXT, percent=None, decimals=1):
        if value is None:
            shown = "--"
        else:
            shown = ("%%.%df" % decimals) % value
        self.value.configure(text=shown + (" " + self.unit if self.unit else ""), fg=color)
        if percent is not None:
            self.percent = max(0, min(100, percent))
            self.color = color
            self._draw_bar()

    def _draw_bar(self):
        if not self.bar:
            return
        self.bar.delete("all")
        width = self.bar.winfo_width()
        self.bar.create_rectangle(0, 0, width * self.percent / 100.0, 8, fill=self.color, outline="")


class Dashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        LOG_DIR.mkdir(exist_ok=True)
        self.title("Jetson Nano 전원 · 성능 모니터")
        self.geometry("980x690")
        self.minsize(900, 640)
        self.configure(bg=BG)
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.proc = None
        self.started = None
        self.end_at = None
        self.previous_cpu = None
        self.rows = []
        self.log_path = None
        self.log_file = None
        self.log_writer = None
        self.stderr_path = None
        self.stderr_file = None
        self.test_duration = None
        self.test_target = None
        self.gpu_target = 85
        self.gpu_stress_duty = 80
        self.gpu_control_ema = None
        self.cpu_stress_threads = None
        self.cpu_stress_duty = None
        self.cpu_control_path = None
        self.cpu_control_ema = None
        self.latest_cpu = None
        self.jetson = None
        if jtop is not None:
            try:
                self.jetson = jtop()
                self.jetson.start()
            except Exception:
                self.jetson = None
        self._build()
        for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
            signal.signal(signum, self._signal_exit)
        self.after(200, self.sample)

    def _build(self):
        header = tk.Frame(self, bg=BG)
        header.pack(fill="x", padx=25, pady=(20, 14))
        tk.Label(header, text="JETSON NANO", bg=BG, fg=BLUE, font=("Sans", 11, "bold")).pack(anchor="w")
        tk.Label(header, text="전원 · 성능 모니터", bg=BG, fg=TEXT, font=("Sans", 24, "bold")).pack(side="left")
        self.status = tk.Label(header, text="● 대기", bg=BG, fg=MUTED, font=("Sans", 13, "bold"))
        self.status.pack(side="right", pady=7)

        grid = tk.Frame(self, bg=BG)
        grid.pack(fill="both", expand=True, padx=25)
        for c in range(3):
            grid.grid_columnconfigure(c, weight=1, uniform="cards")
        for r in range(2):
            grid.grid_rowconfigure(r, weight=1, uniform="cards")

        self.cpu = MetricCard(grid, "CPU 사용률", "%", True)
        self.gpu = MetricCard(grid, "GPU 사용률", "%", True)
        self.power = MetricCard(grid, "전체 소비전력", "W")
        self.cpu_temp = MetricCard(grid, "CPU 온도", "°C")
        self.gpu_temp = MetricCard(grid, "GPU 온도", "°C")
        self.voltage = MetricCard(grid, "5V 입력 전압", "V")
        cards = [self.cpu, self.gpu, self.power, self.cpu_temp, self.gpu_temp, self.voltage]
        for i, card in enumerate(cards):
            card.grid(row=i // 3, column=i % 3, sticky="nsew", padx=6, pady=6)

        details = tk.Frame(self, bg=PANEL, highlightthickness=1, highlightbackground="#374151")
        details.pack(fill="x", padx=31, pady=(12, 8))
        self.detail = tk.Label(details, text="GPU 클럭 -- MHz   |   PMIC -- °C   |   입력 전류 -- A",
                               bg=PANEL, fg=MUTED, font=("Sans", 11))
        self.detail.pack(pady=12)

        controls = tk.Frame(self, bg=BG)
        controls.pack(fill="x", padx=31, pady=(4, 22))
        tk.Label(controls, text="시험 시간", bg=BG, fg=MUTED, font=("Sans", 11)).pack(side="left")
        self.duration = ttk.Combobox(controls, state="readonly", width=10,
                                     values=("1분", "5분", "10분", "15분"))
        self.duration.current(2)
        self.duration.pack(side="left", padx=(8, 15))
        tk.Label(controls, text="CPU 목표", bg=BG, fg=MUTED, font=("Sans", 11)).pack(side="left")
        self.target = ttk.Combobox(controls, state="readonly", width=7,
                                   values=("80%", "85%", "90%"))
        self.target.current(1)
        self.target.pack(side="left", padx=(8, 15))
        tk.Label(controls, text="GPU 목표 85%", bg=BG, fg=BLUE,
                 font=("Sans", 11, "bold")).pack(side="left", padx=(0, 15))
        self.start_button = tk.Button(controls, text="부하 시험 시작", command=self.start_test,
                                      bg="#2563eb", fg="white", activebackground="#1d4ed8",
                                      activeforeground="white", relief="flat", padx=22, pady=9,
                                      font=("Sans", 11, "bold"))
        self.start_button.pack(side="left")
        self.stop_button = tk.Button(controls, text="중지", command=self.stop_test, state="disabled",
                                     bg="#7f1d1d", fg="white", disabledforeground=MUTED,
                                     relief="flat", padx=22, pady=9, font=("Sans", 11, "bold"))
        self.stop_button.pack(side="left", padx=8)
        self.timer = tk.Label(controls, text="", bg=BG, fg=TEXT, font=("Sans", 12, "bold"))
        self.timer.pack(side="right")

    @staticmethod
    def level_color(value, warning, danger):
        if value is None:
            return MUTED
        if value >= danger:
            return RED
        if value >= warning:
            return YELLOW
        return GREEN

    @staticmethod
    def find_power(data, suffix):
        preferred = [k for k in data if k.endswith(suffix) and ("POM_5V_IN" in k or "VDD_IN" in k)]
        return data.get(preferred[0]) if preferred else None

    def read_power(self):
        direct = monitor.power_values()
        if direct or self.jetson is None or not self.jetson.ok():
            return direct
        try:
            info = self.jetson.power
            rails = dict(info.get("rail", {}))
            total = info.get("tot", {})
            if total:
                rails[total.get("name", "POM_5V_IN")] = total
            result = {}
            for name, values in rails.items():
                result[name + "_voltage_mV"] = values.get("volt")
                result[name + "_current_mA"] = values.get("curr")
                result[name + "_power_mW"] = values.get("power")
            return result
        except Exception:
            return {}

    def start_test(self):
        binary = BASE / "jetson_stress"
        if not binary.exists():
            messagebox.showerror("실행 불가", "jetson_stress가 없습니다. 먼저 ./build.sh를 실행하세요.")
            return
        minutes = int(self.duration.get().replace("분", ""))
        seconds = minutes * 60
        target = int(self.target.get().replace("%", ""))
        baseline = self.latest_cpu or 0.0
        cores = os.cpu_count() or 4
        cpu_threads = cores
        cpu_duty = max(1, min(100, int(round(target - baseline))))
        self.rows = []
        self.log_path = LOG_DIR / ("power_test_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.stderr_path = self.log_path.with_name(self.log_path.stem + "_stress.log")
        self.test_duration = seconds
        self.test_target = target
        self.cpu_stress_threads = cpu_threads
        self.cpu_stress_duty = cpu_duty
        self.cpu_control_ema = baseline
        self.gpu_control_ema = self.gpu_target
        self.cpu_control_path = BASE / (".cpu_duty_%d" % os.getpid())
        self.cpu_control_path.write_text("%d %d" % (cpu_duty, self.gpu_stress_duty))
        self._open_logs()
        try:
            self.proc = subprocess.Popen([str(binary), str(seconds), str(cpu_threads),
                                          str(cpu_duty), str(self.gpu_stress_duty),
                                          str(self.cpu_control_path)], stdout=subprocess.DEVNULL,
                                         stderr=self.stderr_file)
        except Exception as exc:
            self._write_log({"event": "start_failed: %s" % exc})
            self._close_logs()
            try:
                self.cpu_control_path.unlink()
            except OSError:
                pass
            self.cpu_control_path = None
            messagebox.showerror("실행 실패", str(exc))
            return
        self.started = time.monotonic()
        self.end_at = self.started + seconds
        self._write_log({"event": "start", "elapsed_s": 0,
                         "duration_s": seconds, "target_pct": target,
                         "gpu_target_pct": self.gpu_target,
                         "gpu_stress_duty": self.gpu_stress_duty,
                         "cpu_stress_threads": cpu_threads,
                         "cpu_stress_duty": cpu_duty})
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status.configure(text="● 부하 시험 중", fg=BLUE)

    def stop_test(self, reason=None):
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
        self.finish_test(reason or "사용자가 중지함")

    def finish_test(self, reason):
        if not self.proc and not self.log_file:
            return
        proc = self.proc
        if proc and proc.poll() is None:
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        exit_code = proc.returncode if proc else None
        elapsed = time.monotonic() - self.started if self.started else None
        self._write_log({"event": reason,
                         "elapsed_s": None if elapsed is None else round(elapsed, 2),
                         "stress_exit_code": exit_code})
        self._close_logs()
        self.proc = None
        if self.cpu_control_path is not None:
            try:
                self.cpu_control_path.unlink()
            except OSError:
                pass
            self.cpu_control_path = None
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.timer.configure(text="")
        bad = "자동" in reason or "비정상" in reason or "조기" in reason
        self.status.configure(text="● " + reason, fg=RED if bad else GREEN)

    def _owner_uid(self):
        value = os.environ.get("PKEXEC_UID") or os.environ.get("SUDO_UID")
        return int(value) if value and value.isdigit() else None

    def _give_to_original_user(self, path):
        uid = self._owner_uid()
        if os.geteuid() == 0 and uid is not None:
            os.chown(path, uid, -1)

    def _open_logs(self):
        self.log_file = self.log_path.open("w", newline="", buffering=1)
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=LOG_FIELDS, extrasaction="ignore")
        self.log_writer.writeheader()
        self.log_file.flush()
        os.fsync(self.log_file.fileno())
        self._give_to_original_user(self.log_path)
        self.stderr_file = self.stderr_path.open("w", buffering=1)
        self._give_to_original_user(self.stderr_path)

    def _write_log(self, row):
        if not self.log_writer or not self.log_file:
            return
        durable_row = {
            "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "duration_s": self.test_duration,
            "target_pct": self.test_target,
            "gpu_target_pct": self.gpu_target,
            "gpu_stress_duty": self.gpu_stress_duty,
            "cpu_stress_threads": self.cpu_stress_threads,
            "cpu_stress_duty": self.cpu_stress_duty,
        }
        durable_row.update(row)
        self.log_writer.writerow(durable_row)
        self.log_file.flush()
        os.fsync(self.log_file.fileno())
        if self.stderr_file:
            self.stderr_file.flush()
            os.fsync(self.stderr_file.fileno())

    def _close_logs(self):
        if self.stderr_file:
            self.stderr_file.flush()
            os.fsync(self.stderr_file.fileno())
            self.stderr_file.close()
        if self.log_file:
            self.log_file.flush()
            os.fsync(self.log_file.fileno())
            self.log_file.close()
        self.stderr_file = None
        self.log_file = None
        self.log_writer = None

    def _signal_exit(self, signum, _frame):
        self.after(0, lambda: self.stop_test("프로그램 신호 종료 SIG%d" % signum))

    def sample(self):
        try:
            cpu, self.previous_cpu = monitor.cpu_sample(self.previous_cpu)
            self.latest_cpu = cpu
            gpu = monitor.read_float(monitor.GPU_LOAD, 10.0)
            freq = monitor.read_float(monitor.GPU_FREQ, 1000000.0)
            temps = monitor.thermal_values()
            power = self.read_power()
            cpu_t = temps.get("CPU-therm")
            gpu_t = temps.get("GPU-therm")
            pmic_t = temps.get("PMIC-Die")
            voltage_mv = self.find_power(power, "_voltage_mV")
            current_ma = self.find_power(power, "_current_mA")
            total_mw = self.find_power(power, "_power_mW")

            self.cpu.set(cpu, self.level_color(cpu, 80, 95), cpu)
            self.gpu.set(gpu, self.level_color(gpu, 80, 95), gpu)
            self.cpu_temp.set(cpu_t, self.level_color(cpu_t, 70, 85))
            self.gpu_temp.set(gpu_t, self.level_color(gpu_t, 70, 85))
            self.power.set(None if total_mw is None else total_mw / 1000.0, BLUE, decimals=2)
            if voltage_mv is None:
                self.voltage.set(None, MUTED)
            else:
                vcolor = RED if voltage_mv < 4750 else (YELLOW if voltage_mv < 4850 else GREEN)
                self.voltage.set(voltage_mv / 1000.0, vcolor, decimals=3)
            self.detail.configure(text="GPU 클럭 %s MHz   |   PMIC %s °C   |   입력 전류 %s A" % (
                "--" if freq is None else "%.0f" % freq,
                "--" if pmic_t is None else "%.1f" % pmic_t,
                "--" if current_ma is None else "%.2f" % (current_ma / 1000.0)))

            if voltage_mv is None:
                self.status.configure(text="● 전력 센서 권한 없음", fg=YELLOW)
            elif voltage_mv < 4750:
                self.status.configure(text="● 위험: 입력 전압 낮음", fg=RED)
            elif voltage_mv < 4850:
                self.status.configure(text="● 주의: 입력 전압 낮음", fg=YELLOW)
            elif self.proc and self.proc.poll() is None:
                self.status.configure(text="● 정상 · 부하 시험 중", fg=GREEN)
            else:
                self.status.configure(text="● 정상 · 대기", fg=GREEN)

            if self.proc:
                elapsed = time.monotonic() - self.started
                if cpu is not None and self.cpu_control_path is not None:
                    self.cpu_control_ema = (0.7 * self.cpu_control_ema + 0.3 * cpu)
                    error = self.test_target - self.cpu_control_ema
                    change = max(-8.0, min(8.0, 0.6 * error))
                    self.cpu_stress_duty = max(1, min(100, int(round(self.cpu_stress_duty + change))))
                if gpu is not None and elapsed >= 3:
                    self.gpu_control_ema = 0.8 * self.gpu_control_ema + 0.2 * gpu
                    change = max(-3.0, min(3.0, 0.25 * (self.gpu_target - self.gpu_control_ema)))
                    self.gpu_stress_duty = max(1, min(100, int(round(self.gpu_stress_duty + change))))
                if self.cpu_control_path is not None:
                    self.cpu_control_path.write_text("%d %d" %
                                                     (self.cpu_stress_duty, self.gpu_stress_duty))
                remaining = max(0, int(self.end_at - time.monotonic()))
                self.timer.configure(text="남은 시간 %02d:%02d" % divmod(remaining, 60))
                row = {"event": "sample", "elapsed_s": round(elapsed, 2),
                       "cpu_pct": cpu, "gpu_pct": gpu,
                       "gpu_mhz": freq, "CPU_C": cpu_t, "GPU_C": gpu_t, "PMIC_C": pmic_t}
                row.update(power)
                self._write_log(row)
                hottest = max((v for v in temps.values() if v is not None), default=0)
                if hottest >= 85:
                    self.stop_test("과열 자동 중지")
                elif self.proc.poll() is not None:
                    code = self.proc.returncode
                    if time.monotonic() < self.end_at - 1:
                        self.finish_test("부하 프로세스 조기 종료 (code %s)" % code)
                    else:
                        self.finish_test("시험 완료")
                elif time.monotonic() >= self.end_at:
                    self.finish_test("시험 완료")
        finally:
            self.after(1000, self.sample)

    def close(self):
        if self.proc and self.proc.poll() is None:
            if not messagebox.askyesno("종료", "부하 시험을 중지하고 닫을까요?"):
                return
            self.stop_test("창 닫힘")
        elif self.log_file:
            self._write_log({"event": "창 닫힘"})
            self._close_logs()
        if self.jetson is not None:
            self.jetson.close()
        self.destroy()


if __name__ == "__main__":
    Dashboard().mainloop()
