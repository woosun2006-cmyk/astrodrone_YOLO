#include <cuda_runtime.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

__global__ void gpu_stress(float *out, unsigned long long seed) {
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
    float x = 1.000001f + (float)((idx + seed) & 1023) * 0.000001f;
    float y = 0.999999f - (float)(idx & 255) * 0.000001f;
    #pragma unroll 8
    for (int i = 0; i < 1024; ++i) {
        x = fmaf(x, y, 0.000001f);
        y = fmaf(y, 0.999999f, x * 0.0000001f);
    }
    out[idx] = x + y;
}

static std::atomic<bool> running(true);
static volatile double cpu_sink = 0.0;
static std::atomic<int> cpu_target_percent(90);
static std::atomic<int> gpu_target_percent(85);
static std::chrono::steady_clock::time_point cycle_epoch;

static bool wait_if_idle_window(int target_percent, long long cycle_us) {
    if (target_percent >= 100) return false;
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - cycle_epoch).count();
    const long long phase_us = elapsed % cycle_us;
    const long long busy_us = cycle_us * target_percent / 100;
    if (phase_us < busy_us) return false;
    std::this_thread::sleep_for(std::chrono::microseconds(cycle_us - phase_us));
    return true;
}

static void cpu_worker(unsigned int worker) {
    double x = 1.000001 + worker * 0.00001;
    double y = 0.999999;
    while (running.load(std::memory_order_relaxed)) {
        if (wait_if_idle_window(cpu_target_percent.load(std::memory_order_relaxed), 100000)) continue;
        for (int i = 0; i < 64; ++i) {
            x = std::fma(x, y, 0.0000001);
            y = std::fma(y, 0.9999999, x * 0.00000001);
        }
        cpu_sink = x + y;
    }
}

static void cuda_check(cudaError_t status, const char *where) {
    if (status != cudaSuccess) {
        std::fprintf(stderr, "CUDA error at %s: %s\n", where, cudaGetErrorString(status));
        std::exit(2);
    }
}

int main(int argc, char **argv) {
    int seconds = argc > 1 ? std::atoi(argv[1]) : 60;
    int cpu_threads = argc > 2 ? std::atoi(argv[2]) : (int)std::thread::hardware_concurrency();
    cpu_target_percent.store(argc > 3 ? std::atoi(argv[3]) : 90);
    gpu_target_percent.store(argc > 4 ? std::atoi(argv[4]) : cpu_target_percent.load());
    const std::string cpu_control_file = argc > 5 ? argv[5] : "";
    if (seconds < 1 || cpu_threads < 0 || cpu_target_percent.load() < 1 || cpu_target_percent.load() > 100 ||
        gpu_target_percent.load() < 1 || gpu_target_percent.load() > 100) {
        std::fprintf(stderr, "Usage: %s SECONDS [CPU_THREADS] [CPU_DUTY] [GPU_DUTY]\n", argv[0]);
        return 2;
    }

    cudaDeviceProp prop{};
    cuda_check(cudaGetDeviceProperties(&prop, 0), "cudaGetDeviceProperties");
    const int threads = 256;
    const int blocks = prop.multiProcessorCount * 32;
    const size_t count = (size_t)threads * blocks;
    float *device_out = nullptr;
    cuda_check(cudaMalloc(&device_out, count * sizeof(float)), "cudaMalloc");
    cycle_epoch = std::chrono::steady_clock::now();

    std::vector<std::thread> workers;
    workers.reserve(cpu_threads);
    for (int i = 0; i < cpu_threads; ++i) workers.emplace_back(cpu_worker, (unsigned int)i);

    std::thread control_worker;
    if (!cpu_control_file.empty()) {
        control_worker = std::thread([&cpu_control_file]() {
            while (running.load(std::memory_order_relaxed)) {
                std::ifstream input(cpu_control_file);
                int cpu_duty = 0, gpu_duty = 0;
                if (input >> cpu_duty && cpu_duty >= 1 && cpu_duty <= 100)
                    cpu_target_percent.store(cpu_duty, std::memory_order_relaxed);
                if (input >> gpu_duty && gpu_duty >= 1 && gpu_duty <= 100)
                    gpu_target_percent.store(gpu_duty, std::memory_order_relaxed);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }

    std::fprintf(stderr, "Stress started: GPU=%s, CPU threads=%d, CPU duty=%d%%, GPU duty=%d%%, duration=%ds\n",
                 prop.name, cpu_threads, cpu_target_percent.load(), gpu_target_percent.load(), seconds);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    unsigned long long launch = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        // A short GPU duty cycle avoids the large utilization swings caused by
        // the previous 100 ms on/off window.
        if (wait_if_idle_window(gpu_target_percent.load(std::memory_order_relaxed), 10000)) continue;
        // Queue a small batch before synchronizing.  Synchronizing every tiny
        // kernel left the GPU idle while the CPU/driver was scheduled, so the
        // measured GPU load stayed far below the requested duty cycle.
        for (int batch = 0; batch < 4; ++batch) {
            gpu_stress<<<blocks, threads>>>(device_out, launch++);
            cuda_check(cudaGetLastError(), "kernel launch");
        }
        cuda_check(cudaDeviceSynchronize(), "kernel execution");
    }

    running.store(false, std::memory_order_relaxed);
    for (auto &worker : workers) worker.join();
    if (control_worker.joinable()) control_worker.join();
    cuda_check(cudaFree(device_out), "cudaFree");
    std::fprintf(stderr, "Stress finished normally (%llu GPU launches).\n", launch);
    return 0;
}
