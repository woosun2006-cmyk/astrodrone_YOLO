#pragma once

// Port of yolo_live.py's col()/describe()/report(): the plain-text
// recording report written to YOLO_MODEL/log/<prefix>_MMDD_HHMM.txt.

#include <string>
#include <vector>

#include "sysinfo.hpp"

// One sampler tick, same shape as Python's sample()+row.update(sysinfo())
// dict, plus elapsed_s which is only meaningful once a row is part of a
// recording (see finish_locked()/sampler() in yolo_live.cpp).
struct SampleRow {
    double cap_fps = 0.0;
    double infer_fps = 0.0;
    double latency_ms = 0.0;
    int det_count = 0;
    double avg_conf = 0.0;
    double top_conf = 0.0;
    double conf_threshold = 0.0;
    SysSample sys;
    double elapsed_s = 0.0;
};

// Builds the same "===...=== / YOLO LOG ... / SUMMARY / VERDICT / INFER FPS
// OVER TIME" text block yolo_live.py's report() writes to disk.
std::string build_report(const std::vector<SampleRow>& rows, const std::string& started_str, double duration,
                          const std::string& weights_name, const std::vector<std::string>& class_names,
                          int img_size);
