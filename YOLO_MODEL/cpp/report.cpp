#include "report.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <numeric>
#include <optional>
#include <sstream>

namespace {

constexpr int kWidth = 68;

std::string bar(char c) { return std::string(kWidth, c); }

double mean_of(const std::vector<double>& v) { return std::accumulate(v.begin(), v.end(), 0.0) / v.size(); }

double stdev_of(const std::vector<double>& v) {
    double m = mean_of(v);
    double ss = 0.0;
    for (double x : v) ss += (x - m) * (x - m);
    return std::sqrt(ss / (v.size() - 1));  // sample stdev, matches statistics.stdev
}

// Mirrors col(rows, key): collects a field across rows, skipping rows where
// that field wasn't available (Python's try/except ValueError on a missing
// dict key becomes an empty std::optional here).
std::vector<double> col(const std::vector<SampleRow>& rows,
                          const std::function<std::optional<double>(const SampleRow&)>& get) {
    std::vector<double> out;
    out.reserve(rows.size());
    for (const auto& r : rows) {
        if (auto v = get(r)) out.push_back(*v);
    }
    return out;
}

std::string describe(const std::vector<double>& vals, const std::string& unit = "") {
    if (vals.empty()) return "n/a";
    std::ostringstream s;
    s << std::fixed << std::setprecision(2);
    s << "mean " << std::setw(7) << mean_of(vals) << unit << "   ";
    s << "min " << std::setw(7) << *std::min_element(vals.begin(), vals.end()) << unit << "   ";
    s << "max " << std::setw(7) << *std::max_element(vals.begin(), vals.end()) << unit;
    if (vals.size() > 1) {
        s << "   stdev " << std::setw(5) << stdev_of(vals);
    }
    return s.str();
}

std::string py_list_repr(const std::vector<std::string>& names) {
    std::ostringstream s;
    s << "[";
    for (size_t i = 0; i < names.size(); ++i) {
        if (i) s << ", ";
        s << "'" << names[i] << "'";
    }
    s << "]";
    return s.str();
}

}  // namespace

std::string build_report(const std::vector<SampleRow>& rows, const std::string& started_str, double duration,
                          const std::string& weights_name, const std::vector<std::string>& class_names,
                          int img_size) {
    auto cap_fps = col(rows, [](const SampleRow& r) { return std::optional<double>(r.cap_fps); });
    auto infer_fps = col(rows, [](const SampleRow& r) { return std::optional<double>(r.infer_fps); });

    std::vector<std::string> L;
    L.push_back(bar('='));
    L.push_back(" YOLO LOG   " + started_str);
    L.push_back(bar('='));

    {
        std::ostringstream s;
        s << " requested      : " << std::fixed << std::setprecision(0) << duration << " s";
        L.push_back(s.str());
    }
    L.push_back(" weights        : " + weights_name);
    L.push_back(" classes        : " + py_list_repr(class_names));
    L.push_back(" img size       : " + std::to_string(img_size));
    {
        std::ostringstream s;
        s << " conf threshold : " << (rows.empty() ? 0.0 : rows.back().conf_threshold);
        L.push_back(s.str());
    }
    L.push_back(" samples        : " + std::to_string(rows.size()));
    L.push_back("");

    L.push_back(bar('-'));
    L.push_back(" SUMMARY");
    L.push_back(bar('-'));
    L.push_back(" capture fps  " + describe(cap_fps));
    L.push_back(" infer fps    " + describe(infer_fps));
    L.push_back(" latency      " +
                describe(col(rows, [](const SampleRow& r) { return std::optional<double>(r.latency_ms); }), " ms"));
    L.push_back(
        " detections   " +
        describe(col(rows, [](const SampleRow& r) { return std::optional<double>(r.det_count); })));
    L.push_back(" avg conf     " +
                describe(col(rows, [](const SampleRow& r) { return std::optional<double>(r.avg_conf); })));
    L.push_back(" top conf     " +
                describe(col(rows, [](const SampleRow& r) { return std::optional<double>(r.top_conf); })));
    L.push_back(" cpu          " +
                describe(col(rows, [](const SampleRow& r) { return r.sys.cpu_pct; }), " %"));
    L.push_back(" temp         " +
                describe(col(rows, [](const SampleRow& r) { return r.sys.temp_c; }), " C"));
    L.push_back(" clock        " +
                describe(col(rows, [](const SampleRow& r) { return r.sys.clock_mhz; }), " MHz"));
    L.push_back("");

    if (!infer_fps.empty() && !cap_fps.empty()) {
        double mean_cap = mean_of(cap_fps);
        double ratio = mean_cap != 0.0 ? mean_of(infer_fps) / mean_cap : 0.0;
        L.push_back(bar('-'));
        L.push_back(" VERDICT");
        L.push_back(bar('-'));
        if (ratio > 0.9) {
            L.push_back(" inference keeps up with capture - not the bottleneck.");
        } else {
            std::ostringstream s;
            s << " inference runs at " << std::fixed << std::setprecision(0) << (ratio * 100) << "% of capture fps -";
            L.push_back(s.str());
            L.push_back(" the model, not the camera, is the ceiling here.");
            L.push_back(" the input size is fixed by the exported onnx (currently " + std::to_string(img_size) +
                        ") - re-export at a smaller size to raise fps, or try");
            L.push_back(" fp16 and verify accuracy still holds on this GPU.");
        }
        L.push_back("");
    }

    L.push_back(bar('-'));
    L.push_back(" INFER FPS OVER TIME");
    L.push_back(bar('-'));
    double top = infer_fps.empty() ? 1.0 : *std::max_element(infer_fps.begin(), infer_fps.end());
    for (const auto& r : rows) {
        int n = top != 0.0 ? static_cast<int>(std::round(r.infer_fps / top * 40)) : 0;
        n = std::clamp(n, 0, 40);
        std::ostringstream s;
        s << " " << std::fixed << std::setprecision(1) << std::setw(6) << r.elapsed_s << "s "
          << std::setprecision(2) << std::setw(6) << r.infer_fps << " |" << std::string(n, '#')
          << std::string(40 - n, '.') << "|  det=" << r.det_count;
        L.push_back(s.str());
    }
    L.push_back("");
    L.push_back(bar('='));

    std::ostringstream out;
    for (size_t i = 0; i < L.size(); ++i) {
        if (i) out << "\n";
        out << L[i];
    }
    return out.str();
}
