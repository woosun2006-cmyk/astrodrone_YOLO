#include "tensorrt_engine.hpp"

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_fp16.h>
#include <sys/stat.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>

namespace {

size_t dtype_elem_size(nvinfer1::DataType t) {
    switch (t) {
        case nvinfer1::DataType::kFLOAT:
            return 4;
        case nvinfer1::DataType::kHALF:
            return 2;
        case nvinfer1::DataType::kINT8:
        case nvinfer1::DataType::kBOOL:
            return 1;
        case nvinfer1::DataType::kINT32:
            return 4;
        default:
            throw std::runtime_error("unsupported TensorRT binding dtype");
    }
}

// Fills `wire` (already sized in bytes) from `src` floats, in whatever
// dtype the binding actually wants - this project's prototype.onnx
// declares images/output0 as tensor(float16) (exported with --half), so
// kFLOAT is the untested-but-supported fallback for a future fp32 export.
void floats_to_wire(const std::vector<float>& src, std::vector<uint8_t>& wire, nvinfer1::DataType dtype) {
    if (dtype == nvinfer1::DataType::kFLOAT) {
        std::memcpy(wire.data(), src.data(), src.size() * sizeof(float));
    } else if (dtype == nvinfer1::DataType::kHALF) {
        __half* out = reinterpret_cast<__half*>(wire.data());
        for (size_t i = 0; i < src.size(); ++i) out[i] = __float2half(src[i]);
    } else {
        throw std::runtime_error("unsupported input binding dtype");
    }
}

void wire_to_floats(const std::vector<uint8_t>& wire, std::vector<float>& dst, nvinfer1::DataType dtype) {
    if (dtype == nvinfer1::DataType::kFLOAT) {
        std::memcpy(dst.data(), wire.data(), dst.size() * sizeof(float));
    } else if (dtype == nvinfer1::DataType::kHALF) {
        const __half* in = reinterpret_cast<const __half*>(wire.data());
        for (size_t i = 0; i < dst.size(); ++i) dst[i] = __half2float(in[i]);
    } else {
        throw std::runtime_error("unsupported output binding dtype");
    }
}

// TensorRT 8.2's object-lifetime API is IObj::destroy() (superseded by
// plain `delete` in newer TensorRT); wrapping it here keeps the
// deprecation warning in one place instead of scattered at every call site.
template <typename T>
void trt_destroy(T* obj) {
    if (!obj) return;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    obj->destroy();
#pragma GCC diagnostic pop
}

class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[TensorRT] " << msg << std::endl;
        }
    }
};

Logger g_logger;

void cuda_check(cudaError_t err, const char* what) {
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(err));
    }
}

// Mirrors yolov5's letterbox(): scales the image to fit inside
// (target_w, target_h) preserving aspect ratio, then centers it on a
// 114-gray canvas of exactly that size. The exported onnx has a fixed
// square input, so unlike yolo_live.py's own (non-square, stride-rounded)
// letterbox used for the .pt path, this one always pads to a perfect
// square to match the engine's fixed binding shape.
struct LetterboxInfo {
    float scale;
    float pad_x, pad_y;
};

cv::Mat letterbox(const cv::Mat& src, int target_w, int target_h, LetterboxInfo& info) {
    float r = std::min(static_cast<float>(target_w) / src.cols, static_cast<float>(target_h) / src.rows);
    int new_w = static_cast<int>(std::round(src.cols * r));
    int new_h = static_cast<int>(std::round(src.rows * r));

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

    float pad_w = static_cast<float>(target_w - new_w) / 2.0f;
    float pad_h = static_cast<float>(target_h - new_h) / 2.0f;
    int top = static_cast<int>(std::round(pad_h - 0.1f));
    int bottom = static_cast<int>(std::round(pad_h + 0.1f));
    int left = static_cast<int>(std::round(pad_w - 0.1f));
    int right = static_cast<int>(std::round(pad_w + 0.1f));

    cv::Mat out;
    cv::copyMakeBorder(resized, out, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    info.scale = r;
    info.pad_x = pad_w;
    info.pad_y = pad_h;
    return out;
}

// Inverse of letterbox(): maps a box from the padded/scaled network-input
// frame back to the original frame's pixel coords, clipped to its bounds.
void scale_box_to_frame(RawBox& b, const LetterboxInfo& info, int frame_w, int frame_h) {
    b.x1 = (b.x1 - info.pad_x) / info.scale;
    b.x2 = (b.x2 - info.pad_x) / info.scale;
    b.y1 = (b.y1 - info.pad_y) / info.scale;
    b.y2 = (b.y2 - info.pad_y) / info.scale;
    b.x1 = std::clamp(b.x1, 0.0f, static_cast<float>(frame_w - 1));
    b.x2 = std::clamp(b.x2, 0.0f, static_cast<float>(frame_w - 1));
    b.y1 = std::clamp(b.y1, 0.0f, static_cast<float>(frame_h - 1));
    b.y2 = std::clamp(b.y2, 0.0f, static_cast<float>(frame_h - 1));
}

float iou(const RawBox& a, const RawBox& b) {
    float ix1 = std::max(a.x1, b.x1), iy1 = std::max(a.y1, b.y1);
    float ix2 = std::min(a.x2, b.x2), iy2 = std::min(a.y2, b.y2);
    float iw = std::max(0.0f, ix2 - ix1), ih = std::max(0.0f, iy2 - iy1);
    float inter = iw * ih;
    float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
    float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
    float uni = area_a + area_b - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}

// Per-class greedy NMS, same semantics as yolov5's non_max_suppression()
// (torchvision.ops.nms per class, sorted by confidence).
std::vector<RawBox> nms(std::vector<RawBox> boxes, float iou_thres) {
    std::sort(boxes.begin(), boxes.end(), [](const RawBox& a, const RawBox& b) { return a.conf > b.conf; });
    std::vector<RawBox> kept;
    std::vector<bool> removed(boxes.size(), false);
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (removed[i]) continue;
        kept.push_back(boxes[i]);
        for (size_t j = i + 1; j < boxes.size(); ++j) {
            if (removed[j] || boxes[j].cls != boxes[i].cls) continue;
            if (iou(boxes[i], boxes[j]) > iou_thres) removed[j] = true;
        }
    }
    return kept;
}

}  // namespace

struct YoloTrt::Impl {
    nvinfer1::IRuntime* runtime = nullptr;
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
};

YoloTrt::YoloTrt(const std::string& onnx_path, const std::string& engine_cache_path, bool fp16)
    : impl_(std::make_unique<Impl>()) {
    build_or_load(onnx_path, engine_cache_path, fp16);
    allocate_buffers();
}

YoloTrt::~YoloTrt() {
    if (dev_input_) cudaFree(dev_input_);
    if (dev_output_) cudaFree(dev_output_);
    if (stream_) cudaStreamDestroy(stream_);
    trt_destroy(impl_->context);
    trt_destroy(impl_->engine);
    trt_destroy(impl_->runtime);
}

void YoloTrt::build_or_load(const std::string& onnx_path, const std::string& engine_cache_path, bool fp16) {
    struct stat onnx_st{}, engine_st{};
    const bool have_engine = stat(engine_cache_path.c_str(), &engine_st) == 0;
    std::vector<char> engine_bytes;
    bool have_cache = have_engine;
    if (have_cache) {
        std::ifstream f(engine_cache_path, std::ios::binary | std::ios::ate);
        if (f) {
            std::streamsize size = f.tellg();
            f.seekg(0, std::ios::beg);
            engine_bytes.resize(static_cast<size_t>(size));
            if (!f.read(engine_bytes.data(), size)) engine_bytes.clear();
        }
        have_cache = !engine_bytes.empty();
    }

    impl_->runtime = nvinfer1::createInferRuntime(g_logger);
    if (!impl_->runtime) throw std::runtime_error("createInferRuntime failed");

    if (have_cache) {
        std::cerr << "[yolo_trt] loading cached engine: " << engine_cache_path << std::endl;
        impl_->engine = impl_->runtime->deserializeCudaEngine(engine_bytes.data(), engine_bytes.size());
        if (!impl_->engine) {
            throw std::runtime_error("failed to deserialize TensorRT engine (original preserved): " +
                                     engine_cache_path);
        }
    }

    if (!impl_->engine) {
        if (stat(onnx_path.c_str(), &onnx_st) != 0) {
            throw std::runtime_error("onnx model not found and no usable engine: " + onnx_path);
        }
        std::cerr << "[yolo_trt] building TensorRT engine from " << onnx_path
                  << " (first run on a new onnx is slow, ~1-2 min on a Jetson Nano)" << std::endl;
        auto* builder = nvinfer1::createInferBuilder(g_logger);
        const auto flag = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        auto* network = builder->createNetworkV2(flag);
        auto* parser = nvonnxparser::createParser(*network, g_logger);

        if (!parser->parseFromFile(onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
            for (int i = 0; i < parser->getNbErrors(); ++i) {
                std::cerr << "[yolo_trt] onnx parse error: " << parser->getError(i)->desc() << std::endl;
            }
            trt_destroy(parser);
            trt_destroy(network);
            trt_destroy(builder);
            throw std::runtime_error("failed to parse " + onnx_path);
        }

        auto* config = builder->createBuilderConfig();
        config->setMaxWorkspaceSize(256ULL * 1024 * 1024);
        if (fp16) {
            if (builder->platformHasFastFp16()) {
                config->setFlag(nvinfer1::BuilderFlag::kFP16);
            } else {
                std::cerr << "[yolo_trt] fp16 requested but not supported on this GPU, staying FP32" << std::endl;
            }
        }

        nvinfer1::IHostMemory* serialized = builder->buildSerializedNetwork(*network, *config);
        if (!serialized) {
            trt_destroy(config);
            trt_destroy(parser);
            trt_destroy(network);
            trt_destroy(builder);
            throw std::runtime_error("TensorRT engine build failed for " + onnx_path);
        }

        impl_->engine = impl_->runtime->deserializeCudaEngine(serialized->data(), serialized->size());

        std::ofstream out(engine_cache_path, std::ios::binary | std::ios::trunc);
        if (out) {
            out.write(reinterpret_cast<const char*>(serialized->data()),
                       static_cast<std::streamsize>(serialized->size()));
            std::cerr << "[yolo_trt] cached engine: " << engine_cache_path << std::endl;
        } else {
            std::cerr << "[yolo_trt] could not write engine cache to " << engine_cache_path << " (continuing anyway)"
                       << std::endl;
        }

        trt_destroy(serialized);
        trt_destroy(config);
        trt_destroy(parser);
        trt_destroy(network);
        trt_destroy(builder);
    }

    if (!impl_->engine) throw std::runtime_error("failed to obtain a CUDA engine for " + onnx_path);

    impl_->context = impl_->engine->createExecutionContext();
    if (!impl_->context) throw std::runtime_error("createExecutionContext failed");

    if (impl_->engine->getNbBindings() != 2) {
        throw std::runtime_error("expected exactly 1 input + 1 output binding, got " +
                                  std::to_string(impl_->engine->getNbBindings()));
    }

    nvinfer1::Dims in_dims = impl_->engine->getBindingDimensions(0);   // [1,3,H,W]
    nvinfer1::Dims out_dims = impl_->engine->getBindingDimensions(1);  // [1,N,5+nc]
    if (in_dims.nbDims != 4 || out_dims.nbDims != 3) {
        throw std::runtime_error("unexpected binding shapes in " + onnx_path);
    }
    input_h_ = in_dims.d[2];
    input_w_ = in_dims.d[3];
    num_boxes_ = out_dims.d[1];
    box_attrs_ = out_dims.d[2];
    num_classes_ = box_attrs_ - 5;
    if (num_classes_ < 1) throw std::runtime_error("model output has too few attributes per box");

    nvinfer1::DataType in_dtype = impl_->engine->getBindingDataType(0);
    nvinfer1::DataType out_dtype = impl_->engine->getBindingDataType(1);
    if (in_dtype != nvinfer1::DataType::kFLOAT && in_dtype != nvinfer1::DataType::kHALF) {
        throw std::runtime_error("unsupported input binding dtype in " + onnx_path);
    }
    if (out_dtype != nvinfer1::DataType::kFLOAT && out_dtype != nvinfer1::DataType::kHALF) {
        throw std::runtime_error("unsupported output binding dtype in " + onnx_path);
    }
    in_dtype_ = static_cast<int32_t>(in_dtype);
    out_dtype_ = static_cast<int32_t>(out_dtype);
    std::cerr << "[yolo_trt] bindings: input " << (in_dtype == nvinfer1::DataType::kHALF ? "float16" : "float32")
              << ", output " << (out_dtype == nvinfer1::DataType::kHALF ? "float16" : "float32") << std::endl;
}

void YoloTrt::allocate_buffers() {
    cuda_check(cudaStreamCreate(&stream_), "cudaStreamCreate");

    size_t input_elems = static_cast<size_t>(1) * 3 * input_h_ * input_w_;
    size_t output_elems = static_cast<size_t>(num_boxes_) * box_attrs_;

    size_t input_bytes = input_elems * dtype_elem_size(static_cast<nvinfer1::DataType>(in_dtype_));
    size_t output_bytes = output_elems * dtype_elem_size(static_cast<nvinfer1::DataType>(out_dtype_));

    cuda_check(cudaMalloc(&dev_input_, input_bytes), "cudaMalloc input");
    cuda_check(cudaMalloc(&dev_output_, output_bytes), "cudaMalloc output");

    host_input_.resize(input_elems);
    host_output_.resize(output_elems);
    input_wire_.resize(input_bytes);
    output_wire_.resize(output_bytes);
}

std::vector<Detection> YoloTrt::infer(const cv::Mat& bgr_frame, float conf_thres, float iou_thres) {
    LetterboxInfo lb{};
    cv::Mat padded = letterbox(bgr_frame, input_w_, input_h_, lb);

    // BGR HWC uint8 -> RGB CHW float32 [0,1], matching yolov5's
    // `im[..., ::-1].transpose(2,0,1) / 255`.
    const int hw = input_h_ * input_w_;
    for (int y = 0; y < input_h_; ++y) {
        const uint8_t* row = padded.ptr<uint8_t>(y);
        for (int x = 0; x < input_w_; ++x) {
            const uint8_t b = row[x * 3 + 0];
            const uint8_t g = row[x * 3 + 1];
            const uint8_t r = row[x * 3 + 2];
            const int idx = y * input_w_ + x;
            host_input_[0 * hw + idx] = r / 255.0f;
            host_input_[1 * hw + idx] = g / 255.0f;
            host_input_[2 * hw + idx] = b / 255.0f;
        }
    }

    floats_to_wire(host_input_, input_wire_, static_cast<nvinfer1::DataType>(in_dtype_));
    cuda_check(
        cudaMemcpyAsync(dev_input_, input_wire_.data(), input_wire_.size(), cudaMemcpyHostToDevice, stream_),
        "H2D copy");

    void* bindings[2] = {dev_input_, dev_output_};
    if (!impl_->context->enqueueV2(bindings, stream_, nullptr)) {
        throw std::runtime_error("TensorRT enqueueV2 failed");
    }

    cuda_check(
        cudaMemcpyAsync(output_wire_.data(), dev_output_, output_wire_.size(), cudaMemcpyDeviceToHost, stream_),
        "D2H copy");
    cuda_check(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
    wire_to_floats(output_wire_, host_output_, static_cast<nvinfer1::DataType>(out_dtype_));

    // Decode: each row is [cx, cy, w, h, obj_conf, cls0_conf, cls1_conf, ...]
    // in network-input pixel space, obj/cls already sigmoided but not yet
    // multiplied together - same convention as yolov5's raw Detect() output
    // consumed by non_max_suppression().
    std::vector<RawBox> candidates;
    candidates.reserve(64);
    for (int i = 0; i < num_boxes_; ++i) {
        const float* row = &host_output_[static_cast<size_t>(i) * box_attrs_];
        float obj = row[4];
        if (obj <= 0.0f) continue;

        int best_cls = 0;
        float best_cls_score = row[5];
        for (int c = 1; c < num_classes_; ++c) {
            if (row[5 + c] > best_cls_score) {
                best_cls_score = row[5 + c];
                best_cls = c;
            }
        }
        float conf = obj * best_cls_score;
        if (conf < conf_thres) continue;

        float cx = row[0], cy = row[1], w = row[2], h = row[3];
        RawBox b{};
        b.x1 = cx - w / 2.0f;
        b.y1 = cy - h / 2.0f;
        b.x2 = cx + w / 2.0f;
        b.y2 = cy + h / 2.0f;
        b.conf = conf;
        b.cls = best_cls;
        candidates.push_back(b);
    }

    std::vector<RawBox> kept = nms(std::move(candidates), iou_thres);

    std::vector<Detection> dets;
    dets.reserve(kept.size());
    for (auto& b : kept) {
        scale_box_to_frame(b, lb, bgr_frame.cols, bgr_frame.rows);
        dets.push_back(Detection{b.x1, b.y1, b.x2, b.y2, b.conf, b.cls});
    }
    return dets;
}
