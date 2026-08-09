#pragma once

// TensorRT/ONNX inference backend for the C++ port of ../yolo_live.py.
//
// Swaps yolo_live.py's `torch.hub` + prototype.pt path for prototype.onnx
// built into a TensorRT engine (prototype.engine, cached next to the onnx
// and rebuilt automatically if the onnx is newer). Everything torch.hub's
// AutoShape did for free in Python - letterbox resize, BGR/RGB + HWC/CHW
// conversion, NMS, box rescale back to the original frame - is implemented
// by hand here, since TensorRT only runs the bare network.

#include <cuda_runtime_api.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "common.hpp"

namespace nvinfer1 {
class IRuntime;
class ICudaEngine;
class IExecutionContext;
enum class DataType : int32_t;
}  // namespace nvinfer1

class YoloTrt {
public:
    // onnx_path: e.g. YOLO_MODEL/prototype.onnx
    // engine_cache_path: e.g. YOLO_MODEL/prototype.engine - reused as-is if
    // its mtime is >= onnx_path's, otherwise rebuilt and overwritten.
    // fp16: see yolo_live.py's HALF constant - this project measured FP16
    // collapsing mAP50 0.995 -> 0.659 on a GTX 1660 SUPER and left it
    // unverified on Jetson hardware, so this defaults to false; only flip it
    // after re-checking accuracy on this GPU.
    YoloTrt(const std::string& onnx_path, const std::string& engine_cache_path, bool fp16);
    ~YoloTrt();

    YoloTrt(const YoloTrt&) = delete;
    YoloTrt& operator=(const YoloTrt&) = delete;

    // Runs one inference pass on a BGR frame (as produced by cv::VideoCapture)
    // and returns NMS-filtered detections in that frame's own pixel coords.
    std::vector<Detection> infer(const cv::Mat& bgr_frame, float conf_thres, float iou_thres);

    int input_w() const { return input_w_; }
    int input_h() const { return input_h_; }
    int num_classes() const { return num_classes_; }

private:
    void build_or_load(const std::string& onnx_path, const std::string& engine_cache_path, bool fp16);
    void allocate_buffers();

    struct Impl;
    std::unique_ptr<Impl> impl_;

    int input_w_ = 0, input_h_ = 0;
    int num_boxes_ = 0;    // e.g. 6300
    int box_attrs_ = 0;    // 5 + num_classes
    int num_classes_ = 0;

    // this project's prototype.onnx declares both images/output0 as
    // tensor(float16) (it was exported with --half) - the binding's wire
    // format is whatever the onnx graph says, independent of the fp16 ctor
    // arg above (which only toggles TensorRT's *internal* kernel precision).
    // in_dtype_/out_dtype_ hold nvinfer1::DataType, kept as int32_t here so
    // this header doesn't need to pull in NvInferRuntimeCommon.h's full enum.
    int32_t in_dtype_ = 0;
    int32_t out_dtype_ = 0;

    void* dev_input_ = nullptr;
    void* dev_output_ = nullptr;
    cudaStream_t stream_ = nullptr;

    // Always-float32 working buffers used by preprocessing/decode math.
    std::vector<float> host_input_;
    std::vector<float> host_output_;
    // Wire-format staging buffers, only populated/used when in_/out_dtype_
    // is kHALF; sized in bytes since the element size depends on the dtype.
    std::vector<uint8_t> input_wire_;
    std::vector<uint8_t> output_wire_;
};
