#ifndef RK3588_DEMO_YOLOV8_CUSTOM_H
#define RK3588_DEMO_YOLOV8_CUSTOM_H

#include "engine/engine.h"

#include <memory>

#include <opencv2/opencv.hpp>
#include "process/preprocess.h"
#include "types/yolo_datatype.h"

class Yolov8Custom
{
public:
    Yolov8Custom();
    ~Yolov8Custom();

    nn_error_e LoadModel(const char *model_path);

    nn_error_e Run(const cv::Mat &img, std::vector<Detection> &objects);

    // Exposed for multi-threaded pipeline
    nn_error_e Preprocess(const cv::Mat &img, cv::Mat &image_letterbox);
    nn_error_e Inference();
    nn_error_e Postprocess(const cv::Mat &img, std::vector<Detection> &objects);

    int GetModelInputWidth() const { return static_cast<int>(input_tensor_.attr.dims[2]); }
    int GetModelInputHeight() const { return static_cast<int>(input_tensor_.attr.dims[1]); }

    // Buffer management for pipeline (re-point internal buffers)
    void AttachBuffers(tensor_data_s &input, std::vector<tensor_data_s> &outputs);
    void ReleaseBuffers();

    // Thresholds
    void SetObjectThreshold(float t) { object_threshold_ = t; }
    void SetNmsThreshold(float t) { nms_threshold_ = t; }
    float GetObjectThreshold() const { return object_threshold_; }
    float GetNmsThreshold() const { return nms_threshold_; }

    // Accessors
    tensor_data_s &GetInputTensor() { return input_tensor_; }
    std::vector<tensor_data_s> &GetOutputTensors() { return output_tensors_; }
    LetterBoxInfo &GetLetterBoxInfo() { return letterbox_info_; }
    bool IsWantFloat() const { return want_float_; }
    const std::vector<int32_t> &GetOutZps() const { return out_zps_; }
    const std::vector<float> &GetOutScales() const { return out_scales_; }
    std::shared_ptr<NNEngine> GetEngine() { return engine_; }
    bool IsReady() const { return ready_; }

private:
    void cleanup_tensors();

    bool owns_buffers_ = true;
    bool ready_;
    LetterBoxInfo letterbox_info_;
    tensor_data_s input_tensor_;
    std::vector<tensor_data_s> output_tensors_;
    bool want_float_;
    float object_threshold_ = 0.2;
    float nms_threshold_ = 0.25;
    std::vector<int32_t> out_zps_;
    std::vector<float> out_scales_;
    std::shared_ptr<NNEngine> engine_;
};

#endif // RK3588_DEMO_YOLOV8_CUSTOM_H
