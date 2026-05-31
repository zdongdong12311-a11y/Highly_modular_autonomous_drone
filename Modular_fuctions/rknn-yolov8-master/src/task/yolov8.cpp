#include "yolov8.h"
#include <random>
#include "utils/logging.h"
#include "process/preprocess.h"
#include "process/postprocess.h"

namespace {
// 相机/图像源分辨率（检测框坐标系）；与 RKNN 模型输入尺寸（通常 640）无关
constexpr int kDefaultSourceWidth = 1920;
constexpr int kDefaultSourceHeight = 1080;
}

// define global classes
// COCO 80 class names (must match coco_80_labels_list.txt order)
static std::vector<std::string> g_classes = {
    "car", "blockhouse","H","bridge","tent","tank","redten",};


Yolov8Custom::Yolov8Custom()
{
    engine_ = CreateRKNNEngine();
    input_tensor_.data = nullptr;
    want_float_ = false;
    ready_ = false;
}

Yolov8Custom::~Yolov8Custom()
{
    if (owns_buffers_)
    {
        cleanup_tensors();
    }
}

void Yolov8Custom::cleanup_tensors()
{
    NN_LOG_DEBUG("release input tensor");
    if (input_tensor_.data != nullptr)
    {
        free(input_tensor_.data);
        input_tensor_.data = nullptr;
    }
    NN_LOG_DEBUG("release output tensor");
    for (auto &tensor : output_tensors_)
    {
        if (tensor.data != nullptr)
        {
            free(tensor.data);
            tensor.data = nullptr;
        }
    }
    output_tensors_.clear();
}

void Yolov8Custom::AttachBuffers(tensor_data_s &input, std::vector<tensor_data_s> &outputs)
{
    if (owns_buffers_)
    {
        cleanup_tensors();
        owns_buffers_ = false;
    }
    input_tensor_ = input;
    output_tensors_ = outputs;
}

void Yolov8Custom::ReleaseBuffers()
{
    owns_buffers_ = true;
    input_tensor_.data = nullptr;
    output_tensors_.clear();
}

nn_error_e Yolov8Custom::LoadModel(const char *model_path)
{
    auto ret = engine_->LoadModelFile(model_path);
    if (ret != NN_SUCCESS)
    {
        NN_LOG_ERROR("yolov8 load model file failed");
        return ret;
    }
    // get input tensor
    auto input_shapes = engine_->GetInputShapes();

    // check number of input and n_dims
    if (input_shapes.size() != 1)
    {
        NN_LOG_ERROR("yolov8 input tensor number is not 1, but %ld", input_shapes.size());
        return NN_RKNN_INPUT_ATTR_ERROR;
    }
    nn_tensor_attr_to_cvimg_input_data(input_shapes[0], input_tensor_);
    input_tensor_.data = malloc(input_tensor_.attr.size);

    const int model_w = static_cast<int>(input_tensor_.attr.dims[2]);
    const int model_h = static_cast<int>(input_tensor_.attr.dims[1]);
    yolo::SetInputShape(model_w, model_h);
    NN_LOG_INFO("RKNN model input: %dx%d | source/detect coords: %dx%d (letterbox, no stretch)",
                model_w, model_h, kDefaultSourceWidth, kDefaultSourceHeight);

    auto output_shapes = engine_->GetOutputShapes();
    if (output_shapes.size() != 6 && output_shapes.size() != 9)
    {
        NN_LOG_ERROR("yolov8 output tensor number must be 6 or 9, but %ld", output_shapes.size());
        return NN_RKNN_OUTPUT_ATTR_ERROR;
    }
    if (output_shapes.size() == 9)
    {
        const int num_classes = static_cast<int>(output_shapes[1].dims[1]);
        const int box_channels = static_cast<int>(output_shapes[0].dims[1]);
        yolo::SetOutputConfig(9, num_classes, box_channels);
        NN_LOG_INFO("yolov8 RKOPT 9-output model: classes=%d, box_channels=%d (dfl_len=%d)",
                    num_classes, box_channels, box_channels / 4);
    }
    else
    {
        const int num_classes = static_cast<int>(output_shapes[1].dims[1]);
        yolo::SetOutputConfig(6, num_classes, 4);
        NN_LOG_INFO("yolov8 legacy 6-output model: classes=%d", num_classes);
    }
    if (output_shapes[0].type == NN_TENSOR_FLOAT16)
    {
        want_float_ = true;
        NN_LOG_WARNING("yolov8 output tensor type is float16, want type set to float32");
    }
    for (int i = 0; i < output_shapes.size(); i++)
    {
        tensor_data_s tensor;
        tensor.attr.n_elems = output_shapes[i].n_elems;
        tensor.attr.n_dims = output_shapes[i].n_dims;
        for (int j = 0; j < output_shapes[i].n_dims; j++)
        {
            tensor.attr.dims[j] = output_shapes[i].dims[j];
        }
        // output tensor needs to be float32
        tensor.attr.type = want_float_ ? NN_TENSOR_FLOAT : output_shapes[i].type;
        tensor.attr.index = 0;
        tensor.attr.size = output_shapes[i].n_elems * nn_tensor_type_to_size(tensor.attr.type);
        tensor.data = malloc(tensor.attr.size);
        output_tensors_.push_back(tensor);
        out_zps_.push_back(output_shapes[i].zp);
        out_scales_.push_back(output_shapes[i].scale);
    }

    ready_ = true;
    return NN_SUCCESS;
}

nn_error_e Yolov8Custom::Preprocess(const cv::Mat &img, cv::Mat &image_letterbox)
{
    const int model_w = static_cast<int>(input_tensor_.attr.dims[2]);
    const int model_h = static_cast<int>(input_tensor_.attr.dims[1]);

    // 在源图（如 1920×1080）上做 letterbox，再缩放到模型输入（如 640×640），避免直接拉伸变形
    const float wh_ratio = static_cast<float>(model_w) / static_cast<float>(model_h);
    letterbox_info_ = letterbox(img, image_letterbox, wh_ratio);
    cvimg2tensor(image_letterbox, static_cast<uint32_t>(model_w), static_cast<uint32_t>(model_h),
                 input_tensor_);

    return NN_SUCCESS;
}

nn_error_e Yolov8Custom::Inference()
{
    std::vector<tensor_data_s> inputs;
    inputs.push_back(input_tensor_);
    return engine_->Run(inputs, output_tensors_, want_float_);
}

static const cv::Scalar &get_class_color(int class_id)
{
    static std::vector<cv::Scalar> colors = []()
    {
        std::vector<cv::Scalar> c(80);
        for (int i = 0; i < 80; i++)
        {
            int hue = i * 180 / 80;
            cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 200));
            cv::Mat rgb;
            cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);
            cv::Vec3b p = rgb.at<cv::Vec3b>(0, 0);
            c[i] = cv::Scalar(p[0], p[1], p[2]);
        }
        return c;
    }();
    return colors[class_id % 80];
}

nn_error_e Yolov8Custom::Postprocess(const cv::Mat &img, std::vector<Detection> &objects)
{
    const int out_num = static_cast<int>(output_tensors_.size());
    std::vector<void *> output_data(out_num);
    for (int i = 0; i < out_num; i++)
    {
        output_data[i] = output_tensors_[i].data;
    }
    std::vector<float> DetectiontRects;
    if (want_float_)
    {
        yolo::GetConvDetectionResult(reinterpret_cast<float **>(output_data.data()), DetectiontRects,
                                     object_threshold_, nms_threshold_);
    }
    else
    {
        yolo::GetConvDetectionResultInt8(reinterpret_cast<int8_t **>(output_data.data()), out_zps_, out_scales_,
                                         DetectiontRects, object_threshold_, nms_threshold_);
    }

    int img_width = img.cols;
    int img_height = img.rows;
    for (int i = 0; i < DetectiontRects.size(); i += 6)
    {
        int classId = int(DetectiontRects[i + 0]);
        float conf = DetectiontRects[i + 1];
        int xmin = int(DetectiontRects[i + 2] * float(img_width) + 0.5);
        int ymin = int(DetectiontRects[i + 3] * float(img_height) + 0.5);
        int xmax = int(DetectiontRects[i + 4] * float(img_width) + 0.5);
        int ymax = int(DetectiontRects[i + 5] * float(img_height) + 0.5);
        Detection result;
        result.class_id = classId;
        result.confidence = conf;

        result.color = get_class_color(classId);
        result.className = g_classes[result.class_id];
        result.box = cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin);

        objects.push_back(result);
    }

    return NN_SUCCESS;
}
void letterbox_decode(std::vector<Detection> &objects, bool hor, int pad)
{
    for (auto &obj : objects)
    {
        if (hor)
        {
            obj.box.x -= pad;
        }
        else
        {
            obj.box.y -= pad;
        }
    }
}

nn_error_e Yolov8Custom::Run(const cv::Mat &img, std::vector<Detection> &objects)
{

    // letterbox后的图像
    cv::Mat image_letterbox;
    Preprocess(img, image_letterbox);
    // 推理
    Inference();
    // 后处理
    Postprocess(image_letterbox, objects);

    letterbox_decode(objects, letterbox_info_.hor, letterbox_info_.pad);

    return NN_SUCCESS;
}
