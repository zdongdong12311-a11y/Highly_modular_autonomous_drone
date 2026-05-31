#undef LOG_TAG
#define LOG_TAG "yolov8_img"

#include <opencv2/opencv.hpp>

#include "task/yolov8.h"
#include "utils/logging.h"
#include "draw/cv_draw.h"

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        NN_LOG_ERROR("Usage: %s <model.rknn> <image.jpg>", argv[0]);
        return -1;
    }

    const char *model_file = argv[1];
    const char *img_file = argv[2];

    cv::Mat img = cv::imread(img_file);
    if (img.empty())
    {
        NN_LOG_ERROR("Failed to read image: %s", img_file);
        return -1;
    }

    NN_LOG_INFO("Source image: %d x %d", img.cols, img.rows);

    Yolov8Custom yolo;
    if (yolo.LoadModel(model_file) != NN_SUCCESS)
    {
        return -1;
    }

    NN_LOG_INFO("Model input: %d x %d", yolo.GetModelInputWidth(), yolo.GetModelInputHeight());

    std::vector<Detection> objects;
    yolo.Run(img, objects);
    DrawDetections(img, objects);

    cv::imwrite("result.jpg", img);
    NN_LOG_INFO("Saved result.jpg (%d x %d)", img.cols, img.rows);

    return 0;
}
