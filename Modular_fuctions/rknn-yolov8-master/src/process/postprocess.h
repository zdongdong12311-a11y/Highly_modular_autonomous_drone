
#ifndef RK3588_DEMO_POSTPROCESS_H
#define RK3588_DEMO_POSTPROCESS_H

#include <stdint.h>
#include <vector>

int get_top(float *pfProb, float *pfMaxProb, uint32_t *pMaxClass, uint32_t outputCount, uint32_t topNum);

namespace yolo
{
    // Model input size (e.g. 640x640). Must match the exported .rknn model.
    void SetInputShape(int width, int height);

    // num_classes / dfl_len from RKNN output attrs (9-output RKOPT: 3 branches x 3 tensors)
    void SetOutputConfig(int num_outputs, int num_classes, int box_channels);

    int GetConvDetectionResult(float **pBlob, std::vector<float> &DetectiontRects,
                               float objectThreshold = 0.2, float nmsThreshold = 0.25);
    int GetConvDetectionResultInt8(int8_t **pBlob, std::vector<int> &qnt_zp, std::vector<float> &qnt_scale,
                                   std::vector<float> &DetectiontRects,
                                   float objectThreshold = 0.2, float nmsThreshold = 0.25);
}

#endif // RK3588_DEMO_POSTPROCESS_H
