
#include "postprocess.h"

#include <string.h>
#include <stdlib.h>

#include <algorithm>

#include "utils/logging.h"

int get_top(float *pfProb, float *pfMaxProb, uint32_t *pMaxClass, uint32_t outputCount, uint32_t topNum)
{
    uint32_t i, j;

#define MAX_TOP_NUM 20
    if (topNum > MAX_TOP_NUM)
        return 0;

    memset(pfMaxProb, 0, sizeof(float) * topNum);
    memset(pMaxClass, 0xff, sizeof(float) * topNum);

    for (j = 0; j < topNum; j++)
    {
        for (i = 0; i < outputCount; i++)
        {
            if ((i == *(pMaxClass + 0)) || (i == *(pMaxClass + 1)) || (i == *(pMaxClass + 2)) || (i == *(pMaxClass + 3)) ||
                (i == *(pMaxClass + 4)))
            {
                continue;
            }

            if (pfProb[i] > *(pfMaxProb + j))
            {
                *(pfMaxProb + j) = pfProb[i];
                *(pMaxClass + j) = i;
            }
        }
    }

    return 1;
}

namespace yolo
{
    typedef struct
    {
        float xmin;
        float ymin;
        float xmax;
        float ymax;
        float score;
        int classId;
    } DetectRect;
    static int input_w = 1920;
    static int input_h = 1080;
    static int headNum = 3;
    static int class_num = 80;
    static int g_num_outputs = 6;
    static int g_dfl_len = 16;
    static int strides[3] = {8, 16, 32};
    static int mapSize[3][2] = {{135, 240}, {68, 120}, {34, 60}};
    static std::vector<float> g_meshgrid;
    static int g_meshgrid_w = 0;
    static int g_meshgrid_h = 0;

    static std::vector<float> GenerateMeshgrid();

    void SetOutputConfig(int num_outputs, int num_classes, int box_channels)
    {
        g_num_outputs = num_outputs;
        if (num_classes > 0)
        {
            class_num = num_classes;
        }
        if (box_channels > 0)
        {
            g_dfl_len = box_channels / 4;
        }
    }

    void SetInputShape(int width, int height)
    {
        if (width <= 0 || height <= 0)
        {
            NN_LOG_ERROR("invalid input shape: %dx%d", width, height);
            return;
        }
        input_w = width;
        input_h = height;
        for (int i = 0; i < headNum; i++)
        {
            mapSize[i][0] = input_h / strides[i];
            mapSize[i][1] = input_w / strides[i];
        }
        g_meshgrid_w = 0;
        g_meshgrid_h = 0;
        NN_LOG_INFO("postprocess input shape: %dx%d (grids: %dx%d, %dx%d, %dx%d)",
                    input_w, input_h,
                    mapSize[0][1], mapSize[0][0],
                    mapSize[1][1], mapSize[1][0],
                    mapSize[2][1], mapSize[2][0]);
    }

    static const std::vector<float> &get_meshgrid()
    {
        if (g_meshgrid_w != input_w || g_meshgrid_h != input_h || g_meshgrid.empty())
        {
            g_meshgrid = GenerateMeshgrid();
            g_meshgrid_w = input_w;
            g_meshgrid_h = input_h;
        }
        return g_meshgrid;
    }
#define ZQ_MAX(a, b) ((a) > (b) ? (a) : (b))
#define ZQ_MIN(a, b) ((a) < (b) ? (a) : (b))
    static inline float fast_exp(float x)
    {
        // return exp(x);
        union
        {
            uint32_t i;
            float f;
        } v;
        v.i = (12102203.1616540672 * x + 1064807160.56887296);
        return v.f;
    }

    float sigmoid(float x)
    {
        return 1 / (1 + fast_exp(-x));
    }

    static inline float IOU(float XMin1, float YMin1, float XMax1, float YMax1, float XMin2, float YMin2, float XMax2, float YMax2)
    {
        float Inter = 0;
        float Total = 0;
        float XMin = 0;
        float YMin = 0;
        float XMax = 0;
        float YMax = 0;
        float Area1 = 0;
        float Area2 = 0;
        float InterWidth = 0;
        float InterHeight = 0;

        XMin = ZQ_MAX(XMin1, XMin2);
        YMin = ZQ_MAX(YMin1, YMin2);
        XMax = ZQ_MIN(XMax1, XMax2);
        YMax = ZQ_MIN(YMax1, YMax2);

        InterWidth = XMax - XMin;
        InterHeight = YMax - YMin;

        InterWidth = (InterWidth >= 0) ? InterWidth : 0;
        InterHeight = (InterHeight >= 0) ? InterHeight : 0;

        Inter = InterWidth * InterHeight;

        Area1 = (XMax1 - XMin1) * (YMax1 - YMin1);
        Area2 = (XMax2 - XMin2) * (YMax2 - YMin2);

        Total = Area1 + Area2 - Inter;

        return float(Inter) / float(Total);
    }

    static float DeQnt2F32(int8_t qnt, int zp, float scale)
    {
        return ((float)qnt - (float)zp) * scale;
    }

    static int8_t QntF32ToAffine(float f32, int32_t zp, float scale)
    {
        float dst_val = (f32 / scale) + static_cast<float>(zp);
        if (dst_val < -128.f)
            return -128;
        if (dst_val > 127.f)
            return 127;
        return static_cast<int8_t>(dst_val);
    }

    static void ComputeDfl(const float *tensor, int dfl_len, float *box)
    {
        for (int b = 0; b < 4; b++)
        {
            float exp_sum = 0.f;
            float acc_sum = 0.f;
            float exp_t[32];
            for (int i = 0; i < dfl_len; i++)
            {
                exp_t[i] = fast_exp(tensor[i + b * dfl_len]);
                exp_sum += exp_t[i];
            }
            for (int i = 0; i < dfl_len; i++)
            {
                acc_sum += (exp_t[i] / exp_sum) * static_cast<float>(i);
            }
            box[b] = acc_sum;
        }
    }

    std::vector<float> GenerateMeshgrid()
    {
        std::vector<float> meshgrid;
        if (headNum == 0)
        {
            NN_LOG_ERROR("=== yolov8 Meshgrid  Generate failed! ");
            exit(-1);
        }

        for (int index = 0; index < headNum; index++)
        {
            for (int i = 0; i < mapSize[index][0]; i++)
            {
                for (int j = 0; j < mapSize[index][1]; j++)
                {
                    meshgrid.push_back(float(j + 0.5));
                    meshgrid.push_back(float(i + 0.5));
                }
            }
        }

        NN_LOG_DEBUG("yolov8 meshgrid generated, cells=%zu", meshgrid.size() / 2);
        return meshgrid;
    }

    static void RunNmsAndPack(std::vector<DetectRect> &detectRects, std::vector<float> &DetectiontRects,
                              float nmsThreshold)
    {
        std::sort(detectRects.begin(), detectRects.end(),
                  [](const DetectRect &a, const DetectRect &b) { return a.score > b.score; });

        NN_LOG_DEBUG("NMS Before num :%ld", detectRects.size());
        for (size_t i = 0; i < detectRects.size(); ++i)
        {
            float xmin1 = detectRects[i].xmin;
            float ymin1 = detectRects[i].ymin;
            float xmax1 = detectRects[i].xmax;
            float ymax1 = detectRects[i].ymax;
            int classId = detectRects[i].classId;
            float score = detectRects[i].score;

            if (classId != -1)
            {
                DetectiontRects.push_back(float(classId));
                DetectiontRects.push_back(score);
                DetectiontRects.push_back(xmin1);
                DetectiontRects.push_back(ymin1);
                DetectiontRects.push_back(xmax1);
                DetectiontRects.push_back(ymax1);

                for (size_t j = i + 1; j < detectRects.size(); ++j)
                {
                    if (detectRects[j].classId == -1)
                        continue;
                    float iou = IOU(xmin1, ymin1, xmax1, ymax1,
                                    detectRects[j].xmin, detectRects[j].ymin,
                                    detectRects[j].xmax, detectRects[j].ymax);
                    if (iou > nmsThreshold)
                    {
                        detectRects[j].classId = -1;
                    }
                }
            }
        }
    }

    static int ProcessNineOutputsInt8(int8_t **pBlob, std::vector<int> &qnt_zp, std::vector<float> &qnt_scale,
                                      std::vector<float> &DetectiontRects,
                                      float objectThreshold, float nmsThreshold)
    {
        constexpr int output_per_branch = 3;
        std::vector<DetectRect> detectRects;

        for (int index = 0; index < headNum; index++)
        {
            const int box_idx = index * output_per_branch + 0;
            const int score_idx = index * output_per_branch + 1;
            const int sum_idx = index * output_per_branch + 2;

            int8_t *box_tensor = pBlob[box_idx];
            int8_t *score_tensor = pBlob[score_idx];
            int8_t *score_sum_tensor = pBlob[sum_idx];

            const int32_t box_zp = qnt_zp[box_idx];
            const float box_scale = qnt_scale[box_idx];
            const int32_t score_zp = qnt_zp[score_idx];
            const float score_scale = qnt_scale[score_idx];
            const int32_t score_sum_zp = qnt_zp[sum_idx];
            const float score_sum_scale = qnt_scale[sum_idx];

            const int grid_h = mapSize[index][0];
            const int grid_w = mapSize[index][1];
            const int grid_len = grid_h * grid_w;
            const int stride = strides[index];
            const int8_t score_thres_i8 = QntF32ToAffine(objectThreshold, score_zp, score_scale);
            const int8_t score_sum_thres_i8 = QntF32ToAffine(objectThreshold, score_sum_zp, score_sum_scale);

            for (int h = 0; h < grid_h; h++)
            {
                for (int w = 0; w < grid_w; w++)
                {
                    int offset = h * grid_w + w;
                    if (score_sum_tensor[offset] < score_sum_thres_i8)
                    {
                        continue;
                    }

                    int max_class_id = -1;
                    int8_t max_score = static_cast<int8_t>(-score_zp);
                    int score_offset = offset;
                    for (int c = 0; c < class_num; c++)
                    {
                        if (score_tensor[score_offset] > score_thres_i8 &&
                            score_tensor[score_offset] > max_score)
                        {
                            max_score = score_tensor[score_offset];
                            max_class_id = c;
                        }
                        score_offset += grid_len;
                    }

                    if (max_score <= score_thres_i8)
                    {
                        continue;
                    }

                    offset = h * grid_w + w;
                    float box[4];
                    float before_dfl[128];
                    for (int k = 0; k < g_dfl_len * 4; k++)
                    {
                        before_dfl[k] = DeQnt2F32(box_tensor[offset], box_zp, box_scale);
                        offset += grid_len;
                    }
                    ComputeDfl(before_dfl, g_dfl_len, box);

                    float x1 = (-box[0] + w + 0.5f) * stride;
                    float y1 = (-box[1] + h + 0.5f) * stride;
                    float x2 = (box[2] + w + 0.5f) * stride;
                    float y2 = (box[3] + h + 0.5f) * stride;

                    x1 = x1 > 0 ? x1 : 0;
                    y1 = y1 > 0 ? y1 : 0;
                    x2 = x2 < input_w ? x2 : input_w;
                    y2 = y2 < input_h ? y2 : input_h;

                    if (x1 < x2 && y1 < y2)
                    {
                        DetectRect temp;
                        temp.xmin = x1 / input_w;
                        temp.ymin = y1 / input_h;
                        temp.xmax = x2 / input_w;
                        temp.ymax = y2 / input_h;
                        temp.classId = max_class_id;
                        temp.score = DeQnt2F32(max_score, score_zp, score_scale);
                        detectRects.push_back(temp);
                    }
                }
            }
        }

        RunNmsAndPack(detectRects, DetectiontRects, nmsThreshold);
        return 0;
    }

    static int ProcessNineOutputsFloat(float **pBlob, std::vector<float> &DetectiontRects,
                                       float objectThreshold, float nmsThreshold)
    {
        constexpr int output_per_branch = 3;
        std::vector<DetectRect> detectRects;

        for (int index = 0; index < headNum; index++)
        {
            const int box_idx = index * output_per_branch + 0;
            const int score_idx = index * output_per_branch + 1;
            const int sum_idx = index * output_per_branch + 2;

            float *box_tensor = pBlob[box_idx];
            float *score_tensor = pBlob[score_idx];
            float *score_sum_tensor = pBlob[sum_idx];

            const int grid_h = mapSize[index][0];
            const int grid_w = mapSize[index][1];
            const int grid_len = grid_h * grid_w;
            const int stride = strides[index];

            for (int h = 0; h < grid_h; h++)
            {
                for (int w = 0; w < grid_w; w++)
                {
                    int offset = h * grid_w + w;
                    if (score_sum_tensor[offset] < objectThreshold)
                    {
                        continue;
                    }

                    int max_class_id = -1;
                    float max_score = 0.f;
                    int score_offset = offset;
                    for (int c = 0; c < class_num; c++)
                    {
                        if (score_tensor[score_offset] > objectThreshold &&
                            score_tensor[score_offset] > max_score)
                        {
                            max_score = score_tensor[score_offset];
                            max_class_id = c;
                        }
                        score_offset += grid_len;
                    }

                    if (max_score <= objectThreshold)
                    {
                        continue;
                    }

                    offset = h * grid_w + w;
                    float box[4];
                    float before_dfl[128];
                    for (int k = 0; k < g_dfl_len * 4; k++)
                    {
                        before_dfl[k] = box_tensor[offset];
                        offset += grid_len;
                    }
                    ComputeDfl(before_dfl, g_dfl_len, box);

                    float x1 = (-box[0] + w + 0.5f) * stride;
                    float y1 = (-box[1] + h + 0.5f) * stride;
                    float x2 = (box[2] + w + 0.5f) * stride;
                    float y2 = (box[3] + h + 0.5f) * stride;

                    x1 = x1 > 0 ? x1 : 0;
                    y1 = y1 > 0 ? y1 : 0;
                    x2 = x2 < input_w ? x2 : input_w;
                    y2 = y2 < input_h ? y2 : input_h;

                    if (x1 < x2 && y1 < y2)
                    {
                        DetectRect temp;
                        temp.xmin = x1 / input_w;
                        temp.ymin = y1 / input_h;
                        temp.xmax = x2 / input_w;
                        temp.ymax = y2 / input_h;
                        temp.classId = max_class_id;
                        temp.score = max_score;
                        detectRects.push_back(temp);
                    }
                }
            }
        }

        RunNmsAndPack(detectRects, DetectiontRects, nmsThreshold);
        return 0;
    }

    // int8版本
    int GetConvDetectionResultInt8(int8_t **pBlob, std::vector<int> &qnt_zp, std::vector<float> &qnt_scale,
                                   std::vector<float> &DetectiontRects,
                                   float objectThreshold, float nmsThreshold)
    {
        if (g_num_outputs == 9)
        {
            return ProcessNineOutputsInt8(pBlob, qnt_zp, qnt_scale, DetectiontRects,
                                          objectThreshold, nmsThreshold);
        }

        const auto &meshgrid = get_meshgrid();
        int ret = 0;

        int gridIndex = -2;
        float xmin = 0, ymin = 0, xmax = 0, ymax = 0;
        float cls_val = 0;
        float cls_max = 0;
        int cls_index = 0;

        int quant_zp_cls = 0, quant_zp_reg = 0;
        float quant_scale_cls = 0, quant_scale_reg = 0;

        DetectRect temp;
        std::vector<DetectRect> detectRects;

        for (int index = 0; index < headNum; index++)
        {
            int8_t *reg = (int8_t *)pBlob[index * 2 + 0];
            int8_t *cls = (int8_t *)pBlob[index * 2 + 1];

            quant_zp_reg = qnt_zp[index * 2 + 0];
            quant_zp_cls = qnt_zp[index * 2 + 1];

            quant_scale_reg = qnt_scale[index * 2 + 0];
            quant_scale_cls = qnt_scale[index * 2 + 1];

            for (int h = 0; h < mapSize[index][0]; h++)
            {
                for (int w = 0; w < mapSize[index][1]; w++)
                {
                    gridIndex += 2;

                    for (int cl = 0; cl < class_num; cl++)
                    {
                        cls_val = sigmoid(
                            DeQnt2F32(cls[cl * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w],
                                      quant_zp_cls, quant_scale_cls));

                        if (0 == cl)
                        {
                            cls_max = cls_val;
                            cls_index = cl;
                        }
                        else
                        {
                            if (cls_val > cls_max)
                            {
                                cls_max = cls_val;
                                cls_index = cl;
                            }
                        }
                    }

                    if (cls_max > objectThreshold)
                    {
                        xmin = (meshgrid[gridIndex + 0] -
                                DeQnt2F32(reg[0 * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w],
                                          quant_zp_reg, quant_scale_reg)) *
                               strides[index];
                        ymin = (meshgrid[gridIndex + 1] -
                                DeQnt2F32(reg[1 * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w],
                                          quant_zp_reg, quant_scale_reg)) *
                               strides[index];
                        xmax = (meshgrid[gridIndex + 0] +
                                DeQnt2F32(reg[2 * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w],
                                          quant_zp_reg, quant_scale_reg)) *
                               strides[index];
                        ymax = (meshgrid[gridIndex + 1] +
                                DeQnt2F32(reg[3 * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w],
                                          quant_zp_reg, quant_scale_reg)) *
                               strides[index];

                        xmin = xmin > 0 ? xmin : 0;
                        ymin = ymin > 0 ? ymin : 0;
                        xmax = xmax < input_w ? xmax : input_w;
                        ymax = ymax < input_h ? ymax : input_h;

                        if (xmin >= 0 && ymin >= 0 && xmax <= input_w && ymax <= input_h)
                        {
                            temp.xmin = xmin / input_w;
                            temp.ymin = ymin / input_h;
                            temp.xmax = xmax / input_w;
                            temp.ymax = ymax / input_h;
                            temp.classId = cls_index;
                            temp.score = cls_max;
                            detectRects.push_back(temp);
                        }
                    }
                }
            }
        }

        std::sort(detectRects.begin(), detectRects.end(),
                  [](DetectRect &Rect1, DetectRect &Rect2) -> bool
                  { return (Rect1.score > Rect2.score); });

        NN_LOG_DEBUG("NMS Before num :%ld", detectRects.size());
        for (int i = 0; i < detectRects.size(); ++i)
        {
            float xmin1 = detectRects[i].xmin;
            float ymin1 = detectRects[i].ymin;
            float xmax1 = detectRects[i].xmax;
            float ymax1 = detectRects[i].ymax;
            int classId = detectRects[i].classId;
            float score = detectRects[i].score;

            if (classId != -1)
            {
                // 将检测结果按照classId、score、xmin1、ymin1、xmax1、ymax1 的格式存放在vector<float>中
                DetectiontRects.push_back(float(classId));
                DetectiontRects.push_back(float(score));
                DetectiontRects.push_back(float(xmin1));
                DetectiontRects.push_back(float(ymin1));
                DetectiontRects.push_back(float(xmax1));
                DetectiontRects.push_back(float(ymax1));

                for (int j = i + 1; j < detectRects.size(); ++j)
                {
                    float xmin2 = detectRects[j].xmin;
                    float ymin2 = detectRects[j].ymin;
                    float xmax2 = detectRects[j].xmax;
                    float ymax2 = detectRects[j].ymax;
                    float iou = IOU(xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2);
                    if (iou > nmsThreshold)
                    {
                        detectRects[j].classId = -1;
                    }
                }
            }
        }

        return ret;
    }
    // 浮点数版本
    int GetConvDetectionResult(float **pBlob, std::vector<float> &DetectiontRects,
                               float objectThreshold, float nmsThreshold)
    {
        if (g_num_outputs == 9)
        {
            return ProcessNineOutputsFloat(pBlob, DetectiontRects, objectThreshold, nmsThreshold);
        }

        const auto &meshgrid = get_meshgrid();
        int ret = 0;

        int gridIndex = -2;
        float xmin = 0, ymin = 0, xmax = 0, ymax = 0;
        float cls_val = 0;
        float cls_max = 0;
        int cls_index = 0;

        DetectRect temp;
        std::vector<DetectRect> detectRects;

        for (int index = 0; index < headNum; index++)
        {
            float *reg = (float *)pBlob[index * 2 + 0];
            float *cls = (float *)pBlob[index * 2 + 1];

            for (int h = 0; h < mapSize[index][0]; h++)
            {
                for (int w = 0; w < mapSize[index][1]; w++)
                {
                    gridIndex += 2;

                    for (int cl = 0; cl < class_num; cl++)
                    {
                        cls_val = sigmoid(
                            cls[cl * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w]);

                        if (0 == cl)
                        {
                            cls_max = cls_val;
                            cls_index = cl;
                        }
                        else
                        {
                            if (cls_val > cls_max)
                            {
                                cls_max = cls_val;
                                cls_index = cl;
                            }
                        }
                    }

                    if (cls_max > objectThreshold)
                    {
                        xmin = (meshgrid[gridIndex + 0] -
                                reg[0 * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w]) *
                               strides[index];
                        ymin = (meshgrid[gridIndex + 1] -
                                reg[1 * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w]) *
                               strides[index];
                        xmax = (meshgrid[gridIndex + 0] +
                                reg[2 * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w]) *
                               strides[index];
                        ymax = (meshgrid[gridIndex + 1] +
                                reg[3 * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w]) *
                               strides[index];

                        xmin = xmin > 0 ? xmin : 0;
                        ymin = ymin > 0 ? ymin : 0;
                        xmax = xmax < input_w ? xmax : input_w;
                        ymax = ymax < input_h ? ymax : input_h;

                        if (xmin >= 0 && ymin >= 0 && xmax <= input_w && ymax <= input_h)
                        {
                            temp.xmin = xmin / input_w;
                            temp.ymin = ymin / input_h;
                            temp.xmax = xmax / input_w;
                            temp.ymax = ymax / input_h;
                            temp.classId = cls_index;
                            temp.score = cls_max;
                            detectRects.push_back(temp);
                        }
                    }
                }
            }
        }

        std::sort(detectRects.begin(), detectRects.end(),
                  [](DetectRect &Rect1, DetectRect &Rect2) -> bool
                  { return (Rect1.score > Rect2.score); });

        NN_LOG_DEBUG("NMS Before num :%ld", detectRects.size());
        for (int i = 0; i < detectRects.size(); ++i)
        {
            float xmin1 = detectRects[i].xmin;
            float ymin1 = detectRects[i].ymin;
            float xmax1 = detectRects[i].xmax;
            float ymax1 = detectRects[i].ymax;
            int classId = detectRects[i].classId;
            float score = detectRects[i].score;

            if (classId != -1)
            {
                // 将检测结果按照classId、score、xmin1、ymin1、xmax1、ymax1 的格式存放在vector<float>中
                DetectiontRects.push_back(float(classId));
                DetectiontRects.push_back(float(score));
                DetectiontRects.push_back(float(xmin1));
                DetectiontRects.push_back(float(ymin1));
                DetectiontRects.push_back(float(xmax1));
                DetectiontRects.push_back(float(ymax1));

                for (int j = i + 1; j < detectRects.size(); ++j)
                {
                    float xmin2 = detectRects[j].xmin;
                    float ymin2 = detectRects[j].ymin;
                    float xmax2 = detectRects[j].xmax;
                    float ymax2 = detectRects[j].ymax;
                    float iou = IOU(xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2);
                    if (iou > nmsThreshold)
                    {
                        detectRects[j].classId = -1;
                    }
                }
            }
        }

        return ret;
    }

}