# Model weights (`.rknn`)

## 推荐配置（当前工程默认）

| 项目 | 尺寸 | 说明 |
|------|------|------|
| **RKNN 模型输入** | **640×640** | 标准 YOLOv8s RK3588 量化模型 |
| **相机/识别坐标** | **1920×1080** | `infer_width` / `infer_height`，检测框在此坐标系 |

无需 1080p 专用 `.rknn`。流程：1080p 源图 → **letterbox** → 640×640 推理 → 框映射回 1080p。

示例文件名：

```
weights/yolov8s_rk3588.rknn
```

## 如何获得 640 模型

1. 导出 ONNX：`yolo export model=yolov8s.pt format=onnx imgsz=640`
2. RKNN-Toolkit2 转 RK3588 INT8，`input_size_list=[[3, 640, 640]]`
3. 复制到本目录

参考：[rknn_model_zoo YOLOv8](https://github.com/airockchip/rknn_model_zoo/tree/main/examples/yolov8)

## 验证

```bash
ls -lh weights/*.rknn
bash scripts/smoke_test.sh --model weights/yolov8s_rk3588.rknn
```

## 可选：1080p 输入的 RKNN 模型

若将模型本身做成 1920×1080，NPU 计算量约为 640 的 5 倍，帧率会降到约 5–10 FPS。  
**大多数场景用 640 模型 + 1080p 源图即可**（见根目录 README）。
