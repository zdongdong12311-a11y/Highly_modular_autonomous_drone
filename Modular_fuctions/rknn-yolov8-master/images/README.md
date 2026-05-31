# Test images

Put sample images here for `yolov8_img` and smoke tests.

Quick start (on the board, with network):

```bash
bash scripts/download_test_assets.sh
# creates images/bus.jpg
```

Manual run:

```bash
./build/yolov8_img weights/yolov8s_rk3588.rknn images/bus.jpg
# writes result.jpg in the project root
```
