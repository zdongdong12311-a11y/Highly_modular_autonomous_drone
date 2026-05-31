# Scripts

| Script | Purpose |
|--------|---------|
| `setup_board.sh` | Install apt deps (OpenCV, ROS Noetic packages), create dirs, check runtime/model |
| `fetch_rknn_runtime.sh` | Copy `librknnrt.so` from common system paths into `librknn_api/aarch64/` |
| `download_test_assets.sh` | Download `images/bus.jpg` for smoke tests |
| `smoke_test.sh` | Run `yolov8_img` and optionally `yolov8_ros_node` after build |

All scripts target **aarch64 Linux on RK3588**. Run from the project root:

```bash
chmod +x scripts/*.sh
bash scripts/setup_board.sh
```
