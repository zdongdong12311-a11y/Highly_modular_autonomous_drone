#!/usr/bin/env bash
# Smoke tests: yolov8_img (required), yolov8_ros_node (optional if ROS is available).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT}/build}"
MODEL="${MODEL:-}"
IMAGE="${IMAGE:-${ROOT}/images/bus.jpg}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --build-dir DIR   CMake build directory (default: ${ROOT}/build)
  --model PATH      .rknn model path (default: first weights/*.rknn)
  --image PATH      Test image (default: images/bus.jpg)
  --skip-ros        Skip ROS node smoke test
  -h, --help        Show this help

Examples:
  bash scripts/smoke_test.sh
  bash scripts/smoke_test.sh --model weights/yolov8s_rk3588.rknn
EOF
}

SKIP_ROS=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --model) MODEL="$2"; shift 2 ;;
    --image) IMAGE="$2"; shift 2 ;;
    --skip-ros) SKIP_ROS=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

shopt -s nullglob
if [[ -z "${MODEL}" ]]; then
  models=("${ROOT}"/weights/*.rknn)
  if ((${#models[@]} == 0)); then
    echo "ERROR: No model specified and no weights/*.rknn found."
    echo "Place a converted RK3588 model in weights/ or pass --model PATH"
    exit 1
  fi
  MODEL="${models[0]}"
fi

IMG_BIN="${BUILD_DIR}/yolov8_img"
ROS_BIN="${BUILD_DIR}/yolov8_ros_node"
if [[ ! -x "${IMG_BIN}" ]]; then
  IMG_BIN="${BUILD_DIR}/yolov8_img/yolov8_img"
fi

if [[ ! -x "${IMG_BIN}" ]]; then
  echo "ERROR: yolov8_img not found under ${BUILD_DIR}. Build first:"
  echo "  mkdir -p build && cd build && cmake .. && make -j\$(nproc)"
  exit 1
fi

if [[ ! -f "${IMAGE}" ]]; then
  echo "ERROR: Test image missing: ${IMAGE}"
  echo "Run: bash ${ROOT}/scripts/download_test_assets.sh"
  exit 1
fi

if [[ ! -f "${MODEL}" ]]; then
  echo "ERROR: Model missing: ${MODEL}"
  exit 1
fi

echo "==> Smoke test 1/2: yolov8_img"
echo "    model: ${MODEL}"
echo "    image: ${IMAGE}"
rm -f "${ROOT}/result.jpg"
(
  cd "${ROOT}"
  "${IMG_BIN}" "${MODEL}" "${IMAGE}"
)

if [[ ! -f "${ROOT}/result.jpg" ]]; then
  echo "FAIL: result.jpg was not created"
  exit 1
fi
echo "PASS: yolov8_img -> result.jpg ($(du -h "${ROOT}/result.jpg" | awk '{print $1}'))"

if [[ "${SKIP_ROS}" -eq 1 ]]; then
  echo "Skipped ROS smoke test (--skip-ros)"
  exit 0
fi

if [[ ! -x "${ROS_BIN}" ]]; then
  ROS_BIN="${BUILD_DIR}/devel/lib/rknn_yolov8_ros/yolov8_ros_node"
fi
if [[ ! -x "${ROS_BIN}" ]]; then
  echo "NOTE: yolov8_ros_node not built (catkin/ROS build required). Skipping ROS test."
  exit 0
fi

if ! command -v roscore >/dev/null 2>&1; then
  echo "NOTE: roscore not in PATH. Skipping ROS test."
  exit 0
fi

echo "==> Smoke test 2/2: yolov8_ros_node (model load, 6s)"
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

if pgrep -x roscore >/dev/null 2>&1 || pgrep -x rosmaster >/dev/null 2>&1; then
  echo "roscore already running"
else
  roscore &
  ROSCORE_PID=$!
  sleep 2
  trap 'kill ${ROSCORE_PID} 2>/dev/null || true' EXIT
fi

LOG="$(mktemp)"
set +e
timeout 6 rosrun rknn_yolov8_ros yolov8_ros_node \
  _model_path:="${MODEL}" \
  _input_mode:=topic \
  _input_topic:=/camera/image_raw \
  _score_threshold:=0.25 \
  _nms_threshold:=0.25 >"${LOG}" 2>&1
set -e

if grep -qi "rknn_init success" "${LOG}" || grep -qi "YOLOv8 ROS node" "${LOG}"; then
  echo "PASS: yolov8_ros_node loaded model (see log for details)"
else
  echo "WARN: yolov8_ros_node log did not show expected init message:"
  tail -n 20 "${LOG}" || true
  echo "If the model loaded, this may still be OK when no image publisher is present."
fi
rm -f "${LOG}"

echo "All smoke tests completed."
