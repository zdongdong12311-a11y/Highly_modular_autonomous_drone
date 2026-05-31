#!/usr/bin/env bash
# RK3588 board setup: system packages, directory layout, runtime library checks.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIB_ARCH="${LIB_ARCH:-aarch64}"
RKNN_SO="${ROOT}/librknn_api/${LIB_ARCH}/librknnrt.so"

echo "==> rknn-yolov8 board setup"
echo "    Project root: ${ROOT}"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "WARN: Expected aarch64 (RK3588). Current: $(uname -m)"
  echo "      Build and run on the board or use a cross-compile toolchain."
fi

echo "==> Creating asset directories"
mkdir -p "${ROOT}/weights" "${ROOT}/images" "${ROOT}/videos"
mkdir -p "${ROOT}/librknn_api/${LIB_ARCH}"
mkdir -p "${ROOT}/build"

echo "==> Installing system dependencies"
if command -v apt-get >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y \
    build-essential cmake pkg-config \
    libopencv-dev

  if [[ -f /opt/ros/noetic/setup.bash ]]; then
    echo "==> Detected ROS Noetic; installing ROS packages"
    sudo apt-get install -y \
      ros-noetic-vision-msgs \
      ros-noetic-cv-bridge \
      ros-noetic-image-transport \
      ros-noetic-sensor-msgs
  else
    echo "NOTE: ROS Noetic not found. Skip ROS apt packages or install ROS first."
  fi
else
  echo "NOTE: apt-get not found; install OpenCV and ROS dependencies manually."
fi

echo "==> Checking RKNN runtime library"
if [[ -f "${RKNN_SO}" ]]; then
  echo "OK: ${RKNN_SO}"
else
  echo "MISSING: ${RKNN_SO}"
  echo "Try copying from the board SDK or system:"
  echo "  bash ${ROOT}/scripts/fetch_rknn_runtime.sh"
  echo "See: ${ROOT}/librknn_api/aarch64/README.md"
fi

echo "==> Checking model weights"
shopt -s nullglob
RKNN_MODELS=("${ROOT}"/weights/*.rknn)
if ((${#RKNN_MODELS[@]} > 0)); then
  echo "OK: found ${#RKNN_MODELS[@]} .rknn model(s) in weights/"
else
  echo "MISSING: no weights/*.rknn"
  echo "See: ${ROOT}/weights/README.md"
fi

echo "==> Setup finished"
echo "Next:"
echo "  1. Place librknnrt.so and a .rknn model (see README files under librknn_api/ and weights/)"
echo "  2. bash ${ROOT}/scripts/download_test_assets.sh   # optional test image"
echo "  3. mkdir -p build && cd build && cmake .. && make -j\$(nproc)"
echo "  4. bash ${ROOT}/scripts/smoke_test.sh"
