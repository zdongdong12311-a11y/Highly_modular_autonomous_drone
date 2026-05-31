#!/usr/bin/env bash
# Copy librknnrt.so from common RK3588 system locations into the project tree.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIB_ARCH="${LIB_ARCH:-aarch64}"
DEST="${ROOT}/librknn_api/${LIB_ARCH}/librknnrt.so"

CANDIDATES=(
  "/usr/lib/librknnrt.so"
  "/usr/lib/aarch64-linux-gnu/librknnrt.so"
  "/opt/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"
  "/usr/local/lib/librknnrt.so"
)

if [[ -f "${DEST}" ]]; then
  echo "Already exists: ${DEST}"
  exit 0
fi

mkdir -p "$(dirname "${DEST}")"

for src in "${CANDIDATES[@]}"; do
  if [[ -f "${src}" ]]; then
    cp -v "${src}" "${DEST}"
    echo "Installed RKNN runtime to ${DEST}"
    exit 0
  fi
done

echo "ERROR: librknnrt.so not found in common paths."
echo "Install Rockchip RKNN runtime (rknpu2) on the board, then re-run this script"
echo "or manually copy librknnrt.so to:"
echo "  ${DEST}"
exit 1
