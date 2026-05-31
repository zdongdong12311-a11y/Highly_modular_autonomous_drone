#!/usr/bin/env bash
# Download a small public-domain style test image for smoke tests.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMG_DIR="${ROOT}/images"
IMG="${IMG_DIR}/bus.jpg"
URL="https://ultralytics.com/images/bus.jpg"

mkdir -p "${IMG_DIR}"

if [[ -f "${IMG}" ]]; then
  echo "Test image already exists: ${IMG}"
  exit 0
fi

if ! command -v curl >/dev/null 2>&1 && ! command -v wget >/dev/null 2>&1; then
  echo "Install curl or wget, or manually save a test image to ${IMG}"
  exit 1
fi

echo "Downloading test image to ${IMG}"
if command -v curl >/dev/null 2>&1; then
  curl -fsSL -o "${IMG}" "${URL}"
else
  wget -q -O "${IMG}" "${URL}"
fi

echo "Done: ${IMG}"
