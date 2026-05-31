# RKNN runtime library (`librknnrt.so`)

CMake expects the NPU runtime at:

```
librknn_api/aarch64/librknnrt.so
```

## Obtain the library

1. **From the board system** (if rknpu2 is already installed):

   ```bash
   bash scripts/fetch_rknn_runtime.sh
   ```

2. **From Rockchip RKNN SDK** (rknpu2 runtime package for RK3588 / aarch64):

   - Download from [Rockchip rknpu2](https://github.com/rockchip-linux/rknpu2) releases or your board vendor SDK.
   - Copy `runtime/Linux/librknn_api/aarch64/librknnrt.so` into this directory.

3. **Verify**:

   ```bash
   file librknn_api/aarch64/librknnrt.so
   # ELF 64-bit LSB shared object, ARM aarch64
   ```

> This file is not redistributed in the repository due to licensing and SoC-specific binaries.
