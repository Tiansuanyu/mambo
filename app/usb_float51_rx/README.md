# usb_float51_rx

Minimal USB bulk receiver for 51 `float32` control values.

- Board: `robomaster_board_c`
- SYNC ID: `0x0101`
- Payload: `51 * sizeof(float)`
- USB/protocol path: same style as `2026_R2usb_armv2`

The code only receives the array and prints `A/B/X/Y` plus `lx/ly/rx/ry`.

Build:

```sh
/home/xiexiang/zephyrproject/.venv/bin/west build -b robomaster_board_c src/usb_float51_rx -d build/usb_float51_rx -p always -- -DUSER_CACHE_DIR=/home/xiexiang/zephyrproject/zephyr_ws/build/zephyr-cache -DUSE_CCACHE=0
```
