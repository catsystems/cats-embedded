# Simulator API

The generated `gs-sim.js` module exports these C ABI entrypoints. Strings are
UTF-8 JSON or button names; framebuffer bytes are the 12,000 packed bytes of
the 400×240 `GFXcanvas1` buffer.

```text
gs_reset()
gs_press(button), gs_release(button), gs_hold(button, milliseconds)
gs_advance(milliseconds)
gs_set_link_json(link, json)
gs_set_navigation_json(json), gs_set_sensor_json(json)
gs_set_device_status_json(json), gs_set_configuration_json(json)
gs_set_logs_json(json), gs_load_replay_json(json)
gs_snapshot_json()
gs_framebuffer(), gs_framebuffer_size(), gs_framebuffer_revision()
```

The browser host calls only these functions and paints the returned one-bit
buffer with nearest-neighbor scaling. State identifiers and action names are
kept in `HmiSnapshot`/`PlatformAction` in `simulator/hmi_controller.hpp`.
