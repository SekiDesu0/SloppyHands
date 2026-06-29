# SloppyHands

SteamVR driver + companion GUI that makes **Meta Quest controllers appear as Valve Index Knuckles controllers**, with optional **ESP32-based fingertip pressure sensors** for per-finger skeletal tracking.

## Overview

SloppyHands has two components:

| Component | Description |
|-----------|-------------|
| **SloppyHandDriver** | SteamVR driver DLL that creates proxy Knuckles devices, captures Quest controller inputs via vtable hooks, and mirrors pose/inputs to the proxies |
| **SloppyHandJob** | SDL2 + Dear ImGui companion app that receives ESP32 finger sensor data over UDP (SLP1 protocol), fuses it with SteamVR controller inputs, and writes per-finger curl values to the driver via shared memory |

## How it works

1. **SloppyHandDriver** hooks `IVRDriverInput` vtable entries to intercept Quest controller input creation and update calls in real time
2. When virtual controllers are enabled, the driver sets Quest controllers to `OptOut` role (so SteamVR ignores them) and registers proxy Knuckles devices with full Index input profiles
3. The driver mirrors the Quest controller pose (position, rotation, velocity) onto the proxy Knuckles and forwards captured inputs (buttons, trigger, thumbstick, grip, trackpad, thumbrest)
4. **SloppyHandJob** listens on UDP port 4242 for [SLP1 protocol](#slp1-protocol) packets from ESP32 microcontrollers with fingertip pressure sensors (12 capacitive electrode channels per hand)
5. The GUI fuses ESP32 electrode data with controller touch/click sensors (trigger, grip, A, B, thumbstick, thumbrest) into per-finger curl values using a configurable source mapping
6. Curl values are written to Windows shared memory and consumed by the driver, which generates full 31-bone Valve Index hand skeletons using a physics-based hand simulation model
7. Haptic events received by proxy Knuckles are forwarded back to the real Quest controllers via the OpenVR client API
8. When disabled, Quest controllers are restored to their original role and render model

## SLP1 Protocol

A custom binary UDP protocol (port 4242, magic `0x534C5031`) for ESP32 ↔ GUI communication:

| Packet | Direction | Purpose |
|--------|-----------|---------|
| HELLO | ESP32 → GUI | Device announces itself (MAC, device type, channel count, hand hint) |
| WELCOME | GUI → ESP32 | GUI acknowledges and assigns a hand slot |
| DATA | ESP32 → GUI | 12-channel filtered electrode readings (uint16), touch bits, RSSI, uptime |
| KEEPALIVE | GUI → ESP32 | Periodic keepalive (1s interval) |
| BYE | GUI → ESP32 | Graceful shutdown notification |

## Build

**Prerequisites**: Visual Studio 2022, CMake 3.7.1+

### SloppyHandDriver

```powershell
cd SloppyHandDriver
mkdir build; cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

Installing: copy the `driver_sloppyhanddriver/` folder from the build output into `<SteamVR>/drivers/`.

### SloppyHandJob

```powershell
cd SloppyHandJob
mkdir build; cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

SDL2 and Dear ImGui are fetched automatically via CMake FetchContent.

## Dependencies

- **OpenVR** — bundled in `thirdparty/openvr/`
- **SDL2** — fetched via CMake (release-2.30.0)
- **Dear ImGui** — fetched via CMake (v1.90.4) with OpenGL3 + SDL2 backends
- **vrmath** — bundled in `thirdparty/vrmath/` (Valve quaternion/vector math utilities)
- Windows only (vtable patching via `VirtualProtect`, Windows shared memory)

## License

GNU General Public License v3.0 — see [LICENSE](LICENSE).
