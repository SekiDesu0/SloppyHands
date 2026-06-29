#pragma once
#include <cstdint>

// Shared memory structure for bidirectional communication between
// the SloppyHandJob GUI and the driver_sample DLL.
//
// Driver creates the shared memory (CreateFileMapping).
// GUI opens it (OpenFileMapping).
// Both sides read/write their respective fields.

#pragma pack(push, 1)
struct DriverSharedState
{
    static constexpr uint32_t kMagic   = 0x534C4F50u; // "SLOP"
    static constexpr uint32_t kVersion = 1;
    static constexpr const char* kShmName = "Local\\SloppyHandJob";

    uint32_t magic;
    uint32_t version;

    // ---- GUI -> Driver ----
    uint32_t virtual_enabled;   // 0 = disabled (Quest controllers active), 1 = enabled (proxy Knuckles active)
    float    curls_left[5];     // thumb, index, middle, ring, pinky (0-1)
    float    curls_right[5];
    uint32_t curls_sequence;    // GUI increments when updating curls
    float    pivot_offset_left[3];   // XYZ offset from Quest IMU to Knuckles pivot (meters)
    float    pivot_offset_right[3];

    // ---- Driver -> GUI ----
    uint32_t quest_left_detected;
    uint32_t quest_right_detected;
    uint32_t quest_left_index;   // TrackedDeviceIndex_t for haptics
    uint32_t quest_right_index;
    uint32_t proxy_left_active;
    uint32_t proxy_right_active;

    // Captured Quest inputs (for GUI to fuse with ESP32 data)
    float trigger_left;
    float trigger_right;
    float grip_left;
    float grip_right;

    // Touch states (for thumb curl estimation)
    uint8_t thumbrest_touch_left;
    uint8_t thumbrest_touch_right;
    uint8_t thumbstick_touch_left;
    uint8_t thumbstick_touch_right;
    uint8_t a_touch_left;
    uint8_t a_touch_right;
    uint8_t b_touch_left;
    uint8_t b_touch_right;

    // Click states (press vs touch distinction for fusion)
    uint8_t a_click_left;
    uint8_t a_click_right;
    uint8_t b_click_left;
    uint8_t b_click_right;
    uint8_t thumbstick_click_left;
    uint8_t thumbstick_click_right;
    uint8_t trigger_click_left;
    uint8_t trigger_click_right;
    uint8_t trigger_touch_left;
    uint8_t trigger_touch_right;
    uint8_t grip_touch_left;
    uint8_t grip_touch_right;

    uint32_t input_sequence;    // Driver increments when updating inputs

    // Haptic data (GUI reads this, triggers on Quest via client API)
    float haptic_amplitude_left;
    float haptic_amplitude_right;
    float haptic_duration_left;
    float haptic_duration_right;
    uint32_t haptic_seq_left;    // Driver increments when new left haptic arrives
    uint32_t haptic_seq_right;   // Driver increments when new right haptic arrives
};
#pragma pack(pop)
