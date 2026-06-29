#pragma once

#include "openvr_driver.h"
#include "shared_state.h"

// ---------------------------------------------------------------------------
// Captured input state from Quest controllers
// Populated by vtable hooks, read by proxy devices
// ---------------------------------------------------------------------------
struct CapturedInputs
{
    bool a_click    = false;
    bool a_touch    = false;
    bool b_click    = false;
    bool b_touch    = false;
    bool trigger_click = false;
    bool trigger_touch = false;
    bool thumbstick_click = false;
    bool thumbstick_touch = false;
    bool system_click = false;
    bool grip_touch  = false;
    bool thumbrest_touch = false;

    float trigger_value   = 0.f;
    float thumbstick_x    = 0.f;
    float thumbstick_y    = 0.f;
    float grip_value       = 0.f;
};

class VTableHook
{
public:
    static bool HookIVRDriverInput();
    static void Unhook();
    static bool IsHooked();

    // Shared memory — driver creates it, GUI opens it
    static bool InitSharedMemory();
    static void CloseSharedMemory();

    // Write captured inputs + status to shared memory (called each frame)
    static void WriteSharedState();

    // Read GUI commands from shared memory (virtual_enabled, curls)
    // Returns true if shared memory is valid and GUI is connected
    static bool ReadSharedState();

    // Per-frame: update skeleton, handle enable/disable
    static void RunFrame(vr::IVRDriverInput* pInput);

    // Get captured input state for a given hand role
    static CapturedInputs* GetCapturedInputs(vr::ETrackedControllerRole role);

    // Get the Quest controller's tracked device index for a given role
    static uint32_t GetQuestDeviceIndex(vr::ETrackedControllerRole role);

    // Check if Quest controllers with the given role have been detected
    static bool IsQuestDetected(vr::ETrackedControllerRole role);

    // Set Quest controllers to OptOut role so proxy takes the hand role
    static void SetQuestRoleOptOut();

    // Restore Quest controllers to their original hand role
    static void RestoreQuestRole();

    // Check if GUI wants virtual controllers enabled
    static bool VirtualEnabled();

    // Write proxy active state to shared memory
    static void SetProxyState(bool left, bool right);

    // Store a haptic event received by a proxy device
    static void StoreHaptic(vr::ETrackedControllerRole role, float amplitude, float duration);
};

// Called by proxy device to register its skeleton handle for per-frame updates
void VTableHook_RegisterSkeleton(vr::PropertyContainerHandle_t container,
    vr::VRInputComponentHandle_t handle, vr::ETrackedControllerRole role);
