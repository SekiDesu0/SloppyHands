#include "vtable_hook.h"
#include "hand_simulation.h"
#include "openvr_driver.h"
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <unordered_map>
#include <unordered_set>

// ---------------------------------------------------------------------------
// Hook state
// ---------------------------------------------------------------------------
static bool g_hooked = false;
static void** g_vtable_ptr = nullptr;

static void* g_orig_create_bool = nullptr;   // idx 0
static void* g_orig_update_bool = nullptr;   // idx 1
static void* g_orig_create_scalar = nullptr; // idx 2
static void* g_orig_update_scalar = nullptr; // idx 3

// ---------------------------------------------------------------------------
// Container -> device info map
// ---------------------------------------------------------------------------
struct DeviceInfo
{
    vr::TrackedDeviceIndex_t index;
    char serial[256];
    char model[256];
    char render_model[256];  // original render model name (for hide/restore)
    vr::ETrackedControllerRole role;
};
static std::unordered_map<vr::PropertyContainerHandle_t, DeviceInfo> g_container_map;

// Quest device indices by role
static uint32_t g_quest_index_left  = vr::k_unTrackedDeviceIndexInvalid;
static uint32_t g_quest_index_right = vr::k_unTrackedDeviceIndexInvalid;

// Track which Quest containers we've set to OptOut
static std::unordered_set<vr::PropertyContainerHandle_t> g_optout_set;

// ---------------------------------------------------------------------------
// Captured input state (two hands)
// ---------------------------------------------------------------------------
static CapturedInputs g_captured[2]; // [0]=LeftHand, [1]=RightHand

// Handle -> (path string, role index 0=left 1=right)
struct HandleInfo
{
    std::string path;
    int role_idx; // 0=left, 1=right
};
static std::unordered_map<vr::VRInputComponentHandle_t, HandleInfo> g_handle_info;

// Role helpers
static int RoleToIdx(vr::ETrackedControllerRole role)
{
    if (role == vr::TrackedControllerRole_LeftHand)  return 0;
    if (role == vr::TrackedControllerRole_RightHand) return 1;
    return -1;
}

static const char* RoleToString(vr::ETrackedControllerRole role)
{
    switch (role)
    {
    case vr::TrackedControllerRole_Invalid:    return "Invalid";
    case vr::TrackedControllerRole_LeftHand:   return "LeftHand";
    case vr::TrackedControllerRole_RightHand:  return "RightHand";
    case vr::TrackedControllerRole_OptOut:     return "OptOut";
    case vr::TrackedControllerRole_Treadmill:  return "Treadmill";
    case vr::TrackedControllerRole_Stylus:     return "Stylus";
    default:                                   return "Unknown";
    }
}

// ---------------------------------------------------------------------------
// Update captured inputs from a path + value
// ---------------------------------------------------------------------------
static void CaptureBoolean(int role_idx, const char* path, bool value)
{
    if (role_idx < 0 || role_idx > 1) return;
    CapturedInputs& c = g_captured[role_idx];

    if      (strcmp(path, "/input/a/click") == 0)         c.a_click = value;
    else if (strcmp(path, "/input/a/touch") == 0)         c.a_touch = value;
    else if (strcmp(path, "/input/b/click") == 0)         c.b_click = value;
    else if (strcmp(path, "/input/b/touch") == 0)         c.b_touch = value;
    else if (strcmp(path, "/input/trigger/click") == 0)   c.trigger_click = value;
    else if (strcmp(path, "/input/trigger/touch") == 0)   c.trigger_touch = value;
    else if (strcmp(path, "/input/joystick/click") == 0)  c.thumbstick_click = value; // joystick -> thumbstick
    else if (strcmp(path, "/input/joystick/touch") == 0)  c.thumbstick_touch = value;
    else if (strcmp(path, "/input/thumbstick/click") == 0) c.thumbstick_click = value;
    else if (strcmp(path, "/input/thumbstick/touch") == 0) c.thumbstick_touch = value;
    else if (strcmp(path, "/input/system/click") == 0)    c.system_click = value;
    else if (strcmp(path, "/input/x/click") == 0 && role_idx == 0) c.a_click = value; // left: x->a
    else if (strcmp(path, "/input/x/touch") == 0 && role_idx == 0) c.a_touch = value;
    else if (strcmp(path, "/input/y/click") == 0 && role_idx == 0) c.b_click = value; // left: y->b
    else if (strcmp(path, "/input/y/touch") == 0 && role_idx == 0) c.b_touch = value;
    else if (strcmp(path, "/input/grip/touch") == 0)      c.grip_touch = value;
    else if (strcmp(path, "/input/thumbrest/touch") == 0) c.thumbrest_touch = value;
}

static void CaptureScalar(int role_idx, const char* path, float value)
{
    if (role_idx < 0 || role_idx > 1) return;
    CapturedInputs& c = g_captured[role_idx];

    if      (strcmp(path, "/input/trigger/value") == 0)   c.trigger_value = value;
    else if (strcmp(path, "/input/joystick/x") == 0)       c.thumbstick_x = value;  // joystick -> thumbstick
    else if (strcmp(path, "/input/joystick/y") == 0)       c.thumbstick_y = value;
    else if (strcmp(path, "/input/thumbstick/x") == 0)      c.thumbstick_x = value;
    else if (strcmp(path, "/input/thumbstick/y") == 0)      c.thumbstick_y = value;
    else if (strcmp(path, "/input/grip/value") == 0)        c.grip_value = value;
    else if (strcmp(path, "/input/grip/force") == 0)        c.grip_value = value;
}

// ---------------------------------------------------------------------------
// Rescan containers
// ---------------------------------------------------------------------------
void VTableHook::RescanContainers()
{
    vr::IVRProperties* props_raw = vr::VRPropertiesRaw();
    if (!props_raw) return;

    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
    {
        vr::PropertyContainerHandle_t container = props_raw->TrackedDeviceToPropertyContainer(i);
        if (container == 0) continue;
        if (g_container_map.find(container) != g_container_map.end()) continue;

        char serial[256] = {};
        vr::ETrackedPropertyError err;
        vr::VRProperties()->GetStringProperty(container, vr::Prop_SerialNumber_String, serial, sizeof(serial), &err);
        if (err != vr::TrackedProp_Success || serial[0] == '\0') continue;

        char model[256] = {};
        vr::VRProperties()->GetStringProperty(container, vr::Prop_ModelNumber_String, model, sizeof(model), &err);

        char render_model[256] = {};
        vr::VRProperties()->GetStringProperty(container, vr::Prop_RenderModelName_String, render_model, sizeof(render_model), &err);

        int32_t role_int = vr::VRProperties()->GetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, &err);
        vr::ETrackedControllerRole role = (err == vr::TrackedProp_Success)
            ? static_cast<vr::ETrackedControllerRole>(role_int)
            : vr::TrackedControllerRole_Invalid;

        DeviceInfo info;
        info.index = i;
        info.role = role;
        strcpy_s(info.serial, serial);
        strcpy_s(info.model, model);
        strcpy_s(info.render_model, render_model);
        g_container_map[container] = info;

        // Track Quest controller indices
        if (strstr(model, "Quest") != nullptr)
        {
            if (role == vr::TrackedControllerRole_LeftHand)  g_quest_index_left = i;
            if (role == vr::TrackedControllerRole_RightHand) g_quest_index_right = i;
        }

        char buf[512];
        sprintf_s(buf, sizeof(buf),
            "[ContainerMap] NEW container=0x%llx serial=\"%s\" index=%u role=%s model=\"%s\"",
            (unsigned long long)container, serial, (unsigned int)i,
            RoleToString(role), model[0] ? model : "(unknown)");
        vr::VRDriverLog()->Log(buf);
    }
}

// ---------------------------------------------------------------------------
// Skeleton handle registry (container -> skeleton) — for proxy skeleton update
// ---------------------------------------------------------------------------
struct SkeletonEntry
{
    vr::VRInputComponentHandle_t handle;
    vr::ETrackedControllerRole role;
};
static std::unordered_map<vr::PropertyContainerHandle_t, SkeletonEntry> g_skeleton_map;

// ---------------------------------------------------------------------------
// Shared memory — bidirectional communication with SloppyHandJob GUI
// ---------------------------------------------------------------------------
static HANDLE g_shm_file = nullptr;
static DriverSharedState* g_shm_state = nullptr;
static uint32_t g_shm_last_curls_seq = 0;
static uint32_t g_input_seq = 0;
static bool g_virtual_enabled = false;       // current enabled state
static bool g_gui_connected = false;         // GUI is writing to shared memory

static MyHandSimulation g_hand_sim;

// Haptic data captured from proxy events
static float g_haptic_amp[2] = {};      // [0]=left, [1]=right
static float g_haptic_dur[2] = {};
static uint32_t g_haptic_seq[2] = {};   // per-hand sequence

// ---------------------------------------------------------------------------
// Hook typedefs
// ---------------------------------------------------------------------------
using CreateBooleanComponent_t = vr::EVRInputError(__fastcall*)(
    vr::IVRDriverInput*, vr::PropertyContainerHandle_t, const char*, vr::VRInputComponentHandle_t*);
using UpdateBooleanComponent_t = vr::EVRInputError(__fastcall*)(
    vr::IVRDriverInput*, vr::VRInputComponentHandle_t, bool, double);
using CreateScalarComponent_t = vr::EVRInputError(__fastcall*)(
    vr::IVRDriverInput*, vr::PropertyContainerHandle_t, const char*, vr::VRInputComponentHandle_t*,
    vr::EVRScalarType, vr::EVRScalarUnits);
using UpdateScalarComponent_t = vr::EVRInputError(__fastcall*)(
    vr::IVRDriverInput*, vr::VRInputComponentHandle_t, float, double);

// ---------------------------------------------------------------------------
// Hook: CreateBooleanComponent (idx 0)
// ---------------------------------------------------------------------------
static vr::EVRInputError __fastcall HookedCreateBooleanComponent(
    vr::IVRDriverInput* pThis,
    vr::PropertyContainerHandle_t ulContainer,
    const char* pchName,
    vr::VRInputComponentHandle_t* pHandle)
{
    auto original = reinterpret_cast<CreateBooleanComponent_t>(g_orig_create_bool);
    vr::EVRInputError result = original(pThis, ulContainer, pchName, pHandle);

    if (result == vr::VRInputError_None && pHandle && *pHandle != 0)
    {
        // Track handle
        auto it = g_container_map.find(ulContainer);
        if (it == g_container_map.end())
        {
            VTableHook::RescanContainers();
            it = g_container_map.find(ulContainer);
        }
        if (it != g_container_map.end() && strstr(it->second.model, "Quest") != nullptr)
        {
            int ridx = RoleToIdx(it->second.role);
            if (ridx >= 0)
            {
                g_handle_info[*pHandle] = { pchName ? pchName : "", ridx };

                char buf[512];
                sprintf_s(buf, sizeof(buf), "[CreateBool] role=%s path=\"%s\" handle=0x%llx",
                    RoleToString(it->second.role), pchName ? pchName : "(null)",
                    (unsigned long long)*pHandle);
                vr::VRDriverLog()->Log(buf);
            }
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// Hook: UpdateBooleanComponent (idx 1)
// ---------------------------------------------------------------------------
static vr::EVRInputError __fastcall HookedUpdateBooleanComponent(
    vr::IVRDriverInput* pThis,
    vr::VRInputComponentHandle_t ulComponent,
    bool bNewValue,
    double fTimeOffset)
{
    auto original = reinterpret_cast<UpdateBooleanComponent_t>(g_orig_update_bool);
    vr::EVRInputError result = original(pThis, ulComponent, bNewValue, fTimeOffset);

    auto it = g_handle_info.find(ulComponent);
    if (it != g_handle_info.end())
    {
        CaptureBoolean(it->second.role_idx, it->second.path.c_str(), bNewValue);
    }

    return result;
}

// ---------------------------------------------------------------------------
// Hook: CreateScalarComponent (idx 2)
// ---------------------------------------------------------------------------
static vr::EVRInputError __fastcall HookedCreateScalarComponent(
    vr::IVRDriverInput* pThis,
    vr::PropertyContainerHandle_t ulContainer,
    const char* pchName,
    vr::VRInputComponentHandle_t* pHandle,
    vr::EVRScalarType eType,
    vr::EVRScalarUnits eUnits)
{
    auto original = reinterpret_cast<CreateScalarComponent_t>(g_orig_create_scalar);
    vr::EVRInputError result = original(pThis, ulContainer, pchName, pHandle, eType, eUnits);

    if (result == vr::VRInputError_None && pHandle && *pHandle != 0)
    {
        auto it = g_container_map.find(ulContainer);
        if (it == g_container_map.end())
        {
            VTableHook::RescanContainers();
            it = g_container_map.find(ulContainer);
        }
        if (it != g_container_map.end() && strstr(it->second.model, "Quest") != nullptr)
        {
            int ridx = RoleToIdx(it->second.role);
            if (ridx >= 0)
            {
                g_handle_info[*pHandle] = { pchName ? pchName : "", ridx };

                char buf[512];
                sprintf_s(buf, sizeof(buf), "[CreateScalar] role=%s path=\"%s\" handle=0x%llx",
                    RoleToString(it->second.role), pchName ? pchName : "(null)",
                    (unsigned long long)*pHandle);
                vr::VRDriverLog()->Log(buf);
            }
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// Hook: UpdateScalarComponent (idx 3)
// ---------------------------------------------------------------------------
static vr::EVRInputError __fastcall HookedUpdateScalarComponent(
    vr::IVRDriverInput* pThis,
    vr::VRInputComponentHandle_t ulComponent,
    float fNewValue,
    double fTimeOffset)
{
    auto original = reinterpret_cast<UpdateScalarComponent_t>(g_orig_update_scalar);
    vr::EVRInputError result = original(pThis, ulComponent, fNewValue, fTimeOffset);

    auto it = g_handle_info.find(ulComponent);
    if (it != g_handle_info.end())
    {
        CaptureScalar(it->second.role_idx, it->second.path.c_str(), fNewValue);
    }

    return result;
}

// ===========================================================================
// Public API
// ===========================================================================

bool VTableHook::HookIVRDriverInput()
{
    if (g_hooked) return true;

    vr::IVRDriverInput* input = vr::VRDriverInput();
    if (!input) return false;

    g_vtable_ptr = *(void***)input;

    g_orig_create_bool   = g_vtable_ptr[0];
    g_orig_update_bool   = g_vtable_ptr[1];
    g_orig_create_scalar = g_vtable_ptr[2];
    g_orig_update_scalar = g_vtable_ptr[3];

    auto install_hook = [](int idx, void* hook_fn) -> bool {
        void** target = &g_vtable_ptr[idx];
        DWORD old;
        if (!VirtualProtect(target, sizeof(void*), PAGE_READWRITE, &old))
            return false;
        *target = hook_fn;
        DWORD dummy;
        VirtualProtect(target, sizeof(void*), old, &dummy);
        return true;
    };

    if (!install_hook(0, reinterpret_cast<void*>(HookedCreateBooleanComponent))) return false;
    if (!install_hook(1, reinterpret_cast<void*>(HookedUpdateBooleanComponent))) return false;
    if (!install_hook(2, reinterpret_cast<void*>(HookedCreateScalarComponent)))   return false;
    if (!install_hook(3, reinterpret_cast<void*>(HookedUpdateScalarComponent)))   return false;

    VTableHook::RescanContainers();

    g_hooked = true;
    vr::VRDriverLog()->Log("[VTableHook] Hooks installed on indices 0-3 (input capture mode)");
    return true;
}

void VTableHook::Unhook()
{
    if (!g_hooked || !g_vtable_ptr) return;

    CloseSharedMemory();

    auto restore = [](int idx, void* orig) {
        if (!orig) return;
        void** target = &g_vtable_ptr[idx];
        DWORD old;
        VirtualProtect(target, sizeof(void*), PAGE_READWRITE, &old);
        *target = orig;
        DWORD dummy;
        VirtualProtect(target, sizeof(void*), old, &dummy);
    };

    restore(0, g_orig_create_bool);
    restore(1, g_orig_update_bool);
    restore(2, g_orig_create_scalar);
    restore(3, g_orig_update_scalar);

    g_orig_create_bool   = nullptr;
    g_orig_update_bool   = nullptr;
    g_orig_create_scalar = nullptr;
    g_orig_update_scalar = nullptr;
    g_container_map.clear();
    g_skeleton_map.clear();
    g_handle_info.clear();
    g_optout_set.clear();
    g_virtual_enabled = false;

    g_hooked = false;
    vr::VRDriverLog()->Log("[VTableHook] All hooks removed");
}

bool VTableHook::IsHooked() { return g_hooked; }

CapturedInputs* VTableHook::GetCapturedInputs(vr::ETrackedControllerRole role)
{
    int idx = RoleToIdx(role);
    if (idx < 0) return nullptr;
    return &g_captured[idx];
}

uint32_t VTableHook::GetQuestDeviceIndex(vr::ETrackedControllerRole role)
{
    if (role == vr::TrackedControllerRole_LeftHand)  return g_quest_index_left;
    if (role == vr::TrackedControllerRole_RightHand) return g_quest_index_right;
    return vr::k_unTrackedDeviceIndexInvalid;
}

bool VTableHook::IsQuestDetected(vr::ETrackedControllerRole role)
{
    return GetQuestDeviceIndex(role) != vr::k_unTrackedDeviceIndexInvalid;
}

void VTableHook::SetQuestRoleOptOut()
{
    vr::CVRPropertyHelpers* props = vr::VRProperties();
    if (!props) return;

    for (auto it = g_container_map.begin(); it != g_container_map.end(); ++it)
    {
        vr::PropertyContainerHandle_t container = it->first;
        const DeviceInfo& info = it->second;

        if (g_optout_set.find(container) != g_optout_set.end())
            continue;
        if (strstr(info.model, "Quest") == nullptr)
            continue;

        // Set role to OptOut so SteamVR doesn't use Quest for hand role
        props->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_OptOut);
        // Clear render model to hide the Quest controller visually
        props->SetStringProperty(container, vr::Prop_RenderModelName_String, "");
        g_optout_set.insert(container);

        char buf[256];
        sprintf_s(buf, sizeof(buf), "[OptOut] container=0x%llx serial=\"%s\" -> OptOut + hidden",
            (unsigned long long)container, info.serial);
        vr::VRDriverLog()->Log(buf);
    }
}

void VTableHook::RestoreQuestRole()
{
    vr::CVRPropertyHelpers* props = vr::VRProperties();
    if (!props) return;

    for (auto it = g_container_map.begin(); it != g_container_map.end(); ++it)
    {
        vr::PropertyContainerHandle_t container = it->first;
        const DeviceInfo& info = it->second;

        if (g_optout_set.find(container) == g_optout_set.end())
            continue;
        if (strstr(info.model, "Quest") == nullptr)
            continue;

        // Restore original role
        props->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, info.role);
        // Restore original render model
        props->SetStringProperty(container, vr::Prop_RenderModelName_String, info.render_model);

        char buf[256];
        sprintf_s(buf, sizeof(buf), "[RestoreRole] container=0x%llx serial=\"%s\" -> %s + model restored",
            (unsigned long long)container, info.serial, RoleToString(info.role));
        vr::VRDriverLog()->Log(buf);
    }
    g_optout_set.clear();
}

// ---------------------------------------------------------------------------
// Shared memory
// ---------------------------------------------------------------------------
bool VTableHook::InitSharedMemory()
{
    // Driver creates the shared memory (so GUI can open it later)
    g_shm_file = CreateFileMappingA(
        INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
        0, sizeof(DriverSharedState), DriverSharedState::kShmName);

    if (!g_shm_file)
    {
        vr::VRDriverLog()->Log("[SharedMem] CreateFileMapping failed");
        return false;
    }

    g_shm_state = (DriverSharedState*)MapViewOfFile(g_shm_file, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(DriverSharedState));
    if (!g_shm_state)
    {
        CloseHandle(g_shm_file);
        g_shm_file = nullptr;
        vr::VRDriverLog()->Log("[SharedMem] MapViewOfFile failed");
        return false;
    }

    // Initialize
    memset(g_shm_state, 0, sizeof(DriverSharedState));
    g_shm_state->magic = DriverSharedState::kMagic;
    g_shm_state->version = DriverSharedState::kVersion;
    g_shm_state->virtual_enabled = 0; // disabled by default

    vr::VRDriverLog()->Log("[SharedMem] Created successfully, waiting for SloppyHandJob GUI");
    return true;
}

void VTableHook::CloseSharedMemory()
{
    if (g_shm_state) { UnmapViewOfFile(g_shm_state); g_shm_state = nullptr; }
    if (g_shm_file) { CloseHandle(g_shm_file); g_shm_file = nullptr; }
}

void VTableHook::WriteSharedState()
{
    if (!g_shm_state) return;

    // Write captured inputs + status for GUI to read
    g_shm_state->quest_left_detected  = (g_quest_index_left  != vr::k_unTrackedDeviceIndexInvalid) ? 1 : 0;
    g_shm_state->quest_right_detected = (g_quest_index_right != vr::k_unTrackedDeviceIndexInvalid) ? 1 : 0;
    g_shm_state->quest_left_index  = g_quest_index_left;
    g_shm_state->quest_right_index = g_quest_index_right;

    g_shm_state->trigger_left  = g_captured[0].trigger_value;
    g_shm_state->trigger_right = g_captured[1].trigger_value;
    g_shm_state->grip_left     = g_captured[0].grip_value;
    g_shm_state->grip_right    = g_captured[1].grip_value;

    g_shm_state->thumbrest_touch_left  = g_captured[0].thumbrest_touch ? 1 : 0;
    g_shm_state->thumbrest_touch_right = g_captured[1].thumbrest_touch ? 1 : 0;
    g_shm_state->thumbstick_touch_left  = g_captured[0].thumbstick_touch ? 1 : 0;
    g_shm_state->thumbstick_touch_right = g_captured[1].thumbstick_touch ? 1 : 0;
    g_shm_state->a_touch_left  = g_captured[0].a_touch ? 1 : 0;
    g_shm_state->a_touch_right = g_captured[1].a_touch ? 1 : 0;
    g_shm_state->b_touch_left  = g_captured[0].b_touch ? 1 : 0;
    g_shm_state->b_touch_right = g_captured[1].b_touch ? 1 : 0;

    // Click states for fusion
    g_shm_state->a_click_left  = g_captured[0].a_click ? 1 : 0;
    g_shm_state->a_click_right = g_captured[1].a_click ? 1 : 0;
    g_shm_state->b_click_left  = g_captured[0].b_click ? 1 : 0;
    g_shm_state->b_click_right = g_captured[1].b_click ? 1 : 0;
    g_shm_state->thumbstick_click_left  = g_captured[0].thumbstick_click ? 1 : 0;
    g_shm_state->thumbstick_click_right = g_captured[1].thumbstick_click ? 1 : 0;
    g_shm_state->trigger_click_left  = g_captured[0].trigger_click ? 1 : 0;
    g_shm_state->trigger_click_right = g_captured[1].trigger_click ? 1 : 0;
    g_shm_state->trigger_touch_left  = g_captured[0].trigger_touch ? 1 : 0;
    g_shm_state->trigger_touch_right = g_captured[1].trigger_touch ? 1 : 0;
    g_shm_state->grip_touch_left  = g_captured[0].grip_touch ? 1 : 0;
    g_shm_state->grip_touch_right = g_captured[1].grip_touch ? 1 : 0;

    g_shm_state->input_sequence = ++g_input_seq;

    // Haptic data (per-hand, GUI checks each sequence independently)
    g_shm_state->haptic_amplitude_left  = g_haptic_amp[0];
    g_shm_state->haptic_amplitude_right = g_haptic_amp[1];
    g_shm_state->haptic_duration_left   = g_haptic_dur[0];
    g_shm_state->haptic_duration_right  = g_haptic_dur[1];
    g_shm_state->haptic_seq_left        = g_haptic_seq[0];
    g_shm_state->haptic_seq_right       = g_haptic_seq[1];
}

bool VTableHook::ReadSharedState()
{
    if (!g_shm_state) return false;
    if (g_shm_state->magic != DriverSharedState::kMagic) return false;

    // Check if GUI is connected (curls_sequence is advancing)
    g_gui_connected = (g_shm_state->curls_sequence > 0);

    return true;
}

bool VTableHook::VirtualEnabled()
{
    if (!g_shm_state) return false;
    return g_shm_state->virtual_enabled != 0;
}

void VTableHook::SetProxyState(bool left, bool right)
{
    if (!g_shm_state) return;
    g_shm_state->proxy_left_active = left ? 1 : 0;
    g_shm_state->proxy_right_active = right ? 1 : 0;
}

void VTableHook::StoreHaptic(vr::ETrackedControllerRole role, float amplitude, float duration)
{
    int idx = RoleToIdx(role);
    if (idx < 0) return;
    g_haptic_amp[idx] = amplitude;
    g_haptic_dur[idx] = duration;
    g_haptic_seq[idx]++;
}

vr::HmdVector3_t VTableHook::GetPivotOffset(vr::ETrackedControllerRole role)
{
    vr::HmdVector3_t offset = { 0.f, 0.f, 0.f };
    if (!g_shm_state || g_shm_state->magic != DriverSharedState::kMagic)
        return offset;

    if (role == vr::TrackedControllerRole_LeftHand)
        { offset.v[0] = g_shm_state->pivot_offset_left[0]; offset.v[1] = g_shm_state->pivot_offset_left[1]; offset.v[2] = g_shm_state->pivot_offset_left[2]; }
    else if (role == vr::TrackedControllerRole_RightHand)
        { offset.v[0] = g_shm_state->pivot_offset_right[0]; offset.v[1] = g_shm_state->pivot_offset_right[1]; offset.v[2] = g_shm_state->pivot_offset_right[2]; }

    return offset;
}

// ---------------------------------------------------------------------------
// Per-frame skeleton update — called by proxy devices
// ---------------------------------------------------------------------------
void VTableHook::RunFrame(vr::IVRDriverInput* pInput)
{
    // Write captured inputs to shared memory for GUI
    WriteSharedState();
    ReadSharedState();

    // Handle enable/disable — this must happen BEFORE the skeleton check
    // so OptOut/restore runs even when no proxy devices exist yet
    bool want_enabled = VirtualEnabled();
    if (want_enabled && !g_virtual_enabled)
    {
        g_virtual_enabled = true;
        SetQuestRoleOptOut();
        vr::VRDriverLog()->Log("[VTableHook] Virtual controllers ENABLED");
    }
    else if (!want_enabled && g_virtual_enabled)
    {
        g_virtual_enabled = false;
        RestoreQuestRole();
        vr::VRDriverLog()->Log("[VTableHook] Virtual controllers DISABLED");
    }

    if (!pInput || g_skeleton_map.empty() || !g_virtual_enabled)
        return;
    // Only update skeleton when virtual controllers are enabled
    if (!g_virtual_enabled)
        return;

    // Snapshot GUI curl data once (BOTH hands) — avoids one-hand-only reads
    static float s_gui_curls[2][5] = {};
    static bool s_have_gui = false;
    if (g_gui_connected && g_shm_state && g_shm_state->curls_sequence != g_shm_last_curls_seq)
    {
        g_shm_last_curls_seq = g_shm_state->curls_sequence;
        memcpy(s_gui_curls[0], g_shm_state->curls_left, sizeof(float) * 5);
        memcpy(s_gui_curls[1], g_shm_state->curls_right, sizeof(float) * 5);
        s_have_gui = true;

        static int ccc = 0;
        if (++ccc >= 60) { ccc = 0;
            char buf[256];
            sprintf_s(buf, sizeof(buf), "[Curls] L T=%.2f I=%.2f M=%.2f R=%.2f P=%.2f  |  R T=%.2f I=%.2f M=%.2f R=%.2f P=%.2f",
                s_gui_curls[0][0], s_gui_curls[0][1], s_gui_curls[0][2], s_gui_curls[0][3], s_gui_curls[0][4],
                s_gui_curls[1][0], s_gui_curls[1][1], s_gui_curls[1][2], s_gui_curls[1][3], s_gui_curls[1][4]);
            vr::VRDriverLog()->Log(buf);
        }
    }

    vr::VRBoneTransform_t bones[31];

    for (auto it = g_skeleton_map.begin(); it != g_skeleton_map.end(); ++it)
    {
        const SkeletonEntry& skel = it->second;
        if (skel.handle == 0) continue;

        int ridx = RoleToIdx(skel.role);
        if (ridx < 0) ridx = 0;

        MyFingerCurls curls = {};
        MyFingerSplays splays = {};
        if (g_shm_state && g_shm_state->magic == DriverSharedState::kMagic)
        {
            float sf = g_shm_state->splay_factor;
            splays.thumb  = sf;
            splays.index  = sf;
            splays.middle = sf;
            splays.ring   = sf;
            splays.pinky  = sf;
        }

        if (s_have_gui)
        {
            curls.thumb  = s_gui_curls[ridx][0];
            curls.index  = s_gui_curls[ridx][1];
            curls.middle = s_gui_curls[ridx][2];
            curls.ring   = s_gui_curls[ridx][3];
            curls.pinky  = s_gui_curls[ridx][4];
        }
        else
        {
            // No GUI data ever received — use trigger/grip fallback
            const CapturedInputs& cap = g_captured[ridx];
            if (cap.trigger_value > 0.f)
                curls.index = cap.trigger_value;
            if (cap.grip_value > 0.f)
            {
                float g = cap.grip_value;
                curls.middle = g;
                curls.ring   = g;
                curls.pinky  = g;
            }
            if (cap.thumbstick_touch || cap.a_touch || cap.b_touch || cap.thumbrest_touch)
                curls.thumb = 0.5f;
        }

        // Smoothing: 50/50 blend + deadband to reduce jitter from ESP32 noise
        static float s_prev[2][5] = {};
        float blend = 0.5f;
        float deadband = 0.02f; // ignore changes smaller than this
        for (int ci = 0; ci < 5; ci++) {
            float target = (&curls.thumb)[ci];
            float prev = s_prev[ridx][ci];
            if (fabsf(target - prev) < deadband && target < 0.95f)
                (&curls.thumb)[ci] = prev; // hold previous within deadband
            else
                (&curls.thumb)[ci] = prev + blend * (target - prev);
            s_prev[ridx][ci] = (&curls.thumb)[ci];
        }

        g_hand_sim.ComputeSkeletonTransforms(skel.role, curls, splays, bones);

        pInput->UpdateSkeletonComponent(skel.handle,
            vr::VRSkeletalMotionRange_WithController, bones, 31);
        pInput->UpdateSkeletonComponent(skel.handle,
            vr::VRSkeletalMotionRange_WithoutController, bones, 31);
    }
}

// ---------------------------------------------------------------------------
// Called by proxy device to register its skeleton handle
// ---------------------------------------------------------------------------
void VTableHook_RegisterSkeleton(vr::PropertyContainerHandle_t container,
    vr::VRInputComponentHandle_t handle, vr::ETrackedControllerRole role)
{
    SkeletonEntry skel;
    skel.handle = handle;
    skel.role = role;
    g_skeleton_map[container] = skel;
}