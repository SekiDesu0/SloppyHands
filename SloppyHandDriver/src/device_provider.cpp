#include "device_provider.h"
#include "vtable_hook.h"
#include <cstdio>
#include <cstring>
#include <windows.h>

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* pDriverContext) {
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    vr::VRDriverLog()->Log("Hello world! (Knuckles Proxy Driver)");

    VTableHook::HookIVRDriverInput();
    VTableHook::InitSharedMemory();

    proxy_left_  = std::make_unique<KnucklesProxyDevice>(vr::TrackedControllerRole_LeftHand);
    proxy_right_ = std::make_unique<KnucklesProxyDevice>(vr::TrackedControllerRole_RightHand);

    return vr::VRInitError_None;
}

void DeviceProvider::Cleanup() {
    VTableHook::Unhook();
    proxy_left_.reset();
    proxy_right_.reset();
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

const char* const* DeviceProvider::GetInterfaceVersions() {
    return vr::k_InterfaceVersions;
}

void DeviceProvider::RunFrame() {
    // Update shared memory + skeleton + enable/disable logic
    VTableHook::RunFrame(vr::VRDriverInput());

    bool want_proxy = VTableHook::VirtualEnabled();

    // If user wants proxies but Quest controllers haven't been discovered yet,
    // force a container rescan. Quest controllers often finish SteamVR init
    // after our driver's Init() runs, so the initial scan may miss them.
    if (want_proxy) {
        if (!VTableHook::IsQuestDetected(vr::TrackedControllerRole_LeftHand) ||
            !VTableHook::IsQuestDetected(vr::TrackedControllerRole_RightHand)) {
            VTableHook::RescanContainers();
        }
    }

    // Only register proxy devices when virtual controllers are enabled,
    // so SteamVR status window shows a clean pair of controllers at boot.
    // Once registered they persist (no TrackedDeviceRemoved API) but report
    // valid/invalid pose per-frame based on the toggle state.
    if (want_proxy && VTableHook::IsQuestDetected(vr::TrackedControllerRole_LeftHand)) {
        if (!proxy_left_added_) {
            proxy_left_added_ = vr::VRServerDriverHost()->TrackedDeviceAdded(
                "sample_knuckles_left", vr::TrackedDeviceClass_Controller, proxy_left_.get());
            vr::VRDriverLog()->Log(proxy_left_added_ ? "[Proxy] Left added" : "[Proxy] Left add FAILED");
        }
    }
    if (want_proxy && VTableHook::IsQuestDetected(vr::TrackedControllerRole_RightHand)) {
        if (!proxy_right_added_) {
            proxy_right_added_ = vr::VRServerDriverHost()->TrackedDeviceAdded(
                "sample_knuckles_right", vr::TrackedDeviceClass_Controller, proxy_right_.get());
            vr::VRDriverLog()->Log(proxy_right_added_ ? "[Proxy] Right added" : "[Proxy] Right add FAILED");
        }
    }

    // Always run proxy frames — they check 'active' to report valid/invalid pose
    if (proxy_left_ && proxy_left_added_)
        proxy_left_->RunFrame(want_proxy);
    if (proxy_right_ && proxy_right_added_)
        proxy_right_->RunFrame(want_proxy);

    // Poll events
    vr::VREvent_t vrevent;
    while (vr::VRServerDriverHost()->PollNextEvent(&vrevent, sizeof(vrevent))) {
        if (proxy_left_ && proxy_left_added_)
            proxy_left_->HandleEvent(vrevent);
        if (proxy_right_ && proxy_right_added_)
            proxy_right_->HandleEvent(vrevent);
    }

    // Update proxy active state in shared memory
    VTableHook::SetProxyState(want_proxy && proxy_left_added_, want_proxy && proxy_right_added_);
}

bool DeviceProvider::ShouldBlockStandbyMode() {
    return false;
}

void DeviceProvider::EnterStandby() {}
void DeviceProvider::LeaveStandby() {}