#pragma once

#include <memory>
#include "openvr_driver.h"
#include "controller_device.h"

class DeviceProvider : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override;
    void Cleanup() override;
    const char* const* GetInterfaceVersions() override;
    void RunFrame() override;
    bool ShouldBlockStandbyMode() override;
    void EnterStandby() override;
    void LeaveStandby() override;

private:
    std::unique_ptr<KnucklesProxyDevice> proxy_left_;
    std::unique_ptr<KnucklesProxyDevice> proxy_right_;
    bool proxy_left_added_  = false;
    bool proxy_right_added_ = false;
};