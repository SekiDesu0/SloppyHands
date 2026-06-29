#pragma once

#include <array>
#include "openvr_driver.h"

enum KnucklesHandle {
    kBool_system_click,
    kBool_system_touch,
    kBool_a_click,
    kBool_a_touch,
    kBool_b_click,
    kBool_b_touch,
    kBool_trigger_click,
    kBool_trigger_touch,
    kBool_thumbstick_click,
    kBool_thumbstick_touch,
    kBool_grip_touch,
    kBool_trackpad_touch,
    kBool_trackpad_click,
    kScalar_trigger_value,
    kScalar_thumbstick_x,
    kScalar_thumbstick_y,
    kScalar_grip_force,
    kScalar_grip_value,
    kHaptic,
    kSkeleton,
    kHandle_COUNT
};

class KnucklesProxyDevice : public vr::ITrackedDeviceServerDriver {
public:
    KnucklesProxyDevice(vr::ETrackedControllerRole role);

    // ITrackedDeviceServerDriver
    virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
    virtual void Deactivate() override;
    virtual void EnterStandby() override;
    virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
    virtual vr::DriverPose_t GetPose() override;

    void RunFrame(bool active);
    void HandleEvent(const vr::VREvent_t& vrevent);

private:
    std::array<vr::VRInputComponentHandle_t, kHandle_COUNT> handles_{};

    vr::ETrackedControllerRole role_;
    vr::TrackedDeviceIndex_t device_id_;
    bool activated_;

    // Rotation smoothing state
    vr::HmdQuaternion_t smoothed_rot_;
    bool rot_initialized_;
    vr::HmdQuaternion_t raw_prev_rot_;
    bool raw_prev_valid_;

    // Position smoothing state
    vr::HmdVector3_t smoothed_pos_;
    bool pos_initialized_;
    vr::HmdVector3_t raw_prev_pos_;
    bool raw_prev_pos_valid_;
};