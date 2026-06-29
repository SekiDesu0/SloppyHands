#include "controller_device.h"
#include "vtable_hook.h"
#include "openvr_driver.h"
#include <cmath>
#include <cstring>

KnucklesProxyDevice::KnucklesProxyDevice(vr::ETrackedControllerRole role)
    : role_(role), device_id_(vr::k_unTrackedDeviceIndexInvalid), activated_(false),
      smoothed_rot_({1.f, 0.f, 0.f, 0.f}), rot_initialized_(false),
      raw_prev_rot_({1.f, 0.f, 0.f, 0.f}), raw_prev_valid_(false),
      smoothed_pos_({0.f, 0.f, 0.f}), pos_initialized_(false),
      raw_prev_pos_({0.f, 0.f, 0.f}), raw_prev_pos_valid_(false)
{
    handles_.fill(0);
}

vr::EVRInitError KnucklesProxyDevice::Activate(uint32_t unObjectId)
{
    device_id_ = unObjectId;

    const char* role_str = (role_ == vr::TrackedControllerRole_LeftHand) ? "Left" : "Right";
    char logbuf[256];
    sprintf_s(logbuf, sizeof(logbuf), "[Proxy] Activate device_id=%u role=%s", unObjectId, role_str);
    vr::VRDriverLog()->Log(logbuf);

    vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

    // --- Knuckles properties ---
    vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, role_);
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ControllerType_String, "knuckles");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_InputProfilePath_String,
        "{indexcontroller}/input/index_controller_profile.json");

    bool is_right = (role_ == vr::TrackedControllerRole_RightHand);
    vr::VRProperties()->SetStringProperty(container, vr::Prop_RenderModelName_String,
        is_right ? "{indexcontroller}valve_controller_knu_1_0_right"
                 : "{indexcontroller}valve_controller_knu_1_0_left");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String,
        is_right ? "Knuckles Right" : "Knuckles Left");
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ManufacturerName_String, "Valve");

    // --- Create Knuckles input components (native paths) ---

    // Boolean components
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/system/click",     &handles_[kBool_system_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/system/touch",     &handles_[kBool_system_touch]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/a/click",          &handles_[kBool_a_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/a/touch",          &handles_[kBool_a_touch]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/b/click",          &handles_[kBool_b_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/b/touch",          &handles_[kBool_b_touch]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/trigger/click",    &handles_[kBool_trigger_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/trigger/touch",    &handles_[kBool_trigger_touch]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/thumbstick/click", &handles_[kBool_thumbstick_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/thumbstick/touch", &handles_[kBool_thumbstick_touch]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/grip/touch",       &handles_[kBool_grip_touch]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/trackpad/touch",    &handles_[kBool_trackpad_touch]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/trackpad/click",    &handles_[kBool_trackpad_click]);

    // Scalar components
    vr::VRDriverInput()->CreateScalarComponent(container, "/input/trigger/value",
        &handles_[kScalar_trigger_value], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    vr::VRDriverInput()->CreateScalarComponent(container, "/input/thumbstick/x",
        &handles_[kScalar_thumbstick_x], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    vr::VRDriverInput()->CreateScalarComponent(container, "/input/thumbstick/y",
        &handles_[kScalar_thumbstick_y], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    vr::VRDriverInput()->CreateScalarComponent(container, "/input/grip/force",
        &handles_[kScalar_grip_force], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    vr::VRDriverInput()->CreateScalarComponent(container, "/input/grip/value",
        &handles_[kScalar_grip_value], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);

    // Haptic
    vr::VRDriverInput()->CreateHapticComponent(container, "/output/haptic", &handles_[kHaptic]);

    // Skeleton
    const char* skel_path = is_right ? "/skeleton/hand/right" : "/skeleton/hand/left";
    const char* input_path = is_right ? "/input/skeleton/right" : "/input/skeleton/left";

    vr::EVRInputError skel_err = vr::VRDriverInput()->CreateSkeletonComponent(
        container, input_path, skel_path, "/pose/raw",
        vr::VRSkeletalTracking_Partial, nullptr, 0, &handles_[kSkeleton]);

    if (skel_err == vr::VRInputError_None && handles_[kSkeleton] != 0)
    {
        VTableHook_RegisterSkeleton(container, handles_[kSkeleton], role_);
        sprintf_s(logbuf, sizeof(logbuf), "[Proxy] Skeleton created role=%s handle=0x%llx", role_str,
            (unsigned long long)handles_[kSkeleton]);
        vr::VRDriverLog()->Log(logbuf);
    }
    else
    {
        sprintf_s(logbuf, sizeof(logbuf), "[Proxy] Skeleton FAILED err=%d", (int)skel_err);
        vr::VRDriverLog()->Log(logbuf);
    }

    activated_ = true;
    return vr::VRInitError_None;
}

void KnucklesProxyDevice::RunFrame(bool active)
{
    if (!activated_) return;

    if (!active)
    {
        // Report invalid pose so SteamVR ignores this device
        vr::DriverPose_t pose = {};
        pose.poseIsValid = false;
        pose.deviceIsConnected = false;
        pose.result = vr::TrackingResult_Uninitialized;
        pose.qWorldFromDriverRotation.w = 1.f;
        pose.qDriverFromHeadRotation.w = 1.f;
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_id_, pose, sizeof(vr::DriverPose_t));
        return;
    }

    // --- Update pose from Quest controller ---
    vr::DriverPose_t pose = GetPose();
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_id_, pose, sizeof(vr::DriverPose_t));

    // --- Forward captured Quest inputs to our Knuckles handles ---
    CapturedInputs* cap = VTableHook::GetCapturedInputs(role_);
    if (!cap) return;

    // Boolean inputs
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_a_click],          cap->a_click, 0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_a_touch],          cap->a_touch, 0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_b_click],          cap->b_click, 0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_b_touch],          cap->b_touch, 0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_trigger_click],    cap->trigger_click, 0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_trigger_touch],    cap->trigger_touch, 0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_thumbstick_click], cap->thumbstick_click, 0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_thumbstick_touch], cap->thumbstick_touch, 0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_system_click],      cap->system_click, 0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_grip_touch],       cap->grip_touch, 0.0);

    // Only thumbrest drives trackpad touch (not stick/buttons)
    bool trackpad_touch = cap->thumbrest_touch;
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_trackpad_touch], trackpad_touch, 0.0);
    // Map thumbstick click to trackpad click as well (some bindings use it)
    vr::VRDriverInput()->UpdateBooleanComponent(handles_[kBool_trackpad_click], cap->thumbstick_click, 0.0);

    // Scalar inputs
    vr::VRDriverInput()->UpdateScalarComponent(handles_[kScalar_trigger_value],  cap->trigger_value, 0.0);
    vr::VRDriverInput()->UpdateScalarComponent(handles_[kScalar_thumbstick_x],   cap->thumbstick_x, 0.0);
    vr::VRDriverInput()->UpdateScalarComponent(handles_[kScalar_thumbstick_y],   cap->thumbstick_y, 0.0);
    vr::VRDriverInput()->UpdateScalarComponent(handles_[kScalar_grip_force],    cap->grip_value, 0.0);
    // Send zero to grip_value so VRChat's gesture system doesn't override skeleton fingers.
    // Grip_force is kept non-zero so the grab action (picking up objects) still works.
    vr::VRDriverInput()->UpdateScalarComponent(handles_[kScalar_grip_value],    0.0f, 0.0);
}

void KnucklesProxyDevice::HandleEvent(const vr::VREvent_t& vrevent)
{
    switch (vrevent.eventType)
    {
    case vr::VREvent_Input_HapticVibration:
    {
        const auto& hv = vrevent.data.hapticVibration;
        if (hv.componentHandle == handles_[kHaptic])
        {
            VTableHook::StoreHaptic(role_, hv.fAmplitude, hv.fDurationSeconds);
            char buf[256];
            const char* hand = (role_ == vr::TrackedControllerRole_LeftHand) ? "L" : "R";
            sprintf_s(buf, sizeof(buf), "[Haptic] %s amp=%.3f dur=%.4f freq=%.1f",
                hand, hv.fAmplitude, hv.fDurationSeconds, hv.fFrequency);
            vr::VRDriverLog()->Log(buf);
        }
        break;
    }
    }
}

vr::DriverPose_t KnucklesProxyDevice::GetPose()
{
    vr::DriverPose_t pose = {};
    pose.poseIsValid = true;
    pose.result = vr::TrackingResult_Running_OK;
    pose.deviceIsConnected = true;

    pose.qWorldFromDriverRotation.w = 1.f;
    pose.qDriverFromHeadRotation.w = 1.f;
    pose.qRotation.w = 1.f;

    uint32_t quest_idx = VTableHook::GetQuestDeviceIndex(role_);
    if (quest_idx != vr::k_unTrackedDeviceIndexInvalid)
    {
        vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
        vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, poses, vr::k_unMaxTrackedDeviceCount);

        const vr::TrackedDevicePose_t& qp = poses[quest_idx];
        if (qp.bPoseIsValid)
        {
            pose.vecPosition[0] = qp.mDeviceToAbsoluteTracking.m[0][3];
            pose.vecPosition[1] = qp.mDeviceToAbsoluteTracking.m[1][3];
            pose.vecPosition[2] = qp.mDeviceToAbsoluteTracking.m[2][3];

            vr::HmdVector3_t raw_pos = { pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2] };

            // Extract raw rotation from matrix (must happen before pivot offset)
            const auto& m = qp.mDeviceToAbsoluteTracking;
            float tr = m.m[0][0] + m.m[1][1] + m.m[2][2];
            if (tr > 0.0f)
            {
                float S = sqrtf(tr + 1.0f) * 2.0f;
                pose.qRotation.w = 0.25f * S;
                pose.qRotation.x = (m.m[2][1] - m.m[1][2]) / S;
                pose.qRotation.y = (m.m[0][2] - m.m[2][0]) / S;
                pose.qRotation.z = (m.m[1][0] - m.m[0][1]) / S;
            }
            else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2])
            {
                float S = sqrtf(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0f;
                pose.qRotation.w = (m.m[2][1] - m.m[1][2]) / S;
                pose.qRotation.x = 0.25f * S;
                pose.qRotation.y = (m.m[0][1] + m.m[1][0]) / S;
                pose.qRotation.z = (m.m[0][2] + m.m[2][0]) / S;
            }
            else if (m.m[1][1] > m.m[2][2])
            {
                float S = sqrtf(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0f;
                pose.qRotation.w = (m.m[0][2] - m.m[2][0]) / S;
                pose.qRotation.x = (m.m[0][1] + m.m[1][0]) / S;
                pose.qRotation.y = 0.25f * S;
                pose.qRotation.z = (m.m[1][2] + m.m[2][1]) / S;
            }
            else
            {
                float S = sqrtf(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0f;
                pose.qRotation.w = (m.m[1][0] - m.m[0][1]) / S;
                pose.qRotation.x = (m.m[0][2] + m.m[2][0]) / S;
                pose.qRotation.y = (m.m[1][2] + m.m[2][1]) / S;
                pose.qRotation.z = 0.25f * S;
            }

            // Apply pivot offset: rotate local offset into world space and add to raw position
            {
                vr::HmdVector3_t offset = VTableHook::GetPivotOffset(role_);
                float qw = (float)pose.qRotation.w, qx = (float)pose.qRotation.x, qy = (float)pose.qRotation.y, qz = (float)pose.qRotation.z;
                float ox = offset.v[0], oy = offset.v[1], oz = offset.v[2];

                // cross1 = q_xyz × offset
                float cx1 = qy * oz - qz * oy;
                float cy1 = qz * ox - qx * oz;
                float cz1 = qx * oy - qy * ox;

                // cross2 = q_xyz × cross1
                float cx2 = qy * cz1 - qz * cy1;
                float cy2 = qz * cx1 - qx * cz1;
                float cz2 = qx * cy1 - qy * cx1;

                // result = offset + 2*qw*cross1 + 2*cross2
                raw_pos.v[0] += ox + 2.0f * qw * cx1 + 2.0f * cx2;
                raw_pos.v[1] += oy + 2.0f * qw * cy1 + 2.0f * cy2;
                raw_pos.v[2] += oz + 2.0f * qw * cz1 + 2.0f * cz2;
            }

            // Adaptive position smoothing: heavy at rest, near-raw on fast moves
            float pos_blend;
            if (raw_prev_pos_valid_)
            {
                float dx = raw_pos.v[0] - raw_prev_pos_.v[0];
                float dy = raw_pos.v[1] - raw_prev_pos_.v[1];
                float dz = raw_pos.v[2] - raw_prev_pos_.v[2];
                float pos_delta = sqrtf(dx * dx + dy * dy + dz * dz);

                const float lo_p = 0.002f;   // m/frame: jitter threshold
                const float hi_p = 0.030f;   // m/frame: fast-arm-swing threshold
                if (pos_delta < lo_p)
                    pos_blend = 0.20f;
                else if (pos_delta > hi_p)
                    pos_blend = 0.90f;
                else
                    pos_blend = 0.20f + (0.90f - 0.20f) * (pos_delta - lo_p) / (hi_p - lo_p);
            }
            else
            {
                pos_blend = 1.0f;
            }
            raw_prev_pos_ = raw_pos;
            raw_prev_pos_valid_ = true;

            if (!pos_initialized_)
            {
                smoothed_pos_ = raw_pos;
                pos_initialized_ = true;
            }
            else
            {
                smoothed_pos_.v[0] += pos_blend * (raw_pos.v[0] - smoothed_pos_.v[0]);
                smoothed_pos_.v[1] += pos_blend * (raw_pos.v[1] - smoothed_pos_.v[1]);
                smoothed_pos_.v[2] += pos_blend * (raw_pos.v[2] - smoothed_pos_.v[2]);
            }
            pose.vecPosition[0] = smoothed_pos_.v[0];
            pose.vecPosition[1] = smoothed_pos_.v[1];
            pose.vecPosition[2] = smoothed_pos_.v[2];

            // Adaptive blend factor: heavy smoothing at rest, near-raw on fast moves
            vr::HmdQuaternion_t raw_rot = pose.qRotation;
            float blend;
            if (raw_prev_valid_)
            {
                float raw_dot = fabsf(raw_prev_rot_.w * raw_rot.w
                                    + raw_prev_rot_.x * raw_rot.x
                                    + raw_prev_rot_.y * raw_rot.y
                                    + raw_prev_rot_.z * raw_rot.z);
                if (raw_dot > 1.0f) raw_dot = 1.0f;
                float angle_delta = acosf(raw_dot);

                const float lo = 0.01f;   // rad/frame: jitter threshold
                const float hi = 0.10f;   // rad/frame: fast-flick threshold
                if (angle_delta < lo)
                    blend = 0.20f;
                else if (angle_delta > hi)
                    blend = 0.90f;
                else
                    blend = 0.20f + (0.90f - 0.20f) * (angle_delta - lo) / (hi - lo);
            }
            else
            {
                blend = 1.0f;  // first raw sample: no smoothing
            }
            raw_prev_rot_ = raw_rot;
            raw_prev_valid_ = true;

            if (!rot_initialized_)
            {
                smoothed_rot_ = pose.qRotation;
                rot_initialized_ = true;
            }
            else
            {
                float dot = smoothed_rot_.w * pose.qRotation.w
                          + smoothed_rot_.x * pose.qRotation.x
                          + smoothed_rot_.y * pose.qRotation.y
                          + smoothed_rot_.z * pose.qRotation.z;
                if (dot < 0.0f)
                {
                    pose.qRotation.w = -pose.qRotation.w;
                    pose.qRotation.x = -pose.qRotation.x;
                    pose.qRotation.y = -pose.qRotation.y;
                    pose.qRotation.z = -pose.qRotation.z;
                    dot = -dot;
                }

                if (dot > 0.9995f)
                {
                    float t1 = 1.0f - blend;
                    smoothed_rot_.w = t1 * smoothed_rot_.w + blend * pose.qRotation.w;
                    smoothed_rot_.x = t1 * smoothed_rot_.x + blend * pose.qRotation.x;
                    smoothed_rot_.y = t1 * smoothed_rot_.y + blend * pose.qRotation.y;
                    smoothed_rot_.z = t1 * smoothed_rot_.z + blend * pose.qRotation.z;
                    float norm = sqrtf(smoothed_rot_.w * smoothed_rot_.w
                                     + smoothed_rot_.x * smoothed_rot_.x
                                     + smoothed_rot_.y * smoothed_rot_.y
                                     + smoothed_rot_.z * smoothed_rot_.z);
                    smoothed_rot_.w /= norm;
                    smoothed_rot_.x /= norm;
                    smoothed_rot_.y /= norm;
                    smoothed_rot_.z /= norm;
                }
                else
                {
                    float omega = acosf(dot);
                    float sin_omega = sinf(omega);
                    float a = sinf((1.0f - blend) * omega) / sin_omega;
                    float b = sinf(blend * omega) / sin_omega;
                    smoothed_rot_.w = a * smoothed_rot_.w + b * pose.qRotation.w;
                    smoothed_rot_.x = a * smoothed_rot_.x + b * pose.qRotation.x;
                    smoothed_rot_.y = a * smoothed_rot_.y + b * pose.qRotation.y;
                    smoothed_rot_.z = a * smoothed_rot_.z + b * pose.qRotation.z;
                }
                pose.qRotation = smoothed_rot_;
            }

            pose.vecVelocity[0] = qp.vVelocity.v[0];
            pose.vecVelocity[1] = qp.vVelocity.v[1];
            pose.vecVelocity[2] = qp.vVelocity.v[2];

            pose.vecAngularVelocity[0] = qp.vAngularVelocity.v[0];
            pose.vecAngularVelocity[1] = qp.vAngularVelocity.v[1];
            pose.vecAngularVelocity[2] = qp.vAngularVelocity.v[2];
        }
    }

    return pose;
}

void KnucklesProxyDevice::Deactivate() { activated_ = false; rot_initialized_ = false; raw_prev_valid_ = false; pos_initialized_ = false; raw_prev_pos_valid_ = false; }
void KnucklesProxyDevice::EnterStandby() {}
void* KnucklesProxyDevice::GetComponent(const char* pchComponentNameAndVersion) { return nullptr; }
void KnucklesProxyDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1) pchResponseBuffer[0] = 0;
}