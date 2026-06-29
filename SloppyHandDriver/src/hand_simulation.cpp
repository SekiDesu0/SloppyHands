//============ Copyright (c) Valve Corporation, All rights reserved. ============
// Inspired by Moshi Turner's code from Monado https://gitlab.freedesktop.org/monado/monado/-/blob/main/src/xrt/auxiliary/util/u_hand_simulation.c
#include "hand_simulation.h"
#include "vrmath.h"

struct HandSimSplayableJoint
{
	vr::HmdVector2_t swing = { 0.f, 0.f };
	float twist = 0.f;
};

struct HandSimJoint
{
	float rotation = 0.f;
};

struct HandSimThumb
{
	HandSimSplayableJoint metacarpal;
	HandSimSplayableJoint proximal;
	HandSimJoint distal;
};

struct HandSimFinger
{
	HandSimSplayableJoint metacarpal;
	HandSimSplayableJoint proximal;
	HandSimJoint intermediate;
	HandSimJoint distal;
};

struct HandSimHand
{
	vr::ETrackedControllerRole role;

	HandSimThumb thumb;

	HandSimFinger fingers[4];
};

static const float finger_joint_lengths[5][5] = {
	{ 0.05f, 0.05f, 0.035f, 0.025f, 0.f },
	{ 0.03f, 0.073f, 0.045f, 0.025f, 0.02f },
	{ 0.01f, 0.091f, 0.049f, 0.03f, 0.02f },
	{ 0.02f, 0.073f, 0.045f, 0.03f, 0.03f },
	{ 0.03f, 0.067f, 0.03f, 0.025f, 0.02f },
};

static void InitHand(HandSimHand& out_hand)
{
	for (auto& finger : out_hand.fingers)
	{
		finger.metacarpal.swing.v[1] = 0.f;
		finger.metacarpal.twist = 0.f;

		finger.proximal.swing.v[1] = DEG_TO_RAD(10);
		finger.intermediate.rotation = DEG_TO_RAD(5.f);

		finger.intermediate.rotation = DEG_TO_RAD(5.f);
		finger.distal.rotation = DEG_TO_RAD(5.f);
	}

	out_hand.thumb.metacarpal.swing.v[0] = DEG_TO_RAD(10);
	out_hand.thumb.metacarpal.swing.v[1] = DEG_TO_RAD(40);
	out_hand.thumb.metacarpal.twist = DEG_TO_RAD(70);

	out_hand.thumb.proximal.swing.v[0] = 0.f;
	out_hand.thumb.proximal.swing.v[1] = 0.f;
	out_hand.thumb.proximal.twist = 0.f;

	out_hand.thumb.distal.rotation = 0.f;

	out_hand.fingers[0].metacarpal.swing.v[1] = DEG_TO_RAD(13.f);
	out_hand.fingers[1].metacarpal.swing.v[1] = DEG_TO_RAD(-0.f);
	out_hand.fingers[2].metacarpal.swing.v[1] = DEG_TO_RAD(-15.f);
	out_hand.fingers[3].metacarpal.swing.v[1] = DEG_TO_RAD(-27.f);

	out_hand.fingers[0].proximal.swing.v[1] = DEG_TO_RAD(3.f);
	out_hand.fingers[1].proximal.swing.v[1] = DEG_TO_RAD(0.f);
	out_hand.fingers[2].proximal.swing.v[1] = DEG_TO_RAD(-1.f);
	out_hand.fingers[3].proximal.swing.v[1] = DEG_TO_RAD(-2.f);
}

static void ApplyGenericFingerTransform(const float curl, const float splay, HandSimFinger& out_finger)
{
	out_finger.metacarpal.swing.v[0] += DEG_TO_RAD(curl * 5.f);

	out_finger.proximal.swing.v[0] += DEG_TO_RAD(curl * 90.f);
	out_finger.proximal.swing.v[1] += DEG_TO_RAD(splay * 15.f);

	out_finger.intermediate.rotation += DEG_TO_RAD(curl * 80.f);
	out_finger.distal.rotation += DEG_TO_RAD(curl * 80.f);
}

static void ComputeBoneTransform(const vr::ETrackedControllerRole role, const vr::HmdQuaternion_t& orientation, const vr::HmdVector3_t& position, vr::VRBoneTransform_t& out_transform)
{
	HmdQuaternion_ConvertQuaternion(orientation, out_transform.orientation);

	HmdVector3_CovertVector(position, out_transform.position);
	out_transform.position.v[3] = 1.f;

	if (role == vr::TrackedControllerRole_RightHand)
	{
		out_transform.position.v[0] *= -1.f;
	}
}

static void ComputeBoneTransform(const vr::ETrackedControllerRole role, const vr::HmdQuaternion_t& orientation, const float joint_length, vr::VRBoneTransform_t& out_transform)
{
	ComputeBoneTransform(role, orientation, { joint_length, 0.f, 0.f }, out_transform);
}

static void ComputeBoneTransformMetacarpal(const vr::ETrackedControllerRole role, const vr::HmdQuaternion_t& orientation, const float joint_length, vr::VRBoneTransform_t& out_transform)
{
	const vr::HmdVector3_t offset = { joint_length, 0.f, 0.f };

	vr::HmdQuaternion_t magic = { 0.5f, 0.5f, -0.5f, 0.5f };

	vr::HmdQuaternion_t bone_orientation = magic * orientation;

	vr::HmdVector3_t bone_position = offset * bone_orientation;

	if (role == vr::TrackedControllerRole_RightHand)
	{
		std::swap(bone_orientation.w, bone_orientation.x);
		std::swap(bone_orientation.y, bone_orientation.z);

		bone_orientation.x *= -1.f;
		bone_orientation.z *= -1.f;
	}

	ComputeBoneTransform(role, bone_orientation, bone_position, out_transform);
}

static int CalculateBoneTransformPositionFromFinger(int finger, int bone_in_finger)
{
	const int bone_transform_finger_start_offset = eBone_IndexFinger0;

	const int result = bone_transform_finger_start_offset + finger * 5 + bone_in_finger;

	return result;
}

static void ComputeSkeletalTransforms(const HandSimHand& hand, vr::VRBoneTransform_t* out_transforms)
{
	ComputeBoneTransformMetacarpal(
		hand.role, HmdQuaternion_FromSwingTwist(hand.thumb.metacarpal.swing, hand.thumb.metacarpal.twist), finger_joint_lengths[0][0], out_transforms[eBone_Thumb0]);
	ComputeBoneTransform(hand.role, HmdQuaternion_FromSwingTwist(hand.thumb.proximal.swing, hand.thumb.metacarpal.twist), finger_joint_lengths[0][1], out_transforms[eBone_Thumb1]);
	ComputeBoneTransform(hand.role, HmdQuaternion_FromEulerAngles(hand.thumb.distal.rotation, 0.f, 0.f), finger_joint_lengths[0][2], out_transforms[eBone_Thumb2]);
	ComputeBoneTransform(hand.role, HmdQuaternion_Identity, finger_joint_lengths[0][3], out_transforms[eBone_Thumb3]);

	for (int finger = 0; finger < 4; finger++)
	{
		ComputeBoneTransformMetacarpal(hand.role, HmdQuaternion_FromSwingTwist(hand.fingers[finger].metacarpal.swing, hand.fingers[finger].metacarpal.twist),
			finger_joint_lengths[finger + 1][0], out_transforms[CalculateBoneTransformPositionFromFinger(finger, 0)]);

		ComputeBoneTransform(hand.role, HmdQuaternion_FromSwingTwist(hand.fingers[finger].proximal.swing, hand.fingers[finger].proximal.twist), finger_joint_lengths[finger + 1][1],
			out_transforms[CalculateBoneTransformPositionFromFinger(finger, 1)]);

		ComputeBoneTransform(hand.role, HmdQuaternion_FromEulerAngles(hand.fingers[finger].intermediate.rotation, 0.f, 0.f), finger_joint_lengths[finger + 1][2],
			out_transforms[CalculateBoneTransformPositionFromFinger(finger, 2)]);

		ComputeBoneTransform(hand.role, HmdQuaternion_FromEulerAngles(hand.fingers[finger].distal.rotation, 0.f, 0.f), finger_joint_lengths[finger + 1][3],
			out_transforms[CalculateBoneTransformPositionFromFinger(finger, 3)]);

		ComputeBoneTransform(hand.role, HmdQuaternion_Identity, finger_joint_lengths[finger + 1][4], out_transforms[CalculateBoneTransformPositionFromFinger(finger, 4)]);
	}
}

void MyHandSimulation::ComputeSkeletonTransforms(vr::ETrackedControllerRole role, const MyFingerCurls& curls, const MyFingerSplays& splays, vr::VRBoneTransform_t* out_transforms)
{
	HandSimHand hand{};

	hand.role = role;

	out_transforms[0] = { { 0.000000f, 0.000000f, 0.000000f, 1.000000f }, { 1.000000f, -0.000000f, -0.000000f, 0.000000f } };

	out_transforms[1] = { { -0.034038f, 0.036503f, 0.164722f, 1.000000f }, { -0.055147f, -0.078608f, -0.920279f, 0.379296f } };

	if (role == vr::TrackedControllerRole_RightHand)
	{
		out_transforms[1].position.v[0] *= -1.f;

		out_transforms[1].orientation.y *= -1.f;
		out_transforms[1].orientation.z *= -1.f;
	}

	InitHand(hand);

	hand.thumb.metacarpal.swing.v[0] += DEG_TO_RAD(curls.thumb * 5.f);
	hand.thumb.metacarpal.swing.v[1] += DEG_TO_RAD(splays.thumb * 5.f);
	hand.thumb.metacarpal.twist = 0.f;

	hand.thumb.proximal.swing.v[0] += DEG_TO_RAD(curls.thumb * 90.f);
	hand.thumb.proximal.swing.v[1] += DEG_TO_RAD(splays.thumb * 20.f);
	hand.thumb.proximal.twist = 0.f;

	hand.thumb.distal.rotation += DEG_TO_RAD(curls.thumb * 90.f);

	ApplyGenericFingerTransform(curls.index, splays.index, hand.fingers[0]);
	ApplyGenericFingerTransform(curls.middle, splays.middle, hand.fingers[1]);
	ApplyGenericFingerTransform(curls.ring, splays.ring, hand.fingers[2]);
	ApplyGenericFingerTransform(curls.pinky, splays.pinky, hand.fingers[3]);

	ComputeSkeletalTransforms(hand, out_transforms);
}
