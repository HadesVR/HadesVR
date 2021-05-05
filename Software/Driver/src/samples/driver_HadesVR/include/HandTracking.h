/*

Original Work Copyright (c) 2021 LucidVR
Modified Work Copyright (c) 2021 HadesVR

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
//my crappy implementation of LucidVR's open vr gloves finger tracking code, check them out here https://github.com/LucidVR/opengloves-driver


#pragma once

#include "openvr/openvr_driver.h"

namespace fingerTracking {
	const int NUM_BONES = 31;
	typedef int32_t BoneIndex_t;
	const BoneIndex_t INVALID_BONEINDEX = -1;

	enum HandSkeletonBone : BoneIndex_t
	{
		eBone_Root = 0,
		eBone_Wrist,
		eBone_Thumb0,
		eBone_Thumb1,
		eBone_Thumb2,
		eBone_Thumb3,
		eBone_IndexFinger0,
		eBone_IndexFinger1,
		eBone_IndexFinger2,
		eBone_IndexFinger3,
		eBone_IndexFinger4,
		eBone_MiddleFinger0,
		eBone_MiddleFinger1,
		eBone_MiddleFinger2,
		eBone_MiddleFinger3,
		eBone_MiddleFinger4,
		eBone_RingFinger0,
		eBone_RingFinger1,
		eBone_RingFinger2,
		eBone_RingFinger3,
		eBone_RingFinger4,
		eBone_PinkyFinger0,
		eBone_PinkyFinger1,
		eBone_PinkyFinger2,
		eBone_PinkyFinger3,
		eBone_PinkyFinger4,
		eBone_Aux_Thumb,
		eBone_Aux_IndexFinger,
		eBone_Aux_MiddleFinger,
		eBone_Aux_RingFinger,
		eBone_Aux_PinkyFinger,
		eBone_Count
	};

	extern vr::VRBoneTransform_t Pose_OpenRight[NUM_BONES];
	extern vr::VRBoneTransform_t Pose_ClosedRight[NUM_BONES];
	extern vr::VRBoneTransform_t Pose_OpenLeft[NUM_BONES];
	extern vr::VRBoneTransform_t Pose_ClosedLeft[NUM_BONES];

	float Lerp(const float a, const float b, const float f);
	extern void CalculateHandBones(vr::VRBoneTransform_t* HandBoneTransform, float thumbFingerFlexion, float indexFingerFlexion, float middleFingerFlexion, float ringFingerFlexion, float pinkyFingerFlexion, const bool isRightHand, int i);
}