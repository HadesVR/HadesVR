//Controller props shamelessly stolen from Driver_leap (https://github.com/SDraw/driver_leap)!

#ifndef _controllerBindings_
#define _controllerBindings_

#pragma once

#define INDEX_AClick           0x0001
#define INDEX_ATouch           0x0002
#define INDEX_BClick           0x0004
#define INDEX_BTouch           0x0008
#define INDEX_SYSClick         0x0010
#define INDEX_ThumbStickClick  0x0020
#define INDEX_FingerIndex      0x0040
#define INDEX_FingerMiddle     0x0080
#define INDEX_FingerRing       0x0100
#define INDEX_FingerPinky      0x0200
#define INDEX_TrackpadTouch    0x0400
#define INDEX_ThumbStickTouch  0x0800

#define HTC_SysClick		0x0001
#define HTC_MenuClick		0x0004
#define HTC_ThumbstickClick 0x0020
#define HTC_GripClick		0x0080
#define HTC_ThumbstickTouch 0x0800



#include <openvr_driver.h>
#include "dataHandler.hpp"

vr::VRInputComponentHandle_t HButtonsCtrl1[11], HAnalogCtrl1[12];
vr::VRInputComponentHandle_t HButtonsCtrl2[11], HAnalogCtrl2[12];

TController MyCtrl, MyCtrl2;

void initController(int contType, int contIndex, vr::PropertyContainerHandle_t m_ulPropertyContainer, vr::VRInputComponentHandle_t m_skeletonHandle, vr::VRInputComponentHandle_t &haptic)
{
	switch (contType) 
	{
	case 0:
		if (contIndex == 1) {
	
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "Knuckles Right");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "{indexcontroller}valve_controller_knu_1_0_right");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_Firmware_ProgrammingTarget_String, "LHR-E217CD01");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, "valve/index_controllerLHR-E217CD01");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{indexcontroller}/icons/right_controller_status_off.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{indexcontroller}/icons/right_controller_status_searching.gif");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{indexcontroller}/icons//right_controller_status_searching_alert.gif");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{indexcontroller}/icons//right_controller_status_ready.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{indexcontroller}/icons//right_controller_status_ready_alert.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{indexcontroller}/icons//right_controller_status_error.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{indexcontroller}/icons//right_controller_status_off.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{indexcontroller}/icons//right_controller_status_ready_low.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, "LHR-E217CD01");
			vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/right", "/skeleton/hand/right", "/pose/raw", vr::VRSkeletalTracking_Partial, nullptr, 0U, &m_skeletonHandle);
			

			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/thumbstick/x", &HAnalogCtrl1[0], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/thumbstick/y", &HAnalogCtrl1[1], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &HAnalogCtrl1[2], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/x", &HAnalogCtrl1[3], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/y", &HAnalogCtrl1[4], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/force", &HAnalogCtrl1[5], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/index", &HAnalogCtrl1[6], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/middle", &HAnalogCtrl1[7], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/ring", &HAnalogCtrl1[8], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/pinky", &HAnalogCtrl1[9], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/grip/force", &HAnalogCtrl1[10], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/grip/value", &HAnalogCtrl1[11], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

			//  Buttons handles
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/click", &HButtonsCtrl1[0]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/touch", &HButtonsCtrl1[1]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/click", &HButtonsCtrl1[2]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/touch", &HButtonsCtrl1[3]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &HButtonsCtrl1[4]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/touch", &HButtonsCtrl1[5]);

			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &HButtonsCtrl1[6]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &HButtonsCtrl1[7]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/touch", &HButtonsCtrl1[8]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/thumbstick/click", &HButtonsCtrl1[9]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/thumbstick/touch", &HButtonsCtrl1[10]);

		}
		else{
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_LeftHand);
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "Knuckles Left");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "{indexcontroller}valve_controller_knu_1_0_Left");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_Firmware_ProgrammingTarget_String, "LHR-E217CD00");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, "valve/index_controllerLHR-E217CD00");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{indexcontroller}/icons/left_controller_status_off.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{indexcontroller}/icons/left_controller_status_searching.gif");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{indexcontroller}/icons//left_controller_status_searching_alert.gif");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{indexcontroller}/icons//left_controller_status_ready.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{indexcontroller}/icons//left_controller_status_ready_alert.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{indexcontroller}/icons//left_controller_status_error.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{indexcontroller}/icons//left_controller_status_off.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{indexcontroller}/icons//left_controller_status_ready_low.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, "LHR-E217CD00");
			vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/left", "/skeleton/hand/left", "/pose/raw", vr::VRSkeletalTracking_Partial, nullptr, 0U, &m_skeletonHandle);
			


			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/thumbstick/x", &HAnalogCtrl2[0], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/thumbstick/y", &HAnalogCtrl2[1], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &HAnalogCtrl2[2], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/x", &HAnalogCtrl2[3], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/y", &HAnalogCtrl2[4], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/force", &HAnalogCtrl2[5], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/index", &HAnalogCtrl2[6], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/middle", &HAnalogCtrl2[7], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/ring", &HAnalogCtrl2[8], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/pinky", &HAnalogCtrl2[9], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/grip/force", &HAnalogCtrl2[10], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/grip/value", &HAnalogCtrl2[11], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

			//  Buttons handles
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/click", &HButtonsCtrl2[0]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/touch", &HButtonsCtrl2[1]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/click", &HButtonsCtrl2[2]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/touch", &HButtonsCtrl2[3]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &HButtonsCtrl2[4]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/touch", &HButtonsCtrl2[5]);

			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &HButtonsCtrl2[6]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &HButtonsCtrl2[7]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/touch", &HButtonsCtrl2[8]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/thumbstick/click", &HButtonsCtrl2[9]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/thumbstick/touch", &HButtonsCtrl2[10]);
		}


		
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_UpdateAvailable_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_ManualUpdate_Bool, false);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_Firmware_ManualUpdateURL_String, "https://developer.valvesoftware.com/wiki/SteamVR/HowTo_Update_Firmware");
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceCanPowerOff_Bool, true);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_ForceUpdateRequired_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Identifiable_Bool, true);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_RemindUpdate_Bool, false);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_TrackPad);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDisplayComponent_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasCameraComponent_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasVirtualDisplayComponent_Bool, false);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerHandSelectionPriority_Int32, 0);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "Valve");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingFirmwareVersion_String, "1562916277 watchman@ValveBuilder02 2019-07-12 FPGA 538(2.26/10/2) BL 0 VRC 1562916277 Radio 1562882729");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_HardwareRevision_String, "product 17 rev 14.1.9 lot 2019/4/20 0");
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_HardwareRevision_Uint64, 286130441U);
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_FirmwareVersion_Uint64, 1562916277U);
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_FPGAVersion_Uint64, 538U);
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_VRCVersion_Uint64, 1562916277U);
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_RadioVersion_Uint64, 1562882729U);
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_DongleVersion_Uint64, 1558748372U);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ResourceRoot_String, "indexcontroller");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, "{indexcontroller}/input/index_controller_profile.json");
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis2Type_Int32, vr::k_eControllerAxis_Trigger);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "knuckles");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingSystemName_String, "PS EYE");

		// create our haptic component
		vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &haptic);

		break;

	case 1:
		if (contIndex == 1) 
		{
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String,"htc/vive_controllerLHR-F94B3BD9");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, "LHR-F94B3BD9");

			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &HButtonsCtrl1[0]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &HButtonsCtrl1[1]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &HButtonsCtrl1[2]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &HButtonsCtrl1[3]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &HButtonsCtrl1[4]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/application_menu/click", &HButtonsCtrl1[5]);

			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &HAnalogCtrl1[0], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/x", &HAnalogCtrl1[1], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/y", &HAnalogCtrl1[2], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
		}
		else 
		{
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_LeftHand);
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, "htc/vive_controllerLHR-F94B3BD8");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, "LHR-F94B3BD8");

			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &HButtonsCtrl2[0]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &HButtonsCtrl2[1]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &HButtonsCtrl2[2]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &HButtonsCtrl2[3]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &HButtonsCtrl2[4]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/application_menu/click", &HButtonsCtrl2[5]);

			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &HAnalogCtrl2[0], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/x", &HAnalogCtrl2[1], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/y", &HAnalogCtrl2[2], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
		}

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingSystemName_String, "lighthouse");
		
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_WillDriftInYaw_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceIsWireless_Bool, true);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceIsCharging_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_UpdateAvailable_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_ManualUpdate_Bool, false);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_Firmware_ManualUpdateURL_String, "https://developer.valvesoftware.com/wiki/SteamVR/HowTo_Update_Firmware");
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceCanPowerOff_Bool, true);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_ForceUpdateRequired_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Identifiable_Bool, true);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_RemindUpdate_Bool, false);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_TrackPad);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDisplayComponent_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasCameraComponent_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasVirtualDisplayComponent_Bool, false);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerHandSelectionPriority_Int32, 0);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "Vive. Controller MV");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "HTC");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingFirmwareVersion_String, "1533720215 htcvrsoftware@firmware-win32 2018-08-08 FPGA 262(1.6/0/0) BL 0 VRC 1533720214 Radio 1532585738");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_HardwareRevision_String, "product 129 rev 1.5.0 lot 2000/0/0 0");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ConnectedWirelessDongle_String, "1E8092840E"); 
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_HardwareRevision_Uint64, 2164327680U);
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_FirmwareVersion_Uint64, 1533720215U);
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_FPGAVersion_Uint64, 262U);
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_VRCVersion_Uint64, 1533720214U);
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_RadioVersion_Uint64, 1532585738U);
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_DongleVersion_Uint64, 1461100729U);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ResourceRoot_String, "htc");

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, "{htc}/input/vive_controller_profile.json");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{htc}/icons/controller_status_off.png");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/controller_status_searching.gif");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/controller_status_searching_alert.gif");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{htc}/icons/controller_status_ready.png");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/controller_status_ready_alert.png");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/controller_status_error.png");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/controller_status_off.png");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/controller_status_ready_low.png");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_controller");

		break;
	}
}


void updateController(int contType, int contIndex){

	switch (contType)
	{
	case 0:
		if (contIndex == 1) {

			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[0], (MyCtrl.Buttons & INDEX_AClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[1], (MyCtrl.Buttons & INDEX_ATouch) != 0, 0);		//a button 
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[2], (MyCtrl.Buttons & INDEX_BClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[3], (MyCtrl.Buttons & INDEX_BTouch) != 0, 0);		//b button
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[4], (MyCtrl.Buttons & INDEX_SYSClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[5], (MyCtrl.Buttons & INDEX_SYSClick) != 0, 0);	//sys button
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[6], (MyCtrl.Buttons & INDEX_TrackpadTouch) != 0, 0);	//trackpad touch
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[7], (MyCtrl.Trigger), 0);							//trigger click
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[8], (MyCtrl.Buttons & INDEX_FingerMiddle) != 0, 0);	//grip touch
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[9], (MyCtrl.Buttons & INDEX_ThumbStickClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[10], (MyCtrl.Buttons & INDEX_ThumbStickTouch) != 0, 0); //joy touch and click

			//analog stuff
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[0], MyCtrl.JoyAxisX, 0); //joyx
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[1], MyCtrl.JoyAxisY, 0); //joyy
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[2], MyCtrl.Trigger, 0);	//Trigger
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[3], 0, 0); //Trackpad x
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[4], MyCtrl.TrackpY, 0); //Trackpad Y

			

			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[6], ((MyCtrl.Buttons & INDEX_FingerIndex) != 0), 0); //Index
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[7], ((MyCtrl.Buttons & INDEX_FingerMiddle) != 0), 0); //middle
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[8], ((MyCtrl.Buttons & INDEX_FingerRing) != 0), 0); //Ring
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[9], ((MyCtrl.Buttons & INDEX_FingerPinky) != 0), 0); //Pinky

			if ((MyCtrl.Buttons & INDEX_TrackpadTouch) != 0) {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[5], 1.f, 0); //Trackpad force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[4], MyCtrl.TrackpY, 0); //Trackpad Y
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[3], 0.f, 0); //Trackpad X
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[5], 0.f, 0); //Trackpad force
			}

			if ((MyCtrl.Buttons & INDEX_FingerMiddle) != 0) {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[10], 0.5f, 0); //grip force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[11], 0.5f, 0); //grip value

				if ((MyCtrl.Buttons & INDEX_FingerRing) != 0) {
					vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[10], 0.75f, 2); //grip force
					vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[11], 0.75f, 2); //grip value

					if ((MyCtrl.Buttons & INDEX_FingerPinky) != 0) {
						vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[10], 1.f, 5); //grip force
						vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[11], 1.f, 5); //grip value
					}
				}
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[10], 0.f, 0); //grip force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[11], 0.f, 0); //grip value
			}
		}
		else
		{
			//Controller2
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[0], (MyCtrl2.Buttons & INDEX_AClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[1], (MyCtrl2.Buttons & INDEX_ATouch) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[2], (MyCtrl2.Buttons & INDEX_BClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[3], (MyCtrl2.Buttons & INDEX_BTouch) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[4], (MyCtrl2.Buttons & INDEX_SYSClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[5], (MyCtrl2.Buttons & INDEX_SYSClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[6], (MyCtrl2.Buttons & INDEX_TrackpadTouch) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[7], (MyCtrl2.Trigger), 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[8], (MyCtrl2.Buttons & INDEX_FingerMiddle) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[9], (MyCtrl2.Buttons & INDEX_ThumbStickClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[10], (MyCtrl2.Buttons & INDEX_ThumbStickTouch) != 0, 0);

			//analog stuff
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[0], MyCtrl2.JoyAxisX, 0); //joyx
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[1], MyCtrl2.JoyAxisY, 0); //joyy
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[2], MyCtrl2.Trigger, 0);	//Trigger
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[3], 0, 0); //Trackpad x
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[4], 0, 0); //Trackpad Y

			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[6], ((MyCtrl2.Buttons & INDEX_FingerIndex) != 0), 0); //Index
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[7], ((MyCtrl2.Buttons & INDEX_FingerMiddle) != 0), 0); //middle
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[8], ((MyCtrl2.Buttons & INDEX_FingerRing) != 0), 0); //Ring
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[9], ((MyCtrl2.Buttons & INDEX_FingerPinky) != 0), 0); //Pinky

			if ((MyCtrl2.Buttons & INDEX_TrackpadTouch) != 0) {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[5], 1.f, 0); //Trackpad force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[4], MyCtrl2.TrackpY, 0); //Trackpad Y
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[3], 0.0f, 0); //Trackpad X
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[5], 0.f, 0); //Trackpad force
			}

			if ((MyCtrl2.Buttons & INDEX_FingerMiddle) != 0) {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[10], 0.5f, 0); //grip force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[11], 0.5f, 0); //grip value

				if ((MyCtrl2.Buttons & INDEX_FingerRing) != 0) {
					vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[10], 0.75f, 2); //grip force
					vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[11], 0.75f, 2); //grip value

					if ((MyCtrl2.Buttons & INDEX_FingerPinky) != 0) {
						vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[10], 1.f, 5); //grip force
						vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[11], 1.f, 5); //grip value
					}
				}
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[10], 0.f, 0); //grip force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[11], 0.f, 0); //grip value
			}
		}

		if ((MyCtrl2.Buttons & INDEX_ThumbStickClick) != 0 && (MyCtrl.Buttons & INDEX_ThumbStickClick) != 0 && (MyCtrl2.Trigger > 0.9f) && (MyCtrl.Trigger > 0.9f))
		{
			SetCentering();
		}
		break;



	case 1:
		if (contIndex == 1) 
		{
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[0], (MyCtrl.Buttons& HTC_SysClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[2], (MyCtrl.Buttons& HTC_ThumbstickTouch) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[3], (MyCtrl.Buttons& HTC_ThumbstickClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[4], (MyCtrl.Buttons& HTC_GripClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[5], (MyCtrl.Buttons& HTC_MenuClick) != 0, 0);

			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[0], MyCtrl.Trigger, 0); 
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[1], MyCtrl.JoyAxisX, 0); 
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl1[2], MyCtrl.JoyAxisY, 0);	

			if (MyCtrl.Trigger > 0.9f)
			{
				vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[1], 1 != 0, 0);
			}
			else
			{
				vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl1[1], 0 != 0, 0);
			}

		}
		else
		{
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[0], (MyCtrl2.Buttons& HTC_SysClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[2], (MyCtrl2.Buttons& HTC_ThumbstickTouch) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[3], (MyCtrl2.Buttons& HTC_ThumbstickClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[4], (MyCtrl2.Buttons& HTC_GripClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[5], (MyCtrl2.Buttons& HTC_MenuClick) != 0, 0);

			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[0], MyCtrl2.Trigger, 0);
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[1], MyCtrl2.JoyAxisX, 0);
			vr::VRDriverInput()->UpdateScalarComponent(HAnalogCtrl2[2], MyCtrl2.JoyAxisY, 0);

			if (MyCtrl2.Trigger > 0.9f) 
			{
				vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[1], 1 != 0, 0);
			}
			else 
			{
				vr::VRDriverInput()->UpdateBooleanComponent(HButtonsCtrl2[1], 0 != 0, 0);
			}

		}

		if ((MyCtrl2.Buttons & HTC_SysClick) != 0 && (MyCtrl.Buttons & HTC_SysClick) != 0 && (MyCtrl2.Trigger > 0.9f) && (MyCtrl.Trigger > 0.9f))
		{
			SetCentering();
		}

		break;
	}
}


std::string getCtrlSerial(int contType, int contIndex) {
	
	switch (contType) 
	{
	case 0:
		if (contIndex == 1) {
			return "LHR-E217CD01";
		}
		else
		{
			return "LHR-E217CD00";
		}
		break;
	case 1:
		if (contIndex == 1) {
			return "LHR-F94B3BD9";
		}
		else 
		{
			return "LHR-F94B3BD8";
		}
		break;
	}
}


#endif