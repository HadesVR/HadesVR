#pragma once

static const char* const k_pch_Display_Section = "Display";


static const char* const k_pch_Sample_WindowX_Int32 = "windowX";
static const char* const k_pch_Sample_WindowY_Int32 = "windowY";
static const char* const k_pch_Sample_WindowWidth_Int32 = "windowWidth";
static const char* const k_pch_Sample_WindowHeight_Int32 = "windowHeight";
static const char* const k_pch_Sample_RenderWidth_Int32 = "renderWidth";
static const char* const k_pch_Sample_RenderHeight_Int32 = "renderHeight";
static const char* const k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char* const k_pch_Sample_DisplayFrequency_Float = "displayFrequency";
static const char* const k_pch_Sample_DistortionK1_Float = "DistortionK1";
static const char* const k_pch_Sample_DistortionK2_Float = "DistortionK2";
static const char* const k_pch_Sample_ZoomWidth_Float = "ZoomWidth";
static const char* const k_pch_Sample_ZoomHeight_Float = "ZoomHeight";
static const char* const k_pch_Sample_FOV_Float = "FOV";
static const char* const k_pch_Sample_IPD_Float = "IPD";
static const char* const k_pch_Sample_DistanceBetweenEyes_Int32 = "DistanceBetweenEyes";
static const char* const k_pch_Sample_ScreenOffsetX_Int32 = "ScreenOffsetX";
static const char* const k_pch_Sample_ScreenOffsetY_Int32 = "ScreenOffsetY";
static const char* const k_pch_Sample_Stereo_Bool = "Stereo";
static const char* const k_pch_Sample_DisplayOnDesktop = "IsDisplayOnDesktop";
static const char* const k_pch_Sample_DisplayReal = "IsDisplayReal";
static const char* const k_pch_Sample_DebugMode_Bool = "DebugMode";

//////////////////////////////////////////////////////////////////////////////////////////////////
static const char* const k_pch_Driver_Section = "Driver";

static const char* const k_pch_HID_PID_Int32 = "HID_PID";
static const char* const k_pch_HID_VID_Int32 = "HID_VID";
static const char* const k_pch_PSMS_UPDATE_RATE_Int32 = "PSMSTrackerFrequency";

///////////////////////////////////////////////////////////////////////////////////////////////////

static const char* const k_pch_HMD_Section = "HMD";

static const char* const k_pch_HMD_SerialNumber_String = "serialNumber";
static const char* const k_pch_HMD_ModelNumber_String = "modelNumber";
static const char* const k_pch_HMD_Enable_Bool = "EnableHMD";
static const char* const k_pch_HMD_FilterBeta_Float = "FilterBeta";

static const char* const k_pch_HMD_YawOffset_Float = "HMDYawOffset";
static const char* const k_pch_HMD_PitchOffset_Float = "HMDPitchOffset";
static const char* const k_pch_HMD_RollOffset_Float = "HMDRollOffset";

///////////////////////////////////////////////////////////////////////////////////////////////////

static const char* const k_pch_Controllers_Section = "Controllers";

static const char* const k_pch_Controller_Enable_Bool = "EnableControllers";
static const char* const k_pch_Controller_Mode_Int32 = "ControllerMode";

static const char* const k_pch_Controller1_YawOffset_Float = "CTRLRightYawOffset";
static const char* const k_pch_Controller1_PitchOffset_Float = "CTRLRightPitchOffset";
static const char* const k_pch_Controller1_RollOffset_Float = "CTRLRightRollOffset";

static const char* const k_pch_Controller2_YawOffset_Float = "CTRLLeftYawOffset";
static const char* const k_pch_Controller2_PitchOffset_Float = "CTRLLeftPitchOffset";
static const char* const k_pch_Controller2_RollOffset_Float = "CTRLLeftRollOffset";

///////////////////////////////////////////////////////////////////////////////////////////////////

static const char* const k_pch_Tracker_Section = "Trackers";

static const char* const k_pch_Tracker_Enable_Bool = "EnableTrackers";
static const char* const k_pch_Tracker_Mode_Int32 = "TrackerMode";

///////////////////////////////////////////////////////////////////////////////////////////////////
//this all goes in the steamvr.vrsettings file
static const char* const k_pch_Calibration_Section = "HadesVR_Calibration";

static const char* const k_pch_Calibration_HMDW_Float = "HMDW";
static const char* const k_pch_Calibration_HMDY_Float = "HMDY";

static const char* const k_pch_Calibration_CONT1W_Float = "CONTRightW";
static const char* const k_pch_Calibration_CONT1Y_Float = "CONTRightY";

static const char* const k_pch_Calibration_CONT2W_Float = "CONTLeftW";
static const char* const k_pch_Calibration_CONT2Y_Float = "CONTLeftY";


static const char* const k_pch_Calibration_TRKWaistW_Float = "TRKWaistW";
static const char* const k_pch_Calibration_TRKWaistY_Float = "TRKWaistY";

static const char* const k_pch_Calibration_TRKLeftW_Float = "TRKLeftW";
static const char* const k_pch_Calibration_TRKLeftY_Float = "TRKLeftY";

static const char* const k_pch_Calibration_TRKRightW_Float = "TRKRightW";
static const char* const k_pch_Calibration_TRKRightY_Float = "TRKRightY";

///////////////////////////////////////////////////////////////////////////////////////////////////