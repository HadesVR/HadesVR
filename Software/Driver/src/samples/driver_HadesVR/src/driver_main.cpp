//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include "openvr/openvr_driver.h"
#include "driverlog.h"


#include <vector>
#include <thread>
#include <chrono>

#include <windows.h>
#include <atlbase.h>
#include <atlstr.h> 
#include "dataHandler.h"
#include "devices.hpp"
#include "settingsAPIKeys.h"

using namespace vr;
using namespace std::chrono;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

#define SUCCESS 0
#define FAILURE 1

bool HMDConnected = false, ctrlsConnected = false, trackersConnected = false;

int controllerMode, trackerMode;

double DegToRad(double f) {
	return f * (M_PI / 180);
}

//-----------------------------------------------------------------------------
// Purpose: HMD mess over here
//-----------------------------------------------------------------------------

class C_HMDDeviceDriver : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent
{
public:
	C_HMDDeviceDriver(  )
	{
		HMDIndex_t = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

		//DriverLog( "Using settings values\n" );
		m_flIPD = vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_IPD_Float);

		char buf[1024];
		vr::VRSettings()->GetString(k_pch_HMD_Section, k_pch_HMD_SerialNumber_String, buf, sizeof( buf ) );
		m_sSerialNumber = buf;

		vr::VRSettings()->GetString(k_pch_HMD_Section, k_pch_HMD_ModelNumber_String, buf, sizeof( buf ) );
		m_sModelNumber = buf;

		m_nWindowX = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_WindowX_Int32 );
		m_nWindowY = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_WindowY_Int32 );
		m_nWindowWidth = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_WindowWidth_Int32 );
		m_nWindowHeight = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_WindowHeight_Int32 );
		m_nRenderWidth = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_RenderWidth_Int32 );
		m_nRenderHeight = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_RenderHeight_Int32 );
		m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat( k_pch_Display_Section, k_pch_Sample_SecondsFromVsyncToPhotons_Float );
		m_flDisplayFrequency = vr::VRSettings()->GetFloat( k_pch_Display_Section, k_pch_Sample_DisplayFrequency_Float );
		m_fViewportZoom = vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_ViewportZoom_Float);
		m_fFOV = (vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_FOV_Float) * 3.14159265358979323846 / 180); //radians
		m_fDistanceBetweenLenses = vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_DistanceBetweenLenses_Float);
		m_fDistanceBetweenViews = vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_DistanceBetweenViews_Float);
		m_fDisplayWidth = vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_DisplayWidth_Float);
		m_bIsSinglePanel = vr::VRSettings()->GetBool(k_pch_Display_Section, k_pch_Sample_IsSinglePanel_Bool);
		m_nScreenOffsetX = vr::VRSettings()->GetInt32(k_pch_Display_Section, k_pch_Sample_ScreenOffsetX_Int32);
		m_nScreenOffsetY = vr::VRSettings()->GetInt32(k_pch_Display_Section, k_pch_Sample_ScreenOffsetY_Int32);
		m_bStereoMode = vr::VRSettings()->GetBool(k_pch_Display_Section, k_pch_Sample_Stereo_Bool);
		m_bDebugMode = vr::VRSettings()->GetBool(k_pch_Display_Section, k_pch_Sample_DebugMode_Bool);
		m_displayOnDesktop = vr::VRSettings()->GetBool(k_pch_Display_Section, k_pch_Sample_DisplayOnDesktop);
		m_displayReal = vr::VRSettings()->GetBool(k_pch_Display_Section, k_pch_Sample_DisplayReal);
		m_displayCantAngle = ((vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_DisplayAngle_Float) / 2) * 3.14159265358979323846 / 180); //radians

		m_fDistortion_Red_K[0] = vr::VRSettings()->GetFloat(k_pch_Distortion_Section, k_pch_Distortion_Red_K1_Float);
		m_fDistortion_Red_K[1] = vr::VRSettings()->GetFloat(k_pch_Distortion_Section, k_pch_Distortion_Red_K2_Float);
		m_fDistortion_Red_K[2] = vr::VRSettings()->GetFloat(k_pch_Distortion_Section, k_pch_Distortion_Red_K3_Float);
		m_fDistortion_Green_K[0] = vr::VRSettings()->GetFloat(k_pch_Distortion_Section, k_pch_Distortion_Green_K1_Float);
		m_fDistortion_Green_K[1] = vr::VRSettings()->GetFloat(k_pch_Distortion_Section, k_pch_Distortion_Green_K2_Float);
		m_fDistortion_Green_K[2] = vr::VRSettings()->GetFloat(k_pch_Distortion_Section, k_pch_Distortion_Green_K3_Float);
		m_fDistortion_Blue_K[0] = vr::VRSettings()->GetFloat(k_pch_Distortion_Section, k_pch_Distortion_Blue_K1_Float);
		m_fDistortion_Blue_K[1] = vr::VRSettings()->GetFloat(k_pch_Distortion_Section, k_pch_Distortion_Blue_K2_Float);
		m_fDistortion_Blue_K[2] = vr::VRSettings()->GetFloat(k_pch_Distortion_Section, k_pch_Distortion_Blue_K3_Float);

		m_enableDirectMode = vr::VRSettings()->GetBool(k_pch_Driver_Section, k_pch_DirectModeEnable_Bool);
		m_directModeEDID_PID = vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_DirectMode_EDID_PID_Int32);
		m_directModeEDID_VID = vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_DirectMode_EDID_VID_Int32);

		DriverLog( "Window: %d %d %d %d\n", m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight );
		DriverLog( "Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight );
		DriverLog( "Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons );
		DriverLog( "Display Frequency: %f\n", m_flDisplayFrequency );
		DriverLog( "IPD: %f\n", m_flIPD );
	}

	virtual ~C_HMDDeviceDriver()
	{
	}


	virtual EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId ) 
	{	
		HMDIndex_t = unObjectId;
		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer( HMDIndex_t );

		vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str() );
		vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str() );
		vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_UserIpdMeters_Float, m_flIPD );
		vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.1f);
		vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency );
		vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons );
		vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2 );
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_HmdTrackingStyle_Int32, HmdTrackingStyle_OutsideInCameras);

		// avoid "not fullscreen" warnings from vrmonitor
		vr::VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_IsOnDesktop_Bool, m_displayOnDesktop);
		vr::VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_HasDriverDirectModeComponent_Bool, false);
		//Debug mode activate Windowed Mode (borderless fullscreen), lock to 30 FPS 
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_DisplayDebugMode_Bool, m_bDebugMode);
		//Direct mode crap
		if (m_enableDirectMode) {
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_EdidVendorID_Int32, m_directModeEDID_VID);
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_EdidProductID_Int32, m_directModeEDID_PID);
			m_displayOnDesktop = false;
			m_displayReal = true;
			vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, m_displayOnDesktop);
		}
		//Apply display cant angle (yaw)
		vr::VRServerDriverHost()->SetDisplayEyeToHead(HMDIndex_t, CalcMatFromEuler(m_displayCantAngle, -(m_flIPD / 2)), CalcMatFromEuler(-m_displayCantAngle, m_flIPD / 2));

		return VRInitError_None;
	}

	virtual void Deactivate() 
	{
		HMDIndex_t = vr::k_unTrackedDeviceIndexInvalid;
	}

	virtual void EnterStandby()
	{
	}

	void *GetComponent( const char *pchComponentNameAndVersion )
	{
		if ( !_stricmp( pchComponentNameAndVersion, vr::IVRDisplayComponent_Version ) )
		{
			return (vr::IVRDisplayComponent*)this;
		}

		// override this to add a component to a driver
		return NULL;
	}

	virtual void PowerOff() 
	{
	}

	/** debug request from a client */
	virtual void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize ) 
	{
		if( unResponseBufferSize >= 1 )
			pchResponseBuffer[0] = 0;
	}

	virtual void GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight ) 
	{
		*pnX = m_nWindowX;
		*pnY = m_nWindowY;
		*pnWidth = m_nWindowWidth;
		*pnHeight = m_nWindowHeight;
	}

	virtual bool IsDisplayOnDesktop() 
	{
		return m_displayOnDesktop;
	}

	virtual bool IsDisplayRealDisplay() 
	{
		return m_displayReal;
	}

	virtual void GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight ) 
	{
		*pnWidth = m_nRenderWidth;
		*pnHeight = m_nRenderHeight;
	}

	virtual void GetEyeOutputViewport( EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight ) 
	{
		if (m_bStereoMode) 
		{
			*pnY = m_nScreenOffsetY;
			*pnWidth = m_nWindowWidth / 2;
			*pnHeight = m_nWindowHeight;

			if (eEye == Eye_Left)
			{
				*pnX = m_nScreenOffsetX;
			}
			else
			{
				*pnX = (m_nWindowWidth / 2) + m_nScreenOffsetX;
			}
		}
		else 
		{ //Mono mode
			pnY = 0;
			*pnWidth = m_nRenderWidth;
			*pnHeight = m_nRenderHeight;

			if (eEye == Eye_Left)
			{
				*pnX = (m_nWindowWidth - m_nRenderWidth) / 2;
			}
			else
			{
				*pnX = m_nWindowWidth;
			}
		}
	}

	virtual void GetProjectionRaw( EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom ) 
	{
		if (m_bStereoMode) 
		{
			*pfTop = -tan(m_fFOV* m_nRenderHeight / m_nRenderWidth / 2);
			*pfBottom = tan(m_fFOV * m_nRenderHeight / m_nRenderWidth / 2);
			if (m_bIsSinglePanel == false)
			{
				*pfLeft = -tan(m_fFOV / 2);
				*pfRight = tan(m_fFOV / 2);
			}
			else
			{
				float projectionLength = 0.25 * m_fDisplayWidth * 1 / tan(m_fFOV / 2);
				float centerDifference = 0.25 * m_fDisplayWidth - 0.5 * m_fDistanceBetweenViews;
				if (eEye == Eye_Left)
				{
					*pfLeft = -((0.25 * m_fDisplayWidth + centerDifference) / projectionLength);
					*pfRight = (0.25 * m_fDisplayWidth - centerDifference) / projectionLength;
				}
				else
				{
					*pfLeft = -((0.25 * m_fDisplayWidth - centerDifference) / projectionLength);
					*pfRight = (0.25 * m_fDisplayWidth + centerDifference) / projectionLength;
				}
			}
		}
		else 
		{//Mono
			*pfLeft = (float)m_nRenderWidth / m_nRenderHeight * -1;
			*pfRight = (float)m_nRenderWidth / m_nRenderHeight;
			*pfTop = -1.0;
			*pfBottom = 1.0;
		}
	}

	virtual DistortionCoordinates_t ComputeDistortion( EVREye eEye, float fU, float fV ) 
	{
		DistortionCoordinates_t coordinates;

		float hX_Red, hX_Green, hX_Blue;
		float hY_Red, hY_Green, hY_Blue;

		double rr, rr2, rr4, rr6;
		double theta;

		double r2_Red, r2_Green, r2_Blue;

		float centerOffsetU, centerOffsetV = 0.5f;
		if (m_bIsSinglePanel == false)
		{
			centerOffsetU = 0.5f;
		}
		else
		{
			float centerDifference = 0.25 * m_fDisplayWidth - 0.5 * m_fDistanceBetweenLenses;
			if (eEye == Eye_Left)
			{
				centerOffsetU = 0.5f * (0.25 * m_fDisplayWidth + centerDifference) / (0.25 * m_fDisplayWidth);
			}
			else
			{
				centerOffsetU = 0.5f * (0.25 * m_fDisplayWidth - centerDifference) / (0.25 * m_fDisplayWidth);
			}
		}
		
		rr = sqrt((fU - centerOffsetU) * (fU - centerOffsetU) + (fV - centerOffsetV) * (fV - centerOffsetV));
		rr2 = rr * rr;
		rr4 = rr2 * rr2;
		rr6 = rr2 * rr4;
		theta = atan2(fU - centerOffsetU, fV - centerOffsetV);

		r2_Red = rr * (1 + m_fDistortion_Red_K[0] * rr2 + m_fDistortion_Red_K[1] * rr4 + m_fDistortion_Red_K[2] * rr6);
		r2_Green = rr * (1 + m_fDistortion_Green_K[0] * rr2 + m_fDistortion_Green_K[1] * rr4 + m_fDistortion_Green_K[2] * rr6);
		r2_Blue = rr * (1 + m_fDistortion_Blue_K[0] * rr2 + m_fDistortion_Blue_K[1] * rr4 + m_fDistortion_Blue_K[2] * rr6);

		hX_Red = sin(theta) * r2_Red * m_fViewportZoom;
		hY_Red = cos(theta) * r2_Red * m_fViewportZoom;
		hX_Green = sin(theta) * r2_Green * m_fViewportZoom;
		hY_Green = cos(theta) * r2_Green * m_fViewportZoom;
		hX_Blue = sin(theta) * r2_Blue * m_fViewportZoom;
		hY_Blue = cos(theta) * r2_Blue * m_fViewportZoom;

		coordinates.rfRed[0] = hX_Red + centerOffsetU;
		coordinates.rfRed[1] = hY_Red + centerOffsetV;
		coordinates.rfGreen[0] = hX_Green + centerOffsetU;
		coordinates.rfGreen[1] = hY_Green + centerOffsetV;
		coordinates.rfBlue[0] = hX_Blue + centerOffsetU;
		coordinates.rfBlue[1] = hY_Blue + centerOffsetV;

		return coordinates;
	}

	virtual DriverPose_t GetPose() 
	{
		return dH.GetHMDPose();
	}
	
	HmdMatrix34_t CalcMatFromEuler(float a, float IPDOffset) {
		HmdMatrix34_t mat;

		mat.m[0][0] = cos(a);
		mat.m[0][1] = 0;
		mat.m[0][2] = sin(a);
		mat.m[0][3] = IPDOffset;
		mat.m[1][0] = 0;
		mat.m[1][1] = 1;
		mat.m[1][2] = 0;
		mat.m[1][3] = 0;
		mat.m[2][0] = -sin(a);
		mat.m[2][1] = 0;
		mat.m[2][2] = cos(a);
		mat.m[2][3] = 0;

		return mat;
	}

	void RunFrame()
	{
		// In a real driver, this should happen from some pose tracking thread.
		// The RunFrame interval is unspecified and can be very irregular if some other
		// driver blocks it for some periodic task.
		if ( HMDIndex_t != vr::k_unTrackedDeviceIndexInvalid )
		{
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated( HMDIndex_t , GetPose(), sizeof( DriverPose_t ) );
		}
	}

	std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
	vr::TrackedDeviceIndex_t HMDIndex_t;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	std::string m_sSerialNumber;
	std::string m_sModelNumber;

	int32_t m_nWindowX;
	int32_t m_nWindowY;
	int32_t m_nWindowWidth;
	int32_t m_nWindowHeight;
	int32_t m_nRenderWidth;
	int32_t m_nRenderHeight;
	float m_flSecondsFromVsyncToPhotons;
	float m_flDisplayFrequency;
	float m_flIPD;
	float m_fViewportZoom;
	float m_fFOV;
	float m_displayCantAngle = 0.f;
	float m_fDistanceBetweenLenses; // distance between the center of the lenses / Unit: m
	float m_fDistanceBetweenViews; // distance between the center of the viewports / Unit: m
	float m_fDisplayWidth; // panel physical length / Unit: m
	bool m_bIsSinglePanel; // single panel or dual panel

	int32_t m_nScreenOffsetY;
	int32_t m_nScreenOffsetX;
	bool m_bStereoMode = true;
	bool m_bDebugMode;
	bool m_displayReal;
	bool m_displayOnDesktop;
	bool m_enableDirectMode = false;
	int32_t m_directModeEDID_PID;
	int32_t m_directModeEDID_VID;

	float m_fDistortion_Red_K[3];
	float m_fDistortion_Green_K[3];
	float m_fDistortion_Blue_K[3];

};

//-----------------------------------------------------------------------------
// Purpose: Controller stuff goes on over here
//-----------------------------------------------------------------------------

class C_ControllerDriver : public vr::ITrackedDeviceServerDriver
{
	vr::VRInputComponentHandle_t m_skeletonHandle;
	int32_t ControllerIndex;
public:
	C_ControllerDriver()
	{
		Ctrl1Index_t = vr::k_unTrackedDeviceIndexInvalid;
		Ctrl2Index_t = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
		m_skeletonHandle = vr::k_ulInvalidInputComponentHandle;
	}

	virtual void SetControllerIndex(int32_t CtrlIndex)
	{
		ControllerIndex = CtrlIndex;
	}

	virtual ~C_ControllerDriver()
	{
	}

	virtual EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId )
	{
		DriverLog("[HadesVR] Initializing controller %d",ControllerIndex);
		switch (ControllerIndex)
		{
		case 1:

			for (size_t i = 0; i < fingerTracking::NUM_BONES; i++)
			{
				m_handBones[i] = fingerTracking::Pose_OpenRight[i];
			}

			Ctrl1Index_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl1Index_t);		
			break;
		case 2:

			for (size_t i = 0; i < fingerTracking::NUM_BONES; i++)
			{
				m_handBones[i] = fingerTracking::Pose_OpenLeft[i];
			}

			Ctrl2Index_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl2Index_t);
			break;
		}

		initDevice(controllerMode, ControllerIndex, m_ulPropertyContainer, m_compHaptic, m_skeletonHandle );

		return VRInitError_None;
	}

	virtual void Deactivate()
	{
		switch (ControllerIndex)
		{
		case 1:
			Ctrl1Index_t = vr::k_unTrackedDeviceIndexInvalid;
			break;
		case 2:
			Ctrl2Index_t = vr::k_unTrackedDeviceIndexInvalid;
			break;
		}
	}


	virtual void EnterStandby()
	{
	}

	void *GetComponent( const char *pchComponentNameAndVersion )
	{
		// override this to add a component to a driver
		return NULL;
	}

	virtual void PowerOff()
	{
	}

	/** debug request from a client */
	virtual void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize )
	{
		if ( unResponseBufferSize >= 1 )
			pchResponseBuffer[0] = 0;
	}

	virtual DriverPose_t GetPose()
	{
		return dH.GetControllersPose(ControllerIndex);
	}

	void RunFrame()
	{
		switch (ControllerIndex)
		{
		case 1:
			if (Ctrl1Index_t != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(Ctrl1Index_t, GetPose(), sizeof(DriverPose_t));
			}
			break;
		case 2:
			if (Ctrl2Index_t != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(Ctrl2Index_t, GetPose(), sizeof(DriverPose_t));
			}
			break;
		}
		dH.GetControllerData(&RightCtrl, &LeftCtrl);

		updateFingerTracking(controllerMode, ControllerIndex, m_skeletonHandle, m_handBones);

		updateDevice(controllerMode, ControllerIndex);
	}

	void UpdateDeviceBattery() 
	{
		vr::VRProperties()->SetFloatProperty(vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl1Index_t), vr::Prop_DeviceBatteryPercentage_Float, RightCtrl.vBat);
		vr::VRProperties()->SetFloatProperty(vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl2Index_t), vr::Prop_DeviceBatteryPercentage_Float, LeftCtrl.vBat);
	}

	void ProcessEvent( const vr::VREvent_t & vrEvent )
	{
		switch ( vrEvent.eventType )
		{
		case vr::VREvent_Input_HapticVibration:
		{
			if ( vrEvent.data.hapticVibration.componentHandle == m_compHaptic )
			{
				// This is where you would send a signal to hardware to trigger actual haptic feedback
			}
		}
		break;
		}
	}

	std::string GetSerialNumber() const { 
		return getDeviceSerial(controllerMode, ControllerIndex);
	}

private:
	vr::TrackedDeviceIndex_t Ctrl1Index_t;
	vr::TrackedDeviceIndex_t Ctrl2Index_t;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;
	
	vr::VRBoneTransform_t m_handBones[31];
	//vr::VRInputComponentHandle_t m_compA;
	//vr::VRInputComponentHandle_t m_compB;
	//vr::VRInputComponentHandle_t m_compC;
	vr::VRInputComponentHandle_t m_compHaptic;
	//std::string m_sSerialNumber;
	//std::string m_sModelNumber;
};

//-----------------------------------------------------------------------------
// Purpose: Tracker stuff goes on down here
//-----------------------------------------------------------------------------

class C_TrackerDriver : public vr::ITrackedDeviceServerDriver
{
	int32_t TrackerIndex;
	vr::VRInputComponentHandle_t m_skeletonHandle;
public:
	C_TrackerDriver()
	{
		TrackerWaist_t = vr::k_unTrackedDeviceIndexInvalid;
		TrackerLeftFoot_t = vr::k_unTrackedDeviceIndexInvalid;
		TrackerRightFoot_t = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
		m_skeletonHandle = vr::k_ulInvalidInputComponentHandle;
	}

	virtual void SetTrackerIndex(int32_t trckIndex)
	{
		TrackerIndex = trckIndex;
	}

	virtual ~C_TrackerDriver()
	{
	}

	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		DriverLog("[HadesVR] Initializing tracker %d", TrackerIndex);
		switch (TrackerIndex)
		{
		case 1:
			TrackerWaist_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(TrackerWaist_t);
			break;
		case 2:
			TrackerLeftFoot_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(TrackerLeftFoot_t);
			break;
		case 3:
			TrackerRightFoot_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(TrackerRightFoot_t);
			break;
		}

		initDevice(100, TrackerIndex, m_ulPropertyContainer, m_compHaptic, m_skeletonHandle);

		return VRInitError_None;
	}

	virtual void Deactivate()
	{
		switch (TrackerIndex)
		{
		case 1:
			TrackerWaist_t = vr::k_unTrackedDeviceIndexInvalid;
			break;
		case 2:
			TrackerLeftFoot_t = vr::k_unTrackedDeviceIndexInvalid;
			break;
		case 3:
			TrackerRightFoot_t = vr::k_unTrackedDeviceIndexInvalid;
			break;
		}
	}


	virtual void EnterStandby()
	{
	}

	void* GetComponent(const char* pchComponentNameAndVersion)
	{
		// override this to add a component to a driver
		return NULL;
	}

	virtual void PowerOff()
	{
	}

	/** debug request from a client */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
	{
		if (unResponseBufferSize >= 1)
			pchResponseBuffer[0] = 0;
	}

	virtual DriverPose_t GetPose()
	{
		DriverPose_t pose = { 0 };
		//pose.poseIsValid = false;
		pose.poseIsValid = true;
		//pose.result = TrackingResult_Calibrating_OutOfRange;
		pose.result = TrackingResult_Running_OK;
		pose.deviceIsConnected = true;

		pose.qWorldFromDriverRotation = HmdQuaternion_t{ 1, 0, 0, 0 };
		pose.qDriverFromHeadRotation = HmdQuaternion_t{ 1, 0, 0, 0 };

		//Controllers positions and rotations
		switch (TrackerIndex)
		{
		case 1:
			//Waist Tracker
			pose.vecPosition[0] = 0.f;
			pose.vecPosition[1] = 0.f;
			pose.vecPosition[2] = 0.f;
			//velocity
			pose.vecVelocity[0] = 0.f;
			pose.vecVelocity[1] = 0.f;
			pose.vecVelocity[2] = 0.f;
			//Rotation
			pose.qRotation = HmdQuaternion_t{ WaistTrk.Rotation.W, WaistTrk.Rotation.X, WaistTrk.Rotation.Y, WaistTrk.Rotation.Z };
			break;
		case 2:
			//Left foot tracker
			pose.vecPosition[0] = 0.f;
			pose.vecPosition[1] = 0.f;
			pose.vecPosition[2] = 0.f;
			//velocity
			pose.vecVelocity[0] = 0.f;
			pose.vecVelocity[1] = 0.f;
			pose.vecVelocity[2] = 0.f;
			//rotation
			pose.qRotation = HmdQuaternion_t{ LeftTrk.Rotation.W, LeftTrk.Rotation.X, LeftTrk.Rotation.Y, LeftTrk.Rotation.Z };
			break;
		case 3:
			//Right foot tracker
			pose.vecPosition[0] = 0.f;
			pose.vecPosition[1] = 0.f;
			pose.vecPosition[2] = 0.f;
			//velocity
			pose.vecVelocity[0] = 0.f;
			pose.vecVelocity[1] = 0.f;
			pose.vecVelocity[2] = 0.f;

			//rotation
			pose.qRotation = HmdQuaternion_t{ RightTrk.Rotation.W, RightTrk.Rotation.X, RightTrk.Rotation.Y, RightTrk.Rotation.Z };
			break;
		}

		return pose;
	}

	void RunFrame()
	{
		switch (TrackerIndex)
		{
		case 1:
			if (TrackerWaist_t != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(TrackerWaist_t, GetPose(), sizeof(DriverPose_t));
			}
			break;
		case 2:
			if (TrackerLeftFoot_t != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(TrackerLeftFoot_t, GetPose(), sizeof(DriverPose_t));
			}
			break;
		case 3:
			if (TrackerRightFoot_t != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(TrackerRightFoot_t, GetPose(), sizeof(DriverPose_t));
			}
			break;
		}
	}

	void UpdateDeviceBattery()
	{
		vr::VRProperties()->SetFloatProperty(vr::VRProperties()->TrackedDeviceToPropertyContainer(TrackerWaist_t), vr::Prop_DeviceBatteryPercentage_Float, WaistTrk.vBat);
		vr::VRProperties()->SetFloatProperty(vr::VRProperties()->TrackedDeviceToPropertyContainer(TrackerLeftFoot_t), vr::Prop_DeviceBatteryPercentage_Float, LeftTrk.vBat);
		vr::VRProperties()->SetFloatProperty(vr::VRProperties()->TrackedDeviceToPropertyContainer(TrackerRightFoot_t), vr::Prop_DeviceBatteryPercentage_Float, RightTrk.vBat);
	}

	void ProcessEvent(const vr::VREvent_t& vrEvent)
	{
		switch (vrEvent.eventType)
		{
		case vr::VREvent_Input_HapticVibration:
		{
			if (vrEvent.data.hapticVibration.componentHandle == m_compHaptic)
			{
				// This is where you would send a signal to hardware to trigger actual haptic feedback
			}
		}
		break;
		}
	}

	std::string GetSerialNumber() const {
		return getDeviceSerial(100, TrackerIndex);
	}

private:
	vr::TrackedDeviceIndex_t TrackerWaist_t;
	vr::TrackedDeviceIndex_t TrackerLeftFoot_t;
	vr::TrackedDeviceIndex_t TrackerRightFoot_t;

	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	//vr::VRInputComponentHandle_t m_compA;
	//vr::VRInputComponentHandle_t m_compB;
	//vr::VRInputComponentHandle_t m_compC;
	vr::VRInputComponentHandle_t m_compHaptic;
	//std::string m_sSerialNumber;
	//std::string m_sModelNumber;
};

//-----------------------------------------------------------------------------
// Purpose: important server driver stuff over here
//-----------------------------------------------------------------------------
class CServerDriver_Sample: public IServerTrackedDeviceProvider
{
public:
	virtual EVRInitError Init( vr::IVRDriverContext *pDriverContext ) ;
	virtual void Cleanup() ;
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame() ;
	virtual bool ShouldBlockStandbyMode()  { return false; }
	virtual void EnterStandby()  {}
	virtual void LeaveStandby()  {}

private:

	void BatteryUpdateThread();
	static void BatteryUpdateThreadEnter(CServerDriver_Sample* ptr) {
		ptr->BatteryUpdateThread();
	}
	bool m_bBatteryUpdateThreadAlive;
	std::thread* m_ptBatteryUpdateThread;
	bool ctrlsEnabled = false, HMDEnabled = false, trackersEnabled = false;
	
	C_HMDDeviceDriver *m_pHmd = nullptr;
	C_ControllerDriver *m_pControllerRight = nullptr;
	C_ControllerDriver *m_pControllerLeft = nullptr;

	C_TrackerDriver* m_pTrackerWaist = nullptr;
	C_TrackerDriver* m_pTrackerLeftFoot = nullptr;
	C_TrackerDriver* m_pTrackerRightFoot = nullptr;
};

CServerDriver_Sample g_serverDriverNull;

EVRInitError CServerDriver_Sample::Init( vr::IVRDriverContext *pDriverContext )
{
	VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );
	InitDriverLog( vr::VRDriverLog() );

	//this is stupid
	ctrlsEnabled = vr::VRSettings()->GetBool(k_pch_Controllers_Section, k_pch_Controller_Enable_Bool);
	controllerMode = vr::VRSettings()->GetInt32(k_pch_Controllers_Section, k_pch_Controller_Mode_Int32);

	HMDEnabled = vr::VRSettings()->GetBool(k_pch_HMD_Section, k_pch_HMD_Enable_Bool);
	trackersEnabled = vr::VRSettings()->GetInt32(k_pch_Tracker_Section, k_pch_Tracker_Enable_Bool);
	trackerMode = vr::VRSettings()->GetInt32(k_pch_Tracker_Section, k_pch_Tracker_Mode_Int32);

	if (trackersEnabled) {
		DriverLog("[TRACKERS] Trackers enabled!");
		if (trackerMode == 0) {
			DriverLog("[TRACKERS] Tracker Mode: Full body");
		}
		else {
			DriverLog("[TRACKERS] Tracker Mode: Waist");
		}
	}

	if (controllerMode > 9) { controllerMode = 0; }

	dH.StartData(vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_HID_PID_Int32), vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_HID_VID_Int32));

	DriverLog("[DataStream] HID value PID = %d , VID = %d\n", vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_HID_PID_Int32), vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_HID_VID_Int32));

	if (dH.HIDConnected)
	{
		if (HMDEnabled) {
			HMDConnected = true;
		}
		if (ctrlsEnabled) {
			ctrlsConnected = true;
		}
		if (trackersEnabled) {
			trackersConnected = true;
		}
	}
	else 
	{
		HMDConnected = false;
		ctrlsConnected = false;
		trackersConnected = false;
		return vr::VRInitError_Init_InterfaceNotFound;
	}

	if (HMDConnected) 
	{
		m_pHmd = new C_HMDDeviceDriver();
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pHmd->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_pHmd);
	}

	if (ctrlsConnected) 
	{
		m_pControllerRight = new C_ControllerDriver();
		m_pControllerRight->SetControllerIndex(1);
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pControllerRight->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pControllerRight);


		m_pControllerLeft = new C_ControllerDriver();
		m_pControllerLeft->SetControllerIndex(2);
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pControllerLeft->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pControllerLeft);
	}

	if (trackersConnected) 
	{
		m_pTrackerWaist = new C_TrackerDriver();
		m_pTrackerWaist->SetTrackerIndex(1);
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pTrackerWaist->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker, m_pTrackerWaist);
		if (trackerMode == 0) {
			m_pTrackerLeftFoot = new C_TrackerDriver();
			m_pTrackerLeftFoot->SetTrackerIndex(2);
			vr::VRServerDriverHost()->TrackedDeviceAdded(m_pTrackerLeftFoot->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker, m_pTrackerLeftFoot);

			m_pTrackerRightFoot = new C_TrackerDriver();
			m_pTrackerRightFoot->SetTrackerIndex(3);
			vr::VRServerDriverHost()->TrackedDeviceAdded(m_pTrackerRightFoot->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker, m_pTrackerRightFoot);
		}
	}

	// battery thread
	m_bBatteryUpdateThreadAlive = true;
	m_ptBatteryUpdateThread = new std::thread(this->BatteryUpdateThreadEnter, this);
	if (!m_bBatteryUpdateThreadAlive) {
		return VRInitError_IPC_Failed;
	}

	return VRInitError_None;
}



void CServerDriver_Sample::Cleanup() 
{
	if (HMDConnected) {
		delete m_pHmd;
		m_pHmd = NULL;
	}

	if (ctrlsConnected) {
		delete m_pControllerRight;
		m_pControllerRight = NULL;
		delete m_pControllerLeft;
		m_pControllerLeft = NULL;
	}

	if (trackersConnected) {
		delete m_pTrackerWaist;
		m_pTrackerWaist = NULL;
		if (trackerMode == 0) {
			delete m_pTrackerLeftFoot;
			m_pTrackerLeftFoot = NULL;
			delete m_pTrackerRightFoot;
			m_pTrackerRightFoot = NULL;
		}
	}

	if (dH.HIDConnected) {
		dH.HIDConnected = false;
		if (dH.pHIDthread) {
			dH.pHIDthread->join();
			delete dH.pHIDthread;
			dH.pHIDthread = nullptr;
			dH.StopData();
		}
	}
	if (dH.PSMConnected) {
		dH.PSMConnected = false;
		PSM_Shutdown();
		if (dH.pPSMUpdatethread) {
			dH.pPSMUpdatethread->join();
			delete dH.pPSMUpdatethread;
			dH.pPSMUpdatethread = nullptr;
		}
	}
	if (m_bBatteryUpdateThreadAlive) {
		m_bBatteryUpdateThreadAlive = false;
		if (m_ptBatteryUpdateThread) {
			m_ptBatteryUpdateThread->join();
			delete m_ptBatteryUpdateThread;
			m_ptBatteryUpdateThread = nullptr;
		}
	}

	CleanupDriverLog();
}

void CServerDriver_Sample::RunFrame()
{
	if ( m_pHmd )
	{
		m_pHmd->RunFrame();
	}
	if (ctrlsConnected) 
	{
		if (m_pControllerRight)
		{
			m_pControllerRight->RunFrame();
		}
		if (m_pControllerLeft)
		{
			m_pControllerLeft->RunFrame();
		}

		vr::VREvent_t vrEvent;
		while ( vr::VRServerDriverHost()->PollNextEvent( &vrEvent, sizeof( vrEvent ) ) )
		{
			if ( m_pControllerRight )
			{
				m_pControllerRight->ProcessEvent(vrEvent);
			}
			if (m_pControllerLeft)
			{
				m_pControllerLeft->ProcessEvent(vrEvent);
			}
		}
	}
	if (trackersConnected) 
	{
		dH.GetTrackersData(&WaistTrk, &LeftTrk, &RightTrk);

		if (m_pTrackerWaist) 
		{
			m_pTrackerWaist->RunFrame();
		}
		if (trackerMode == 0) {
			if (m_pTrackerLeftFoot)
			{
				m_pTrackerLeftFoot->RunFrame();
			}
			if (m_pTrackerRightFoot)
			{
				m_pTrackerRightFoot->RunFrame();
			}
		}

		vr::VREvent_t vrEvent;
		while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
		{
			if (m_pTrackerWaist)
			{
				m_pTrackerWaist->ProcessEvent(vrEvent);
			}
			if (m_pTrackerLeftFoot)
			{
				m_pTrackerLeftFoot->ProcessEvent(vrEvent);
			}
			if (m_pTrackerRightFoot)
			{
				m_pTrackerRightFoot->ProcessEvent(vrEvent);
			}
		}
	}
}

void CServerDriver_Sample::BatteryUpdateThread() {
	
	while (m_bBatteryUpdateThreadAlive) 
	{
		//controllers
		if (m_pControllerRight)
		{
			m_pControllerRight->UpdateDeviceBattery();
		}
		if (m_pControllerLeft)
		{
			m_pControllerLeft->UpdateDeviceBattery();
		}
		//trackers
		if (m_pTrackerWaist) 
		{
			m_pTrackerWaist->UpdateDeviceBattery();
		}
		if (m_pTrackerLeftFoot)
		{
			m_pTrackerLeftFoot->UpdateDeviceBattery();
		}
		if (m_pTrackerRightFoot)
		{
			m_pTrackerRightFoot->UpdateDeviceBattery();
		}
		//night night
		std::this_thread::sleep_for(std::chrono::seconds(10));
	}
	m_bBatteryUpdateThreadAlive = false;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void *HmdDriverFactory( const char *pInterfaceName, int *pReturnCode )
{
	if( 0 == strcmp( IServerTrackedDeviceProvider_Version, pInterfaceName ) )
	{
		return &g_serverDriverNull;
	}
	/*if( 0 == strcmp( IVRWatchdogProvider_Version, pInterfaceName ) )
	{
		return &g_watchdogDriverNull;
	}*/

	if( pReturnCode )
		*pReturnCode = VRInitError_Init_InterfaceNotFound;

	return NULL;
}

