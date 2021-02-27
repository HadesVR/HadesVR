//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>
#include "driverlog.h"


#include <vector>
#include <thread>
#include <chrono>

#include <windows.h>
#include <atlbase.h>
#include <atlstr.h> 
#include "dataHandler.hpp"

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

inline HmdQuaternion_t HmdQuaternion_Init( double w, double x, double y, double z )
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

inline void HmdMatrix_SetIdentity( HmdMatrix34_t *pMatrix )
{
	pMatrix->m[0][0] = 1.f;
	pMatrix->m[0][1] = 0.f;
	pMatrix->m[0][2] = 0.f;
	pMatrix->m[0][3] = 0.f;
	pMatrix->m[1][0] = 0.f;
	pMatrix->m[1][1] = 1.f;
	pMatrix->m[1][2] = 0.f;
	pMatrix->m[1][3] = 0.f;
	pMatrix->m[2][0] = 0.f;
	pMatrix->m[2][1] = 0.f;
	pMatrix->m[2][2] = 1.f;
	pMatrix->m[2][3] = 0.f;
}


// keys for use with the settings API
static const char * const k_pch_Display_Section = "DiyVR";
static const char * const k_pch_Sample_SerialNumber_String = "serialNumber";
static const char * const k_pch_Sample_ModelNumber_String = "modelNumber";
static const char * const k_pch_Sample_WindowX_Int32 = "windowX";
static const char * const k_pch_Sample_WindowY_Int32 = "windowY";
static const char * const k_pch_Sample_WindowWidth_Int32 = "windowWidth";
static const char * const k_pch_Sample_WindowHeight_Int32 = "windowHeight";
static const char * const k_pch_Sample_RenderWidth_Int32 = "renderWidth";
static const char * const k_pch_Sample_RenderHeight_Int32 = "renderHeight";
static const char * const k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char * const k_pch_Sample_DisplayFrequency_Float = "displayFrequency";
static const char * const k_pch_Sample_DistortionK1_Float = "DistortionK1";
static const char * const k_pch_Sample_DistortionK2_Float = "DistortionK2";
static const char * const k_pch_Sample_ZoomWidth_Float = "ZoomWidth";
static const char * const k_pch_Sample_ZoomHeight_Float = "ZoomHeight";
static const char * const k_pch_Sample_FOV_Float = "FOV";
static const char * const k_pch_Sample_IPD_Float = "IPD";
static const char * const k_pch_Sample_DistanceBetweenEyes_Int32 = "DistanceBetweenEyes";
static const char * const k_pch_Sample_ScreenOffsetX_Int32 = "ScreenOffsetX";
static const char * const k_pch_Sample_ScreenOffsetY_Int32 = "ScreenOffsetY";
static const char * const k_pch_Sample_Stereo_Bool = "Stereo";
static const char * const k_pch_Sample_DisplayOnDesktop = "IsDisplayOnDesktop";
static const char * const k_pch_Sample_DisplayReal = "IsDisplayReal";
static const char * const k_pch_Sample_DebugMode_Bool = "DebugMode";
static const char * const k_pch_Sample_EnableControllers = "EnableControllers";
static const char * const k_pch_Sample_Controller_Type = "ControllerType";
static const char * const k_pch_Sample_ComPort = "ComPort";

#define SUCCESS 0
#define FAILURE 1

#define IB_AClick           0x0001
#define IB_ATouch           0x0002
#define IB_BClick           0x0004
#define IB_BTouch           0x0008
#define IB_SYSClick         0x0010
#define IB_ThumbStickClick  0x0020
#define IB_FingerIndex      0x0040
#define IB_FingerMiddle     0x0080
#define IB_FingerRing       0x0100
#define IB_FingerPinky      0x0200
#define IB_TrackpadTouch    0x0400
#define IB_ThumbStickTouch  0x0800

HMODULE hDll;
THMD MyHMD;
TController MyCtrl, MyCtrl2;

bool HMDConnected = false, ctrlsConnected = false, ctrlsEnabled = false;

double DegToRad(double f) {
	return f * (3.14159265358979323846 / 180);
}

int comPort = 3;
//Velocity
double FirstCtrlLastPos[3] = { 0, 0, 0 }, SecondCtrlLastPos[3] = { 0, 0, 0 };
milliseconds deltaTime;

inline vr::HmdQuaternion_t retquat(double qW, double qX, double qY, double qZ)
{
	vr::HmdQuaternion_t q;
	// Abbreviations for the various angular functions
	q.w = qW;
	q.x = qX;
	q.y = qY;
	q.z = qZ;

	return q;
}
//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------

/*class CWatchdogDriver_Sample : public IVRWatchdogProvider
{
public:
	CWatchdogDriver_Sample()
	{
		m_pWatchdogThread = nullptr;
	}

	virtual EVRInitError Init( vr::IVRDriverContext *pDriverContext ) ;
	virtual void Cleanup() ;

private:
	std::thread *m_pWatchdogThread;
};

CWatchdogDriver_Sample g_watchdogDriverNull;


EVRInitError CWatchdogDriver_Sample::Init( vr::IVRDriverContext *pDriverContext )
{
	VR_INIT_WATCHDOG_DRIVER_CONTEXT( pDriverContext );
	//InitDriverLog( vr::VRDriverLog() );

	// Watchdog mode on Windows starts a thread that listens for the 'Y' key on the keyboard to 
	// be pressed. A real driver should wait for a system button event or something else from the 
	// the hardware that signals that the VR system should start up.

	return VRInitError_None;
}


void CWatchdogDriver_Sample::Cleanup()
{
	//CleanupDriverLog();
}*/


//-----------------------------------------------------------------------------
// Purpose:
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
		vr::VRSettings()->GetString( k_pch_Display_Section, k_pch_Sample_SerialNumber_String, buf, sizeof( buf ) );
		m_sSerialNumber = buf;

		vr::VRSettings()->GetString( k_pch_Display_Section, k_pch_Sample_ModelNumber_String, buf, sizeof( buf ) );
		m_sModelNumber = buf;

		m_nWindowX = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_WindowX_Int32 );
		m_nWindowY = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_WindowY_Int32 );
		m_nWindowWidth = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_WindowWidth_Int32 );
		m_nWindowHeight = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_WindowHeight_Int32 );
		m_nRenderWidth = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_RenderWidth_Int32 );
		m_nRenderHeight = vr::VRSettings()->GetInt32( k_pch_Display_Section, k_pch_Sample_RenderHeight_Int32 );
		m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat( k_pch_Display_Section, k_pch_Sample_SecondsFromVsyncToPhotons_Float );
		m_flDisplayFrequency = vr::VRSettings()->GetFloat( k_pch_Display_Section, k_pch_Sample_DisplayFrequency_Float );
		m_fDistortionK1 = vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_DistortionK1_Float);
		m_fDistortionK2 = vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_DistortionK2_Float);
		m_fZoomWidth = vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_ZoomWidth_Float);
		m_fZoomHeight = vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_ZoomHeight_Float);
		m_fFOV = (vr::VRSettings()->GetFloat(k_pch_Display_Section, k_pch_Sample_FOV_Float) * 3.14159265358979323846 / 180); //radians
		m_nDistanceBetweenEyes = vr::VRSettings()->GetInt32(k_pch_Display_Section, k_pch_Sample_DistanceBetweenEyes_Int32);
		m_nScreenOffsetX = vr::VRSettings()->GetInt32(k_pch_Display_Section, k_pch_Sample_ScreenOffsetX_Int32);
		m_nScreenOffsetY = vr::VRSettings()->GetInt32(k_pch_Display_Section, k_pch_Sample_ScreenOffsetY_Int32);
		m_bStereoMode = vr::VRSettings()->GetBool(k_pch_Display_Section, k_pch_Sample_Stereo_Bool);
		m_bDebugMode = vr::VRSettings()->GetBool(k_pch_Display_Section, k_pch_Sample_DebugMode_Bool);
		m_displayOnDesktop = vr::VRSettings()->GetBool(k_pch_Display_Section, k_pch_Sample_DisplayOnDesktop);
		m_displayReal = vr::VRSettings()->GetBool(k_pch_Display_Section, k_pch_Sample_DisplayReal);
		ctrlsEnabled = vr::VRSettings()->GetBool(k_pch_Display_Section, k_pch_Sample_EnableControllers);

		
		DriverLog( "Window: %d %d %d %d\n", m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight );
		DriverLog( "Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight );
		DriverLog( "Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons );
		DriverLog( "Display Frequency: %f\n", m_flDisplayFrequency );
		DriverLog( "IPD: %f\n", m_flIPD );
		DriverLog("Controllers enabled: %d\n", ctrlsEnabled);
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
		vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.f );
		vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency );
		vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons );

		
		vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2 );

		// avoid "not fullscreen" warnings from vrmonitor
		vr::VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_IsOnDesktop_Bool, m_displayOnDesktop);

		//Debug mode activate Windowed Mode (borderless fullscreen), lock to 30 FPS 
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_DisplayDebugMode_Bool, m_bDebugMode);

		// Icons can be configured in code or automatically configured by an external file "drivername\resources\driver.vrresources".
		// Icon properties NOT configured in code (post Activate) are then auto-configured by the optional presence of a driver's "drivername\resources\driver.vrresources".
		// In this manner a driver can configure their icons in a flexible data driven fashion by using an external file.
		//
		// The structure of the driver.vrresources file allows a driver to specialize their icons based on their HW.
		// Keys matching the value in "Prop_ModelNumber_String" are considered first, since the driver may have model specific icons.
		// An absence of a matching "Prop_ModelNumber_String" then considers the ETrackedDeviceClass ("HMD", "Controller", "GenericTracker", "TrackingReference")
		// since the driver may have specialized icons based on those device class names.
		//
		// An absence of either then falls back to the "system.vrresources" where generic device class icons are then supplied.
		//
		// Please refer to "bin\drivers\sample\resources\driver.vrresources" which contains this sample configuration.
		//
		// "Alias" is a reserved key and specifies chaining to another json block.
		//
		// In this sample configuration file (overly complex FOR EXAMPLE PURPOSES ONLY)....
		//
		// "Model-v2.0" chains through the alias to "Model-v1.0" which chains through the alias to "Model-v Defaults".
		//
		// Keys NOT found in "Model-v2.0" would then chase through the "Alias" to be resolved in "Model-v1.0" and either resolve their or continue through the alias.
		// Thus "Prop_NamedIconPathDeviceAlertLow_String" in each model's block represent a specialization specific for that "model".
		// Keys in "Model-v Defaults" are an example of mapping to the same states, and here all map to "Prop_NamedIconPathDeviceOff_String".
	
	//	vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{oculus}/resources/icons/cv1_headset_off.png" );
	//	vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{oculus}/resources/icons/cv1_headset_searching.b4bfb144.gif" );
	//	vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{oculus}/resources/icons/cv1_headset_alert_searching.b4bfb144.gif" );
	//	vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{oculus}/resources/icons/cv1_headset_ready.b4bfb144.png" );
	//	vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{oculus}/resources/icons/cv1_headset_ready_alert.b4bfb144.png" );
	//	vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{oculus}/resources/icons/cv1_headset_error.b4bfb144.png" );
	//	vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{oculus}/resources/icons/cv1_headset_standby.b4bfb144.png" );
	//	vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{oculus}/resources/icons/headset_sample_status_ready_low.png" );

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
		/**pnY = 0;
				*pnWidth = m_nWindowWidth / 2;
				*pnHeight = m_nWindowHeight;

				if ( eEye == Eye_Left )
				{
					*pnX = 0;
				}
				else
				{
					*pnX = m_nWindowWidth / 2;
				}*/

		if (m_bStereoMode) {

			*pnY = m_nScreenOffsetY;
			*pnWidth = m_nWindowWidth / 2;
			*pnHeight = m_nWindowHeight;

			if (eEye == Eye_Left)
			{
				*pnX = m_nDistanceBetweenEyes + m_nScreenOffsetX;
			}
			else
			{
				*pnX = (m_nWindowWidth / 2) - m_nDistanceBetweenEyes + m_nScreenOffsetX;
			}
		}
		else { //Mono mode
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
		if (m_bStereoMode) {
			
			*pfLeft = -m_fFOV;
			*pfRight = m_fFOV;
			*pfTop = -m_fFOV;
			*pfBottom = m_fFOV;
		}
		else { //Mono
			*pfLeft = (float)m_nRenderWidth / m_nRenderHeight * -1;
			*pfRight = (float)m_nRenderWidth / m_nRenderHeight;
			*pfTop = -1.0;
			*pfBottom = 1.0;
		}
	}

	virtual DistortionCoordinates_t ComputeDistortion( EVREye eEye, float fU, float fV ) 
	{
		DistortionCoordinates_t coordinates;

		float hX;
		float hY;
		double rr;
		double r2;
		double theta;

		rr = sqrt((fU - 0.5f)*(fU - 0.5f) + (fV - 0.5f)*(fV - 0.5f));
		r2 = rr * (1 + m_fDistortionK1 * (rr*rr) + m_fDistortionK2 * (rr*rr*rr*rr));
		theta = atan2(fU - 0.5f, fV - 0.5f);
		hX = sin(theta)*r2*m_fZoomWidth;
		hY = cos(theta)*r2*m_fZoomHeight;

		coordinates.rfBlue[0] = hX + 0.5f;
		coordinates.rfBlue[1] = hY + 0.5f;
		coordinates.rfGreen[0] = hX + 0.5f;
		coordinates.rfGreen[1] = hY + 0.5f;
		coordinates.rfRed[0] = hX + 0.5f;
		coordinates.rfRed[1] = hY + 0.5f;

		return coordinates;
	}

	virtual DriverPose_t GetPose() 
	{
		DriverPose_t pose = { 0 };

		if (HMDConnected) {
			pose.poseIsValid = true;
			pose.result = TrackingResult_Running_OK;
			pose.deviceIsConnected = true;
		}
		else
		{
			pose.poseIsValid = false;
			pose.result = TrackingResult_Uninitialized;
			pose.deviceIsConnected = false;
		}

		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

		if (HMDConnected) {

			//hmd data
			GetHMDData(&MyHMD);

			//Set head tracking rotation
			HmdQuaternion_t HMDQuat;
			HMDQuat.w = MyHMD.qW;
			HMDQuat.x = MyHMD.qX;
			HMDQuat.y = MyHMD.qY;
			HMDQuat.z = MyHMD.qZ;
			pose.qRotation = HMDQuat;
			//Set head position tracking
			pose.vecPosition[0] = MyHMD.X;
			pose.vecPosition[1] = MyHMD.Z;
			pose.vecPosition[2] = MyHMD.Y;
		}

		return pose;
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

	float realfreq;

	int32_t m_nWindowX;
	int32_t m_nWindowY;
	int32_t m_nWindowWidth;
	int32_t m_nWindowHeight;
	int32_t m_nRenderWidth;
	int32_t m_nRenderHeight;
	float m_flSecondsFromVsyncToPhotons;
	float m_flDisplayFrequency;
	float m_flIPD;
	float m_fDistortionK1;
	float m_fDistortionK2;
	float m_fZoomWidth;
	float m_fZoomHeight;
	float m_fFOV;
	int32_t m_nDistanceBetweenEyes;
	int32_t m_nScreenOffsetY;
	int32_t m_nScreenOffsetX;
	bool m_bStereoMode = true;
	bool m_bDebugMode;
	bool m_displayReal;
	bool m_displayOnDesktop;
};

//-----------------------------------------------------------------------------
// Purpose:
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
		DriverLog("[DIYVR] Initializing controller %d",ControllerIndex);
		switch (ControllerIndex)
		{
		case 1:
			Ctrl1Index_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl1Index_t);											
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
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-E217CD01");
			vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/right", "/skeleton/hand/right", "/pose/raw", vr::VRSkeletalTracking_Partial, nullptr, 0U, &m_skeletonHandle);
			break;
		case 2:
			Ctrl2Index_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl2Index_t);
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
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-E217CD00");
			vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/left", "/skeleton/hand/left", "/pose/raw", vr::VRSkeletalTracking_Partial, nullptr, 0U, &m_skeletonHandle);
			break;
		}

	

		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_DeviceBatteryPercentage_Float, 1.f);
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
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingSystemName_String, "PS EYE");
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_DeviceBatteryPercentage_Float, 1.f);

		// avoid "not fullscreen" warnings from vrmonitor
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);


		vr::VRDriverInput()->CreateScalarComponent( m_ulPropertyContainer, "/input/thumbstick/x", &HAnalog[0], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/thumbstick/Y", &HAnalog[1], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &HAnalog[2], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/x", &HAnalog[3], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/y", &HAnalog[4], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/force", &HAnalog[5], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/index", &HAnalog[6], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/middle", &HAnalog[7], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/ring", &HAnalog[8], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/finger/pinky", &HAnalog[9], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/grip/force", &HAnalog[10], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/grip/value", &HAnalog[11], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

		//  Buttons handles
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/click", &HButtons[0]);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/touch", &HButtons[1]);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/click", &HButtons[2]);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/touch", &HButtons[3]);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &HButtons[4]);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/touch", &HButtons[5]);

		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &HButtons[6]);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &HButtons[7]);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/touch", &HButtons[8]);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/thumbstick/click", &HButtons[9]);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/thumbstick/touch", &HButtons[10]);

		// create our haptic component
		vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);

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
		DriverPose_t pose = { 0 };
		//pose.poseIsValid = false;
		pose.poseIsValid = true;
		//pose.result = TrackingResult_Calibrating_OutOfRange;
		pose.result = TrackingResult_Running_OK;
		pose.deviceIsConnected = true;

		pose.qWorldFromDriverRotation = HmdQuaternion_Init( 1, 0, 0, 0 );
		pose.qDriverFromHeadRotation = HmdQuaternion_Init( 1, 0, 0, 0 );

		//Controllers positions and rotations
		if (ControllerIndex == 1) {

			pose.vecPosition[0] = MyCtrl.X;
			pose.vecPosition[1] = MyCtrl.Z;
			pose.vecPosition[2] = MyCtrl.Y;

			//Velocity, right?
			pose.vecVelocity[0] = (pose.vecPosition[0] - FirstCtrlLastPos[0]) * 1000 / max((int)deltaTime.count(), 1) / 3; // div 3 - ghosting fix, there are right ways to remove ghosting?
			pose.vecVelocity[1] = (pose.vecPosition[1] - FirstCtrlLastPos[1]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			pose.vecVelocity[2] = (pose.vecPosition[2] - FirstCtrlLastPos[2]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			FirstCtrlLastPos[0] = pose.vecPosition[0];
			FirstCtrlLastPos[1] = pose.vecPosition[1];
			FirstCtrlLastPos[2] = pose.vecPosition[2];

			//Rotation first controller
			pose.qRotation = retquat(MyCtrl.qW, MyCtrl.qX, MyCtrl.qY, MyCtrl.qZ);

		} else { 
			//Controller2
			pose.vecPosition[0] = MyCtrl2.X;
			pose.vecPosition[1] = MyCtrl2.Z;
			pose.vecPosition[2] = MyCtrl2.Y;

			//Velocity
			pose.vecVelocity[0] = (pose.vecPosition[0] - SecondCtrlLastPos[0]) * 1000 / max((int)deltaTime.count(), 1) / 3; 
			pose.vecVelocity[1] = (pose.vecPosition[1] - SecondCtrlLastPos[1]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			pose.vecVelocity[2] = (pose.vecPosition[2] - SecondCtrlLastPos[2]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			SecondCtrlLastPos[0] = pose.vecPosition[0];
			SecondCtrlLastPos[1] = pose.vecPosition[1];
			SecondCtrlLastPos[2] = pose.vecPosition[2];

			pose.qRotation = retquat(MyCtrl2.qW, MyCtrl2.qX, MyCtrl2.qY, MyCtrl2.qZ);
		}

		return pose;
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
		
		if (ControllerIndex == 1) {

			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[0], (MyCtrl.Buttons & IB_AClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[1], (MyCtrl.Buttons & IB_ATouch) != 0, 0);		//a button 
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[2], (MyCtrl.Buttons & IB_BClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[3], (MyCtrl.Buttons & IB_BTouch) != 0, 0);		//b button
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[4], (MyCtrl.Buttons & IB_SYSClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[5], (MyCtrl.Buttons & IB_SYSClick) != 0, 0);	//sys button

			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[6], (MyCtrl.Buttons & IB_TrackpadTouch) != 0, 0);	//trackpad touch
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[7], (MyCtrl.Trigger), 0);							//trigger click
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[8], (MyCtrl.Buttons & IB_FingerMiddle) != 0, 0);	//grip touch
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[9], (MyCtrl.Buttons & IB_ThumbStickClick) != 0, 0);	
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[10], (MyCtrl.Buttons & IB_ThumbStickTouch) != 0, 0); //joy touch and click

			//analog stuff
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[0], MyCtrl.JoyAxisX, 0); //joyx
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[1], MyCtrl.JoyAxisY, 0); //joyy
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[2], MyCtrl.Trigger, 0);	//Trigger
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[3], 0, 0); //Trackpad x
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[4], MyCtrl.TrackpY, 0); //Trackpad Y

			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[6], ((MyCtrl.Buttons & IB_FingerIndex) != 0), 0); //Index
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[7], ((MyCtrl.Buttons & IB_FingerMiddle) != 0), 0); //middle
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[8], ((MyCtrl.Buttons & IB_FingerRing) != 0), 0); //Ring
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[9], ((MyCtrl.Buttons & IB_FingerPinky) != 0), 0); //Pinky
			
			if ((MyCtrl.Buttons & IB_TrackpadTouch) != 0) {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[5], 1.f, 0); //Trackpad force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[4], MyCtrl.TrackpY, 0); //Trackpad Y
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[3], 0.1f, 0); //Trackpad Y
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[5], 0.f, 0); //Trackpad force
			}

			if ((MyCtrl.Buttons & IB_FingerMiddle) != 0) {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[10], 0.5f, 0); //grip force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[11], 0.5f, 0); //grip value

				if ((MyCtrl.Buttons & IB_FingerRing) != 0) {
					vr::VRDriverInput()->UpdateScalarComponent(HAnalog[10], 0.75f, 2); //grip force
					vr::VRDriverInput()->UpdateScalarComponent(HAnalog[11], 0.75f, 2); //grip value

					if ((MyCtrl.Buttons & IB_FingerPinky) != 0) {
						vr::VRDriverInput()->UpdateScalarComponent(HAnalog[10], 1.f, 5); //grip force
						vr::VRDriverInput()->UpdateScalarComponent(HAnalog[11], 1.f, 5); //grip value
					}
				}
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[10], 0.f, 0); //grip force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[11], 0.f, 0); //grip value
			}

		} else {
			//Controller2
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[0], (MyCtrl2.Buttons & IB_AClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[1], (MyCtrl2.Buttons & IB_ATouch) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[2], (MyCtrl2.Buttons & IB_BClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[3], (MyCtrl2.Buttons & IB_BTouch) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[4], (MyCtrl2.Buttons & IB_SYSClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[5], (MyCtrl2.Buttons & IB_SYSClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[6], (MyCtrl2.Buttons & IB_TrackpadTouch) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[7], (MyCtrl2.Trigger), 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[8], (MyCtrl2.Buttons & IB_FingerMiddle) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[9], (MyCtrl2.Buttons & IB_ThumbStickClick) != 0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(HButtons[10], (MyCtrl2.Buttons & IB_ThumbStickTouch) != 0, 0);

			//analog stuff
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[0], MyCtrl2.JoyAxisX, 0); //joyx
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[1], MyCtrl2.JoyAxisY, 0); //joyy
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[2], MyCtrl2.Trigger, 0);	//Trigger
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[3], 0, 0); //Trackpad x
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[4], 0, 0); //Trackpad Y

			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[6], ((MyCtrl2.Buttons& IB_FingerIndex) != 0), 0); //Index
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[7], ((MyCtrl2.Buttons& IB_FingerMiddle) != 0), 0); //middle
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[8], ((MyCtrl2.Buttons& IB_FingerRing) != 0), 0); //Ring
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[9], ((MyCtrl2.Buttons& IB_FingerPinky) != 0), 0); //Pinky

			if ((MyCtrl2.Buttons & IB_TrackpadTouch) != 0) {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[5], 1.f, 0); //Trackpad force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[4], MyCtrl2.TrackpY, 0); //Trackpad Y
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[3], 0.1f, 0); //Trackpad Y
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[5], 0.f, 0); //Trackpad force
			}
		
			if ((MyCtrl2.Buttons & IB_FingerMiddle) != 0) {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[10], 0.5f, 0); //grip force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[11], 0.5f, 0); //grip value

				if ((MyCtrl2.Buttons & IB_FingerRing) != 0) {
					vr::VRDriverInput()->UpdateScalarComponent(HAnalog[10], 0.75f, 2); //grip force
					vr::VRDriverInput()->UpdateScalarComponent(HAnalog[11], 0.75f, 2); //grip value

					if ((MyCtrl2.Buttons & IB_FingerPinky) != 0) {
						vr::VRDriverInput()->UpdateScalarComponent(HAnalog[10], 1.f, 5); //grip force
						vr::VRDriverInput()->UpdateScalarComponent(HAnalog[11], 1.f, 5); //grip value
					}
				}
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[10], 0.f, 0); //grip force
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[11], 0.f, 0); //grip value
			}
		}

		if ((MyCtrl2.Buttons & IB_ThumbStickClick) != 0 && (MyCtrl.Buttons & IB_ThumbStickClick) != 0 && (MyCtrl2.Trigger == 1) && (MyCtrl.Trigger == 1))
		{
			SetCentering();
		}

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
		
		switch (ControllerIndex)
		{
		case 1:
			return "LHR-E217CD01";
			break;
		case 2:
			return "LHR-E217CD00";
			break;
		}
	}

private:
	vr::TrackedDeviceIndex_t Ctrl1Index_t;
	vr::TrackedDeviceIndex_t Ctrl2Index_t;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	//vr::VRInputComponentHandle_t m_compA;
	//vr::VRInputComponentHandle_t m_compB;
	//vr::VRInputComponentHandle_t m_compC;
	vr::VRInputComponentHandle_t m_compHaptic;

	vr::VRInputComponentHandle_t HButtons[11], HAnalog[12];
	//std::string m_sSerialNumber;
	//std::string m_sModelNumber;
};

//-----------------------------------------------------------------------------
// Purpose:
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
	C_HMDDeviceDriver *m_pNullHmdLatest = nullptr;
	C_ControllerDriver *m_pController = nullptr;
	C_ControllerDriver *m_pController2 = nullptr;
};

CServerDriver_Sample g_serverDriverNull;

EVRInitError CServerDriver_Sample::Init( vr::IVRDriverContext *pDriverContext )
{
	VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );
	InitDriverLog( vr::VRDriverLog() );

	//this is stupid
	comPort = vr::VRSettings()->GetInt32(k_pch_Display_Section, k_pch_Sample_ComPort);
	ctrlsEnabled = vr::VRSettings()->GetBool(k_pch_Display_Section, k_pch_Sample_EnableControllers);

	StartData(comPort);
	DriverLog("[DIYVR] Starting data stream on COMPort: =%d\n", comPort);

	if (!PSMConnected)
	{
		DriverLog("[DIYVR] PSMoveService not connected!!!!");
		return VRInitError_Driver_Failed;
	}

	if (SerialConnected)
	{
		HMDConnected = true;
		if (ctrlsEnabled) {
			ctrlsConnected = true;
		}
		
	}
	else 
	{
		HMDConnected = false;
		ctrlsConnected = false;
	}

	if (HMDConnected) 
	{
		m_pNullHmdLatest = new C_HMDDeviceDriver();
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pNullHmdLatest->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_pNullHmdLatest);
	}

	if (ctrlsConnected) {
		m_pController = new C_ControllerDriver();
		m_pController->SetControllerIndex(1);
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController);


		m_pController2 = new C_ControllerDriver();
		m_pController2->SetControllerIndex(2);
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController2->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController2);
	}

	return VRInitError_None;
}

void CServerDriver_Sample::Cleanup() 
{
	if (HMDConnected) {
		delete m_pNullHmdLatest;
		m_pNullHmdLatest = NULL;
	}

	if (ctrlsConnected) {
		delete m_pController;
		m_pController = NULL;
		delete m_pController2;
		m_pController2 = NULL;
	}
	if (hDll != NULL) {
		FreeLibrary(hDll);
		hDll = nullptr;
	}

	if (SerialConnected) {
		SerialConnected = false;
		if (pCtrlthread) {
			pCtrlthread->join();
			delete pCtrlthread;
			pCtrlthread = nullptr;
		}
		CloseHandle(hSerial);
	}
	if (PSMConnected) {
		PSMConnected = false;
		PSM_Shutdown();
		if (pPSMUpdatethread) {
			pPSMUpdatethread->join();
			delete pPSMUpdatethread;
			pPSMUpdatethread = nullptr;
		}
	}

	CleanupDriverLog();
}

void CServerDriver_Sample::RunFrame()
{
	//Velocity
	static milliseconds lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	deltaTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()) - lastMillis;
	lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

	if ( m_pNullHmdLatest )
	{
		m_pNullHmdLatest->RunFrame();
	}
	if (ctrlsConnected) {
		GetControllersData(&MyCtrl, &MyCtrl2);

		if (m_pController)
		{
			m_pController->RunFrame();
		}
		if (m_pController2)
		{
			m_pController2->RunFrame();
		}

		vr::VREvent_t vrEvent;
		while ( vr::VRServerDriverHost()->PollNextEvent( &vrEvent, sizeof( vrEvent ) ) )
		{
			if ( m_pController )
			{
				m_pController->ProcessEvent(vrEvent);
			}
			if (m_pController2)
			{
				m_pController2->ProcessEvent(vrEvent);
			}
		}
	}
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

