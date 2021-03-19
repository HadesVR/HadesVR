//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>
#include "controllerBindings.hpp"
#include "driverlog.h"
#include "settingsAPIKeys.h"

#include <vector>
#include <thread>
#include <chrono>

#include <windows.h>
#include <atlbase.h>
#include <atlstr.h> 
#include "dataHandler.h"

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


#define SUCCESS 0
#define FAILURE 1

HMODULE hDll;
THMD HMDdata;

bool HMDConnected = false, ctrlsConnected = false;

int controllerType;

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
		m_flIPD = vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_Sample_IPD_Float);

		char buf[1024];
		vr::VRSettings()->GetString( k_pch_HMD_Section, k_pch_Sample_SerialNumber_String, buf, sizeof( buf ) );
		m_sSerialNumber = buf;

		vr::VRSettings()->GetString( k_pch_HMD_Section, k_pch_Sample_ModelNumber_String, buf, sizeof( buf ) );
		m_sModelNumber = buf;

		m_nWindowX = vr::VRSettings()->GetInt32( k_pch_HMD_Section, k_pch_Sample_WindowX_Int32 );
		m_nWindowY = vr::VRSettings()->GetInt32( k_pch_HMD_Section, k_pch_Sample_WindowY_Int32 );
		m_nWindowWidth = vr::VRSettings()->GetInt32( k_pch_HMD_Section, k_pch_Sample_WindowWidth_Int32 );
		m_nWindowHeight = vr::VRSettings()->GetInt32( k_pch_HMD_Section, k_pch_Sample_WindowHeight_Int32 );
		m_nRenderWidth = vr::VRSettings()->GetInt32( k_pch_HMD_Section, k_pch_Sample_RenderWidth_Int32 );
		m_nRenderHeight = vr::VRSettings()->GetInt32( k_pch_HMD_Section, k_pch_Sample_RenderHeight_Int32 );
		m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat( k_pch_HMD_Section, k_pch_Sample_SecondsFromVsyncToPhotons_Float );
		m_flDisplayFrequency = vr::VRSettings()->GetFloat( k_pch_HMD_Section, k_pch_Sample_DisplayFrequency_Float );
		m_fDistortionK1 = vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_Sample_DistortionK1_Float);
		m_fDistortionK2 = vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_Sample_DistortionK2_Float);
		m_fZoomWidth = vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_Sample_ZoomWidth_Float);
		m_fZoomHeight = vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_Sample_ZoomHeight_Float);
		m_fFOV = (vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_Sample_FOV_Float) * 3.14159265358979323846 / 180); //radians
		m_nDistanceBetweenEyes = vr::VRSettings()->GetInt32(k_pch_HMD_Section, k_pch_Sample_DistanceBetweenEyes_Int32);
		m_nScreenOffsetX = vr::VRSettings()->GetInt32(k_pch_HMD_Section, k_pch_Sample_ScreenOffsetX_Int32);
		m_nScreenOffsetY = vr::VRSettings()->GetInt32(k_pch_HMD_Section, k_pch_Sample_ScreenOffsetY_Int32);
		m_bStereoMode = vr::VRSettings()->GetBool(k_pch_HMD_Section, k_pch_Sample_Stereo_Bool);
		m_bDebugMode = vr::VRSettings()->GetBool(k_pch_HMD_Section, k_pch_Sample_DebugMode_Bool);
		m_displayOnDesktop = vr::VRSettings()->GetBool(k_pch_HMD_Section, k_pch_Sample_DisplayOnDesktop);
		m_displayReal = vr::VRSettings()->GetBool(k_pch_HMD_Section, k_pch_Sample_DisplayReal);

		controllerType = vr::VRSettings()->GetInt32(k_pch_Controllers_Section, k_pch_Sample_Controller_Type_Int32);
		
		DriverLog( "Window: %d %d %d %d\n", m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight );
		DriverLog( "Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight );
		DriverLog( "Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons );
		DriverLog( "Display Frequency: %f\n", m_flDisplayFrequency );
		DriverLog( "IPD: %f\n", m_flIPD );
		DriverLog("Controller Type: %d\n", controllerType);
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
		vr::VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_HasDriverDirectModeComponent_Bool, false);
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
			dH.GetHMDData(&HMDdata);

			//Set head tracking rotation
			HmdQuaternion_t HMDQuat;
			HMDQuat.w = HMDdata.qW;
			HMDQuat.x = HMDdata.qX;
			HMDQuat.y = HMDdata.qY;
			HMDQuat.z = HMDdata.qZ;
			pose.qRotation = HMDQuat;
			//Set head position tracking
			pose.vecPosition[0] = HMDdata.X;
			pose.vecPosition[1] = HMDdata.Z;
			pose.vecPosition[2] = HMDdata.Y;
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
		DriverLog("[HadesVR] Initializing controller %d",ControllerIndex);
		switch (ControllerIndex)
		{
		case 1:
			Ctrl1Index_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl1Index_t);		
			break;
		case 2:
			Ctrl2Index_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl2Index_t);
			break;
		}

		initController(controllerType, ControllerIndex, m_ulPropertyContainer, m_skeletonHandle, m_compHaptic);

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

			pose.vecPosition[0] = RightCtrl.X;
			pose.vecPosition[1] = RightCtrl.Z;
			pose.vecPosition[2] = RightCtrl.Y;

			//Velocity, right?
			pose.vecVelocity[0] = (pose.vecPosition[0] - FirstCtrlLastPos[0]) * 1000 / max((int)deltaTime.count(), 1) / 3; // div 3 - ghosting fix, there are right ways to remove ghosting?
			pose.vecVelocity[1] = (pose.vecPosition[1] - FirstCtrlLastPos[1]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			pose.vecVelocity[2] = (pose.vecPosition[2] - FirstCtrlLastPos[2]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			FirstCtrlLastPos[0] = pose.vecPosition[0];
			FirstCtrlLastPos[1] = pose.vecPosition[1];
			FirstCtrlLastPos[2] = pose.vecPosition[2];

			//Rotation first controller
			pose.qRotation = retquat(RightCtrl.qW, RightCtrl.qX, RightCtrl.qY, RightCtrl.qZ);

		} else { 
			//Controller2
			pose.vecPosition[0] = LeftCtrl.X;
			pose.vecPosition[1] = LeftCtrl.Z;
			pose.vecPosition[2] = LeftCtrl.Y;

			//Velocity
			pose.vecVelocity[0] = (pose.vecPosition[0] - SecondCtrlLastPos[0]) * 1000 / max((int)deltaTime.count(), 1) / 3; 
			pose.vecVelocity[1] = (pose.vecPosition[1] - SecondCtrlLastPos[1]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			pose.vecVelocity[2] = (pose.vecPosition[2] - SecondCtrlLastPos[2]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			SecondCtrlLastPos[0] = pose.vecPosition[0];
			SecondCtrlLastPos[1] = pose.vecPosition[1];
			SecondCtrlLastPos[2] = pose.vecPosition[2];

			pose.qRotation = retquat(LeftCtrl.qW, LeftCtrl.qX, LeftCtrl.qY, LeftCtrl.qZ);
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
		
		updateController(controllerType, ControllerIndex);
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
		return getCtrlSerial(controllerType, ControllerIndex);
	}

private:
	vr::TrackedDeviceIndex_t Ctrl1Index_t;
	vr::TrackedDeviceIndex_t Ctrl2Index_t;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	//vr::VRInputComponentHandle_t m_compA;
	//vr::VRInputComponentHandle_t m_compB;
	//vr::VRInputComponentHandle_t m_compC;
	vr::VRInputComponentHandle_t m_compHaptic;
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

	void BatteryUpdateThread();
	static void BatteryUpdateThreadEnter(CServerDriver_Sample* ptr) {
		ptr->BatteryUpdateThread();
	}
	bool m_bBatteryUpdateThreadAlive;
	std::thread* m_ptBatteryUpdateThread;
	bool ctrlsEnabled = false, HMDEnabled = false;
	C_HMDDeviceDriver *m_pNullHmdLatest = nullptr;
	C_ControllerDriver *m_pControllerRight = nullptr;
	C_ControllerDriver *m_pControllerLeft = nullptr;
};

CServerDriver_Sample g_serverDriverNull;

EVRInitError CServerDriver_Sample::Init( vr::IVRDriverContext *pDriverContext )
{
	VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );
	InitDriverLog( vr::VRDriverLog() );

	//this is stupid
	comPort = vr::VRSettings()->GetInt32(k_pch_HMD_Section, k_pch_Sample_ComPort_Int32);
	ctrlsEnabled = vr::VRSettings()->GetBool(k_pch_Controllers_Section, k_pch_Sample_EnableControllers_Bool);
	HMDEnabled = vr::VRSettings()->GetBool(k_pch_HMD_Section, k_pch_Sample_EnableHMD_Bool);
	controllerType = vr::VRSettings()->GetInt32(k_pch_Controllers_Section, k_pch_Sample_Controller_Type_Int32);

	dH.StartData(comPort);
	DriverLog("[DataStream] Starting data stream on COMPort: =%d\n", comPort);

	if (dH.SerialConnected)
	{
		if (HMDEnabled) {
			HMDConnected = true;
		}
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
		m_pControllerRight = new C_ControllerDriver();
		m_pControllerRight->SetControllerIndex(1);
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pControllerRight->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pControllerRight);


		m_pControllerLeft = new C_ControllerDriver();
		m_pControllerLeft->SetControllerIndex(2);
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pControllerLeft->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pControllerLeft);
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
		delete m_pNullHmdLatest;
		m_pNullHmdLatest = NULL;
	}

	if (ctrlsConnected) {
		delete m_pControllerRight;
		m_pControllerRight = NULL;
		delete m_pControllerLeft;
		m_pControllerLeft = NULL;
	}
	if (hDll != NULL) {
		FreeLibrary(hDll);
		hDll = nullptr;
	}

	if (dH.SerialConnected) {
		dH.SerialConnected = false;
		if (dH.pCtrlthread) {
			dH.pCtrlthread->join();
			delete dH.pCtrlthread;
			dH.pCtrlthread = nullptr;
		}
		CloseHandle(dH.hSerial);
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
	//Velocity
	static milliseconds lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	deltaTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()) - lastMillis;
	lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

	if ( m_pNullHmdLatest )
	{
		m_pNullHmdLatest->RunFrame();
	}
	if (ctrlsConnected) {
		dH.GetControllersData(&RightCtrl, &LeftCtrl);

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
}

void CServerDriver_Sample::BatteryUpdateThread() {
	
	while (m_bBatteryUpdateThreadAlive) 
	{
		if (m_pControllerRight)
		{
			m_pControllerRight->UpdateDeviceBattery();
		}
		if (m_pControllerLeft)
		{
			m_pControllerLeft->UpdateDeviceBattery();
		}

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

