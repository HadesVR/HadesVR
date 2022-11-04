#include "HIDTransport.hpp"
#include "driverlog.h"
#include "hidapi/hidapi.h"
#include "openvr/openvr_driver.h"
#include "settingsAPIKeys.h"

int HIDTransport::Start()
{
	DriverLog("[HIDTransport] Initializing HID transport");

	if (hid_init() != 0) {
		DriverLog("[HIDTransport] HID init failed.");
		return 1;
	}

	const unsigned short vid = (const unsigned short)vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_HID_PID_Int32);
	const unsigned short pid = (const unsigned short)vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_HID_VID_Int32);
	hHID = hid_open((unsigned short)vid, (unsigned short)pid, NULL);
	if (!hHID) {
		DriverLog("[HIDTransport] Unable to start data stream of device with pid=%d and vid=%d.\n", pid, vid);
		HIDConnected = false;
		return 1;
	}

	HIDInit = true;
	HIDConnected = true;

	DriverLog("[DataStream] HID value PID = %d , VID = %d\n", pid, vid);

	return 0;
}

void HIDTransport::Stop()
{
	hid_close(hHID);
	hid_exit();
	HIDConnected = false;
	HIDInit = false;
}

bool HIDTransport::IsConnected()
{
	return HIDConnected;
}

int HIDTransport::ReadPacket(uint8_t* buffer, size_t length)
{
	return hid_read(hHID, buffer, length); //Result should be greater than 0.
}
