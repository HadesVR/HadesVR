#include <windows.h>
#include "UARTTransport.hpp"
#include "driverlog.h"
#include "settingsAPIKeys.h"

#define FRAME_MAGIC             ((uint8_t)0xAA)
#define FRAME_HEADER_SIZE       (3u)  // Magic + seqno + size

#define FRAME_MAGIC_INDEX       (0u)
#define FRAME_SEQNO_INDEX       (1u)
#define FRAME_DATA_SIZE_INDEX   (2u)

UARTTransport::UARTTransport()
	: connected(false),
      lastSeqno(0),
      hasReceivedFirstFrame(false),
      hSerial(NULL)
{
    ring_buffer_init(&ringbuffer);
}

int UARTTransport::Start()
{
    char portName[16];
    DWORD baudrate = 230400;
    vr::EVRSettingsError error;

    vr::VRSettings()->GetString(k_pch_Driver_Section, k_pch_UART_Port, portName, sizeof(portName));

    const int32_t br = vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_UART_Baudrate, &error);
    if (error == vr::VRSettingsError_None) {
        baudrate = (DWORD)br;
    }

    DriverLog("[UARTTransport] Opening serial port %s at %d baud", portName, baudrate);
    hSerial = CreateFile(("\\\\.\\" + std::string(portName)).c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        DriverLog("[UARTTransport] Error: Could not open serial port.");
        return 1;
    }

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (GetCommState(hSerial, &dcbSerialParams) == 0) {
        DriverLog("[UARTTransport] Error getting device state");
        CloseHandle(hSerial);
        return 1;
    }

    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (SetCommState(hSerial, &dcbSerialParams) == 0) {
        DriverLog("[UARTTransport] Error setting device parameters");
        CloseHandle(hSerial);
        return 1;
    }

    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if (SetCommTimeouts(hSerial, &timeouts) == 0) {
        DriverLog("[UARTTransport] Error setting timeouts");
        CloseHandle(hSerial);
        return 1;
    }

    DriverLog("[UARTTransport] UART opened successfully");

	connected = true;
	return 0;
}

void UARTTransport::Stop()
{
    DriverLog("[UARTTransport] Closing serial port...");
    if (CloseHandle(hSerial) == 0) {
        DriverLog("[UARTTransport] Error: Could not close serial port.");
    }
    else {
        DriverLog("[UARTTransport] Closed serial port");
    }
    connected = false;
}

bool UARTTransport::IsConnected()
{
	return connected;
}

int UARTTransport::ReadPacket(uint8_t* outBuf, size_t length)
{
    DWORD bytesRead = 0;
    uint8_t tmpBuf[UART_BUFFER_SIZE];

    while (true) {
        // Fill up the internal ring buffer with data from UART if it's not full (or at least as much data that
        // is currently available)
        if (!ring_buffer_is_full(&ringbuffer)) {
            const DWORD readSize = (DWORD)min(UART_BUFFER_SIZE, UART_BUFFER_SIZE - ring_buffer_num_items(&ringbuffer)) - 1;
            if (!ReadFile(hSerial, tmpBuf, readSize, &bytesRead, NULL)) {
                DriverLog("[UARTTransport] Error: Could not read serial port.");
                return 0;
            }

            ring_buffer_queue_arr(&ringbuffer, tmpBuf, (ring_buffer_size_t)bytesRead);
        }

        // All frames start with FRAME_MAGIC, so drop everything until we find it
        uint8_t magic;
        while (ring_buffer_peek(&ringbuffer, &magic, FRAME_MAGIC_INDEX)) {
            if (magic == FRAME_MAGIC) {
                break;  // We found the magic first in ring buffer
            }

            // Did not match, just drop this byte
            ring_buffer_dequeue(&ringbuffer, &magic);
        }

        // Make sure we have enough data for the header so we can find frame length
        const ring_buffer_size_t totalSize = ring_buffer_num_items(&ringbuffer);
        if (totalSize < FRAME_HEADER_SIZE) {
            break;
        }

        // Handle frame if we have all data is there
        uint8_t dataSize = 0u;
        ring_buffer_peek(&ringbuffer, &dataSize, FRAME_DATA_SIZE_INDEX);
        if (totalSize >= (FRAME_MAGIC + dataSize)) {
            // Some logic to log if we miss frames (for debugging performance issues)
            const uint8_t expectedSeqno = (lastSeqno + 1) & UINT8_MAX;
            uint8_t currentSeqno;
            ring_buffer_peek(&ringbuffer, &currentSeqno, FRAME_SEQNO_INDEX);

            if (!hasReceivedFirstFrame) {
                hasReceivedFirstFrame = true;
            }
            else if (currentSeqno != expectedSeqno) {
                DriverLog("[UARTTransport] Expected seqno 0x%02x, got 0x%02x", expectedSeqno, currentSeqno);
            }

            lastSeqno = currentSeqno;

            ring_buffer_dequeue_arr(&ringbuffer, outBuf, FRAME_HEADER_SIZE); // Drop header bytes
            ring_buffer_dequeue_arr(&ringbuffer, outBuf, dataSize);

            // If the received data size does not match the requested one, assume we should fail
            if (length != dataSize) {
                DriverLog("[UARTTransport] Warning: Expected %d bytes, got %d bytes", length, dataSize);
                return 0;
            }

            return (int)dataSize;
        }

        // We should never reach this point since data size is one byte (max 256 bytes in payload)
        // and the ring buffer _should_ be bigger than that. But left here for sake of completeness.
        if (totalSize == UART_BUFFER_SIZE) {
            DriverLog("[UARTTransport] Warning: buffer overrun");
            ring_buffer_init(&ringbuffer);
        }
    }

    return 0;
}
