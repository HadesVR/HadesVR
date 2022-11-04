#pragma once

#ifndef _UART_TRANSPORT_H_
#define _UART_TRANSPORT_H_

#include "DataTransport.hpp"
#include "ringbuffer.h"

#define UART_BUFFER_SIZE (RING_BUFFER_SIZE)

class UARTTransport : public DataTransport {
public:
	UARTTransport();

	// Inherited via DataTransport
	virtual int Start() override;
	virtual void Stop() override;
	virtual bool IsConnected() override;
	virtual int ReadPacket(uint8_t* buffer, size_t length) override;

private:
	bool connected;
	ring_buffer_t ringbuffer;
	uint8_t lastSeqno;
	bool hasReceivedFirstFrame;

	HANDLE hSerial;
	DCB dcbSerialParams = { 0 };
	COMMTIMEOUTS timeouts = { 0 };
};

#endif /* _UART_TRANSPORT_H_ */
