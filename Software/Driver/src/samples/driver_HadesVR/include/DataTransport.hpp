#pragma once

#ifndef _DATA_TRANSPORT_H_
#define _DATA_TRANSPORT_H_

#include <inttypes.h>

class DataTransport {
public:
	virtual ~DataTransport() {}

	virtual int Start() = 0;
	virtual void Stop() = 0;

	virtual bool IsConnected() = 0;
	virtual int ReadPacket(uint8_t* buffer, size_t length) = 0;
};

#endif /* _DATA_TRANSPORT_H */