/// ---------------------------------------------------------------------------
/// Copyright (C) 2016 by SIA. All rights reserved.
/// @file SerialPort.h
/// @date 2016/11/10
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Serial port communication
///
/// Represents a serial port resource
///
/// @version 1.0.0
/// @note Camel case
/// ---------------------------------------------------------------------------

#ifdef __QNXNTO__

#pragma once

class SerialPort
{
public:
	SerialPort(char *portName, int baudRate, int parity, int dataBits, int stopBits) :
		portName(portName), baudRate(baudRate), parity(parity), dataBits(dataBits), stopBits(stopBits)
	{
	}
	int Open();
	void DiscardInBuffer();
	int Read(unsigned char *buffer, int offset, int count);
	void DiscardOutBuffer();
	void Write(unsigned char *buffer, int offset, int count);
	void Close();
private:
	int fildes;
	char *portName;
	int baudRate;
	int parity;
	int dataBits;
	int stopBits;
};

#endif
