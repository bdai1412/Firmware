/// ---------------------------------------------------------------------------
/// Copyright (C) 2016 by SIA. All rights reserved.
/// @file UdpClient.h
/// @date 2016/11/10
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief UDP socket communication
///
/// Provides User Datagram Protocol (UDP) network services
/// Please specify the socket library using the linker options
///
/// @version 1.0.0
/// @note Camel case
/// ---------------------------------------------------------------------------

#ifdef __QNXNTO__

#pragma once

#include <netinet/in.h>

class UdpClient
{
public:
	UdpClient(const char *ip, int port);
	int Bind(const char *ip, int port);
	int Receive(unsigned char *dgram, int offset, int bytes);
	void SetBroadcast();
	void Send(unsigned char *dgram, int offset, int bytes);
	void Close();
private:
	int fildes;
	struct sockaddr_in addr;
	int length;
};

#endif
