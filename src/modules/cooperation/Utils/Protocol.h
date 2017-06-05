/// ---------------------------------------------------------------------------
/// Copyright (C) 2016 by SIA. All rights reserved.
/// @file Protocol.h
/// @date 2016/11/02
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Standard packet protocol
///
/// Packet structure: header LRC (u8), packet ID (u8), packet length (u8), CRC16 (u16), packet data
///
/// @version 1.0.0
/// @note Camel case
/// ---------------------------------------------------------------------------

#pragma once

const int MaxPacketDataLength = 255;
const int PacketHeaderLength = 5;
const int BufferLength = (PacketHeaderLength + MaxPacketDataLength) * 2;

typedef struct
{
	unsigned char *data;
	int id, length;
} Packet;

typedef struct
{
	unsigned char data[BufferLength];
	int length;
	unsigned int errors;
} Buffer;

class Protocol
{
public:
	Buffer PacketEncode(Packet packet);
	Packet BufferDecode(Buffer *buffer);
private:
	unsigned char CalculateHeaderLrc(unsigned char *data);
	unsigned short CalculateCrc16Ccitt(unsigned char *data, int length);
};
