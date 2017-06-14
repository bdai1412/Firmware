/// ---------------------------------------------------------------------------
/// Copyright (C) 2016 by SIA. All rights reserved.
/// @file Common.h
/// @date 2016/11/04
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Common functions
///
/// Functions: MemoryPrint, GetTimestamp, StatusPrint
///
/// @version 1.0.0
/// @note Camel case
/// ---------------------------------------------------------------------------

#pragma once

const double Pi = 3.14159265358979;
const int Radius = 6367560;

class Common
{
public:
	void MemoryPrint(unsigned char *data, int length);
	void BinaryPrint(int number);
	double GetTimestamp();
	void StatusPrint(const char *message, const char *file, const char *function, const int line);
};
