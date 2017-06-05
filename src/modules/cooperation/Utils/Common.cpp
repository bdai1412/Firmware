#include <iostream>

#ifdef _WIN32
#include <Windows.h>
#include <ctime>
#else
#include <sys/time.h>
#endif

#include "Common.h"

using namespace std;

void Common::MemoryPrint(unsigned char *data, int length)
{
	if (length == 0)
		return;
	int i;
	for (i = 0; i < length - 1; i++)
		fprintf(stdout, "%02X ", data[i]);
	fprintf(stdout, "%02X", data[length - 1]);
}

void Common::BinaryPrint(int number)
{
	int binary = number % 2;
	number = number >> 1;
	if (number)
		BinaryPrint(number);
	cout << binary;
}

double Common::GetTimestamp()
{
	struct timeval tv;
#ifdef _WIN32
	SYSTEMTIME systm;
	GetLocalTime(&systm);
	struct tm tm;
	tm.tm_year = systm.wYear - 1900;
	tm.tm_mon = systm.wMonth - 1;
	tm.tm_mday = systm.wDay;
	tm.tm_hour = systm.wHour;
	tm.tm_min = systm.wMinute;
	tm.tm_sec = systm.wSecond;
	tm.tm_isdst = -1;
	time_t clock = mktime(&tm);
	tv.tv_sec = (long)clock;
	tv.tv_usec = systm.wMilliseconds * 1000;
#else
	gettimeofday(&tv, NULL);
#endif
	return tv.tv_sec + tv.tv_usec / 1000000.0;
}

void Common::StatusPrint(const char *message, const char *file, const char *function, const int line)
{
	cout << message << ", at file: " << file << ", function: " << function << ", line: " << line << ", timestamp: " << fixed
		<< GetTimestamp() << endl;
}
