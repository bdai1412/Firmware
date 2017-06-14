#ifdef __QNXNTO__

#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include "SerialPort.h"

/// @return 0: success, 1: unable to open serial port, 2: invalid baud rate, 3: invalid parity, 4: invalid number of data bits, 5: invalid number of stop bits
int SerialPort::Open()
{
	/// 1 Opens a new serial port connection
	fildes = open(portName, O_RDWR | O_NOCTTY);
	if (fildes == -1)
		return 1;

	/// 2 Set termios
	struct termios termios;

	/// General
	memset(&termios, 0, sizeof(struct termios));
	termios.c_cflag = CLOCAL | CREAD; //Ignore modem status lines, enable receiver

	/// Baud Rate
	int baudRateX;
	switch (baudRate)
	{
	case 600:
		baudRateX = B600;
		break;
	case 1200:
		baudRateX = B1200;
		break;
	case 2400:
		baudRateX = B2400;
		break;
	case 4800:
		baudRateX = B4800;
		break;
	case 9600:
		baudRateX = B9600;
		break;
	case 19200:
		baudRateX = B19200;
		break;
	case 38400:
		baudRateX = B38400;
		break;
	case 57600:
		baudRateX = B57600;
		break;
	case 115200:
		baudRateX = B115200;
		break;
	default:
		close(fildes);
		return 2;
	}
	cfsetispeed(&termios, baudRateX);
	cfsetospeed(&termios, baudRateX);

	/// Parity
	switch (parity)
	{
	case 1:
		termios.c_cflag &= ~PARENB; //No parity
		break;
	case 2:
		termios.c_cflag |= PARENB;
		termios.c_cflag |= PARODD; //Odd parity
		break;
	case 3:
		termios.c_cflag |= PARENB;
		termios.c_cflag &= ~PARODD; //Even parity
		break;
	default:
		close(fildes);
		return 3;
	}

	/// Data bits
	switch (dataBits)
	{
	case 5:
		termios.c_cflag |= CS5;
		break;
	case 6:
		termios.c_cflag |= CS6;
		break;
	case 7:
		termios.c_cflag |= CS7;
		break;
	case 8:
		termios.c_cflag |= CS8;
		break;
	default:
		close(fildes);
		return 4;
	}

	/// Stop bits
	switch (stopBits)
	{
	case 1:
		termios.c_cflag &= ~CSTOPB;
		break;
	case 2:
		termios.c_cflag |= CSTOPB;
		break;
	default:
		close(fildes);
		return 5;
	}

	/// Make the change immediately
	tcsetattr(fildes, TCSANOW, &termios);

	return 0;
}

void SerialPort::DiscardInBuffer()
{
	tcflush(fildes, TCIFLUSH);
}

int SerialPort::Read(unsigned char *buffer, int offset, int count)
{
	return read(fildes, buffer + offset, count);
}

void SerialPort::DiscardOutBuffer()
{
	tcflush(fildes, TCOFLUSH);
}

void SerialPort::Write(unsigned char *buffer, int offset, int count)
{
	write(fildes, buffer + offset, count);
}

void SerialPort::Close()
{
	close(fildes);
}

#endif
