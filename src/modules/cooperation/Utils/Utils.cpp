#include <iostream>

#ifdef __QNXNTO__
#include <sys/neutrino.h>
#define TestA
#define TestB
#endif

#include "Common.h"
#include "Protocol.h"
#include "SharedMemory.h"
#include "ShmX.h"
#include "SerialPort.h"
#include "UdpClient.h"

using namespace std;

int mainX()
{
	cout << "Welcome to the Utils Process" << endl;

	/// 1 Common function test
	Common common;
	cout << endl << "1 Common function test" << endl;
	unsigned char memory[16] =
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
	cout << "Memory(0-15): ";
	common.MemoryPrint(memory, 16);
	cout << endl << "Binary(255): ";
	common.BinaryPrint(255);
	cout << endl << "Timestamp: " << fixed << common.GetTimestamp() << endl;
	common.StatusPrint("Status: status printing test", __FILE__, __FUNCTION__, __LINE__);

	/// 2 Standard packet protocol test
	cout << endl << "2 Standard packet protocol test" << endl;
	/// 2.1 Packet set
	unsigned char packetData[100] =
	{ 0x00, 0x00, 0x2B, 0x78, 0xA7, 0xEF, 0x26, 0x52, 0xB6, 0x77, 0x07, 0x00, 0x4E, 0x57, 0x89, 0xA3, 0xFF, 0x53, 0xE6,
	0x3F, 0xF2, 0x4D, 0xB2, 0x57, 0x16, 0x40, 0x00, 0x40, 0x00, 0x00, 0x00, 0xE0, 0x5F, 0x58, 0x51, 0x40, 0xCC,
	0x84, 0xCA, 0x3C, 0xCA, 0x6E, 0xB8, 0x3B, 0xEF, 0x58, 0x89, 0x3C, 0x62, 0x2A, 0x51, 0x3C, 0x33, 0x16, 0x95,
	0x3C, 0x66, 0xAA, 0xA1, 0x3B, 0x97, 0xD9, 0x7F, 0x3F, 0x34, 0x04, 0xAB, 0x3C, 0x89, 0x68, 0x14, 0xBD, 0x84,
	0xAD, 0x9C, 0x40, 0x96, 0x22, 0x28, 0xBB, 0x82, 0x13, 0xB2, 0xBA, 0x28, 0xEA, 0xFF, 0xBA, 0xAD, 0x76, 0xA2,
	0x3E, 0xCE, 0x4B, 0xBF, 0x3E, 0xEB, 0xC3, 0xA2, 0x3E };
	Packet packet =
	{ packetData, 20, 100 };
	/// 2.2 Encode packet
	Protocol protocol;
	Buffer buffer = protocol.PacketEncode(packet);
	cout << "Buffer: data = ";
	common.MemoryPrint(buffer.data, buffer.length);
	cout << endl << "length = " << buffer.length << endl;
	/// 2.3 Decode packet
	packet = protocol.BufferDecode(&buffer);
	cout << "Packet: data = ";
	common.MemoryPrint(packet.data, packet.length);
	cout << endl << "id = " << packet.id << endl << "length = " << packet.length << endl << "errors = " << buffer.errors
		<< endl;
	/// 2.4 Ensure that you free the packet when you done with it or you will leak memory
	delete[] packet.data;

	/// 3 Shared memory test
	cout << endl << "3 Shared memory test" << endl;
#ifdef __QNXNTO__
	SharedMemory sharedMemory;
	ShmX *shmXPtr;
	if (sharedMemory.Initialize("/dev/shmem/ShmX", &shmXPtr) == 1)
	{
		common.StatusPrint("Error: unable to create shared memory", __FILE__, __FUNCTION__, __LINE__);
		return EXIT_FAILURE;
	}
	pthread_rwlock_wrlock(&(shmXPtr->rwlock));
	shmXPtr->timestamp = common.GetTimestamp();
	shmXPtr->parameter = 16;
	pthread_rwlock_unlock(&(shmXPtr->rwlock));

	pthread_rwlock_rdlock(&(shmXPtr->rwlock));
	cout << shmXPtr->timestamp << endl;
	cout << shmXPtr->parameter << endl;
	pthread_rwlock_unlock(&(shmXPtr->rwlock));
#else
	cout << "Shared memory is only available for QNX" << endl;
#endif

#ifdef TestA
	/// 4 Serial port communicating test
	cout << endl << "4 Serial port communicating test" << endl;
#ifdef __QNXNTO__
	/// 4.1 Set serial port
	int portNumber = 1;
	int intr = 4;
	/// 4.2 Open serial port
	char portName[10];
	sprintf(portName, "/dev/ser%d", portNumber);
	SerialPort serialPort(portName, 115200, 1, 8, 1);
	switch (serialPort.Open())
	{
	case 1:
		common.StatusPrint("Error: unable to open serial port", __FILE__, __FUNCTION__, __LINE__);
		return EXIT_FAILURE;
	case 2:
		common.StatusPrint("Error: invalid baud rate", __FILE__, __FUNCTION__, __LINE__);
		return EXIT_FAILURE;
	case 3:
		common.StatusPrint("Error: invalid parity", __FILE__, __FUNCTION__, __LINE__);
		return EXIT_FAILURE;
	case 4:
		common.StatusPrint("Error: invalid number of data bits", __FILE__, __FUNCTION__, __LINE__);
		return EXIT_FAILURE;
	case 5:
		common.StatusPrint("Error: invalid number of stop bits", __FILE__, __FUNCTION__, __LINE__);
		return EXIT_FAILURE;
	}
	/// 4.3 Set serial interrupt
	struct sigevent event;
	event.sigev_notify = SIGEV_INTR;
	ThreadCtl(_NTO_TCTL_IO, NULL); //Before calling InterruptAttachEvent, the thread must request I/O privileges
	int id = InterruptAttachEvent(intr, &event, 0);
	InterruptUnmask(intr, id);
	/// 4.4 Read
	unsigned char bufferRead[1024];
	int bytesRead;
	serialPort.DiscardInBuffer();
	while (1)
	{
		InterruptWait(0, NULL);
		bytesRead = serialPort.Read(bufferRead, 0, 1024);
		bufferRead[bytesRead] = '\0';
		cout << bufferRead;
		InterruptUnmask(intr, id);
	}
	/// 4.5 Write
	serialPort.DiscardOutBuffer();
	while (1)
	{
		serialPort.Write((unsigned char *) "Serial port write test", 0, 22);
		usleep(10000); //Suspend a thread for a 10ms
	}
	/// 4.6 Close serial port
	serialPort.Close();
#else
	cout << "Serial port communicating test is only available for QNX" << endl;
#endif
#endif

#ifdef TestB
	/// 4 UDP client communicating test
	cout << endl << "4 UDP client communicating test" << endl;
#ifdef __QNXNTO__
	/// 4.1 Initializes UDP client by specifying server address
	UdpClient udpClient("255.255.255.255", 8001);
	/// 4.2 Bind localhost to UDP client and receive
	if (udpClient.Bind("0.0.0.0", 8001) == 1)
	{
		common.StatusPrint("Error: unable to bind the specified ip and port to udp client", __FILE__, __FUNCTION__, __LINE__);
		return EXIT_FAILURE;
	}
	unsigned char dgramReceive[1024];
	int bytesReceive;
	while (1)
	{
		bytesReceive = udpClient.Receive(dgramReceive, 0, 1024);
		dgramReceive[bytesReceive] = '\0';
		cout << dgramReceive;
	}
	/// 4.3 Set broadcast and send
	udpClient.SetBroadcast();
	while (1)
	{
		udpClient.Send((unsigned char *) "UDP client send test", 0, 20);
		usleep(10000); //Suspend a thread for a 10ms
	}
	/// 4.4 Close UDP client
	udpClient.Close();
#else
	cout << "UDP socket communicating test is only available for QNX" << endl;
#endif
#endif

	cout << endl;
	system("pause");
	return EXIT_SUCCESS;
}
