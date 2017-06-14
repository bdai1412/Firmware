/****************************************************************************
 *
 *   Copyright (c) 2013 - 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file cooperation_main.cpp
 * Multicopter position controller.
 *
 * Original publication for the desired attitude generation:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011
 *
 * Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>

//subscribe topics
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/battery_status.h>

//advertise topics
#include <uORB/topics/uground_data.h>

#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#include <matrix/math.hpp>
#include "Utils/Protocol.h"
#define erroNotFound
#define ReceiveWrongData
//#define uartWrite
#define uartRead
static bool thread_should_exit = false; /*app exit flag */
static bool thread_running = false;/*app status flag */
static int cooperation_task;    /*handle of cooperation thread*/
const double Pi = 3.14159265358979;
/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int cooperation_main(int argc, char *argv[]);
static void usage(const char *reason);
int open_uart(int baud, const char *uart_name);
int cooperation_thread_main(int argc, char *argv[]);

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage:rotation_platform_uart {start|stop|status} [-p <additional params>]\n\n");
}

int open_uart(int baud, const char *uart_name)
{
    //    serial port setting
    int	_uart_fd = 0;
    int _baudrate = baud;
    /* open uart */
    _uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    /* if this is a config link, stay here and wait for it to open */
    while(_uart_fd < 0) {
        usleep(100000);
        warnx("open /dev/ttyS2 failed, reopening...");
        _uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    }
    if(_uart_fd >= 0)    warnx("serial port /dev/ttyS2 opened");
    /* Try to set baud rate */

    struct termios uart_config;
    int termios_state;
    /* Initialize the uart config */
    if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
        warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
        ::close(_uart_fd);
        return -1;
    }
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    uart_config.c_cflag &= ~CRTSCTS;
    uart_config.c_cflag |= CS8;

    uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input*/
    uart_config.c_oflag &= ~OPOST; /*Output*/


    /* Set baud rate */
    if (cfsetispeed(&uart_config, _baudrate) < 0 || cfsetospeed(&uart_config, _baudrate) < 0) {
        warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
        ::close(_uart_fd);
        return -1;
    }
    if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
        PX4_WARN("ERR SET CONF %s\n", uart_name);
        ::close(_uart_fd);
        return -1;
    }
    return _uart_fd;
}
int cooperation_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: cooperation {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            warnx("already running");
            exit(0);;
        }
        thread_should_exit = false;
        cooperation_task = px4_task_spawn_cmd("cooperation",
								  SCHED_DEFAULT,
								  SCHED_PRIORITY_MAX - 5,
								  4000,
								  cooperation_thread_main,
								  (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("\t running\n");
        } else {
            warnx("\t not started\n");
        }
        exit(0);;
    }
    usage("unrecognized command");
    return 1;
}

int cooperation_thread_main(int argc, char *argv[])
{
    thread_running = true;
    struct vehicle_attitude_s _vehicle_attitude;
    struct vehicle_gps_position_s _vehicle_gps_position;
    struct sensor_gyro_s _sensor_gyro;
    struct sensor_accel_s _sensor_accel;
    struct battery_status_s _battery_status;

    int		_vehicle_attitude_sub = -1;		/**< vehicle attitude subscription */
    int		_vehicle_gps_position_sub = -1;	/**< vehicle gps position subscription */
    int		_sensor_gyro_sub = -1;		/**< sensor gyroscope subscription */
    int		_sensor_accel_sub = -1;		/**< sensor accelerometer subscription */
    int     _battery_status_sub = -1;    /**< battery subscription */
    //advertise
    //topics
    struct uground_data_s _uground_data;
    orb_advert_t	_uground_data_pub = nullptr;			/**< uground data publication */
    //packet code and decode
    Protocol protol;
    Packet packetReceived = { (unsigned char *) "", 1, 45 };
    packetReceived.id = 1;
    packetReceived.length = 45;
//    packetReceived.data = (unsigned char *)calloc(45, sizeof(unsigned char));
    unsigned char data[45] = {0};
    packetReceived.data = data;
    memset(packetReceived.data, 0, packetReceived.length);

    Buffer bufferReceived;
    memset(bufferReceived.data, 0, BufferLength);

    Packet packetSend;// = { (unsigned char *) "", 32, 87 };
    packetSend.id = 1;
    packetSend.length = 87;
    unsigned char sendData[87] = {0};
    packetSend.data = sendData;         //(unsigned char*)calloc(1,87);
    memset(packetSend.data, 0, packetSend.length);

    Buffer bufferSend;
    memset(bufferSend.data, 0, BufferLength);

    _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
    _sensor_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
    _sensor_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
    _battery_status_sub = orb_subscribe(ORB_ID(battery_status));

    PX4_INFO("Hello, running cooperation_main");
    int	_uart_fd = 0;

    _uart_fd = open_uart(B57600, "/dev/ttyS6");
    tcflush(_uart_fd, TCIOFLUSH);
    PX4_INFO("_uart_fd:%d", _uart_fd);
    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds;
    fds.fd = _vehicle_attitude_sub;
    fds.events = POLLIN;
    while(!thread_should_exit)
    {
        int poll_ret = px4_poll(&fds, 1, 1000);

        if (poll_ret <= 0){
            continue;
        } else {
            int bytesRead  = 0;
//            int bytesWrite = 0;
            //read uart port
            bytesRead = read(_uart_fd, bufferReceived.data, 1024);
            static int i = 0;
            PX4_INFO("bytesRead:%d, i: %d", bytesRead,i);
            if(bytesRead > 0) {

                packetReceived = protol.BufferDecode(&bufferReceived);
                if(packetReceived.id == 1 && packetReceived.length == 45)
                {
                    memcpy(&_uground_data.usvTimestamp, packetReceived.data, sizeof(double));
                    memcpy(&_uground_data.setting, packetReceived.data + 8, sizeof(unsigned char));
                    memcpy(&_uground_data.targetLatitude, packetReceived.data + 9, sizeof(double));
                    memcpy(&_uground_data.targetLongitude, packetReceived.data + 17, sizeof(double));
                    memcpy(&_uground_data.targetAltitude, packetReceived.data + 25, sizeof(double));
                    memcpy(&_uground_data.targetHeading, packetReceived.data + 33, sizeof(float));
                    memcpy(&_uground_data.targetVelocity, packetReceived.data + 37, sizeof(float));
                    memcpy(&_uground_data.desiredAltitude, packetReceived.data + 41, sizeof(float));
                    /* publish uground data */
                    if (_uground_data_pub != nullptr) {
                        orb_publish(ORB_ID(uground_data), _uground_data_pub, &_uground_data);

                    } else {
                        _uground_data_pub = orb_advertise(ORB_ID(uground_data), &_uground_data);
                    }
                }
            }
            //write uart port
            //subscribe topic and poll data
            bool updated = false;
            orb_check(_vehicle_gps_position_sub, &updated); //gps
            if (updated) {
                orb_copy(ORB_ID(vehicle_gps_position), _vehicle_gps_position_sub, &_vehicle_gps_position);
                double latitude = (double)_vehicle_gps_position.lat * 1E-7 / 180 * Pi;
                double longtitude = (double)_vehicle_gps_position.lon * 1E-7 / 180 * Pi;
                double altitude = (double)_vehicle_gps_position.alt / 1000.0;
                memcpy(packetSend.data + 9,  &latitude, sizeof(double));
                memcpy(packetSend.data + 17, &longtitude , sizeof(double));
                memcpy(packetSend.data + 25, &altitude, sizeof(double));

                memcpy(packetSend.data + 45, &_vehicle_gps_position.vel_n_m_s, sizeof(float));
                memcpy(packetSend.data + 49, &_vehicle_gps_position.vel_e_m_s, sizeof(float));
                memcpy(packetSend.data + 53, &_vehicle_gps_position.vel_d_m_s, sizeof(float));

                memcpy(packetSend.data + 81, &_vehicle_gps_position.satellites_used, sizeof(unsigned char));
            }
            orb_check(_vehicle_attitude_sub, &updated); //gps
            if (updated) {
                orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_attitude);

            }
            matrix::Eulerf euler = matrix::Quatf(_vehicle_attitude.q);
            memcpy(packetSend.data + 33, &euler.phi(), sizeof(float));
            memcpy(packetSend.data + 37, &euler.theta(), sizeof(float));
            memcpy(packetSend.data + 41, &euler.psi(), sizeof(float));

            orb_check(_sensor_gyro_sub, &updated);  //gyroscope
            if (updated) {
                orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub, &_sensor_gyro);
                memcpy(packetSend.data + 57, &_sensor_gyro.x, sizeof(float));
                memcpy(packetSend.data + 61, &_sensor_gyro.y, sizeof(float));
                memcpy(packetSend.data + 65, &_sensor_gyro.z, sizeof(float));

            }
            orb_check(_sensor_accel_sub, &updated);   //accelerometer
            if (updated) {
                orb_copy(ORB_ID(sensor_accel), _sensor_accel_sub, &_sensor_accel);
                memcpy(packetSend.data + 69, &_sensor_accel.x, sizeof(float));
                memcpy(packetSend.data + 73, &_sensor_accel.y, sizeof(float));
                memcpy(packetSend.data + 77, &_sensor_accel.z, sizeof(float));
            }
            orb_check(_battery_status_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
                memcpy(packetSend.data + 83, &_battery_status.remaining, sizeof(float));
            }
            double uavTimeStamp = hrt_absolute_time() / 1000.0;
            memcpy(packetSend.data,  &uavTimeStamp, sizeof(double));
            bufferSend = protol.PacketEncode(packetSend);
            write(_uart_fd, bufferSend.data, bufferSend.length);
            usleep(200000); //20000us = 5Hz
        }
    }
    thread_running = false;

    close(_uart_fd);
    warnx("cooperation stopped");
    return 0;
}
