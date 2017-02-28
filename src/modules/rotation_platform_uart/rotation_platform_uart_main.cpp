/*
 * rotation_platform_uart_main.cpp
 * @file rotation_platform_uart_main.cpp
 * @created on: Apr 29, 2016
 * @author: bdai<bdai1412@gmail.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <unistd.h>

#include <nuttx/config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <math.h>
#include <termios.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <fcntl.h>
#include <poll.h>
#include <float.h>
#include "uart/uart.h"
#include <matrix/math.hpp>

#include <uORB/uORB.h>
//#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

static bool thread_should_exit = false; /*app exit flag */
static bool thread_running = false;/*app status flag */
static int rotation_platform_uart_task;/*Handle of rotation_platform_uart task/thread */

extern "C" __EXPORT int rotation_platform_uart_main(int argc, char *argv[]);

static void usage(const char *reason);

int rotation_platform_uart_thread_main(int argc, char *argv[]);

int rotation_platform_uart_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		rotation_platform_uart_task = px4_task_spawn_cmd("rotation_platform_uart",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 rotation_platform_uart_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}


int rotation_platform_uart_thread_main(int argc, char *argv[])
{
	thread_running = true;
	PX4_INFO("Hello, running rotation_platform_uart_main");

	/*subscribe the topic needed*/
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_set_interval(vehicle_attitude_sub, 20);

	unsigned int out_buffer[50];
	static int count_uart = 0;


/*
	 waiting for vehicle_attitude_sub
	px4_pollfd_struct_t fds[1];
	fds[0].fd = vehicle_attitude_sub;
	fds[0].events = POLLIN;
*/


/*该初始化方法适用于.c file ，.cpp will cause a error -bdai<2016-04-30> */
//	/* one could wait for multiple topics with this technique, just using one here */
//	px4_pollfd_struct_t fds[] = {
//		{ .fd = sensor_sub_fd,   .events = POLLIN },
//		/* there could be more file descriptors here, in the form like:
//		 * { .fd = other_sub_fd,   .events = POLLIN },
//		 */
//	};


    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
	int uart_read = uart_init("/dev/ttyS6");
    if(false == uart_read) {
    	PX4_ERR("uart open failed!");
    	//return -1;
    }

    if(false == set_uart_baudrate(uart_read,115200))
    {
    	PX4_ERR("[rotation_platform_uart]set_uart_baudrate is failed\n");
        //return -1;
    }
    sleep(1);
    PX4_INFO("[rotation_platform_uart]uart init is successful!\n");



    hrt_abstime pre_time = hrt_absolute_time();

	while (!thread_should_exit) {

/*		int ret = px4_poll(fds, 1, 500);
		PX4_WARN("ret = %d", ret);
		if (ret == 0){
			PX4_ERR("[rotation_platform_uart] Got on data with in 0.5 senconds");
			//continue;
		}
		else if (ret < 0) {
			PX4_WARN("Rotation_platform_uart POLL ERROR");
			continue;
		}*/


		struct vehicle_attitude_s raw;
		bool updated = false;

		orb_check(vehicle_attitude_sub, &updated);

//		if (hrt_absolute_time() - pre_time < 1000000)
//			continue;

		if(updated) {
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &raw);
			matrix::Eulerf euler = matrix::Quatf(raw.q);

			out_buffer[0] = 0xAAFFFFFF;

			memcpy(&out_buffer[1],&euler.phi(),sizeof(euler.phi()));
			memcpy(&out_buffer[2],&euler.theta(),sizeof(euler.theta()));
			memcpy(&out_buffer[3],&euler.psi(),sizeof(euler.psi()));
			memcpy(&out_buffer[4],&raw.yawspeed,sizeof(raw.yawspeed));
			memcpy(&out_buffer[5],&raw.rollspeed,sizeof(raw.rollspeed));
			memcpy(&out_buffer[6],&raw.pitchspeed,sizeof(raw.pitchspeed));

			out_buffer[7] = 0xBBFFFFFF;


			int ret = write(uart_read,out_buffer,32);
			usleep(50000);
			pre_time = hrt_absolute_time();
			PX4_INFO("time = %d, ret = %d, count = %d,", pre_time, ret,count_uart++);
		}

	}

	thread_running = false;
	return 0;

}


static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage:rotation_platform_uart {start|stop|status} [-p <additional params>]\n\n");
}
