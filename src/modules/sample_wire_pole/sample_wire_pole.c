/*
 * sample_wire_pole.cpp
 *
 *  Created on: Apr 10, 2017
 *      Author: bdai
 */

#include <px4_config.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <float.h>
#include <errno.h>
#include <limits.h>
#include <math.h>

#include <lib/geo/geo.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/wire_pole.h>
//#include <uORB/topics/rc_channels.h>	//rc in
#include <uORB/topics/manual_control_setpoint.h>

static orb_advert_t	mavlink_log_pub = NULL;		/**< mavlink log advert */
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */


/**
 * sample_wire_pole management function.
 */
__EXPORT int sample_wire_pole_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int sample_wire_pole_thread_main(int argc, char *argv[]);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: sample_wire_pole {start|stop|status} [-p <additional params>]\n\n");
}

int sample_wire_pole_thread_main(int argc, char *argv[])
{
	int gps_sub	=	orb_subscribe(ORB_ID(vehicle_gps_position));
	int manual_sub	=	orb_subscribe(ORB_ID(manual_control_setpoint));

	struct vehicle_gps_position_s	gps;
	memset(&gps, 0, sizeof(gps));
	struct manual_control_setpoint_s	manual;
	memset(&manual, 0, sizeof(manual));
	struct wire_pole_s	wire_pole;
	memset(&wire_pole, 0, sizeof(wire_pole));
	struct map_projection_reference_s ref;
	memset(&ref, 0, sizeof(ref));

	orb_advert_t wire_pole_pub = orb_advertise(ORB_ID(wire_pole), &wire_pole);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = manual_sub,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	while(!thread_should_exit) {
		int poll_ret = px4_poll(fds, 1, 1000);
		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
//			PX4_ERR("Got no manual data within a second");
			continue;

		} else if (poll_ret < 0) {
//			PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			continue;
		}

		static bool is_sw_init = true;	//is sw in init position

		orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
		if (is_sw_init && manual.aux1 > 0.75f) {	//ready to sample gps point
			is_sw_init  = false;

			bool updated = false;
			orb_check(gps_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps);
				wire_pole.timestamp = hrt_absolute_time();

				static uint32_t num = 0;
				wire_pole.num = ++num;

				wire_pole.lat = gps.lat;
				wire_pole.lon = gps.lon;
				wire_pole.alt = gps.alt;

				if (num == 1) {
					wire_pole.distance = 0.0f;
					map_projection_init(&ref, wire_pole.lat, wire_pole.lon);
				} else {
					float x = 0.0f, y = 0.0f;
					map_projection_project(&ref, wire_pole.lat, wire_pole.lon, &x, &y);
					wire_pole.distance = sqrtf(x*x + y*y);
					map_projection_init(&ref, wire_pole.lat, wire_pole.lon);
				}

				mavlink_and_console_log_info(&mavlink_log_pub, "sampled: num %d,alt %4.2f,dist %4.2f",
						wire_pole.num,(double)(wire_pole.alt * 1.0e-3f), (double)wire_pole.distance);

				orb_publish(ORB_ID(wire_pole), wire_pole_pub, &wire_pole);	//finally publish the wire pole position
			} else {
				mavlink_and_console_log_info(&mavlink_log_pub, "sample fail, no gps info!");
			}

		} else if(manual.aux1 <= 0.75f) {
			is_sw_init = true;
		}
	}

	return 0;
}

int sample_wire_pole_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("sample_wire_pole running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("sample_wire_pole",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 sample_wire_pole_thread_main,
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






