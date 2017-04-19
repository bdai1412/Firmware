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
#include <uORB/topics/actuator_armed.h>

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
	thread_running = true;

	int gps_sub	=	orb_subscribe(ORB_ID(vehicle_gps_position));
	int manual_sub	=	orb_subscribe(ORB_ID(manual_control_setpoint));
	int armed_sub 	=	orb_subscribe(ORB_ID(actuator_armed));

	struct vehicle_gps_position_s	gps;
	memset(&gps, 0, sizeof(gps));
	struct manual_control_setpoint_s	manual;
	memset(&manual, 0, sizeof(manual));
	struct wire_pole_s	wire_pole;
	memset(&wire_pole, 0, sizeof(wire_pole));
	struct map_projection_reference_s ref, pre_ref, test_ref;
	memset(&ref, 0, sizeof(ref));
	memset(&ref, 0, sizeof(pre_ref));
	memset(&test_ref, 0, sizeof(test_ref));

	struct  actuator_armed_s	armed;
	memset(&armed, 0, sizeof(armed));

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

		static bool swith_on = true;	//is sw in init position

		orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
		if (manual.aux1 > 0.75f) {	//ready to sample gps point
			bool updated = false;
			orb_check(armed_sub, &updated);
			if(updated) {
				orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
			}
			bool gps_updated = false;
			orb_check(gps_sub, &gps_updated);

			if (swith_on) {
				swith_on = false;
				if (!gps_updated) {
					mavlink_and_console_log_info(&mavlink_log_pub,
							"sampled fail, no gps!");
				} else {
					orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps);
					wire_pole.timestamp = hrt_absolute_time();

					static uint32_t num = 0;

					bool reinit_test = false;
					if(manual.aux2 <= -0.5f) {	//sample point normally
						num++;
						memcpy(&pre_ref, &ref, sizeof(pre_ref));
					} else {	//num == 0 is used for testing the precision of gps
						if(manual.aux2 > -0.5f && manual.aux2 < 0.5f && num == 0) {
							reinit_test = true;		//reinit ref when num = 0 to test the precision of gps
						} else if (num > 1){	// re-sample the point
							// if not the first point
							memcpy(&ref, &pre_ref, sizeof(ref));
						}
					}
					wire_pole.num = num;

					wire_pole.lat = gps.lat;
					wire_pole.lon = gps.lon;
					wire_pole.alt = gps.alt;
					double lat = wire_pole.lat * 1.0e-7;
					double lon = wire_pole.lon * 1.0e-7;
					float x = 0.0f, y = 0.0f;

					if(reinit_test) {	//reinit test reference gps point
						map_projection_init(&test_ref, lat, lon);
					}

					if (num == 0) {	//testing mode
						map_projection_project(&test_ref, lat, lon, &x, &y);
					} else {
						if (num == 1){ // the first or in testing status
							map_projection_init(&ref, lat, lon);
						}
						map_projection_project(&ref, lat, lon, &x, &y);
						map_projection_init(&ref, lat, lon);
					}

					wire_pole.distance = sqrtf(x*x + y*y);

					mavlink_and_console_log_info(&mavlink_log_pub, "Num %d,Alt %2.4f,Dist %6.4f",
							wire_pole.num,(double)(wire_pole.alt * 1.0e-3f), (double)wire_pole.distance);
				}
			}
			orb_publish(ORB_ID(wire_pole), wire_pole_pub, &wire_pole);	//finally publish the wire pole position

		} else if(manual.aux1 <= 0.75f) {
			swith_on = true;
		}
	}

	thread_running = false;
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






