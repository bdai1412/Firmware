/****************************************************************************
 *
 *   Copyright (c) 2013 - 2016 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/mc_virtual_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/angacc_acc.h>
#include <uORB/topics/acc_ff.h>
#include <uORB/topics/coupling_force.h>


#include <uORB/topics/battery_status.h>

/*gyzhang <Jul 11, 2017>*/
#include <uORB/topics/target_info.h>
// blocks.hpp include Subscriptions.hpp and Publications.hpp
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include "manipulator_control/BlockManipulatorControl.hpp"


#include <systemlib/systemlib.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <platforms/px4_defines.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ONE_G	9.8066f
#define ACC_FF

#define PI 3.14159f

#define DEG2RAD ((float)M_PI / 180.0f)

/*target tracking gyzhang <Jul 11, 2017>*/
static float ELEC_FENCE[3][2] = {-2.5f, 2.5f, -2.5f, 2.5f, -0.45f -2.0f};
static bool OUT_FENCE = false;
/*follow mode means use velocity feed-forward control -bdai<12 Nov 2016>*/
static bool FOLLOW_MODE = true;
static float pos_sp_condition[4] = { 0.45f, 0.02f,
		24.0f * DEG2RAD, 30.0f * DEG2RAD};
enum {R_max = 0, R_min, ANGLE_MIN, ANGLE_MAX};
enum {MIN = 0, MAX};
// static orb_advert_t mavlink_log_pub = nullptr;

bool RISE_UP = false;

namespace Path_Tracking {

struct Next {	// next desire
	Next(math::Vector<3> pos = math::Vector<3>(0.0f,0.0f,0.0f),
		 math::Vector<3> vel = math::Vector<3>(0.0f,0.0f,0.0f)):
		_pos(pos),_vel(vel)
	{
	};
	math::Vector<3> _pos;	//next position desire if follows shape
	math::Vector<3> _vel;	//next velocity desire if follows shape
};

////////////////////////////////////////////// basic shape /////////////////////////////////////////////

class Shape {
public:
	Shape(){_duration = 0.0f;};
	virtual ~Shape(){};
	virtual Next next_sp(float dt) = 0;
	float _duration;
};

class Scale { // constant velocity and accelrate movement
public:
	// len, startvel middlvel endvel should all be positive
	Scale() {_iserrorstay = true;};
	Scale(float len,float middvel, float startvel, float endvel,
		float accdura1, float accdura2):
			_dura(.0f),_len(len),
			_middvel(middvel), _startvel(startvel), _endvel(endvel),
			_accdura1(accdura1),_accdura2(accdura2),
			_iserrorstay(false)
	{
		if (len < 0 || startvel < 0 || middvel < 0 || endvel < 0 || accdura1 < 0 || accdura2 < 0) {
			_iserrorstay = true; return;}

		float len1 = (_startvel + _middvel) * 0.5f * _accdura1;
		float len2 = (_middvel + _endvel) * 0.5f * _accdura2;
		if(len1 + len2 > len){
			_iserrorstay = true;
		} else if (fabsf(len1 + len2 - len) < FLT_EPSILON) {
			_constdura = 0.0f;
			if (len < FLT_EPSILON) _iserrorstay = true;	// keep stay
		} else{
			if (fabsf(_middvel) < FLT_EPSILON) _iserrorstay = true; //if the middle vel is 0
			else _constdura = (len - len1 - len2) / _middvel;
		}
		_dura = _accdura1 + _constdura + _accdura2;

	};

	int get_value(float dt, float& pos, float& vel) {
		if (_iserrorstay) {
			pos = vel = 0.0f;
			return 1;
		} else {
			if (dt < 0.0f) dt = 0.0f;
			if (dt > _dura) dt = _dura;

			if (dt < _accdura1) {
				vel = _startvel + dt / _accdura1 * (_middvel - _startvel);
				pos =  (_startvel + vel) / 2.0f * dt;
			} else if (dt - _accdura1 - _constdura > FLT_EPSILON) {
				vel = _middvel + (_endvel - _middvel) * (dt - _accdura1 - _constdura) / _accdura2;
				pos = (_startvel + _middvel) / 2.0f * _accdura1 + _middvel * _constdura +
						(_middvel + _endvel) / 2.0f * (dt - _accdura1 - _constdura);
			} else {
				vel = _middvel;
				pos = (_startvel + _middvel) / 2.0f * _accdura1 + vel * (dt - _accdura1);
			}
		}
		return 0;
	};

	float _dura;
//private:
	float _len;
	float _middvel, _startvel, _endvel;	// start velocity, constantly movement velocity and end velocity
	float _accdura1, _accdura2, _constdura;// accelerate duration at phase 1, contantly movement duration
	bool _iserrorstay;	// if there if error or stay
};

class Point : public Shape {
public:
	Point(){};
	Point(math::Vector<3> pos_d, float duration = 0.0f): //default hovering forever
		_hovering_point(pos_d)
	{
		_duration = duration;
	};
	virtual Next next_sp(float dt) {
		return Next(_hovering_point);
	}
private:
	math::Vector<3> _hovering_point;
};

class Line : public Shape {
public:
	Line(math::Vector<3> start, math::Vector<3> end, float middvel, float accdura1 = .0f, float accdura2 = .0f,
			float startvel = .0f ,float endvel = .0f): //default hovering forever
		_scale((end-start).length(), middvel,startvel, endvel, accdura1, accdura2),
		_start(start),_end(end)
	{
		_duration = _scale._dura;
	};
	virtual Next next_sp(float dt) {
		static float pos_norm, vel_norm;
		_scale.get_value(dt, pos_norm, vel_norm);

		math::Vector<3> vel_d = (_end - _start).normalized() * vel_norm;
		return Next(_start +  (_end - _start).normalized() * pos_norm, vel_d);
	}
private:
	Scale _scale;
	math::Vector<3> _start,_end;
};

class Arc: public Shape {
public:
	Arc(){_angle = 0.0f; _duration = 0.0f;};
	// rotate in a fix plane. the angle between plane and horizon is the angle between vector (end -start) and horizon
	
	Arc(math::Vector<3> start, math::Vector<3> end, bool clockwise, float ang,
			float middvel,float accdura1 = .0f, float accdura2 = .0f,
			float startvel = .0f, float endvel = .0f)
	{
		
		float vertical = (clockwise == true) ? 1.0f : -1.0f; //clockwise or anticlockwise
		math::Vector<3> left_or_right(0.0f, 0.0f, vertical);

		bool iscicle = (ang >= 2.0f*PI);

		//ang = 2Pi mean there is an cicle. and end- start is the D of the cicle
		if(iscicle) {
			_angle = ang;
			_start = start;
			_center = (end + start) / 2.0f;

			ang = PI; // to get the rotation axis
		}
		float dist = (end - start).length()/2.0f * (1.0f/sinf(ang/2.0f) - 1.0f/tanf(ang/2.0f));

		math::Vector<3> middle = (end + start) / 2.0f +
				((end - start) % left_or_right).normalized() * dist;

		if (iscicle) {
			_rataxis = (start - _center) % (middle-_center).normalized();
			float radius = (_start - _center).length();
			_scale = Scale(_angle, middvel/radius, startvel/radius, endvel/radius, accdura1, accdura2);
			_duration = _scale._dura;
		} else {
			*this = Arc(start, middle, end, middvel, accdura1, accdura2, startvel, endvel);
		}
	};

	Arc(math::Vector<3> start, math::Vector<3> middle, math::Vector<3> end,
			float middvel, float accdura1 = .0f, float accdura2 = .0f,
			float startvel = .0f, float endvel = .0f)
	{
		// determine rotation direction
		math::Vector<3> axis = (middle - start).normalized() % (end - middle).normalized();
		if (axis.length() < 0.0001f) {	//if the angle is two small
			PX4_INFO("R of Arc is too large");
		} else {		//three plane cross one point is center of the arc
			// x,y,z is the center of the arc
			// A0 B0 C0 D0 constrained by three point in one common plain
			// A1 B1 C1 D1 perpendicular bisecting plane between two points
			// A2 B2 C2 D2 perpendicular bisecting plane between another two points
			// |A0 B0 C0|   |x|   |D0|
			// |A1 B1 C1| * |y| + |D1| = 0
			// |A2 B2 C2|   |z|   |D2|
			// reference: http://blog.csdn.net/yanmy2012/article/details/8111600

			// point O A B C represent center start middle and end, X is the value
			// AX orth. with _rataxis
			_rataxis = axis.normalized();
			matrix::Matrix3f P;
			matrix::Vector3f Q;
//					float A0,B0,C0,D0;
			P(0,0) = _rataxis(0);
			P(0,1) = _rataxis(1);
			P(0,2) = _rataxis(2);
			Q(0)    = -P(0,0)*start(0)-P(0,1)*start(1)-P(0,2)*start(2);

			// contrained by R of the cicle
//					float A1,B1,C1,D1;
			// OA = OB = R
			P(1,0) = 2.0f*(middle(0)-start(0));
			P(1,1) = 2.0f*(middle(1)-start(1));
			P(1,2) = 2.0f*(middle(2)-start(2));
			Q(1)    = start(0)*start(0) + start(1)*start(1) + start(2)*start(2) -
					(middle(0)*middle(0) + middle(1)*middle(1) + middle(2)*middle(2));

			// contrained by R of the cicle
//					float A2,B2,C2,D2;
			// OB = OC = R
			P(2,0) = 2.0f*(end(0)-start(0));
			P(2,1) = 2.0f*(end(1)-start(1));
			P(2,2) = 2.0f*(end(2)-start(2));
			Q(2)    = start(0)*start(0) + start(1)*start(1) + start(2)*start(2) -
					(end(0)*end(0) + end(1)*end(1) + end(2)*end(2));



			// for(int i = 0;i<3;i++) {
			// 		PX4_INFO("%.7f, %.7f, %.7f",
			// 				(double)P(i,0),(double)P(i,1),(double)P(i,2));
			// }

			// matrix::Matrix3f PP = P.I();
			// for(int i = 0;i<3;i++) {
			// 		PX4_INFO("%8.4f, %8.4f, %8.4f, %8.4f",
			// 				(double)PP(i,0),(double)PP(i,1),(double)PP(i,2));
			// }

			matrix::Vector3f cent = -P.I()*Q;
			// PX4_INFO("center: %.7f,%.7f,%.7f", (double)cent(0),(double)cent(1),(double)cent(2));

			// point O A B C represent center start middle and end
			_center = math::Vector<3>(cent(0), cent(1), cent(2));
			//vector product
			math::Vector<3> OA_OC = (start-_center).normalized() % (end - _center).normalized();
			math::Vector<3> AB_BC = axis;
			//scalar product
			float OAOC = (start-_center).normalized()*(end - _center).normalized();

			float theta = acosf(OAOC);
			_angle = (AB_BC*OA_OC > 0.0f) ? theta : 2.0f*PI - theta;
			_start = start;
			float radius = (_start - _center).length();
			_scale = Scale(_angle, middvel/radius, startvel/radius, endvel/radius, accdura1, accdura2);
			_duration = _scale._dura;
		}
	};

	//get next desire when in Arc with rotation of ang
	virtual Next next_sp(float dt) {
		if (dt > _duration) dt = _duration;
		float ang, omega;
		_scale.get_value(dt, ang, omega);

		math::Vector<3> start_vector = _start - _center;

		matrix::AxisAnglef axis_angle(matrix::Vector3f(_rataxis(0),_rataxis(1),_rataxis(2)), ang);
		matrix::Dcmf R(axis_angle);
		matrix::Vector3f pos_d = R*matrix::Vector3f(start_vector(0),start_vector(1),start_vector(2));

		//velocity is PI/2 ahead position
		matrix::AxisAnglef axis_angle_v(matrix::Vector3f(_rataxis(0),_rataxis(1),_rataxis(2)), ang + PI/2.0f);
		matrix::Dcmf R_v(axis_angle_v);
		matrix::Vector3f vel_d = R_v*matrix::Vector3f(start_vector(0),start_vector(1),start_vector(2));

		vel_d = vel_d.normalized() * omega * (_start -  _center).length();		//get velocity

		return Next(math::Vector<3>(pos_d(0),pos_d(1),pos_d(2)) + _center,
				math::Vector<3>(vel_d(0),vel_d(1),vel_d(2)));
	};
private:
	Scale _scale;
	math::Vector<3> _start, _center, _rataxis;
	float _angle;
};

class Sin:public Shape{
public:
	// start and end is one period, and sin function height is same with start
	Sin(math::Vector<3> start, math::Vector<3> end, float amp, float phi,
			float middvel, float accdura1 = .0f, float accdura2 = .0f,
			 float startvel = .0f, float endvel = .0f):
		_scale((end-start).length(), middvel,startvel, endvel, accdura1, accdura2),
		_start(start), _end(end),
		_amplitude(amp),_phi(phi)
	{
		float pitch = asinf((_end - _start).normalized()(2));

		math::Vector<3> proj = math::Vector<3>(_end(0) - _start(0), _end(1) - _start(1), 0.0f);
		float yaw = acosf(proj.normalized()(0));
		if (proj(1) < 0.0f) yaw = -yaw;
		matrix::Eulerf eula(0.0f, pitch, yaw);

		_R = matrix::Dcmf(eula);
		_length = (_end - _start).length();
		_w = 2*PI / _length;
		_duration = _scale._dura;
	};
	virtual Next next_sp(float dt) {
		// x,y is in right hand coordination, _start as (0,0), _end-_start as x direction
		if (dt > _duration) dt = _duration;
		float x, vx;
		_scale.get_value(dt, x, vx);

		float y = _amplitude * sinf(_w*x + _phi);
		float vy = _amplitude * _w * cosf(_w*x + _phi);

		matrix::Vector3f pos_d = _R * matrix::Vector3f(x, y, .0f);
		matrix::Vector3f vel_d = _R * matrix::Vector3f(vx, vy, .0f);

		return Next(math::Vector<3> (pos_d(0),pos_d(1),pos_d(2)) + _start,
				math::Vector<3>(vel_d(0), vel_d(1),vel_d(2)));
	};
private:
	Scale _scale;
	math::Vector<3> _start, _end;
	float _amplitude,_length,_w,_phi;
	matrix::Dcmf _R; // rotate from x: end-start, y: right hand to x:1,0,0 y: 0,1,011
};

////////////////////////////////////////////// path element /////////////////////////////////////////////
class Path {
public:
	int get_next(float dt, Next& next) {

		float tim = 0.0f;
		int index = 0;
		if(_num == 0) return 1;	//if they is no path element
		for (index = 0; index < _num; index++) {
			tim += _shapes[index]->_duration;
			if (dt < tim) break;
		}
		if(index == _num) index--; // if this is the last shape and time is run out
		next = _shapes[index]->next_sp(_shapes[index]->_duration -(tim - dt));

		if(dt > tim) next._vel.zero(); // the end of the path
		return 0;
	}
	int _num;
//	Shape** _shapes;	//not sure what will happend
	Shape* _shapes[20];
};

float plane_height = -1.2f;

math::Vector<3> Fix_Points(-2.0f, 0.0f, plane_height);
struct Path_Point: public Path {
	Path_Point(){
		_shapes[0] = new Point(Fix_Points);
		_num = 1;
	}
};

math::Vector<3> Lines_Points[2] = {
	math::Vector<3>(2.2f, 0.0f, plane_height),
	math::Vector<3>(-2.0f, 0.0f, plane_height)};
struct Path_Lines: public Path {
	Path_Lines(float vel) {
		_shapes[0] = new Line(Lines_Points[0],Lines_Points[1],vel,2.0f,2.0f);
		_shapes[1] = new Point(Lines_Points[1], 5.0f);
		_shapes[2] = new Line(Lines_Points[1],Lines_Points[0],vel,2.0f,2.0f);
		_num = 3;
	}
};

float amp = 1.5f;
math::Vector<3> Sins_Points[6] = {
	math::Vector<3>(2.5f, amp, plane_height),
	math::Vector<3>(2.0f, amp, plane_height),
	math::Vector<3>(2.0f, 0.0f, plane_height),
	math::Vector<3>(-1.5f, 0.0f, plane_height),
	math::Vector<3>(-1.5f, amp, plane_height),
	math::Vector<3>(-2.0f, amp, plane_height)};
struct Path_Sins: public Path {
	Path_Sins(float vel) {
		_shapes[0] = new Line(Sins_Points[0],Sins_Points[1], vel, 2.0f, 0.0f);
		_shapes[1] = new Sin(Sins_Points[2],Sins_Points[3], amp, -PI/2.0f, vel);
		_shapes[2] = new Line(Sins_Points[4],Sins_Points[5], vel, .0f, 2.0f);
		_shapes[3] = new Point(Sins_Points[5], 5.0f);
		_shapes[4] = new Line(Sins_Points[5],Sins_Points[4], vel, 2.0f, 0.0f);
		_shapes[5] = new Sin(Sins_Points[3],Sins_Points[2], amp, PI/2.0f, vel);
		_shapes[6] = new Line(Sins_Points[1],Sins_Points[0], vel, 0.0f, 2.0f);
		_num = 7;
	}
};

math::Vector<3> Cicles_Shape[8] = {
	math::Vector<3>(2.0f,-2.0f,plane_height),
	math::Vector<3>(2.0f,0.0f,plane_height),
	math::Vector<3>(0.0f,0.0f,plane_height),
	math::Vector<3>(0.0f,1.0f,plane_height),
	math::Vector<3>(0.0f,-1.0f,plane_height),
	math::Vector<3>(-1.0f,1.0f,plane_height),
	math::Vector<3>(-1.0f,-1.0f,plane_height),
	math::Vector<3>(-2.0f,-2.0f,plane_height),
};

struct Path_Cicles : public Path{
	Path_Cicles(float vel){
		_shapes[0] = new Line(Cicles_Shape[0],Cicles_Shape[1], vel, 2.0f, 0.0f);
		_shapes[1] = new Arc(Cicles_Shape[1],Cicles_Shape[2], true, PI*5.0f/2.0f, vel);
		Next next = _shapes[1]->next_sp(_shapes[1]->_duration);
		_shapes[3] = new Line(next._pos,Cicles_Shape[3], vel);
		_shapes[4] = new Arc(Cicles_Shape[3],Cicles_Shape[4], true, PI*2.0f, vel);
		next = _shapes[4]->next_sp(_shapes[4]->_duration);
		_shapes[5] = new Line(next._pos,Cicles_Shape[5], vel);
		_shapes[6] = new Arc(Cicles_Shape[5],Cicles_Shape[6], true, PI*5.0f/2.0f, vel);
		next = _shapes[6]->next_sp(_shapes[6]->_duration);
		_shapes[8] = new Line(next._pos,Cicles_Shape[7], vel, 0.0f, 2.0f);
		_num = 9;
	}
};

// shape of S
math::Vector<3> S_Shape[10] = {
	math::Vector<3>(2.0f,-2.0f,plane_height),
	math::Vector<3>(2.0f,1.5f,plane_height),
	math::Vector<3>(1.0f,1.5f,plane_height),
	math::Vector<3>(1.0f,-1.5f,plane_height),
	math::Vector<3>(0.0f,-1.5f,plane_height),
	math::Vector<3>(0.0f,1.5f,plane_height),
	math::Vector<3>(-1.0f,1.5f,plane_height),
	math::Vector<3>(-1.0f,-1.5f,plane_height),
	math::Vector<3>(-2.0f,-1.5f,plane_height),
	math::Vector<3>(-2.0f,2.0f,plane_height),
};

struct Path_Shape_S:public Path{
	Path_Shape_S(float vel){
		_shapes[0] = new Line(S_Shape[0],S_Shape[1], vel, 2.0f, .0f);
		_shapes[1] = new Arc(S_Shape[1],S_Shape[2], true, PI, vel);
		_shapes[2] = new Line(S_Shape[2],S_Shape[3], vel);
		_shapes[3] = new Arc(S_Shape[3],S_Shape[4], false, PI, vel);
		_shapes[4] = new Line(S_Shape[4],S_Shape[5], vel);
		_shapes[5] = new Arc(S_Shape[5],S_Shape[6], true, PI, vel);
		_shapes[6] = new Line(S_Shape[6],S_Shape[7], vel);
		_shapes[7] = new Arc(S_Shape[7],S_Shape[8], false, PI, vel);
		_shapes[8] = new Line(S_Shape[8],S_Shape[9], vel, .0f, 2.0f);
		_num = 9;
	}
};

enum shapes {
	POINT = 0,	//fix position setpoint
	LINES,	// shape of two line
	SINS,
	CICLES,	//shape of cicles
	SHAPE_S,	// shape of SSS
};

//static Path* get_path(shapes shape, float vel = .0f){
//	Path* path;
//	switch(shape) {
//	case POINT: path = new Path_Point(); break;
//	case LINES:	path = new Path_Lines(vel); break;
//	case SINS:  path = new Path_Sins(vel); break;
//	case CICLES:	path = new Path_Cicles(vel); break;
//	case SHAPE_S:	path = new Path_Shape_S(vel); break;
//	default:path = nullptr; break;
//	}
//	return path;
//};
}

enum status {
	DISABLE = 0,	//path is disable
	PAUSE,			//hold in the first point of the path
	FOLLOWING			//start following the path
};
static status statu = DISABLE;
static bool manual_loss = false;

static Path_Tracking::Next next;

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	MulticopterPositionControl();

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterPositionControl();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	bool		cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
					  const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res);

private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_control_task;			/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */

	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_local_pos_sp_sub;		/**< offboard local position setpoint */
	int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */

	int		_angacc_acc_sub;		/* angular acceleration and acceleration*/
	int 	_battery_status_sub;

	int		_target_sub;/*target status gyzhang <Jul 11, 2017>*/
	bool	_target_updated;

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */
	orb_advert_t	_acc_ff_pub;		// angular acceleration and acceleration feedforwad --bdai
	orb_advert_t coup_force_pub;
	orb_id_t _attitude_setpoint_id;

	struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
	struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
	struct control_state_s				_ctrl_state;		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;		/**< vehicle global velocity setpoint */
	struct angacc_acc_s			_angacc_acc;
	struct acc_ff_s				_acc_ff;
	struct target_info_s	_target;

	struct coupling_force_s coup_force;

	math::LowPassFilter2p _lp_accff_x;
	math::LowPassFilter2p _lp_accff_y;
	math::LowPassFilter2p _lp_accff_z;


	struct battery_status_s		_battery_status;

	control::BlockParamFloat _manual_thr_min;
	control::BlockParamFloat _manual_thr_max;
	control::BlockParamFloat _acc_ff_a;
	control::BlockParamFloat _mass;
	control::BlockParamFloat _hovering_thr;
	control::BlockParamFloat _ff_max_horizon;
	control::BlockParamFloat _ff_max_vertical;

	control::BlockParamInt _path_type;

	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;

	control::BlockDerivative _pos_x_deriv;
	control::BlockDerivative _pos_y_deriv;
	control::BlockDerivative _pos_z_deriv;

	struct {
		param_t thr_min;
		param_t thr_max;
		param_t thr_hover;
		param_t alt_ctl_dz;
		param_t alt_ctl_dy;
		param_t z_p;
		param_t z_i;
		param_t z_d;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max_up;
		param_t z_vel_max_down;
		param_t z_ff;
		param_t xy_p;
		param_t xy_i;
		param_t xy_d;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_vel_cruise;
		param_t xy_ff;
		param_t tilt_max_air;
		param_t land_speed;
		param_t tko_speed;
		param_t tilt_max_land;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t global_yaw_max;
		param_t mc_att_yaw_p;
		param_t hold_xy_dz;
		param_t hold_max_xy;
		param_t hold_max_z;
		param_t acc_hor_max;
		param_t alt_mode;
		param_t opt_recover;

	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float thr_hover;
		float alt_ctl_dz;
		float alt_ctl_dy;
		float tilt_max_air;
		float land_speed;
		float tko_speed;
		float tilt_max_land;
		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		float global_yaw_max;
		float mc_att_yaw_p;
		float hold_xy_dz;
		float hold_max_xy;
		float hold_max_z;
		float acc_hor_max;
		float vel_max_up;
		float vel_max_down;
		uint32_t alt_mode;

		int opt_recover;

		math::Vector<3> pos_p;
		math::Vector<3> pos_i;
		math::Vector<3> pos_d;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> vel_cruise;
		math::Vector<3> sp_offs_max;
	}		_params;

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	hrt_abstime _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool _do_reset_alt_pos_flag;
	bool _mode_auto;
	bool _pos_hold_engaged;
	bool _alt_hold_engaged;
	bool _run_pos_control;
	bool _run_alt_control;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _pos_err_d;	/**< derivative of current position error */
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _vel_sp_prev;
	math::Vector<3> _vel_err_d;		/**< derivative of current velocity */

	math::Vector<3> _pre_thrust_sp;
	math::Vector<3> _pre_pid_thrust_sp;

	math::Matrix<3, 3> _R;			/**< rotation matrix from attitude quaternions */
	float _yaw;				/**< yaw angle (euler) */
	bool _in_landing;	/**< the vehicle is in the landing descent */
	bool _lnd_reached_ground; /**< controller assumes the vehicle has reached the ground after landing */
	bool _takeoff_jumped;
	float _vel_z_lp;
	float _acc_z_lp;
	float _takeoff_thrust_sp;
	bool _control_vel_enabled_prev;	/**< previous loop was in velocity controlled mode (control_state.flag_control_velocity_enabled) */

	bool _angacc_acc_updated;

	// counters for reset events on position and velocity states
	// they are used to identify a reset event
	uint8_t _z_reset_counter;
	uint8_t _xy_reset_counter;
	uint8_t _vz_reset_counter;
	uint8_t _vxy_reset_counter;
	uint8_t _heading_reset_counter;

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	static float	scale_control(float ctl, float end, float dz, float dy);
	static float    throttle_curve(float ctl, float ctr);

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();
	/**
	 * Reset position setpoint to current position.
	 *
	 * This reset will only occur if the _reset_pos_sp flag has been set.
	 * The general logic is to first "activate" the flag in the flight
	 * regime where a switch to a position control mode should hold the
	 * very last position. Once switching to a position control mode
	 * the last position is stored once.
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude.
	 *
	 * This reset will only occur if the _reset_alt_sp flag has been set.
	 * The general logic follows the reset_pos_sp() architecture.
	 */
	void		reset_alt_sp();

	/**
	 * Check if position setpoint is too far from current position and adjust it if needed.
	 */
	void		limit_pos_sp_offset();

	/**
	 * Set position setpoint using manual control
	 */
	void		control_manual(float dt);

	/**
	 * Set position setpoint using offboard control
	 */
	void		control_offboard(float dt);

	/**
	 * Set position setpoint for AUTO
	 */
	void		control_auto(float dt);

	/**
	 * Select between barometric and global (AMSL) altitudes
	 */
	void		select_alt(bool global);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};

namespace pos_control
{

MulticopterPositionControl	*g_control;
}

MulticopterPositionControl::MulticopterPositionControl() :
	SuperBlock(NULL, "MPC"),
	_task_should_exit(false),
	_control_task(-1),
	_mavlink_log_pub(nullptr),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_global_vel_sp_sub(-1),
	_angacc_acc_sub(-1),
	_battery_status_sub(-1),

	/*gyzhang <Jul 11, 2017>*/
	_target_sub(-1),
	_target_updated(false),

	/* publications */
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),
	_global_vel_sp_pub(nullptr),
	_acc_ff_pub(nullptr),
	coup_force_pub(nullptr),
	_attitude_setpoint_id(0),
	_vehicle_status{},
	_vehicle_land_detected{},
	_ctrl_state{},
	_att_sp{},
	_manual{},
	_control_mode{},
	_arming{},
	_local_pos{},
	_pos_sp_triplet{},
	_local_pos_sp{},
	_global_vel_sp{},
	_angacc_acc{},
	_acc_ff{},
	_target{},
	_lp_accff_x(13.0f, 2.0f),
	_lp_accff_y(13.0f, 2.0f),
	_lp_accff_z(13.0f, 2.0f),

	_battery_status{},

	_manual_thr_min(this, "MANTHR_MIN"),
	_manual_thr_max(this, "MANTHR_MAX"),
	_acc_ff_a(this, "FF_ACC"),
	_mass(this, "MASS"),
	_hovering_thr(this, "HOVER_THR"),
	_ff_max_horizon(this,"FF_MAX_H"),
	_ff_max_vertical(this,"FF_MAX_V"),
	_path_type(this, "PATH_TYPE"),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_pos_x_deriv(this, "POSD"),
	_pos_y_deriv(this, "POSD"),
	_pos_z_deriv(this, "POSD"),
	_ref_alt(0.0f),
	_ref_timestamp(0),

	_reset_pos_sp(true),
	_reset_alt_sp(true),
	_do_reset_alt_pos_flag(true),
	_mode_auto(false),
	_pos_hold_engaged(false),
	_alt_hold_engaged(false),
	_run_pos_control(true),
	_run_alt_control(true),
	_yaw(0.0f),
	_in_landing(false),
	_lnd_reached_ground(false),
	_takeoff_jumped(false),
	_vel_z_lp(0),
	_acc_z_lp(0),
	_takeoff_thrust_sp(0.0f),
	_control_vel_enabled_prev(false),
	_angacc_acc_updated(false),
	_z_reset_counter(0),
	_xy_reset_counter(0),
	_vz_reset_counter(0),
	_vxy_reset_counter(0),
	_heading_reset_counter(0)
{
	// Make the quaternion valid for control state
	_ctrl_state.q[0] = 1.0f;

	memset(&_ref_pos, 0, sizeof(_ref_pos));

	_params.pos_p.zero();
	_params.pos_i.zero();
	_params.pos_d.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_cruise.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_pos.zero();
	_pos_sp.zero();
	_pos_err_d.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_ff.zero();
	_vel_sp_prev.zero();
	_vel_err_d.zero();
	_pre_thrust_sp.zero();
	_pre_pid_thrust_sp.zero();

	_R.identity();

	_params_handles.thr_min		= param_find("MPC_THR_MIN");
	_params_handles.thr_max		= param_find("MPC_THR_MAX");
	_params_handles.thr_hover	= param_find("MPC_THR_HOVER");
	_params_handles.alt_ctl_dz	= param_find("MPC_ALTCTL_DZ");
	_params_handles.alt_ctl_dy	= param_find("MPC_ALTCTL_DY");
	_params_handles.z_p		= param_find("MPC_Z_P");
	_params_handles.z_i		= param_find("MPC_Z_I");
	_params_handles.z_d		= param_find("MPC_Z_D");
	_params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	_params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	_params_handles.z_vel_max_up	= param_find("MPC_Z_VEL_MAX_UP");
	_params_handles.z_vel_max_down	= param_find("MPC_Z_VEL_MAX");

	// transitional support: Copy param values from max to down
	// param so that max param can be renamed in 1-2 releases
	// (currently at 1.3.0)
	float p;
	param_get(param_find("MPC_Z_VEL_MAX"), &p);
	param_set(param_find("MPC_Z_VEL_MAX_DN"), &p);

	_params_handles.z_ff		= param_find("MPC_Z_FF");
	_params_handles.xy_p		= param_find("MPC_XY_P");
	_params_handles.xy_i		= param_find("MPC_XY_I");
	_params_handles.xy_d		= param_find("MPC_XY_D");
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	_params_handles.xy_vel_cruise	= param_find("MPC_XY_CRUISE");
	_params_handles.xy_ff		= param_find("MPC_XY_FF");
	_params_handles.tilt_max_air	= param_find("MPC_TILTMAX_AIR");
	_params_handles.land_speed	= param_find("MPC_LAND_SPEED");
	_params_handles.tko_speed	= param_find("MPC_TKO_SPEED");
	_params_handles.tilt_max_land	= param_find("MPC_TILTMAX_LND");
	_params_handles.man_roll_max = param_find("MPC_MAN_R_MAX");
	_params_handles.man_pitch_max = param_find("MPC_MAN_P_MAX");
	_params_handles.man_yaw_max = param_find("MPC_MAN_Y_MAX");
	_params_handles.global_yaw_max = param_find("MC_YAWRATE_MAX");
	_params_handles.mc_att_yaw_p = param_find("MC_YAW_P");
	_params_handles.hold_xy_dz = param_find("MPC_HOLD_XY_DZ");
	_params_handles.hold_max_xy = param_find("MPC_HOLD_MAX_XY");
	_params_handles.hold_max_z = param_find("MPC_HOLD_MAX_Z");
	_params_handles.acc_hor_max = param_find("MPC_ACC_HOR_MAX");
	_params_handles.alt_mode = param_find("MPC_ALT_MODE");
	_params_handles.opt_recover = param_find("VT_OPT_RECOV_EN");

	/* fetch initial parameter values */
	parameters_update(true);
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	pos_control::g_control = nullptr;
}

int
MulticopterPositionControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_params_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		/* update C++ param system */
		updateParams();

		/* update legacy C interface params */
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.thr_hover, &_params.thr_hover);
		param_get(_params_handles.alt_ctl_dz, &_params.alt_ctl_dz);
		param_get(_params_handles.alt_ctl_dy, &_params.alt_ctl_dy);
		param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
		_params.tilt_max_air = math::radians(_params.tilt_max_air);
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.tko_speed, &_params.tko_speed);
		param_get(_params_handles.tilt_max_land, &_params.tilt_max_land);
		_params.tilt_max_land = math::radians(_params.tilt_max_land);

		float v;
		uint32_t v_i;
		param_get(_params_handles.xy_p, &v);
		_params.pos_p(0) = v;
		_params.pos_p(1) = v;
		param_get(_params_handles.xy_i, &v);
		_params.pos_i(0) = v;
		_params.pos_i(1) = v;
		param_get(_params_handles.xy_d, &v);
		_params.pos_d(0) = v;
		_params.pos_d(1) = v;
		param_get(_params_handles.z_p, &v);
		_params.pos_p(2) = v;
		param_get(_params_handles.z_i, &v);
		_params.pos_i(2) = v;
		param_get(_params_handles.z_d, &v);
		_params.pos_d(2) = v;
		param_get(_params_handles.xy_vel_p, &v);
		_params.vel_p(0) = v;
		_params.vel_p(1) = v;
		param_get(_params_handles.z_vel_p, &v);
		_params.vel_p(2) = v;
		param_get(_params_handles.xy_vel_i, &v);
		_params.vel_i(0) = v;
		_params.vel_i(1) = v;
		param_get(_params_handles.z_vel_i, &v);
		_params.vel_i(2) = v;
		param_get(_params_handles.xy_vel_d, &v);
		_params.vel_d(0) = v;
		_params.vel_d(1) = v;
		param_get(_params_handles.z_vel_d, &v);
		_params.vel_d(2) = v;
		param_get(_params_handles.xy_vel_max, &v);
		_params.vel_max(0) = v;
		_params.vel_max(1) = v;
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_max_up = v;
		_params.vel_max(2) = v;
		param_get(_params_handles.z_vel_max_down, &v);
		_params.vel_max_down = v;
		param_get(_params_handles.xy_vel_cruise, &v);
		_params.vel_cruise(0) = v;
		_params.vel_cruise(1) = v;
		/* using Z max up for now */
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_cruise(2) = v;
		param_get(_params_handles.xy_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(2) = v;
		param_get(_params_handles.hold_xy_dz, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.hold_xy_dz = v;
		param_get(_params_handles.hold_max_xy, &v);
		_params.hold_max_xy = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.hold_max_z, &v);
		_params.hold_max_z = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.acc_hor_max, &v);
		_params.acc_hor_max = v;
		/*
		 * increase the maximum horizontal acceleration such that stopping
		 * within 1 s from full speed is feasible
		 */
		_params.acc_hor_max = math::max(_params.vel_cruise(0), _params.acc_hor_max);
		param_get(_params_handles.alt_mode, &v_i);
		_params.alt_mode = v_i;

		int i;
		param_get(_params_handles.opt_recover, &i);
		_params.opt_recover = i;

		_params.sp_offs_max = _params.vel_cruise.edivide(_params.pos_p) * 2.0f;

		/* mc attitude control parameters*/
		/* manual control scale */
		param_get(_params_handles.man_roll_max, &_params.man_roll_max);
		param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
		param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
		param_get(_params_handles.global_yaw_max, &_params.global_yaw_max);
		_params.man_roll_max = math::radians(_params.man_roll_max);
		_params.man_pitch_max = math::radians(_params.man_pitch_max);
		_params.man_yaw_max = math::radians(_params.man_yaw_max);
		_params.global_yaw_max = math::radians(_params.global_yaw_max);

		param_get(_params_handles.mc_att_yaw_p, &v);
		_params.mc_att_yaw_p = v;

		/* takeoff and land velocities should not exceed maximum */
		_params.tko_speed = fminf(_params.tko_speed, _params.vel_max_up);
		_params.land_speed = fminf(_params.land_speed, _params.vel_max_down);
	}

	return OK;
}

void
MulticopterPositionControl::poll_subscriptions()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_attitude_setpoint_id) {
			if (_vehicle_status.is_vtol) {
				_attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);

			} else {
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

	orb_check(_ctrl_state_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

		/* get current rotation matrix and euler angles from control state quaternions */
		math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
		_R = q_att.to_dcm();
		math::Vector<3> euler_angles;
		euler_angles = _R.to_euler();
		_yaw = euler_angles(2);

		if (_control_mode.flag_control_manual_enabled) {
			if (_heading_reset_counter != _ctrl_state.quat_reset_counter) {
				_heading_reset_counter = _ctrl_state.quat_reset_counter;
				math::Quaternion delta_q(_ctrl_state.delta_q_reset[0], _ctrl_state.delta_q_reset[1], _ctrl_state.delta_q_reset[2],
							 _ctrl_state.delta_q_reset[3]);

				// we only extract the heading change from the delta quaternion
				math::Vector<3> delta_euler = delta_q.to_euler();
				_att_sp.yaw_body += delta_euler(2);
			}
		}

	}

	orb_check(_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	orb_check(_manual_sub, &updated);

	hrt_abstime now = hrt_absolute_time();
	static hrt_abstime manual_time = now;
	if (updated) {
		manual_time = now; manual_loss = false;
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	} else {
		if(now - manual_time > 5*1e5) manual_loss = true;
	}

	orb_check(_arming_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
	}

	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

		// check if a reset event has happened
		// if the vehicle is in manual mode we will shift the setpoints of the
		// states which were reset. In auto mode we do not shift the setpoints
		// since we want the vehicle to track the original state.
		if (_control_mode.flag_control_manual_enabled) {
			if (_z_reset_counter != _local_pos.z_reset_counter) {
				_pos_sp(2) += _local_pos.delta_z;
			}

			if (_xy_reset_counter != _local_pos.xy_reset_counter) {
				_pos_sp(0) += _local_pos.delta_xy[0];
				_pos_sp(1) += _local_pos.delta_xy[1];
			}

			if (_vz_reset_counter != _local_pos.vz_reset_counter) {
				_vel_sp(2) += _local_pos.delta_vz;
				_vel_sp_prev(2) +=  _local_pos.delta_vz;
			}

			if (_vxy_reset_counter != _local_pos.vxy_reset_counter) {
				_vel_sp(0) += _local_pos.delta_vxy[0];
				_vel_sp(1) += _local_pos.delta_vxy[1];
				_vel_sp_prev(0) += _local_pos.delta_vxy[0];
				_vel_sp_prev(1) += _local_pos.delta_vxy[1];
			}
		}

		// update the reset counters in any case
		_z_reset_counter = _local_pos.z_reset_counter;
		_xy_reset_counter = _local_pos.xy_reset_counter;
		_vz_reset_counter = _local_pos.vz_reset_counter;
		_vxy_reset_counter = _local_pos.vxy_reset_counter;
	}

	orb_check(_angacc_acc_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(angacc_acc), _angacc_acc_sub, &_angacc_acc);
		_angacc_acc_updated = true;
//		PX4_INFO("_angacc_acc: %d", _angacc_acc.valid_acc);
	} else {
		_angacc_acc_updated = false;
	}

	orb_check(_battery_status_sub, &updated);
	if(updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}

	orb_check(_target_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(target_info), _target_sub, &_target);
		_target_updated = true;
	}
}

float
MulticopterPositionControl::scale_control(float ctl, float end, float dz, float dy)
{
	if (ctl > dz) {
		return dy + (ctl - dz) * (1.0f - dy) / (end - dz);

	} else if (ctl < -dz) {
		return -dy + (ctl + dz) * (1.0f - dy) / (end - dz);

	} else {
		return ctl * (dy / dz);
	}
}

float
MulticopterPositionControl::throttle_curve(float ctl, float ctr)
{
	/* piecewise linear mapping: 0:ctr -> 0:0.5
	 * and ctr:1 -> 0.5:1 */
	if (ctl < 0.5f) {
		return 2 * ctl * ctr;

	} else {
		return ctr + 2 * (ctl - 0.5f) * (1.0f - ctr);
	}
}

void
MulticopterPositionControl::task_main_trampoline(int argc, char *argv[])
{
	pos_control::g_control->task_main();
}

void
MulticopterPositionControl::update_ref()
{
	if (_local_pos.ref_timestamp != _ref_timestamp) {
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) {
			/* calculate current position setpoint in global frame */
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
			alt_sp = _ref_alt - _pos_sp(2);
		}

		/* update local projection reference */
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		_ref_alt = _local_pos.ref_alt;

		if (_ref_timestamp != 0) {
			/* reproject position setpoint to new reference */
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
			_pos_sp(2) = -(alt_sp - _ref_alt);
		}

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}

void
MulticopterPositionControl::reset_pos_sp()
{
	if (_reset_pos_sp) {
		_reset_pos_sp = false;

		// we have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// position in a special way. In position control mode the position will be reset anyway until the vehicle has reduced speed.
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
	}
}

void
MulticopterPositionControl::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;

		// we have logic in the main function which choosed the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// altitude in a special way
		_pos_sp(2) = _pos(2);
	}
}

void
MulticopterPositionControl::limit_pos_sp_offset()
{
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();

	if (_control_mode.flag_control_position_enabled) {
		pos_sp_offs(0) = (_pos_sp(0) - _pos(0)) / _params.sp_offs_max(0);
		pos_sp_offs(1) = (_pos_sp(1) - _pos(1)) / _params.sp_offs_max(1);
	}

	if (_control_mode.flag_control_altitude_enabled) {
		pos_sp_offs(2) = (_pos_sp(2) - _pos(2)) / _params.sp_offs_max(2);
	}

	float pos_sp_offs_norm = pos_sp_offs.length();

	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		_pos_sp = _pos + pos_sp_offs.emult(_params.sp_offs_max);
	}
}


void
MulticopterPositionControl::control_manual(float dt)
{
	/* Entering manual control from non-manual control mode, reset alt/pos setpoints */
	if (_mode_auto) {
		_mode_auto = false;

		/* Reset alt pos flags if resetting is enabled */
		if (_do_reset_alt_pos_flag) {
			_reset_pos_sp = true;
			_reset_alt_sp = true;
		}
	}

//	if (_control_mode.flag_control_altitude_enabled &&_control_mode.flag_control_position_enabled) {

//		hrt_abstime now = hrt_absolute_time();
//		static hrt_abstime last_pause_stamp = now;
//
//		static float path_time = 0.0f;
//		static float path_over_time = 0.0f;
//		static status last_statu = statu;
//		last_statu = statu;
//		if (true/*_manual.aux3 < -0.5f*/) {
//			path_over_time = path_time = 0.0f;
//			last_pause_stamp = now;
//			statu = DISABLE;
//		}else if (_manual.aux3 < 0.5f || manual_loss){
//			path_over_time = path_time;
//			last_pause_stamp = now;
//			statu = PAUSE;
//		}else {
//			statu = FOLLOWING;
//			path_time = path_over_time + (now - last_pause_stamp)*1.0e-6f;
//		}
//
//		if (statu != DISABLE) {
//			// get the path at the first time
//			static Path_Tracking::Path* path = Path_Tracking::get_path(Path_Tracking::POINT);;
//			if (last_statu == DISABLE && _path_type.get() != Path_Tracking::POINT)
//				path = Path_Tracking::get_path((Path_Tracking::shapes)_path_type.get(), 0.5f);
//
//			int ret = path->get_next(path_time, next);
//			if (statu == PAUSE) next._vel.zero();
//			// else PX4_INFO("dt: %8.4f, %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f", (double)path_time,
//			// 		(double)_pos_sp(0),(double)_pos_sp(1),(double)_pos_sp(2),
//			// 		(double)next._vel(0),(double)next._vel(1),(double)next._vel(2));
//			_pos_sp = next._pos;
//
//			if(ret == 0) return;
//		}
//
//		// if switch back to disable
//		if (last_statu != DISABLE) {
//			_reset_pos_sp = true;
//			_reset_alt_sp = true;
//		}
//	}


	if(_manual.aux3 > -0.5f && _control_mode.flag_control_altitude_enabled &&_control_mode.flag_control_position_enabled){

		/*subscribe target position -bdai<1 Nov 2016>*/
		matrix::Vector3f target_pos(_target.x, _target.y, _target.z);
	//	matrix::Vector3f target_pos(0.0f, 0.0f, 0.0f);

		// rise up when grap successed --bdai
		hrt_abstime now = hrt_absolute_time();
 		static float rise_hight = 0.20f;
 		static hrt_abstime rise_delay_start =  now;
		static hrt_abstime rise_delay_time = 800000; // waiting for manipulator grap

 		if (REQUEST_RISE_UP) {
 			if ((now - rise_delay_start) > rise_delay_time) {
				target_pos(2) = target_pos(2) - rise_hight;				 
 				RISE_UP = true;
 			} else {
 				RISE_UP = false;
 			}
 		} else {
 			rise_delay_start = now;
 			RISE_UP = false;
 		}

		matrix::Dcmf R_BN;
		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 3; j++){
				R_BN(i,j) = _R(i,j);
			}
		}
		matrix::Vector3f pos(_pos(0), _pos(1), _pos(2));

	//	matrix::Vector3f pos = matrix::Vector3f(1.5f, .0f, .0f);
	//	Quatf qq;
	//	qq.from_axis_angle(matrix::Vector3f(.0f, 1.0f, .0f), (now - first_time)*3.0f * DEG2RAD / 1.0e6f);
	//	matrix::Dcmf RR(qq);
	//	pos = RR * pos - R_BN * (MANI_FIRST_JOINT + MANI_OFFSET);;

		matrix::Vector3f pos_first_joint = pos +  R_BN * (MANI_FIRST_JOINT + MANI_OFFSET);

		matrix::Vector3f direction = (target_pos - pos_first_joint).normalized();

		matrix::Vector3f r = (matrix::Vector3f(0.0f, 0.0f, 1.0f) % direction).normalized();

		int in_range = 7;

		if (direction(2) < sinf(pos_sp_condition[ANGLE_MIN])) {
			Quatf q;
			q.from_axis_angle(r, (float)M_PI/2.0f - pos_sp_condition[ANGLE_MIN]);
			matrix::Dcmf R(q);
			direction = R * matrix::Vector3f(0.0f, 0.0f, 1.0f);
			in_range &= 0<<1;
		} else if (direction(2) > sinf(pos_sp_condition[ANGLE_MAX])){
			Quatf q;
			q.from_axis_angle(r, (float)M_PI/2.0f - pos_sp_condition[ANGLE_MAX]);
			matrix::Dcmf R(q);
			direction = R * matrix::Vector3f(0.0f, 0.0f, 1.0f);
			in_range &= 0;
		}
		matrix::Vector3f relative_pos_sp = -direction * pos_sp_condition[R_max];
		
		math::Vector<3> pos_sp_temp;
		/*if current pos is not in right position -bdai<28 Nov 2016>*/
		matrix::Vector3f err_sp = target_pos - pos_first_joint + relative_pos_sp;
		if (err_sp.norm() > pos_sp_condition[R_min]) {
			in_range &= 0<<2;
			pos_sp_temp = _pos + math::Vector<3>(err_sp(0), err_sp(1), err_sp(2));
		} else {
			// not change pos_sp_temp
			pos_sp_temp = _pos_sp;
		}

		for (int i = 0; i < 3; i++){
			if (pos_sp_temp(i) < ELEC_FENCE[i][MIN]){
				pos_sp_temp(i) = ELEC_FENCE[i][MIN];
				OUT_FENCE = true;
			} else if (pos_sp_temp(i) > ELEC_FENCE[i][MAX]){
				pos_sp_temp(i) = ELEC_FENCE[i][MAX];
				OUT_FENCE = true;
			}
		}


		_pos_sp = pos_sp_temp;

		/*calculate yaw  -bdai<17 Nov 2016>*/

		direction = (target_pos - pos).normalized();
		/*if there are too close with target -bdai<17 Nov 2016>*/
		if (math::Vector<3>(direction(0), direction(1), 0).length() > sinf(15 * DEG2RAD)) {
			float yaw_sp = atan2f(direction(1), direction(0));
			static float yaw_sp_last = yaw_sp;

			PX4_INFO("yaw_err: %8.4f",(double)fabsf((yaw_sp -_yaw) / DEG2RAD));
			if (fabsf(yaw_sp -_yaw) > 10.0f * DEG2RAD) {
				yaw_sp = _yaw + (yaw_sp -_yaw) / fabsf(yaw_sp -_yaw) * 5.0f * DEG2RAD;
			} else if(fabsf(yaw_sp -_yaw) > 5.0f * DEG2RAD){
				yaw_sp = yaw_sp - (yaw_sp -_yaw) / fabsf(yaw_sp -_yaw) * 5.0f * DEG2RAD;
			} else {
				yaw_sp = yaw_sp_last;
			}
			_att_sp.yaw_body = yaw_sp_last = yaw_sp;
			PX4_INFO("yaw_sp: %8.4f, _yaw:%8.4f",(double)(yaw_sp / DEG2RAD), (double)(_yaw / DEG2RAD));
		}
	}
	else {

		statu = DISABLE; //no path
		math::Vector<3> req_vel_sp; // X,Y in local frame and Z in global (D), in [-1,1] normalized range
		req_vel_sp.zero();

		if (_control_mode.flag_control_altitude_enabled) {
			/* set vertical velocity setpoint with throttle stick */
			req_vel_sp(2) = -scale_control(_manual.z - 0.5f, 0.5f, _params.alt_ctl_dz, _params.alt_ctl_dy); // D
		}

		if (_control_mode.flag_control_position_enabled) {
			/* set horizontal velocity setpoint with roll/pitch stick */
			req_vel_sp(0) = _manual.x;
			req_vel_sp(1) = _manual.y;
		}

		if (_control_mode.flag_control_altitude_enabled) {
			/* reset alt setpoint to current altitude if needed */
			reset_alt_sp();
		}

		if (_control_mode.flag_control_position_enabled) {
			/* reset position setpoint to current position if needed */
			reset_pos_sp();
		}

		/* limit velocity setpoint */
		float req_vel_sp_norm = req_vel_sp.length();

		if (req_vel_sp_norm > 1.0f) {
			req_vel_sp /= req_vel_sp_norm;
		}

		/* _req_vel_sp scaled to 0..1, scale it to max speed and rotate around yaw */
		math::Matrix<3, 3> R_yaw_sp;
		R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp.yaw_body);
		math::Vector<3> req_vel_sp_scaled = R_yaw_sp * req_vel_sp.emult(
				_params.vel_cruise); // in NED and scaled to actual velocity

		/*
		 * assisted velocity mode: user controls velocity, but if	velocity is small enough, position
		 * hold is activated for the corresponding axis
		 */

		/* horizontal axes */
		if (_control_mode.flag_control_position_enabled) {
			/* check for pos. hold */
			if (fabsf(req_vel_sp(0)) < _params.hold_xy_dz && fabsf(req_vel_sp(1)) < _params.hold_xy_dz) {
				if (!_pos_hold_engaged) {

					float vel_xy_mag = sqrtf(_vel(0) * _vel(0) + _vel(1) * _vel(1));

					if (_params.hold_max_xy < FLT_EPSILON || vel_xy_mag < _params.hold_max_xy) {
						/* reset position setpoint to have smooth transition from velocity control to position control */
						_pos_hold_engaged = true;
						_pos_sp(0) = _pos(0);
						_pos_sp(1) = _pos(1);

					} else {
						_pos_hold_engaged = false;
					}
				}

			} else {
				_pos_hold_engaged = false;
			}

			/* set requested velocity setpoint */
			if (!_pos_hold_engaged) {
				_pos_sp(0) = _pos(0);
				_pos_sp(1) = _pos(1);
				_run_pos_control = false; /* request velocity setpoint to be used, instead of position setpoint */
				_vel_sp(0) = req_vel_sp_scaled(0);
				_vel_sp(1) = req_vel_sp_scaled(1);
			}
		}

		/* vertical axis */
		if (_control_mode.flag_control_altitude_enabled) {
			/* check for pos. hold */
			if (fabsf(req_vel_sp(2)) < FLT_EPSILON) {
				if (!_alt_hold_engaged) {
					if (_params.hold_max_z < FLT_EPSILON || fabsf(_vel(2)) < _params.hold_max_z) {
						/* reset position setpoint to have smooth transition from velocity control to position control */
						_alt_hold_engaged = true;
						_pos_sp(2) = _pos(2);

					} else {
						_alt_hold_engaged = false;
					}
				}

			} else {
				_alt_hold_engaged = false;
				_pos_sp(2) = _pos(2);
			}

			/* set requested velocity setpoint */
			if (!_alt_hold_engaged) {
				_run_alt_control = false; /* request velocity setpoint to be used, instead of altitude setpoint */
				_vel_sp(2) = req_vel_sp_scaled(2);
			}
		}
	}
}

void
MulticopterPositionControl::control_offboard(float dt)
{
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}

	if (_pos_sp_triplet.current.valid) {
		if (_control_mode.flag_control_position_enabled && _pos_sp_triplet.current.position_valid) {
			/* control position */
			_pos_sp(0) = _pos_sp_triplet.current.x;
			_pos_sp(1) = _pos_sp_triplet.current.y;

		} else if (_control_mode.flag_control_velocity_enabled && _pos_sp_triplet.current.velocity_valid) {
			/* control velocity */
			/* reset position setpoint to current position if needed */
			reset_pos_sp();

			/* set position setpoint move rate */
			_vel_sp(0) = _pos_sp_triplet.current.vx;
			_vel_sp(1) = _pos_sp_triplet.current.vy;

			_run_pos_control = false; /* request velocity setpoint to be used, instead of position setpoint */
		}

		if (_pos_sp_triplet.current.yaw_valid) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;

		} else if (_pos_sp_triplet.current.yawspeed_valid) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;
		}

		if (_control_mode.flag_control_altitude_enabled && _pos_sp_triplet.current.alt_valid) {
			/* control altitude as it is enabled */
			_pos_sp(2) = _pos_sp_triplet.current.z;
			_run_alt_control = true;

		} else if (_control_mode.flag_control_altitude_enabled && _pos_sp_triplet.current.position_valid) {
			/* control altitude because full position control is enabled */
			_pos_sp(2) = _pos_sp_triplet.current.z;
			_run_alt_control = true;

		} else if (_control_mode.flag_control_climb_rate_enabled && _pos_sp_triplet.current.velocity_valid) {
			/* reset alt setpoint to current altitude if needed */
			reset_alt_sp();

			/* set altitude setpoint move rate */
			_vel_sp(2) = _pos_sp_triplet.current.vz;

			_run_alt_control = false; /* request velocity setpoint to be used, instead of position setpoint */
		}

	} else {
		reset_pos_sp();
		reset_alt_sp();
	}
}

bool
MulticopterPositionControl::cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
		const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res)
{
	/* project center of sphere on line */
	/* normalized AB */
	math::Vector<3> ab_norm = line_b - line_a;
	ab_norm.normalize();
	math::Vector<3> d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		/* we have triangle CDX with known CD and CX = R, find DX */
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			/* target waypoint is already behind us */
			res = line_b;

		} else {
			/* target is in front of us */
			res = d + ab_norm * dx_len; // vector A->B on line
		}

		return true;

	} else {
		/* have no roots, return D */
		res = d; /* go directly to line */

		/* previous waypoint is still in front of us */
		if ((sphere_c - line_a) * ab_norm < 0.0f) {
			res = line_a;
		}

		/* target waypoint is already behind us */
		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			res = line_b;
		}

		return false;
	}
}

void MulticopterPositionControl::control_auto(float dt)
{
	/* reset position setpoint on AUTO mode activation or if we are not in MC mode */
	if (!_mode_auto || !_vehicle_status.is_rotary_wing) {
		if (!_mode_auto) {
			_mode_auto = true;
		}

		_reset_pos_sp = true;
		_reset_alt_sp = true;
	}

	// Always check reset state of altitude and position control flags in auto
	reset_pos_sp();
	reset_alt_sp();

	//Poll position setpoint
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		//Make sure that the position setpoint is valid
		if (!PX4_ISFINITE(_pos_sp_triplet.current.lat) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.lon) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
			_pos_sp_triplet.current.valid = false;
		}
	}

	bool current_setpoint_valid = false;
	bool previous_setpoint_valid = false;

	math::Vector<3> prev_sp;
	math::Vector<3> curr_sp;

	if (_pos_sp_triplet.current.valid) {

		/* project setpoint to local frame */
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
				       &curr_sp.data[0], &curr_sp.data[1]);
		curr_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

		if (PX4_ISFINITE(curr_sp(0)) &&
		    PX4_ISFINITE(curr_sp(1)) &&
		    PX4_ISFINITE(curr_sp(2))) {
			current_setpoint_valid = true;
		}
	}

	if (_pos_sp_triplet.previous.valid) {
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon,
				       &prev_sp.data[0], &prev_sp.data[1]);
		prev_sp(2) = -(_pos_sp_triplet.previous.alt - _ref_alt);

		if (PX4_ISFINITE(prev_sp(0)) &&
		    PX4_ISFINITE(prev_sp(1)) &&
		    PX4_ISFINITE(prev_sp(2))) {
			previous_setpoint_valid = true;
		}
	}

	if (current_setpoint_valid &&
	    (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_IDLE)) {

		/* scaled space: 1 == position error resulting max allowed speed */

		math::Vector<3> cruising_speed = _params.vel_cruise;

		if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
		    _pos_sp_triplet.current.cruising_speed > 0.1f) {
			cruising_speed(0) = _pos_sp_triplet.current.cruising_speed;
			cruising_speed(1) = _pos_sp_triplet.current.cruising_speed;
		}

		math::Vector<3> scale = _params.pos_p.edivide(cruising_speed);

		/* convert current setpoint to scaled space */
		math::Vector<3> curr_sp_s = curr_sp.emult(scale);

		/* by default use current setpoint as is */
		math::Vector<3> pos_sp_s = curr_sp_s;

		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION  ||
		     _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) &&
		    previous_setpoint_valid) {

			/* follow "previous - current" line */

			if ((curr_sp - prev_sp).length() > MIN_DIST) {

				/* find X - cross point of unit sphere and trajectory */
				math::Vector<3> pos_s = _pos.emult(scale);
				math::Vector<3> prev_sp_s = prev_sp.emult(scale);
				math::Vector<3> prev_curr_s = curr_sp_s - prev_sp_s;
				math::Vector<3> curr_pos_s = pos_s - curr_sp_s;
				float curr_pos_s_len = curr_pos_s.length();

				if (curr_pos_s_len < 1.0f) {
					/* copter is closer to waypoint than unit radius */
					/* check next waypoint and use it to avoid slowing down when passing via waypoint */
					if (_pos_sp_triplet.next.valid) {
						math::Vector<3> next_sp;
						map_projection_project(&_ref_pos,
								       _pos_sp_triplet.next.lat, _pos_sp_triplet.next.lon,
								       &next_sp.data[0], &next_sp.data[1]);
						next_sp(2) = -(_pos_sp_triplet.next.alt - _ref_alt);

						if ((next_sp - curr_sp).length() > MIN_DIST) {
							math::Vector<3> next_sp_s = next_sp.emult(scale);

							/* calculate angle prev - curr - next */
							math::Vector<3> curr_next_s = next_sp_s - curr_sp_s;
							math::Vector<3> prev_curr_s_norm = prev_curr_s.normalized();

							/* cos(a) * curr_next, a = angle between current and next trajectory segments */
							float cos_a_curr_next = prev_curr_s_norm * curr_next_s;

							/* cos(b), b = angle pos - curr_sp - prev_sp */
							float cos_b = -curr_pos_s * prev_curr_s_norm / curr_pos_s_len;

							if (cos_a_curr_next > 0.0f && cos_b > 0.0f) {
								float curr_next_s_len = curr_next_s.length();

								/* if curr - next distance is larger than unit radius, limit it */
								if (curr_next_s_len > 1.0f) {
									cos_a_curr_next /= curr_next_s_len;
								}

								/* feed forward position setpoint offset */
								math::Vector<3> pos_ff = prev_curr_s_norm *
											 cos_a_curr_next * cos_b * cos_b * (1.0f - curr_pos_s_len) *
											 (1.0f - expf(-curr_pos_s_len * curr_pos_s_len * 20.0f));
								pos_sp_s += pos_ff;
							}
						}
					}

				} else {
					bool near = cross_sphere_line(pos_s, 1.0f, prev_sp_s, curr_sp_s, pos_sp_s);

					if (!near) {
						/* we're far away from trajectory, pos_sp_s is set to the nearest point on the trajectory */
						pos_sp_s = pos_s + (pos_sp_s - pos_s).normalized();
					}
				}
			}
		}

		/* move setpoint not faster than max allowed speed */
		math::Vector<3> pos_sp_old_s = _pos_sp.emult(scale);

		/* difference between current and desired position setpoints, 1 = max speed */
		math::Vector<3> d_pos_m = (pos_sp_s - pos_sp_old_s).edivide(_params.pos_p);
		float d_pos_m_len = d_pos_m.length();

		if (d_pos_m_len > dt) {
			pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).emult(_params.pos_p);
		}

		/* scale result back to normal space */
		_pos_sp = pos_sp_s.edivide(scale);

		/* update yaw setpoint if needed */

		if (_pos_sp_triplet.current.yawspeed_valid
		    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;

		} else if (PX4_ISFINITE(_pos_sp_triplet.current.yaw)) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
		}

		/*
		 * if we're already near the current takeoff setpoint don't reset in case we switch back to posctl.
		 * this makes the takeoff finish smoothly.
		 */
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
		     || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER)
		    && _pos_sp_triplet.current.acceptance_radius > 0.0f
		    /* need to detect we're close a bit before the navigator switches from takeoff to next waypoint */
		    && (_pos - _pos_sp).length() < _pos_sp_triplet.current.acceptance_radius * 1.2f) {
			_do_reset_alt_pos_flag = false;

			/* otherwise: in case of interrupted mission don't go to waypoint but stay at current position */

		} else {
			_do_reset_alt_pos_flag = true;
		}

		// During a mission or in loiter it's safe to retract the landing gear.
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
		     _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) &&
		    !_vehicle_land_detected.landed) {
			_att_sp.landing_gear = 1.0f;

			// During takeoff and landing, we better put it down again.

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ||
			   _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
			_att_sp.landing_gear = -1.0f;

		} else {
			// For the rest of the setpoint types, just leave it as is.
		}

	} else {
		/* no waypoint, do nothing, setpoint was already reset */
	}
}

void
MulticopterPositionControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	_global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));
	_angacc_acc_sub = orb_subscribe(ORB_ID(angacc_acc));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	_target_sub = orb_subscribe(ORB_ID(target_info));

	parameters_update(true);

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	poll_subscriptions();

	/* We really need to know from the beginning if we're landed or in-air. */
	orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);

	bool reset_int_z = true;
	bool reset_int_z_manual = false;
	bool reset_int_xy = true;
	bool reset_yaw_sp = true;
	bool was_armed = false;

	hrt_abstime t_prev = 0;

	math::Vector<3> pos_err_i;
	pos_err_i.zero();
	math::Vector<3> thrust_int;
	thrust_int.zero();
	math::Vector<3> vel_ff;
	vel_ff.zero();

	// Let's be safe and have the landing gear down by default
	_att_sp.landing_gear = -1.0f;


	matrix::Dcmf R;
	R.identity();

	/* wakeup source */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		/* wait for up to 20ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			// Go through the loop anyway to copy manual input at 50 Hz.
		}

		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		poll_subscriptions();

		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
		float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
		t_prev = t;

		// set dt for control blocks
		setDt(dt);

		if (_control_mode.flag_armed && !was_armed) {
			/* reset setpoints and integrals on arming */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_do_reset_alt_pos_flag = true;
			_vel_sp_prev.zero();
			reset_int_z = true;
			reset_int_xy = true;
			reset_yaw_sp = true;
		}

		/* reset yaw and altitude setpoint for VTOL which are in fw mode */
		if (_vehicle_status.is_vtol) {
			if (!_vehicle_status.is_rotary_wing) {
				reset_yaw_sp = true;
				_reset_alt_sp = true;
			}
		}

		//Update previous arming state
		was_armed = _control_mode.flag_armed;

		update_ref();

		/* Update velocity derivative,
		 * independent of the current flight mode
		 */
		if (_local_pos.timestamp > 0) {

			if (PX4_ISFINITE(_local_pos.x) &&
			    PX4_ISFINITE(_local_pos.y) &&
			    PX4_ISFINITE(_local_pos.z)) {

				_pos(0) = _local_pos.x;
				_pos(1) = _local_pos.y;

				if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {
					_pos(2) = -_local_pos.dist_bottom;

				} else {
					_pos(2) = _local_pos.z;
				}
			}
			_pos_err_d(0) = _pos_x_deriv.update(-_pos(0));
			_pos_err_d(1) = _pos_y_deriv.update(-_pos(1));
			_pos_err_d(2) = _pos_z_deriv.update(-_pos(2));

			if (PX4_ISFINITE(_local_pos.vx) &&
			    PX4_ISFINITE(_local_pos.vy) &&
			    PX4_ISFINITE(_local_pos.vz)) {

				_vel(0) = _local_pos.vx;
				_vel(1) = _local_pos.vy;

				if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {
					_vel(2) = -_local_pos.dist_bottom_rate;

				} else {
					_vel(2) = _local_pos.vz;
				}
			}

			_vel_err_d(0) = _vel_x_deriv.update(-_vel(0));
			_vel_err_d(1) = _vel_y_deriv.update(-_vel(1));
			_vel_err_d(2) = _vel_z_deriv.update(-_vel(2));
		}

		// reset the horizontal and vertical position hold flags for non-manual modes
		// or if position / altitude is not controlled
		if (!_control_mode.flag_control_position_enabled || !_control_mode.flag_control_manual_enabled) {
			_pos_hold_engaged = false;
		}

		if (!_control_mode.flag_control_altitude_enabled || !_control_mode.flag_control_manual_enabled) {
			_alt_hold_engaged = false;
		}

		 if (_control_mode.flag_control_altitude_enabled ||
		     _control_mode.flag_control_position_enabled ||
		     _control_mode.flag_control_climb_rate_enabled ||
		     _control_mode.flag_control_velocity_enabled ||
		     _control_mode.flag_control_acceleration_enabled) {

			_vel_ff.zero();

			/* by default, run position/altitude controller. the control_* functions
			 * can disable this and run velocity controllers directly in this cycle */
			_run_pos_control = true;
			_run_alt_control = true;
			OUT_FENCE = false;

			/* select control source */
			if (_control_mode.flag_control_manual_enabled) {
				/* manual control */
				control_manual(dt);

			} else if (_control_mode.flag_control_offboard_enabled) {
				/* offboard control */
				control_offboard(dt);
				_mode_auto = false;

			} else {
				/* AUTO */
				control_auto(dt);
				_target_updated = false;
			}

			/* weather-vane mode for vtol: disable yaw control */
			if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.disable_mc_yaw_control == true) {
				_att_sp.disable_mc_yaw_control = true;

			} else {
				/* reset in case of setpoint updates */
				_att_sp.disable_mc_yaw_control = false;
			}

			if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
			    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
				/* idle state, don't run controller and set zero thrust */
				R.identity();
				matrix::Quatf qd = R;
				memcpy(&_att_sp.q_d[0], qd.data(), sizeof(_att_sp.q_d));
				_att_sp.q_d_valid = true;

				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = _yaw;
				_att_sp.thrust = 0.0f;

				_att_sp.timestamp = hrt_absolute_time();

				/* publish attitude setpoint */
				if (_att_sp_pub != nullptr) {
					orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

				} else if (_attitude_setpoint_id) {
					_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				}

			} else if (_control_mode.flag_control_manual_enabled
				   && _vehicle_land_detected.landed) {
				/* don't run controller when landed */
				_reset_pos_sp = true;
				_reset_alt_sp = true;
				_do_reset_alt_pos_flag = true;
				_mode_auto = false;
				reset_int_z = true;
				reset_int_xy = true;

				R.identity();

				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = _yaw;
				_att_sp.thrust = 0.0f;

				_att_sp.timestamp = hrt_absolute_time();

				/* publish attitude setpoint */
				if (_att_sp_pub != nullptr) {
					orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

				} else if (_attitude_setpoint_id) {
					_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				}

			} else {
				/* position error */
				math::Vector<3> pos_err = _pos_sp - _pos;

				/* run position & altitude controllers, if enabled (otherwise use already computed velocity setpoints) */
				if (_run_pos_control) {
					// _vel_sp(0) = (_pos_sp(0) - _pos(0)) * _params.pos_p(0);
					// _vel_sp(1) = (_pos_sp(1) - _pos(1)) * _params.pos_p(1);
					_vel_sp(0) = pos_err(0) * _params.pos_p(0) + _pos_err_d(0) * _params.pos_d(0) +  pos_err_i(0);
					_vel_sp(1) = pos_err(1) * _params.pos_p(1) + _pos_err_d(1) * _params.pos_d(1) +  pos_err_i(1);
					// PX4_INFO("XY PID: %4.4f,%4.4f,%4.4f",(double)_params.pos_p(0),(double)_params.pos_i(0),(double)_params.pos_d(0));

					/*gyzhang <Jul 11, 2017>*/
					if (_mode_auto ==  true && OUT_FENCE == false && FOLLOW_MODE == true){
						vel_ff(0) = _target.vx;
						vel_ff(1) = _target.vy;
					} else {
						vel_ff(0) = 0;
						vel_ff(1) = 0;
					}
					_vel_sp(0) = _vel_sp(0) + vel_ff(0);
					_vel_sp(1) = _vel_sp(1) + vel_ff(1);

					if (statu != DISABLE){
						_vel_sp(0) += next._vel(0);
						_vel_sp(1) += next._vel(1);
					}
				}

				// guard against any bad velocity values

				bool velocity_valid = PX4_ISFINITE(_pos_sp_triplet.current.vx) &&
						      PX4_ISFINITE(_pos_sp_triplet.current.vy) &&
						      _pos_sp_triplet.current.velocity_valid;

				// do not go slower than the follow target velocity when position tracking is active (set to valid)

				if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
				    velocity_valid &&
				    _pos_sp_triplet.current.position_valid) {

					math::Vector<3> ft_vel(_pos_sp_triplet.current.vx, _pos_sp_triplet.current.vy, 0);

					float cos_ratio = (ft_vel * _vel_sp) / (ft_vel.length() * _vel_sp.length());

					// only override velocity set points when uav is traveling in same direction as target and vector component
					// is greater than calculated position set point velocity component

					if (cos_ratio > 0) {
						ft_vel *= (cos_ratio);
						// min speed a little faster than target vel
						ft_vel += ft_vel.normalized() * 1.5f;

					} else {
						ft_vel.zero();
					}

					_vel_sp(0) = fabs(ft_vel(0)) > fabs(_vel_sp(0)) ? ft_vel(0) : _vel_sp(0);
					_vel_sp(1) = fabs(ft_vel(1)) > fabs(_vel_sp(1)) ? ft_vel(1) : _vel_sp(1);

					// track target using velocity only

				} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
					   velocity_valid) {

					_vel_sp(0) = _pos_sp_triplet.current.vx;
					_vel_sp(1) = _pos_sp_triplet.current.vy;
				}

				if (_run_alt_control) {
					// _vel_sp(2) = (_pos_sp(2) - _pos(2)) * _params.pos_p(2);
					_vel_sp(2) = pos_err(2) * _params.pos_p(2) + _pos_err_d(2) * _params.pos_d(2) + pos_err_i(2);
					// PX4_INFO("Z PID: %4.4f,%4.4f,%4.4f",(double)_params.pos_p(2),(double)_params.pos_i(2),(double)_params.pos_d(2));

					/*gyzhang <Jul 11, 2017>*/
					if (_mode_auto ==  true && OUT_FENCE == false && FOLLOW_MODE == true){
						vel_ff(2) = _target.vz;
					} else {
						vel_ff(2) = 0;
					}
					_vel_sp(2) = _vel_sp(2) + vel_ff(2);

					if (statu != DISABLE){
						_vel_sp(2) += next._vel(2);
					}
				}

				/* make sure velocity setpoint is saturated in xy*/
				float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) +
							  _vel_sp(1) * _vel_sp(1));

				if (vel_norm_xy > _params.vel_max(0)) {
					/* note assumes vel_max(0) == vel_max(1) */
					_vel_sp(0) = _vel_sp(0) * _params.vel_max(0) / vel_norm_xy;
					_vel_sp(1) = _vel_sp(1) * _params.vel_max(1) / vel_norm_xy;
				} else if (_run_pos_control){
					pos_err_i(0) += pos_err(0) * _params.pos_i(0) * dt;
					pos_err_i(1) += pos_err(1) * _params.pos_i(1) * dt;
				} else {
					pos_err_i(1) = pos_err_i(0) = 0; //if no position control, clean init
				}

				/* make sure velocity setpoint is saturated in z*/
				if (_vel_sp(2) < -1.0f * _params.vel_max_up) {
					_vel_sp(2) = -1.0f * _params.vel_max_up;
				}

				if (_vel_sp(2) >  _params.vel_max_down) {
					_vel_sp(2) = _params.vel_max_down;
				} else if(_run_alt_control){
					pos_err_i(2) += pos_err(2) * _params.pos_i(2) * dt;
				} else {
					pos_err_i(2) = 0;
				}

				if (!_control_mode.flag_control_position_enabled) {
					_reset_pos_sp = true;
				}

				if (!_control_mode.flag_control_altitude_enabled) {
					_reset_alt_sp = true;
				}

				if (!_control_mode.flag_control_velocity_enabled) {
					_vel_sp_prev(0) = _vel(0);
					_vel_sp_prev(1) = _vel(1);
					_vel_sp(0) = 0.0f;
					_vel_sp(1) = 0.0f;
					_control_vel_enabled_prev = false;
				}

				if (!_control_mode.flag_control_climb_rate_enabled) {
					_vel_sp(2) = 0.0f;
				}

				/* use constant descend rate when landing, ignore altitude setpoint */
				if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
				    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
					_vel_sp(2) = _params.land_speed;
				}

				/* special thrust setpoint generation for takeoff from ground */
				if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
				    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
				    && _control_mode.flag_armed) {

					// check if we are not already in air.
					// if yes then we don't need a jumped takeoff anymore
					if (!_takeoff_jumped && !_vehicle_land_detected.landed && fabsf(_takeoff_thrust_sp) < FLT_EPSILON) {
						_takeoff_jumped = true;
					}

					if (!_takeoff_jumped) {
						// ramp thrust setpoint up
						if (_vel(2) > -(_params.tko_speed / 2.0f)) {
							_takeoff_thrust_sp += 0.5f * dt;
							_vel_sp.zero();
							_vel_prev.zero();

						} else {
							// copter has reached our takeoff speed. split the thrust setpoint up
							// into an integral part and into a P part
							thrust_int(2) = _takeoff_thrust_sp - _params.vel_p(2) * fabsf(_vel(2));
							thrust_int(2) = -math::constrain(thrust_int(2), _params.thr_min, _params.thr_max);
							_vel_sp_prev(2) = -_params.tko_speed;
							_takeoff_jumped = true;
							reset_int_z = false;
						}
					}

					if (_takeoff_jumped) {
						_vel_sp(2) = -_params.tko_speed;
					}

				} else {
					_takeoff_jumped = false;
					_takeoff_thrust_sp = 0.0f;
				}

				// limit total horizontal acceleration
				math::Vector<2> acc_hor;
				acc_hor(0) = (_vel_sp(0) - _vel_sp_prev(0)) / dt;
				acc_hor(1) = (_vel_sp(1) - _vel_sp_prev(1)) / dt;

				if ((acc_hor.length() > _params.acc_hor_max) & !_reset_pos_sp) {
					acc_hor.normalize();
					acc_hor *= _params.acc_hor_max;
					math::Vector<2> vel_sp_hor_prev(_vel_sp_prev(0), _vel_sp_prev(1));
					math::Vector<2> vel_sp_hor = acc_hor * dt + vel_sp_hor_prev;
					_vel_sp(0) = vel_sp_hor(0);
					_vel_sp(1) = vel_sp_hor(1);
				}

				// limit vertical acceleration

				float acc_v = (_vel_sp(2) - _vel_sp_prev(2)) / dt;

				if ((fabsf(acc_v) > 2 * _params.acc_hor_max) & !_reset_alt_sp) {
					acc_v /= fabsf(acc_v);
					_vel_sp(2) = acc_v * 2 * _params.acc_hor_max * dt + _vel_sp_prev(2);
				}

				_vel_sp_prev = _vel_sp;

				_global_vel_sp.vx = _vel_sp(0);
				_global_vel_sp.vy = _vel_sp(1);
				_global_vel_sp.vz = _vel_sp(2);

				/* publish velocity setpoint */
				if (_global_vel_sp_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);

				} else {
					_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &_global_vel_sp);
				}

				if (_control_mode.flag_control_climb_rate_enabled || _control_mode.flag_control_velocity_enabled ||
				    _control_mode.flag_control_acceleration_enabled) {
					/* reset integrals if needed */
					if (_control_mode.flag_control_climb_rate_enabled) {
						if (reset_int_z) {
							reset_int_z = false;
							float i = _params.thr_min;

							if (reset_int_z_manual) {
								i = _params.thr_hover;

								if (i < _params.thr_min) {
									i = _params.thr_min;

								} else if (i > _params.thr_max) {
									i = _params.thr_max;
								}
							}

							thrust_int(2) = -i;
						}

					} else {
						reset_int_z = true;
					}

					if (_control_mode.flag_control_velocity_enabled) {
						if (reset_int_xy) {
							reset_int_xy = false;
							thrust_int(0) = 0.0f;
							thrust_int(1) = 0.0f;
						}

					} else {
						reset_int_xy = true;
					}

					/* velocity error */
					math::Vector<3> vel_err = _vel_sp - _vel;

					// check if we have switched from a non-velocity controlled mode into a velocity controlled mode
					// if yes, then correct xy velocity setpoint such that the attitude setpoint is continuous
					if (!_control_vel_enabled_prev && _control_mode.flag_control_velocity_enabled) {

						matrix::Dcmf Rb = matrix::Quatf(_att_sp.q_d[0], _att_sp.q_d[1], _att_sp.q_d[2], _att_sp.q_d[3]);

						// choose velocity xyz setpoint such that the resulting thrust setpoint has the direction
						// given by the last attitude setpoint
						_vel_sp(0) = _vel(0) + (-Rb(0,
									    2) * _att_sp.thrust - thrust_int(0) - _vel_err_d(0) * _params.vel_d(0)) / _params.vel_p(0);
						_vel_sp(1) = _vel(1) + (-Rb(1,
									    2) * _att_sp.thrust - thrust_int(1) - _vel_err_d(1) * _params.vel_d(1)) / _params.vel_p(1);
						_vel_sp(2) = _vel(2) + (-Rb(2,
									    2) * _att_sp.thrust - thrust_int(2) - _vel_err_d(2) * _params.vel_d(2)) / _params.vel_p(2);
						_vel_sp_prev(0) = _vel_sp(0);
						_vel_sp_prev(1) = _vel_sp(1);
						_vel_sp_prev(2) = _vel_sp(2);
						_control_vel_enabled_prev = true;

						// compute updated velocity error
						vel_err = _vel_sp - _vel;
					}

					/* thrust vector in NED frame */
					math::Vector<3> thrust_sp;

					if (_control_mode.flag_control_acceleration_enabled && _pos_sp_triplet.current.acceleration_valid) {
						thrust_sp = math::Vector<3>(_pos_sp_triplet.current.a_x, _pos_sp_triplet.current.a_y, _pos_sp_triplet.current.a_z);

					} else {
						thrust_sp = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d) + thrust_int;
					}
					
		
					if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
					    && !_takeoff_jumped && !_control_mode.flag_control_manual_enabled) {
						// for jumped takeoffs use special thrust setpoint calculated above
						thrust_sp.zero();
						thrust_sp(2) = -_takeoff_thrust_sp;
					}

					if (!_control_mode.flag_control_velocity_enabled && !_control_mode.flag_control_acceleration_enabled) {
						thrust_sp(0) = 0.0f;
						thrust_sp(1) = 0.0f;
					}

					if (!_control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_acceleration_enabled) {
						thrust_sp(2) = 0.0f;
					}


					bool saturation_xy = false;
					bool saturation_z = false;

					/* limit min lift */
					float thr_min = _params.thr_min;

					if (!_control_mode.flag_control_velocity_enabled && thr_min < 0.0f) {
						/* don't allow downside thrust direction in manual attitude mode */
						thr_min = 0.0f;
					}

					float thrust_abs = thrust_sp.length();
					float tilt_max = _params.tilt_max_air;
					float thr_max = _params.thr_max;
					/* filter vel_z over 1/8sec */
					_vel_z_lp = _vel_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * _vel(2);
					/* filter vel_z change over 1/8sec */
					float vel_z_change = (_vel(2) - _vel_prev(2)) / dt;
					_acc_z_lp = _acc_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * vel_z_change;

					/* adjust limits for landing mode */
					if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
					    _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
						/* limit max tilt and min lift when landing */
						tilt_max = _params.tilt_max_land;

						if (thr_min < 0.0f) {
							thr_min = 0.0f;
						}

						/* descend stabilized, we're landing */
						if (!_in_landing && !_lnd_reached_ground
						    && (float)fabs(_acc_z_lp) < 0.1f
						    && _vel_z_lp > 0.5f * _params.land_speed) {
							_in_landing = true;
						}

						/* assume ground, cut thrust */
						if (_in_landing
						    && _vel_z_lp < 0.1f) {
							thr_max = 0.0f;
							_in_landing = false;
							_lnd_reached_ground = true;
						}

						/* once we assumed to have reached the ground always cut the thrust.
							Only free fall detection below can revoke this
						*/
						if (!_in_landing && _lnd_reached_ground) {
							thr_max = 0.0f;
						}

						/* if we suddenly fall, reset landing logic and remove thrust limit */
						if (_lnd_reached_ground
						    /* XXX: magic value, assuming free fall above 4m/s2 acceleration */
						    && (_acc_z_lp > 4.0f
							|| _vel_z_lp > 2.0f * _params.land_speed)) {
							thr_max = _params.thr_max;
							_in_landing = false;
							_lnd_reached_ground = false;
						}

					} else {
						_in_landing = false;
						_lnd_reached_ground = false;
					}

					/* limit min lift */
					if (-thrust_sp(2) < thr_min) {
						thrust_sp(2) = -thr_min;
						saturation_z = true;
					}

					if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {

						/* limit max tilt */
						if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
							/* absolute horizontal thrust */
							float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();

							if (thrust_sp_xy_len > 0.01f) {
								/* max horizontal thrust for given vertical thrust*/
								float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);

								if (thrust_sp_xy_len > thrust_xy_max) {
									float k = thrust_xy_max / thrust_sp_xy_len;
									thrust_sp(0) *= k;
									thrust_sp(1) *= k;
									saturation_xy = true;
								}
							}
						}
					}

					if (_control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_velocity_enabled) {
						/* thrust compensation when vertical velocity but not horizontal velocity is controlled */
						float att_comp;

						if (_R(2, 2) > TILT_COS_MAX) {
							att_comp = 1.0f / _R(2, 2);

						} else if (_R(2, 2) > 0.0f) {
							att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * _R(2, 2) + 1.0f;
							saturation_z = true;

						} else {
							att_comp = 1.0f;
							saturation_z = true;
						}

						thrust_sp(2) *= att_comp;
					}

					/* limit max thrust */
					thrust_abs = thrust_sp.length(); /* recalculate because it might have changed */

					if (thrust_abs > thr_max) {
						if (thrust_sp(2) < 0.0f) {
							if (-thrust_sp(2) > thr_max) {
								/* thrust Z component is too large, limit it */
								thrust_sp(0) = 0.0f;
								thrust_sp(1) = 0.0f;
								thrust_sp(2) = -thr_max;
								saturation_xy = true;
								saturation_z = true;

							} else {
								/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
								float thrust_xy_max = sqrtf(thr_max * thr_max - thrust_sp(2) * thrust_sp(2));
								float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
								float k = thrust_xy_max / thrust_xy_abs;
								thrust_sp(0) *= k;
								thrust_sp(1) *= k;
								saturation_xy = true;
							}

						} else {
							/* Z component is negative, going down, simply limit thrust vector */
							float k = thr_max / thrust_abs;
							thrust_sp *= k;
							saturation_xy = true;
							saturation_z = true;
						}

						thrust_abs = thr_max;
					}

					/* update integrals */
					if (_control_mode.flag_control_velocity_enabled && !saturation_xy) {
						thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
						thrust_int(1) += vel_err(1) * _params.vel_i(1) * dt;
					}

					if (_control_mode.flag_control_climb_rate_enabled && !saturation_z) {
						thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;

						/* protection against flipping on ground when landing */
						if (thrust_int(2) > 0.0f) {
							thrust_int(2) = 0.0f;
						}
					}
					_pre_thrust_sp = _pre_pid_thrust_sp;
					_pre_pid_thrust_sp = thrust_sp;

#ifdef ACC_FF
					static bool acc_ff_flag = false;
					static math::Vector<3> ff_value(0.0f,0.0f,0.0f);
					bool ff_saturation_xy = false;
					bool ff_saturation_z = false;
					if (_manual.aux1 > 0.6f) {
						
						hrt_abstime timeNow = hrt_absolute_time();
						static hrt_abstime pretimeStamp = timeNow;
//						PX4_INFO("pos: %d, %d",_angacc_acc_updated,_angacc_acc.valid_acc);
						if (_angacc_acc_updated && _angacc_acc.valid_acc) {
							
							// first time in acc feed back mode
							if (acc_ff_flag == false ) {
								ff_value.zero();
								pretimeStamp = timeNow;
								// PX4_INFO("acc_ff_flag = false!");
								acc_ff_flag = true;
								mavlink_log_critical(&_mavlink_log_pub,"Acc FF Started!");
							}

							float dt_acc_ff = (timeNow - pretimeStamp) * 1e-6f;

//							PX4_INFO("dt_acc_ff: %8.4f",(double)dt_acc_ff);

							pretimeStamp = timeNow;
							math::Vector<3> acc(_angacc_acc.acc_x, _angacc_acc.acc_y, _angacc_acc.acc_z - 9.806f);

							float hovering_thrust = -0.0263f * _battery_status.voltage_filtered_v + 1.315f;

							math::Vector<3> diff =  (_pre_thrust_sp - acc*(hovering_thrust / 9.806f)) * (_acc_ff_a.get() * dt_acc_ff);
							
							math::Vector<3> ff_delta = diff;
							// ff_delta(0) = _lp_accff_x.apply(diff(0));
							// ff_delta(1) = _lp_accff_y.apply(diff(1));
							
							// ff_delta(2) = _lp_accff_z.apply(diff(2) / _acc_ff_a.get() * 1.5f);
							
							// PX4_INFO("time: %8.4f",hrt_absolute_time()*1e-6);

//							mavlink_log_critical(&_mavlink_log_pub, "ff_delta:%8.4f, %8.4f, %8.4f",(double)ff_delta(0),
//									(double)ff_delta(1),(double)ff_delta(2));
//
//							mavlink_log_critical(&_mavlink_log_pub, "_pre_thrust_sp:%8.4f, %8.4f, %8.4f",(double)_pre_thrust_sp(0),
//									(double)_pre_thrust_sp(1),(double)_pre_thrust_sp(2));

							math::Vector<3> ff_value_temp = ff_value + ff_delta;
							float ff_value_temp_norm = sqrtf(ff_value_temp(0) * ff_value_temp(0) + ff_value_temp(1) * ff_value_temp(1));
							if ( ff_value_temp_norm > _ff_max_horizon.get()) {
								ff_value_temp(0) = ff_value_temp(0) / ff_value_temp_norm * _ff_max_horizon.get();
								ff_value_temp(1) = ff_value_temp(1) / ff_value_temp_norm * _ff_max_horizon.get();
								ff_saturation_xy = true;
							}
							if (fabsf(ff_value_temp(2)) > _ff_max_vertical.get()) {
								ff_value_temp(2) = ff_value_temp(2)/fabsf(ff_value_temp(2)) * _ff_max_vertical.get();
								ff_saturation_z = true;
							}
							ff_value = ff_value_temp;
//							PX4_INFO("ff_acc: %8.4f,%8.4f,%8.4f", (double)ff_value(0), (double)ff_value(1),(double)ff_value(2));

							math::Vector<3> thrust_sp_temp = thrust_sp + ff_value;

							/* limit min lift */
							if (-thrust_sp_temp(2) < thr_min) {
								thrust_sp_temp(2) = -thr_min;
								saturation_z = true;
							}

							if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {

								/* limit max tilt */
								if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
									/* absolute horizontal thrust */
									float thrust_temp_xy_len = math::Vector<2>(thrust_sp_temp(0), thrust_sp_temp(1)).length();

									if (thrust_temp_xy_len > 0.01f) {
										/* max horizontal thrust for given vertical thrust*/
										float thrust_xy_max = -thrust_sp_temp(2) * tanf(tilt_max);

										if (thrust_temp_xy_len > thrust_xy_max) {
											float k = thrust_xy_max / thrust_temp_xy_len;
											thrust_sp_temp(0) *= k;
											thrust_sp_temp(1) *= k;
											saturation_xy = true;
										}
									}
								}
							}

//							if (_control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_velocity_enabled) {
//								/* thrust compensation when vertical velocity but not horizontal velocity is controlled */
//								float att_comp;
//
//								if (_R(2, 2) > TILT_COS_MAX) {
//									att_comp = 1.0f / _R(2, 2);
//
//								} else if (_R(2, 2) > 0.0f) {
//									att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * _R(2, 2) + 1.0f;
//									tatal_saturation_z = true;
//
//								} else {
//									att_comp = 1.0f;
//									tatal_saturation_z = true;
//								}
//
//								thrust_sp_temp(2) *= att_comp;
//							}

							/* limit max thrust */
							float thrust_sp_temp_abs = thrust_sp_temp.length(); /* recalculate because it might have changed */

							if (thrust_sp_temp_abs > thr_max) {
								if (thrust_sp_temp(2) < 0.0f) {
									if (-thrust_sp_temp(2) > thr_max) {
										/* thrust Z component is too large, limit it */
										thrust_sp_temp(0) = 0.0f;
										thrust_sp_temp(1) = 0.0f;
										thrust_sp_temp(2) = -thr_max;
										saturation_xy = true;
										saturation_z = true;

									} else {
										/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
										float thrust_xy_max = sqrtf(thr_max * thr_max - thrust_sp_temp(2) * thrust_sp_temp(2));
										float thrust_xy_abs = math::Vector<2>(thrust_sp_temp(0), thrust_sp_temp(1)).length();
										float k = thrust_xy_max / thrust_xy_abs;
										thrust_sp_temp(0) *= k;
										thrust_sp_temp(1) *= k;
										saturation_xy = true;
									}

								} else {
									/* Z component is negative, going down, simply limit thrust vector */
									float k = thr_max / thrust_sp_temp_abs;
									thrust_sp_temp *= k;
									saturation_xy = true;
									saturation_z = true;
								}

								thrust_sp_temp_abs = thr_max;
								ff_value = thrust_sp_temp - thrust_sp;
							}
							thrust_abs = thrust_sp_temp_abs;
//							mavlink_log_critical(&_mavlink_log_pub, "ff_value:%8.4f, %8.4f, %8.4f",(double)ff_value(0),
//									(double)ff_value(1),(double)ff_value(2));
						}
					} else {
						acc_ff_flag = false;
						ff_value.zero();
					}
//					ff_value.zero();
					thrust_sp += ff_value;

					_acc_ff.acc_ff[0] = ff_value(0);
					_acc_ff.acc_ff[1] = ff_value(1);
					_acc_ff.acc_ff[2] = ff_value(2);

//					PX4_INFO("ff_acc: %8.4f,%8.4f,%8.4f", (double)ff_value(0), (double)ff_value(1),(double)ff_value(2));
					_acc_ff.saturation = ff_saturation_xy | (ff_saturation_z << 1);

					if (_acc_ff_pub == nullptr) {
						_acc_ff_pub = orb_advertise(ORB_ID(acc_ff), &_acc_ff);
					} else {
						orb_publish(ORB_ID(acc_ff), _acc_ff_pub, &_acc_ff);
					}

#endif
					coup_force.force_x=thrust_sp(0);
					coup_force.force_y=thrust_sp(1);
					coup_force.force_z=thrust_sp(2);

					if (coup_force_pub == nullptr) {
						coup_force_pub = orb_advertise(ORB_ID(coupling_force), &coup_force);
					} else {
						orb_publish(ORB_ID(coupling_force), coup_force_pub, &coup_force);
					}

					/* calculate attitude setpoint from thrust vector */
					if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {
						/* desired body_z axis = -normalize(thrust_vector) */
						math::Vector<3> body_x;
						math::Vector<3> body_y;
						math::Vector<3> body_z;

						if (thrust_abs > SIGMA) {
							body_z = -thrust_sp / thrust_abs;

						} else {
							/* no thrust, set Z axis to safe value */
							body_z.zero();
							body_z(2) = 1.0f;
						}

						/* vector of desired yaw direction in XY plane, rotated by PI/2 */
						math::Vector<3> y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);

						if (fabsf(body_z(2)) > SIGMA) {
							/* desired body_x axis, orthogonal to body_z */
							body_x = y_C % body_z;

							/* keep nose to front while inverted upside down */
							if (body_z(2) < 0.0f) {
								body_x = -body_x;
							}

							body_x.normalize();

						} else {
							/* desired thrust is in XY plane, set X downside to construct correct matrix,
							 * but yaw component will not be used actually */
							body_x.zero();
							body_x(2) = 1.0f;
						}

						/* desired body_y axis */
						body_y = body_z % body_x;

						/* fill rotation matrix */
						for (int i = 0; i < 3; i++) {
							R(i, 0) = body_x(i);
							R(i, 1) = body_y(i);
							R(i, 2) = body_z(i);
						}

						/* copy quaternion setpoint to attitude setpoint topic */
						matrix::Quatf q_sp = R;
						memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
						_att_sp.q_d_valid = true;

						/* calculate euler angles, for logging only, must not be used for control */
						matrix::Eulerf euler = R;
						_att_sp.roll_body = euler(0);
						_att_sp.pitch_body = euler(1);
						/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */

					} else if (!_control_mode.flag_control_manual_enabled) {
						/* autonomous altitude control without position control (failsafe landing),
						 * force level attitude, don't change yaw */
						R = matrix::Eulerf(0.0f, 0.0f, _att_sp.yaw_body);

						/* copy quaternion setpoint to attitude setpoint topic */
						matrix::Quatf q_sp = R;
						memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
						_att_sp.q_d_valid = true;

						_att_sp.roll_body = 0.0f;
						_att_sp.pitch_body = 0.0f;
					}

					_att_sp.thrust = thrust_abs;

					/* save thrust setpoint for logging */
					_local_pos_sp.acc_x = thrust_sp(0) * ONE_G;
					_local_pos_sp.acc_y = thrust_sp(1) * ONE_G;
					_local_pos_sp.acc_z = thrust_sp(2) * ONE_G;

					_att_sp.timestamp = hrt_absolute_time();


				} else {
					reset_int_z = true;
				}
			}

			/* fill local position, velocity and thrust setpoint */
			_local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp.x = _pos_sp(0);
			_local_pos_sp.y = _pos_sp(1);
			_local_pos_sp.z = _pos_sp(2);
			_local_pos_sp.yaw = _att_sp.yaw_body;
			_local_pos_sp.vx = _vel_sp(0);
			_local_pos_sp.vy = _vel_sp(1);
			_local_pos_sp.vz = _vel_sp(2);

			/* publish local position setpoint */
			if (_local_pos_sp_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);

			} else {
				_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
			}

		} else {
			/* position controller disabled, reset setpoints */
			_reset_alt_sp = true;
			_reset_pos_sp = true;
			_do_reset_alt_pos_flag = true;
			_mode_auto = false;
			reset_int_z = true;
			reset_int_xy = true;
			_control_vel_enabled_prev = false;

			/* store last velocity in case a mode switch to position control occurs */
			_vel_sp_prev = _vel;
		}

		/* generate attitude setpoint from manual controls */
		if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_attitude_enabled) {

			/* reset yaw setpoint to current position if needed */
			if (reset_yaw_sp) {
				reset_yaw_sp = false;
				_att_sp.yaw_body = _yaw;
			}

			/* do not move yaw while sitting on the ground */
			else if (!_vehicle_land_detected.landed &&
				 !(!_control_mode.flag_control_altitude_enabled && _manual.z < 0.1f)) {

				/* we want to know the real constraint, and global overrides manual */
				const float yaw_rate_max = (_params.man_yaw_max < _params.global_yaw_max) ? _params.man_yaw_max :
							   _params.global_yaw_max;
				const float yaw_offset_max = yaw_rate_max / _params.mc_att_yaw_p;

				_att_sp.yaw_sp_move_rate = _manual.r * yaw_rate_max;
				float yaw_target = _wrap_pi(_att_sp.yaw_body + _att_sp.yaw_sp_move_rate * dt);
				float yaw_offs = _wrap_pi(yaw_target - _yaw);

				// If the yaw offset became too big for the system to track stop
				// shifting it, only allow if it would make the offset smaller again.
				if (fabsf(yaw_offs) < yaw_offset_max ||
				    (_att_sp.yaw_sp_move_rate > 0 && yaw_offs < 0) ||
				    (_att_sp.yaw_sp_move_rate < 0 && yaw_offs > 0)) {
					_att_sp.yaw_body = yaw_target;
				}
			}

			/* control throttle directly if no climb rate controller is active */
			if (!_control_mode.flag_control_climb_rate_enabled) {
				float thr_val = throttle_curve(_manual.z, _params.thr_hover);
				_att_sp.thrust = math::min(thr_val, _manual_thr_max.get());

				/* enforce minimum throttle if not landed */
				if (!_vehicle_land_detected.landed) {
					_att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get());
				}
			}

			/* control roll and pitch directly if no aiding velocity controller is active */
			if (!_control_mode.flag_control_velocity_enabled) {
				_att_sp.roll_body = _manual.y * _params.man_roll_max;
				_att_sp.pitch_body = -_manual.x * _params.man_pitch_max;

				/* only if optimal recovery is not used, modify roll/pitch */
				if (_params.opt_recover <= 0) {
					// construct attitude setpoint rotation matrix. modify the setpoints for roll
					// and pitch such that they reflect the user's intention even if a yaw error
					// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
					// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
					// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
					// heading of the vehicle.

					// calculate our current yaw error
					float yaw_error = _wrap_pi(_att_sp.yaw_body - _yaw);

					// compute the vector obtained by rotating a z unit vector by the rotation
					// given by the roll and pitch commands of the user
					math::Vector<3> zB = {0, 0, 1};
					math::Matrix<3, 3> R_sp_roll_pitch;
					R_sp_roll_pitch.from_euler(_att_sp.roll_body, _att_sp.pitch_body, 0);
					math::Vector<3> z_roll_pitch_sp = R_sp_roll_pitch * zB;


					// transform the vector into a new frame which is rotated around the z axis
					// by the current yaw error. this vector defines the desired tilt when we look
					// into the direction of the desired heading
					math::Matrix<3, 3> R_yaw_correction;
					R_yaw_correction.from_euler(0.0f, 0.0f, -yaw_error);
					z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

					// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
					// to calculate the new desired roll and pitch angles
					// R_tilt can be written as a function of the new desired roll and pitch
					// angles. we get three equations and have to solve for 2 unknowns
					_att_sp.pitch_body = asinf(z_roll_pitch_sp(0));
					_att_sp.roll_body = -atan2f(z_roll_pitch_sp(1), z_roll_pitch_sp(2));
				}

				/* copy quaternion setpoint to attitude setpoint topic */
				matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
				memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
				_att_sp.q_d_valid = true;
			}

			if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON &&
			    !_vehicle_land_detected.landed) {
				_att_sp.landing_gear = 1.0f;

			} else if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
				_att_sp.landing_gear = -1.0f;
			}


			_att_sp.timestamp = hrt_absolute_time();

		} else {
			reset_yaw_sp = true;
			_att_sp.yaw_sp_move_rate = 0.0f;
		}

		/* update previous velocity for velocity controller D part */
		_vel_prev = _vel;

		/* publish attitude setpoint
		 * Do not publish if offboard is enabled but position/velocity/accel control is disabled,
		 * in this case the attitude setpoint is published by the mavlink app. Also do not publish
		 * if the vehicle is a VTOL and it's just doing a transition (the VTOL attitude control module will generate
		 * attitude setpoints for the transition).
		 */
		if (!(_control_mode.flag_control_offboard_enabled &&
		      !(_control_mode.flag_control_position_enabled ||
			_control_mode.flag_control_velocity_enabled ||
			_control_mode.flag_control_acceleration_enabled))) {

			if (_att_sp_pub != nullptr) {
				orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

			} else if (_attitude_setpoint_id) {
				_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
			}
		}

		/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
		reset_int_z_manual = _control_mode.flag_armed && _control_mode.flag_control_manual_enabled
				     && !_control_mode.flag_control_climb_rate_enabled;
	}

	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");

	_control_task = -1;
}

int
MulticopterPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_pos_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5000,
					   (px4_main_t)&MulticopterPositionControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_pos_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (pos_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		pos_control::g_control = new MulticopterPositionControl;

		if (pos_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != pos_control::g_control->start()) {
			delete pos_control::g_control;
			pos_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (pos_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete pos_control::g_control;
		pos_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (pos_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
