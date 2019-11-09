#ifndef _JS_CONTROL_H_
#define _JS_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <little_ant_msgs/ControlCmd1.h>
#include <little_ant_msgs/ControlCmd2.h>
#include <sensor_msgs/Joy.h>
#include <iostream>

#define MAX_SPEED 25
#define MAX_AXES_VAL 1 
#define MAX_ST_ANGLE 15
class jsControl
{
	public:
		jsControl();
		~jsControl();
		void init(int , char**);
		
	private:
		//void run();
		void callBack(const sensor_msgs::Joy::ConstPtr& joy_msg);
		void t_callBack(const ros::TimerEvent& event);
	private:
		bool driverless_mode_flag;
		bool hand_brake_flag;
		uint8_t gear_flag;
		float vehicle_speed;
		float steering_angle;
		
		little_ant_msgs::ControlCmd1 cmd1_msg;
		little_ant_msgs::ControlCmd2 cmd2_msg;
		
		ros::Subscriber joy_msg_sub;
		ros::Publisher cmd1_pub;
		ros::Publisher cmd2_pub;
		ros::Timer timer;
		
};

enum
{
	button_handBrake = 0,
	button_setDriverless = 1,
	button_setGear = 2 ,
	axes_setSpeed = 1,
	axes_steeringAngle = 3,
};


#endif
