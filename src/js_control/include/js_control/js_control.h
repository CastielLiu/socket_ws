#ifndef _JS_CONTROL_H_
#define _JS_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <little_ant_msgs/ControlCmd.h>
#include <little_ant_msgs/ControlCmd1.h>
#include <little_ant_msgs/ControlCmd2.h>
#include <little_ant_msgs/State2.h>
#include <sensor_msgs/Joy.h>
#include <js_control/Info.h>
#include <iostream>

#define SpeedIncrement 5  //MAX DRIVE SPEED (kmph)
#define MAX_SPEED_R 5   //MAX REVERSE SPEED (kmph)
#define MAX_AXES_VAL 1 
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
		void speed_callback(const little_ant_msgs::State2::ConstPtr& msg);
	private:
		//control cmd
		bool driverless_mode_flag;
		bool hand_brake_flag;
		uint8_t gear_flag;
		float set_speed;
		float set_brake;
		float road_wheel_angle;
		float max_roadWheel_angle;
		std_msgs::Bool is_manual_flag;
		int soft_gear;
		float offsetVal;
		//parameters
		float offsetMax_;
		
		js_control::Info info_;
		
		little_ant_msgs::ControlCmd1 cmd1_msg;
		little_ant_msgs::ControlCmd2 cmd2_msg;
		little_ant_msgs::ControlCmd cmd_msg;
		
		ros::Subscriber joy_msg_sub;
		ros::Subscriber speed_sub;
		ros::Publisher jsManualCmd_pub;
		ros::Publisher offsetMsg_pub;
		ros::Publisher cmd_pub;
		ros::Publisher brakingCmd_pub;
		ros::Publisher info_pub;
		ros::Timer timer;
		
		ros::NodeHandle nh;
		ros::NodeHandle nh_private;
};

enum
{
	//button
	button_handBrake = 0,
	button_setDriverless = 1,
	button_setGear = 2 ,
	button_speedRangeDec = 4,
	button_speedRangeAdd = 5,
	button_isManual = 8,
	//axes
	axes_setSpeed = 1,
	axes_leftOffset = 2,
	axes_steeringAngle = 3,
	axes_rightOffset = 5,
};
enum
{
	init_gear = 0,
	drive_gear = 1,
	neutral_gear = 0xA,
	reverse_gear = 0x9,
};


#endif
