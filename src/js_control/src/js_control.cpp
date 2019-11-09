#include <js_control/js_control.h>
using namespace std;

jsControl::jsControl()
{
	driverless_mode_flag = 0;
	hand_brake_flag = 1;//default : applied
	gear_flag = 0;
	vehicle_speed = 0;
	steering_angle = 0;
}

jsControl::~jsControl(){}


void jsControl::init(int argc, char** argv)
{
	ros::init(argc, argv, "js_control_node");
	ros::NodeHandle nh;
	//define publisher and subscriber
	cmd1_pub = nh.advertise<little_ant_msgs::ControlCmd1>("/controlCmd1",10);
	cmd2_pub = nh.advertise<little_ant_msgs::ControlCmd2>("/controlCmd2",10);
	joy_msg_sub = nh.subscribe("/joy_out", 10, &jsControl::callBack, this);
	timer = nh.createTimer(ros::Duration(0.1), &jsControl::t_callBack, this);
	
	cmd1_msg.set_driverlessMode = 0;
	cmd1_msg.set_handBrake = 1;
	
	cmd2_msg.set_gear = 0;
	cmd2_msg.set_speed = 0;
	cmd2_msg.set_roadWheelAngle = 0;
}

void jsControl::callBack(const sensor_msgs::Joy::ConstPtr& joy_msg)
{	
	if (joy_msg->buttons[button_setDriverless] == 1)
		driverless_mode_flag = !driverless_mode_flag;
		//hand_brake_flag = !driverless_mode_flag;
		ROS_INFO("driving mode is changed to %d .....", driverless_mode_flag);
	
	if(driverless_mode_flag == 1)
	{
		if (joy_msg->buttons[button_handBrake] == 1)
			hand_brake_flag = !hand_brake_flag;

		if (joy_msg->buttons[button_setGear] == 1 )
		{
			if(gear_flag ==0)
				gear_flag = 1;
			else if(gear_flag ==1)
				gear_flag = 0xA;
			else if(gear_flag == 0xA)
				gear_flag = 0x9;
			else if(gear_flag == 0x9)
				gear_flag = 0;  
		}
		
		vehicle_speed = MAX_SPEED*(joy_msg->axes[axes_setSpeed])/MAX_AXES_VAL;
		if(vehicle_speed < 0) vehicle_speed = 0;
		steering_angle = MAX_ST_ANGLE*(joy_msg->axes[axes_steeringAngle])/MAX_AXES_VAL;
		//ROS_INFO("speed key_value : %f", joy_msg->axes[axes_setSpeed]);
		//ROS_INFO("steering key_value : %f",joy_msg->axes[axes_steeringAngle]);
	}
	
	cmd1_msg.set_driverlessMode = driverless_mode_flag;
	cmd1_msg.set_handBrake = hand_brake_flag;
	
	cmd2_msg.set_gear = gear_flag;
	cmd2_msg.set_speed = vehicle_speed;
	cmd2_msg.set_roadWheelAngle = steering_angle;
	//publish conmmond message
	
	cmd1_pub.publish(cmd1_msg);
	cmd2_pub.publish(cmd2_msg);
}

void jsControl::t_callBack(const ros::TimerEvent& event)
{
	cmd1_pub.publish(cmd1_msg);
	cmd2_pub.publish(cmd2_msg);
	//ROS_INFO("t_callBack function publishing now....");
}

int main(int argc, char** argv)
{	
	jsControl js_control_;
	js_control_.init(argc, argv);
	ros::spin();
	return 0;
}
