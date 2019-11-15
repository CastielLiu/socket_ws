#include <js_control/js_control.h>
using namespace std;

jsControl::jsControl():
	nh_private("~")
{
	driverless_mode_flag = 1;
	hand_brake_flag = 0;
	gear_flag = 1;
	set_speed = 0;
	set_brake = 0;
	road_wheel_angle = 0;
	is_manual_flag.data = 0;
	soft_gear = 1;
	offsetVal = 0;
	max_roadWheel_angle = 15.0;
}

jsControl::~jsControl(){}


void jsControl::init(int argc, char** argv)
{
	nh_private.param<float>("offsetMax",offsetMax_,3.5);
	//define publisher and subscriber
	cmd1_pub = nh.advertise<little_ant_msgs::ControlCmd1>("/controlCmd1",1);
	cmd2_pub = nh.advertise<little_ant_msgs::ControlCmd2>("/controlCmd2",1);
	offsetMsg_pub = nh.advertise<std_msgs::Float32>("/start_avoiding",1);
	jsManualCmd_pub = nh.advertise<std_msgs::Bool>("/isManual",1);
	brakingCmd_pub = nh.advertise<std_msgs::Float32>("/jsBrakingCmd",1);
	info_pub = nh.advertise<js_control::Info>("/joy_info",1);
	
	speed_sub = nh.subscribe("/vehicleState2",1,&jsControl::speed_callback,this);
	joy_msg_sub = nh.subscribe("/joy_out", 100, &jsControl::callBack, this);
	timer = nh.createTimer(ros::Duration(0.1), &jsControl::t_callBack, this);
	
}

float generateRoadwheelAngleByRadius(const float& radius)
{
	return atan(1.5/radius)*180/M_PI; //Laxis = 1.5m
}

float maxRoadwheelAngleBySpeed(const float& speed)
{
	float min_radius = speed*speed/1.5; //1.5m/s2
	if(min_radius <3.0)
		min_radius = 3.0;
	
	float maxAngle = fabs(generateRoadwheelAngleByRadius(min_radius));
	if(maxAngle > 25.0)
		maxAngle = 25.0;
	return maxAngle;
}

void jsControl::speed_callback(const little_ant_msgs::State2::ConstPtr& msg)
{
	max_roadWheel_angle = maxRoadwheelAngleBySpeed(msg->vehicle_speed);
}

void jsControl::callBack(const sensor_msgs::Joy::ConstPtr& joy_msg)
{	
	static bool is_first = true;
	if(is_first)
	{
		is_first = false;
		return;
	}
//switch between maunal and auto mode ,topic name: "/isManual"
	if (joy_msg->buttons[button_isManual] == 1)
		is_manual_flag.data = !is_manual_flag.data;
	
	if (joy_msg->buttons[button_setDriverless] == 1)
		driverless_mode_flag = !driverless_mode_flag;
		//hand_brake_flag = !driverless_mode_flag;
		
	
	if(joy_msg->buttons[button_speedRangeAdd] == 1)
	{
		soft_gear++;
		if (soft_gear>5)
		soft_gear = 5;
	}
	
	if(joy_msg->buttons[button_speedRangeDec] == 1)
	{
		soft_gear--;
		if (soft_gear<1)
		soft_gear = 1;
	}
	

	if(driverless_mode_flag == 1)
	{
		if (joy_msg->buttons[button_handBrake] == 1)
			hand_brake_flag = !hand_brake_flag;

	
		if (joy_msg->buttons[button_setGear] == 1)
		{
			switch(gear_flag)
			{
				case init_gear:
					gear_flag = drive_gear;
					break;
				case drive_gear:
					gear_flag = neutral_gear;
					break;
				case neutral_gear:
					gear_flag = reverse_gear;
					break;
				case reverse_gear:
					gear_flag = init_gear;
					break;
				default:
					break;
			}
		}
		road_wheel_angle = max_roadWheel_angle*(joy_msg->axes[axes_steeringAngle])/MAX_AXES_VAL;
		//calculate offset value
		//ROS_INFO("speed key_value : %f", joy_msg->axes[axes_setSpeed]);
		//ROS_INFO("steering key_value : %f",joy_msg->axes[axes_steeringAngle]);
	}
	
	if(gear_flag == 1)
		set_speed = (soft_gear-1)*SpeedIncrement + (joy_msg->axes[axes_setSpeed])/MAX_AXES_VAL * SpeedIncrement;
		
	if(gear_flag ==0x9)
		set_speed = MAX_SPEED_R*(joy_msg->axes[axes_setSpeed])/MAX_AXES_VAL;
	
	if(set_speed < 0) set_speed = 0;
	
	set_brake = -100*(joy_msg->axes[axes_setSpeed])/MAX_AXES_VAL;
	if(set_brake < 0) set_brake = 0;
	
	if(joy_msg->axes[axes_leftOffset] != 1)
		offsetVal = (joy_msg->axes[axes_leftOffset] - 1)*offsetMax_/2;
		
	else if(joy_msg->axes[axes_rightOffset] != 1)
			offsetVal = -(joy_msg->axes[axes_rightOffset] - 1)*offsetMax_/2;
	else
		offsetVal = 0.0;
	
	cmd1_msg.set_driverlessMode = driverless_mode_flag;
	cmd1_msg.set_handBrake = hand_brake_flag;
	
	cmd2_msg.set_gear = gear_flag;
	cmd2_msg.set_speed = set_speed;
	cmd2_msg.set_brake = set_brake;
	cmd2_msg.set_roadWheelAngle = road_wheel_angle;
	
	//pub offset
	std_msgs::Float32 offsetMsg;
	offsetMsg.data = offsetVal;
	offsetMsg_pub.publish(offsetMsg);
	
	jsManualCmd_pub.publish(is_manual_flag);

	if(!is_manual_flag.data)
	{
		std_msgs::Float32 brakingCmdMsg;
		brakingCmdMsg.data = set_brake;
		brakingCmd_pub.publish(brakingCmdMsg);
	}
	

	//debug message
//	ROS_INFO("speed key_value : %f", joy_msg->axes[axes_setSpeed]);
//	ROS_INFO("steering key_value : %f",joy_msg->axes[axes_steeringAngle]);
	ROS_INFO("left offset key_value : %f",joy_msg->axes[axes_leftOffset]);
	ROS_INFO("left right offset key_value : %f",joy_msg->axes[axes_rightOffset]);
//	ROS_INFO("Conguradulation, init success ...");
}

//for manual control
void jsControl::t_callBack(const ros::TimerEvent& event)
{
	info_.soft_gear = soft_gear;
	info_.offset = offsetVal;
	info_.is_manual = is_manual_flag.data;
	
	info_pub.publish(info_);
	
	if(!is_manual_flag.data)
		return ;
	
	// send base control cmd only when switched to maunal driving  mode
	cmd1_pub.publish(cmd1_msg);
	cmd2_pub.publish(cmd2_msg);
		
	
	//debug message
//	ROS_INFO("==========================");
//	ROS_INFO("Speed Range: %d -- %d km/h", (soft_gear-1)*SpeedIncrement , soft_gear*SpeedIncrement);
//	ROS_INFO("Speed now : %f", set_speed);
//	ROS_INFO("gear_flag : %d", gear_flag);
//	ROS_INFO("offset value : %f",offsetVal);
//	ROS_INFO("driving mode is changed to %d .....", driverless_mode_flag);
//	ROS_INFO("t_callBack function publishing now....");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "js_control_node");
	jsControl js_control_;
	js_control_.init(argc, argv);
	ros::spin();
	return 0;
}
