#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using std::string;

#ifndef PACK
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

PACK(
typedef struct 
{
	uint8_t act_gear :4;
	uint8_t driverless_mode :1;
	uint8_t hand_brake :1;
	uint8_t emergency_brake :1;
	uint8_t car_state :1;
	uint16_t speed;
	uint16_t roadwheelAngle;
	
	uint8_t is_manual :1;
	uint8_t speed_grade :3;
    uint8_t steer_grade :3;
}) StateFeedback_t;


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


uint8_t generateCheckValue(const uint8_t* buf,int len)
{
	uint8_t result = 0;
	for(int i=0; i<len; ++i)
		result += buf[i];
	return result;
}

class RemoteControlFixedStation
{
public:
	RemoteControlFixedStation(int argc,char** argv);
	~RemoteControlFixedStation();
	void closeSocket();
	bool init();

private:
	bool initRosParams();
	bool initSocket();
	void recvThread();
	void showThread();
	void sendJoyCmd();
	void timerCallback(const ros::TimerEvent& event);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
	void showImage(const std::vector<uint8_t>& image_data);
private:
	string image_topic_;
	ros::Subscriber sub_image_;
	ros::Subscriber sub_joy_;
	ros::Timer timer_;
	
	std::string connect_code_;
	struct sockaddr_in sockaddr_;
	string socket_ip_;
	int socket_port_;
	int udp_fd_;
	int tcp_fd_;
	bool is_tcp_;
	
	std::mutex reload_seq_mutex_;
	int reload_seq_;
	
	float offset_;
	sensor_msgs::Joy joy_msg_;
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	
	std::vector<uint8_t> image_data_;
	StateFeedback_t state_feedback_;
	std::mutex state_mutex_;
	std::mutex image_mutex_;
};

RemoteControlFixedStation::RemoteControlFixedStation(int argc,char** argv):
	connect_code_("fixed"),
	offset_(0)
{	
	std::cout << " connect_code_: " <<connect_code_ << std::endl;
	nh_private_ = ros::NodeHandle("~");
	udp_fd_ = -1;
	tcp_fd_ = -1;
	is_tcp_ = false;
}

RemoteControlFixedStation::~RemoteControlFixedStation()
{
	
}

void RemoteControlFixedStation::closeSocket()
{
	if(udp_fd_ != -1)
		close(udp_fd_);
	if(tcp_fd_ != -1)
		close(tcp_fd_);
}

bool RemoteControlFixedStation::initSocket()
{
	bzero(&sockaddr_,sizeof(sockaddr_));//init 0

	sockaddr_.sin_port = htons(socket_port_);
	sockaddr_.sin_family = AF_INET;
	int convert_ret = inet_pton(AF_INET, socket_ip_.c_str(), &sockaddr_.sin_addr);
	if(convert_ret !=1)
	{
		ROS_ERROR("convert socket ip failed, please check the format!");
		return false;
	}
	else
		ROS_INFO("convert socket ip complete .");
	
	//UDP
	udp_fd_ = socket(PF_INET,SOCK_DGRAM , 0);
	if(udp_fd_ < 0)
	{
		ROS_ERROR("build socket error");
		return false;
	}
	else
		ROS_INFO("build socket ok .");
	
	// 设置超时
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 200000;
	if (setsockopt(udp_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
	{
		ROS_ERROR("setsockopt failed !!!");
		return false;
	}
	
	int cnt = 0;
	while(ros::ok())
	{
		int send_ret   = sendto(udp_fd_, connect_code_.c_str(), connect_code_.length(),0, 
						 (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
		ROS_INFO("send: %s -> try to connect to server",connect_code_.c_str());
		if(send_ret <= 0)
		{
			printf("%d: try to connect the server failed! try again...\n",++cnt);
			continue;
		}
		
		char ans[21];
		socklen_t clientLen = sizeof(sockaddr_);
		int len = recvfrom(udp_fd_, ans, 20,0,(struct sockaddr*)&sockaddr_, &clientLen);
		if(len > 0 && string(ans) == connect_code_+"ok")
		{
			printf("connect server ok.\n");
			break;
		}
		
		usleep(300000);
	}
	return true;
}

bool RemoteControlFixedStation::initRosParams()
{
	
	image_topic_ = nh_private_.param<std::string>("image_topic", "/image_raw");
	socket_ip_   = nh_private_.param<std::string>("socket_ip","");
	socket_port_ = nh_private_.param<int>("socket_port",-1);
	
	ROS_INFO("ip:%s,port:%d",socket_ip_.c_str(),socket_port_);
	
	if(socket_ip_.empty() || socket_port_==-1)
	{
		ROS_ERROR("please config the socket ip and port!!!");
		return false;
	}
	return true;
}

bool RemoteControlFixedStation::init()
{
	if(!initRosParams())
		return false;
		
	timer_ = nh_.createTimer(ros::Duration(0.03),&RemoteControlFixedStation::timerCallback,this);
	sub_joy_ = nh_.subscribe("/joy",1,&RemoteControlFixedStation::joyCallback,this);
	
	if(!initSocket())
		return false;
		
	std::thread t1 = std::thread(std::bind(&RemoteControlFixedStation::recvThread,this));
//	std::thread t2 = std::thread(std::bind(&RemoteControlFixedStation::showThread,this));
	t1.detach();
//	t2.detach();
	return true;
}

void RemoteControlFixedStation::sendJoyCmd()
{
	static int last_len = 0;
	static uint8_t* buf = NULL;
	
	reload_seq_mutex_.lock();
	if(joy_msg_.buttons.size())
		joy_msg_.buttons[joy_msg_.buttons.size()-1] = reload_seq_;
	if(reload_seq_)
		reload_seq_ = 0;
	reload_seq_mutex_.unlock();
	
	int axes_size = joy_msg_.axes.size();
	int buttons_size = joy_msg_.buttons.size();
	
	//5byte header,2bytes size, nbytes data, 1byte checkValue
	int len = 7 + axes_size*4 + buttons_size + 1;
	if(len!=last_len)
	{
		if(buf!=NULL)
			delete [] buf;
		buf = new uint8_t[len];
		char header[] = "joy00";
		memcpy(buf, header, 5);
		last_len = len;
	}
	buf[5] = axes_size;
	buf[6] = buttons_size;
	
	memcpy(buf+7,joy_msg_.axes.data(),axes_size*4);
	for(int i=0; i<joy_msg_.buttons.size(); ++i)
		buf[7+axes_size*4+i] = joy_msg_.buttons[i];
	
	buf[len-1] = generateCheckValue(buf+7,len-8);
	
	int send_ret   = sendto(udp_fd_, buf, len,
							0, (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
	if(send_ret < 0)
		ROS_ERROR("send joy to server failed!");
}

void RemoteControlFixedStation::timerCallback(const ros::TimerEvent& event)  
{
	//sendJoyCmd();
}

void RemoteControlFixedStation::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	joy_msg_ = *msg;
	joy_msg_.buttons.push_back(0); //append a member
	
	float offsetMax = 3.5;
	if(msg->axes[axes_leftOffset] != 1)
		offset_ = (msg->axes[axes_leftOffset] - 1)*offsetMax/2;
		
	else if(msg->axes[axes_rightOffset] != 1)
			offset_ = -(msg->axes[axes_rightOffset] - 1)*offsetMax/2;
	else
		offset_ = 0.0;	
	sendJoyCmd();
}


void RemoteControlFixedStation::recvThread()
{
	const int BufLen = 100000;
	uint8_t *recvbuf = new uint8_t [BufLen+1];
	char msg_type[6] = "12345"; msg_type[5] = '\0';
	socklen_t clientLen = sizeof(sockaddr_);
	int len = 0;
	while(ros::ok())
	{
		len = recvfrom(udp_fd_, recvbuf, BufLen,0,(struct sockaddr*)&sockaddr_, &clientLen);
		
		if(len < 0) //reconnect
		{
			sendto(udp_fd_, connect_code_.c_str(), connect_code_.length(),0, (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
			continue;
		}
		
		memcpy(msg_type,recvbuf,5);
		const std::string type(msg_type);

		ROS_INFO("received msg, len: %d",len);
		
		if(type == "state") //vehicle_info
		{
			if(recvbuf[13] != generateCheckValue(recvbuf+5,8))
				continue;
				
			state_mutex_.lock();
			state_feedback_ = *(StateFeedback_t *)(recvbuf+5);
			state_mutex_.unlock();
		}
		else if(len > 5000)
		{
			std::vector<uint8_t> data(recvbuf, recvbuf+len);
			showImage(data);
		}
	}
	delete [] recvbuf;
}

void RemoteControlFixedStation::showImage(const std::vector<uint8_t>& image_data)
{
	const StateFeedback_t& state = state_feedback_;

	if(image_data.size() ==0)
	{
		ROS_ERROR("No image!");
		return;
	}
	cv::namedWindow("result",0);
	cv::Mat img_decode = cv::imdecode(image_data,1);
	cv::resize(img_decode, img_decode,img_decode.size()*2);
	
//		std::cout << int(state.act_gear) << "\t"
//				  << int(state.driverless_mode) << "\t"
//				  << int(state.hand_brake) << "\t"
//				  << int(state.emergency_brake) << "\t"
//				  << int(state.car_state) << "\t"
//				  << state.speed*0.01 << "km/h\t"
//				  << (state.roadwheelAngle-5000)*0.01 << "deg\n";
	
	static int current_road_seq = 1;
	
	//put text
	double fontScale = 1.0;
	int text_pos_x = 30, text_pos_y = 30;
	std::stringstream manual; manual << "is_manual: ";
	if(state.is_manual) manual << 1;
	else manual << 0;
	cv::putText(img_decode,manual.str(),cv::Point(text_pos_x,text_pos_y),cv::FONT_HERSHEY_SIMPLEX,fontScale,cv::Scalar(255,23,0),2,8);
	text_pos_y += 30*fontScale;
	
	std::stringstream speed_grade; speed_grade << "speed_grade: " << int(state.speed_grade);
	cv::putText(img_decode,speed_grade.str(),cv::Point(text_pos_x,text_pos_y),cv::FONT_HERSHEY_SIMPLEX,fontScale,cv::Scalar(255,23,0),2,8);
	text_pos_y += 30*fontScale;

	std::stringstream steer_grade; steer_grade << "steer_grade: " << int(state.steer_grade);
	cv::putText(img_decode,steer_grade.str(),cv::Point(text_pos_x,text_pos_y),cv::FONT_HERSHEY_SIMPLEX,fontScale,cv::Scalar(255,23,0),2,8);
	text_pos_y += 30*fontScale;
	
	std::stringstream gear; gear << "gear: ";
	if(state.act_gear == 11) gear << "D";
	else if(state.act_gear == 13) gear << "R";
	else gear << "N";
	cv::putText(img_decode,gear.str(),cv::Point(text_pos_x,text_pos_y),cv::FONT_HERSHEY_SIMPLEX,fontScale,cv::Scalar(255,23,0),2,8);
	text_pos_y += 30*fontScale;
	
	std::stringstream speed; speed << "speed: " << state.speed*0.01 << " km/h";
	cv::putText(img_decode,speed.str(),cv::Point(text_pos_x,text_pos_y),cv::FONT_HERSHEY_SIMPLEX,fontScale,cv::Scalar(0,255,0),2,8);
	text_pos_y += 30*fontScale;
	
	std::stringstream offset_ss; offset_ss.setf(std::ios::fixed);  offset_ss.precision(1);
	if(offset_<0) offset_ss << "offset: left  " << offset_ ;
	else if(offset_ >0) offset_ss << "offset: right " << offset_ ;
	else	offset_ss << "offset: 0";
	cv::putText(img_decode,offset_ss.str(),cv::Point(text_pos_x,text_pos_y),cv::FONT_HERSHEY_SIMPLEX,fontScale,cv::Scalar(0,255,0),2,8);
	text_pos_y += 30*fontScale;
	
	
	std::stringstream ss_road_seq; ss_road_seq << "road_seq: " << current_road_seq;
	cv::putText(img_decode,ss_road_seq.str(),cv::Point(800,30),cv::FONT_HERSHEY_SIMPLEX,fontScale,cv::Scalar(0,255,0),2,8);
	
	std::stringstream angle; angle.setf(std::ios::fixed);  angle.precision(1);
	angle << "angle: " << (state.roadwheelAngle-5000)*0.01 ;
	cv::putText(img_decode,angle.str(),cv::Point(text_pos_x,text_pos_y),cv::FONT_HERSHEY_SIMPLEX,fontScale,cv::Scalar(0,255,0),2,8);
	text_pos_y += 30*fontScale;
	
	imshow("result",img_decode);
	
	int key = cv::waitKey(10);
	if(key != -1)
		std::cout << "key: " << key << std::endl;
	key -= 48;
	if(key >0 && key <= 5)
	{
		reload_seq_mutex_.lock();
		reload_seq_ = key;
		reload_seq_mutex_.unlock();
		current_road_seq = reload_seq_;
		std::cout << "key: " << key << std::endl;
	}
}

void RemoteControlFixedStation::showThread()
{
	cv::namedWindow("result",0);
	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		
		loop_rate.sleep();
	}
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"fixed_station_node");
	
	RemoteControlFixedStation sender(argc, argv);
	if(sender.init())
	{
		printf("fixed station init ok ^0^\n");
		ros::spin();
	}
	sender.closeSocket();
	return 0;
}


