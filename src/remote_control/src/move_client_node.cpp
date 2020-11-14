#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <js_control/Info.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include<cstring>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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
	uint8_t soft_gear :3;
	
}) StateMsg_t;

union StateUnion_t
{
	StateMsg_t state;
	uint64_t data;
};

using std::string;

uint8_t generateCheckValue(const uint8_t* buf,int len)
{
	uint8_t result = 0;
	for(int i=0; i<len; ++i)
		result += buf[i];
	return result;
}

class MsgSender
{
public:
	MsgSender();
	~MsgSender();
	void closeSocket();
	bool init();

private:
	bool initRosParams();
	bool initSocket();
	void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
	void timerCallback(const ros::TimerEvent& event);
	void vehicleInfoCallback(const std_msgs::UInt64::ConstPtr &msg);
	void joyInfoCallback(const js_control::Info::ConstPtr &msg);
	void recvThread();
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	string image_topic_;
	ros::Subscriber sub_image_;
	ros::Subscriber sub_car_info_;
	ros::Subscriber sub_joy_info_;
	ros::Publisher pub_reload_seq_;
	ros::Publisher pub_joy_;
	ros::Timer timer_;
	
	struct sockaddr_in sockaddr_;
	string socket_ip_;
	int socket_port_;
	int udp_fd_;
	int tcp_fd_;
	bool is_tcp_;
	std::string connect_code_;
	
	int image_cut_h_;
	int image_quality_;
	
	uint8_t *vehicle_info_buf_;
	js_control::Info joy_info_;
};

MsgSender::MsgSender():
	connect_code_("move0"),
	vehicle_info_buf_(NULL)
{
	nh_private_ = ros::NodeHandle("~");
	udp_fd_ = -1; tcp_fd_ = -1;
	is_tcp_ = false;
	
}

MsgSender::~MsgSender()
{
	if(vehicle_info_buf_ != NULL)
		delete [] vehicle_info_buf_;
}

void MsgSender::closeSocket()
{
	if(udp_fd_ != -1)
		close(udp_fd_);
	if(tcp_fd_ != -1)
		close(tcp_fd_);
}


bool MsgSender::initSocket()
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
	
	//UDP
	udp_fd_ = socket(PF_INET,SOCK_DGRAM , 0);
	if(udp_fd_ < 0)
	{
		ROS_ERROR("build socket error");
		return false;
	}
	
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
			printf("%d: try to connect the server failed! try again...",++cnt);
			continue;
		}
		
		char ans[21];
		socklen_t clientLen = sizeof(sockaddr_);
		int len = recvfrom(udp_fd_, ans, 20,0,(struct sockaddr*)&sockaddr_, &clientLen);
		if(len > 0 && string(ans)== connect_code_+"ok")
		{
			printf("connect server ok.\n");
			break;
		}
		
		usleep(300000);
	}
	
	return true;
}

bool MsgSender::initRosParams()
{
	image_topic_ = nh_private_.param<std::string>("image_topic", "/image_raw");
	socket_ip_   = nh_private_.param<std::string>("socket_ip","");
	socket_port_ = nh_private_.param<int>("socket_port",-1);
	image_cut_h_ = nh_private_.param<int>("image_cut_h",0);
	image_quality_ = nh_private_.param<int>("image_quality",50);
	
	
	ROS_INFO("ip:%s,port:%d",socket_ip_.c_str(),socket_port_);
	
	if(socket_ip_.empty() || socket_port_==-1)
	{
		ROS_ERROR("please config the socket ip and port!!!");
		return false;
	}
	return true;
}

bool MsgSender::init()
{
	if(!initRosParams())
		return false;
	std::cout << "ros param init ok." << std:: endl;
	
	if(!initSocket())
		return false;
	
	vehicle_info_buf_ = new uint8_t[14];
	strcpy((char*)vehicle_info_buf_,"state");
		
	sub_image_ = nh_.subscribe(image_topic_, 1, &MsgSender::imageCallback, this);
	sub_joy_info_ = nh_.subscribe("/joy_info",1,&MsgSender::joyInfoCallback, this);
	sub_car_info_ = nh_.subscribe("/vehicle_info",1,&MsgSender::vehicleInfoCallback, this);
	pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy_out",1);
	pub_reload_seq_ = nh_.advertise<std_msgs::UInt8>("/reload_seq",1);
	//timer_ = nh_.createTimer(ros::Duration(0.03),&MsgSender::timerCallback,this);
	
	std::thread t = std::thread(std::bind(&MsgSender::recvThread,this));
	t.detach();
	
	return true;
}

void MsgSender::recvThread()
{
	const int BufLen = 200;
	uint8_t *recvbuf = new uint8_t [BufLen+1];
	char msg_type[6]; msg_type[5]='\0';
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
		else if(len < 5)
			continue;
			
		memcpy(msg_type,recvbuf,5);
		const std::string type(msg_type);
		if(type == "joy00")
		{
			static sensor_msgs::Joy last_joy_msg;
			sensor_msgs::Joy joy_msg;
			int axes_cnt = recvbuf[5];
			int buttons_cnt = recvbuf[6];
			int data_len = axes_cnt*4+buttons_cnt;
			
			if(generateCheckValue(recvbuf+7, data_len) != recvbuf[data_len+7])
				continue;
			
			std::vector<float> axes((float*)(recvbuf+7),(float*)(recvbuf+7)+axes_cnt);
			joy_msg.axes.swap(axes);
			joy_msg.buttons.resize(buttons_cnt);
			for(size_t i=0; i<buttons_cnt; ++i)
				joy_msg.buttons[i] = recvbuf[7+axes_cnt*4+i];
			
			if(joy_msg.axes != last_joy_msg.axes || joy_msg.buttons != last_joy_msg.buttons)
			{
				pub_joy_.publish(joy_msg);
				
				if(joy_msg.buttons.size())
				{
					std_msgs::UInt8 reload_seq;
					reload_seq.data = joy_msg.buttons[joy_msg.buttons.size()-1];
					if(reload_seq.data)
						pub_reload_seq_.publish(reload_seq);
				}
			}
				
			last_joy_msg = joy_msg;
		}
		
	}
	delete [] recvbuf;
}

void MsgSender::timerCallback(const ros::TimerEvent& event)
{

}

void MsgSender::joyInfoCallback(const js_control::Info::ConstPtr &msg)
{
	joy_info_ = *msg;
}

void MsgSender::vehicleInfoCallback(const std_msgs::UInt64::ConstPtr &info)
{
	static StateUnion_t msg;
	msg.data = info->data;
	msg.state.is_manual = joy_info_.is_manual;
	msg.state.soft_gear = joy_info_.soft_gear;
	
	memcpy(vehicle_info_buf_+5,(char *)&(msg.data),8);
	vehicle_info_buf_[13] = generateCheckValue(vehicle_info_buf_+5,8);
	
	sendto(udp_fd_, vehicle_info_buf_, 14,
			0, (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
			
//	static StateUnion_t msg;
//	msg.data = info->data;
//	auto state = msg.state;

//	std::cout << int(state.act_gear) << "\t"
//			  << int(state.driverless_mode) << "\t"
//			  << int(state.hand_brake) << "\t"
//			  << int(state.emergency_brake) << "\t"
//			  << int(state.car_state) << "\t"
//			  << state.speed*0.01 << "km/h\t"
//			  << (state.roadwheelAngle-5000)*0.01 << "deg\n";
}

void MsgSender::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(msg, "bgr8");
	cv::Size size = cv_image->image.size()/2;

	cv::Mat dstImage;
	cv::resize(cv_image->image,dstImage, size);
	
	cv::Rect rect(0,image_cut_h_,size.width ,size.height-image_cut_h_);
	std::vector<uint8_t> image_data;
	std::vector<int> param= {cv::IMWRITE_JPEG_QUALITY, image_quality_};
	cv::imencode(".jpg", dstImage(rect), image_data, param);
	
	int send_ret   = sendto(udp_fd_, image_data.data(), image_data.size(),
							0, (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
	if(send_ret < 0)
		printf("send image to server failed!");
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "remote_msg_sender_node");
	MsgSender sender;
	if(sender.init())
	{
		printf("mobile station init ok ^0^\n");
		ros::spin();
	}
	sender.closeSocket();
	return 0;
}


