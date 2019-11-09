#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <sensor_msgs/Joy.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using std::string;

uint8_t generateCheckValue(const uint8_t* buf,int len)
{
	uint8_t result = 0;
	for(int i=0; i<len; ++i)
		result += buf[i];
	return result;
}

class MsgReceiver
{
public:
	MsgReceiver(int argc,char** argv);
	~MsgReceiver();
	void closeSocket();
	bool init();

private:
	bool initRosParams();
	bool initSocket();
	void recvThread();
	void timerCallback(const ros::TimerEvent& event);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
private:
	string image_topic_;
	ros::Subscriber sub_image_;
	ros::Subscriber sub_joy_;
	ros::Timer timer_;
	
	struct sockaddr_in sockaddr_;
	string socket_ip_;
	int socket_port_;
	int udp_fd_;
	int tcp_fd_;
	bool is_tcp_;
	std::string connect_code_;
	
	sensor_msgs::Joy joy_msg_;
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
};

MsgReceiver::MsgReceiver(int argc,char** argv):
	connect_code_("fixed")
{	
	std::cout << " connect_code_: " <<connect_code_ << std::endl;
	nh_private_ = ros::NodeHandle("~");
	udp_fd_ = -1;
	tcp_fd_ = -1;
	is_tcp_ = false;
}

MsgReceiver::~MsgReceiver()
{
	
}

void MsgReceiver::closeSocket()
{
	if(udp_fd_ != -1)
		close(udp_fd_);
	if(tcp_fd_ != -1)
		close(tcp_fd_);
}

bool MsgReceiver::initSocket()
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

bool MsgReceiver::initRosParams()
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

bool MsgReceiver::init()
{
	if(!initRosParams())
		return false;
		
	timer_ = nh_.createTimer(ros::Duration(0.03),&MsgReceiver::timerCallback,this);
	sub_joy_ = nh_.subscribe("/joy",1,&MsgReceiver::joyCallback,this);
	
	if(!initSocket())
		return false;
		
	std::thread t = std::thread(std::bind(&MsgReceiver::recvThread,this));
	t.detach();
	return true;
}

void MsgReceiver::timerCallback(const ros::TimerEvent& event)
{
	static int last_len = 0;
	static uint8_t* buf = NULL;
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
		ROS_ERROR("send image to server failed!");
		
}

void MsgReceiver::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	joy_msg_ = *msg;
}


void MsgReceiver::recvThread()
{
	const int BufLen = 100000;
	uint8_t *recvbuf = new uint8_t [BufLen+1];
	socklen_t clientLen = sizeof(sockaddr_);
	int len = 0;
	while(ros::ok())
	{
		len = recvfrom(udp_fd_, recvbuf, BufLen,0,(struct sockaddr*)&sockaddr_, &clientLen);
		
		if(len > 5000)
		{
			ROS_INFO("received image, len: %d",len);
			std::vector<uint8_t> data(recvbuf, recvbuf+len);
			cv::Mat img_decode = cv::imdecode(data,1);
			cv::namedWindow("result",0);
			imshow("result",img_decode);
			cv::waitKey(1);
		}
		else if(len < 0) //reconnect
		{
			sendto(udp_fd_, connect_code_.c_str(), connect_code_.length(),0, (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
		}
	}
	delete [] recvbuf;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"remote_msg_receiver_node");
	
	MsgReceiver sender(argc, argv);
	if(sender.init())
	{
		printf("fixed station init ok ^0^\n");
		ros::spin();
	}
	sender.closeSocket();
	return 0;
}


