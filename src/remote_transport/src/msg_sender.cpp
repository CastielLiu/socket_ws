#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <string>
#include <vector>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using std::string;

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
	void timerCallback(const ros::TimerEvent& event);
private:
	string image_topic_;
	ros::Subscriber sub_image_;
	ros::Timer timer_;
	
	string socket_ip_;
	int socket_port_;
	int udp_fd_;
	int tcp_fd_;
	bool is_tcp_;
	struct sockaddr_in sockaddr_;

};

MsgSender::MsgSender()
{
	udp_fd_ = -1;
	tcp_fd_ = -1;
	is_tcp_ = false;
}

MsgSender::~MsgSender()
{
	
}

void MsgSender::closeSocket()
{
	if(udp_fd_ != -1)
		close(udp_fd_);
	if(tcp_fd_ != -1)
		close(tcp_fd_);
}

bool MsgSender::init()
{
	if(!initRosParams())
		return false;
	std::cout << "ros param init ok." << std:: endl;
	
	if(!initSocket())
		return false;
		
	ros::NodeHandle nh;
	sub_image_ = nh.subscribe(image_topic_, 1, &MsgSender::imageCallback, this);
	//timer_ = nh.createTimer(ros::Duration(0.5),&MsgSender::timerCallback,this);
	return true;
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
	
	int cnt = 0;
	while(ros::ok())
	{
		char code[] = "cmd02";
		int send_ret   = sendto(udp_fd_, code, sizeof(code),0, 
						 (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
						 
		if(send_ret <= 0)
		{
			ROS_INFO("%d: try to connect the server failed! try again...",++cnt);
			continue;
		}
		
		char ans[21];
		socklen_t clientLen = sizeof(sockaddr_);
		int len = recvfrom(udp_fd_, ans, 20,0,(struct sockaddr*)&sockaddr_, &clientLen);
		if(len > 0 && string(ans)=="cmd02ok")
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
	ros::NodeHandle nh_private("~");
	image_topic_ = nh_private.param<std::string>("image_topic", "/image_raw");
	socket_ip_   = nh_private.param<std::string>("socket_ip","");
	socket_port_ = nh_private.param<int>("socket_port",-1);
	
	ROS_INFO("ip:%s,port:%d",socket_ip_.c_str(),socket_port_);
	
	if(socket_ip_.empty() || socket_port_==-1)
	{
		ROS_ERROR("please config the socket ip and port!!!");
		return false;
	}
	return true;
}

void MsgSender::timerCallback(const ros::TimerEvent& event)
{
	ROS_INFO("send msgs...");
	uint8_t buf[10] = "test";
	
	int send_ret   = sendto(udp_fd_, buf, 10,
							0, (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
	if(send_ret < 0)
		ROS_ERROR("send image to server failed!");
}

void MsgSender::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
	cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(msg, "bgr8");
	cv::Mat tmp_image = cv_image->image;

	std::vector<uint8_t> image_data;
	std::vector<int> param= {cv::IMWRITE_JPEG_QUALITY, 50};
	cv::imencode(".jpg", cv_image->image, image_data, param);
	
	int send_ret   = sendto(udp_fd_, image_data.data(), image_data.size(),
							0, (struct sockaddr*)&sockaddr_, sizeof(sockaddr_));
	if(send_ret < 0)
		ROS_ERROR("send image to server failed!");
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "remote_msg_sender_node");
	MsgSender sender;
	if(sender.init())
		ros::spin();
	sender.closeSocket();
	return 0;
}


