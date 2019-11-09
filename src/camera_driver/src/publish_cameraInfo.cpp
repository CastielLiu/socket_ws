#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CameraInfo.h"

#define _NODE_NAME_ "cameraInfo_node"

class ImageTalker
{
public:
	ImageTalker()
	{
	}
	bool init(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
	{
		std::string calibration_file_path;
		nh_private.param<std::string>("calibration_file_path",calibration_file_path,"a.yaml");
		ROS_INFO("%s",calibration_file_path.c_str());
	
		if(!Loadintrinsics(calibration_file_path))
		{
			ROS_INFO("%s open failed",calibration_file_path.c_str());
			return false;
		}
		
		camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/camera_info", 1);
		timer_ = nh.createTimer(ros::Duration(0.2), &ImageTalker::timerCallback, this);
		return true;
	}
	
	void timerCallback(const ros::TimerEvent& event)
	{
		this->pubCameraInfo();
	}
	
	
	bool Loadintrinsics(const std::string &calibration_file_path)
	{
		if (calibration_file_path.empty())
		{
			ROS_ERROR("[%s] missing calibration file path", _NODE_NAME_);
			return false;
		}

		cv::FileStorage fs(calibration_file_path, cv::FileStorage::READ);

		if (!fs.isOpened())
		{
			ROS_INFO("[%s] cannot open calibration file %s", _NODE_NAME_, calibration_file_path.c_str());
	 		return false;
		}

		camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
		distortion_coefficients_ = cv::Mat(1, 5, CV_64F);

		cv::Mat dis_tmp;
		
		fs["CameraMat"] >> camera_instrinsics_;
		fs["DistCoeff"] >> dis_tmp;
		fs["ImageSize"] >> imgSize_;
		fs["DistModel"] >> distModel_;
		
		for (int col = 0; col < 5; col++)
		{
			distortion_coefficients_.at<double>(col) = dis_tmp.at<double>(col);
		}
		fs.release();	//释放
		return true;
	}
	
	void pubCameraInfo()
	{
		static bool instrinsics_parsed = false;
		if (instrinsics_parsed)
			goto PublishCameraInfo;
		
		for (int row = 0; row < 3; row++)
			for (int col = 0; col < 3; col++)
				camera_info_msg_.K[row * 3 + col] = camera_instrinsics_.at<double>(row, col);

		for (int row = 0; row < 3; row++)
			for (int col = 0; col < 4; col++)
			{
				if (col == 3)
					camera_info_msg_.P[row * 4 + col] = 0.0f;
				else
					camera_info_msg_.P[row * 4 + col] = camera_instrinsics_.at<double>(row, col);
			}
		for (int row = 0; row < distortion_coefficients_.rows; row++)
			for (int col = 0; col < distortion_coefficients_.cols; col++)
				camera_info_msg_.D.push_back(distortion_coefficients_.at<double>(row, col));
		camera_info_msg_.distortion_model = distModel_;
		camera_info_msg_.height = imgSize_.height;
		camera_info_msg_.width = imgSize_.width;
		instrinsics_parsed = true;
		camera_info_msg_.header.frame_id = "camera";
	
	PublishCameraInfo:
		camera_info_msg_.header.stamp = ros::Time::now();
		camera_info_pub_.publish(camera_info_msg_);
	}

private:
	ros::Publisher camera_info_pub_;
	ros::Timer timer_;
	
	cv::Mat camera_instrinsics_;
	cv::Mat distortion_coefficients_;
	std::string distModel_;
	sensor_msgs::CameraInfo camera_info_msg_;
	cv::Size imgSize_;
	
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, _NODE_NAME_);
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	ImageTalker image_talker;
	if(image_talker.init(nh,nh_private))
		ros::spin();
	return 0;
}
