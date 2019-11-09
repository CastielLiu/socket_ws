#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CameraInfo.h"
#include "opencv2/imgproc/detail/distortion_model.hpp"

#define _NODE_NAME_ "camera_driver"
using namespace cv;

class ImageTalker
{
public:
	ImageTalker()
	{
	}
	void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private,int cameraId)
	{
		camera_id_ = cameraId;
		
		image_transport::ImageTransport it(nh);
	
		std::string calibration_file_path;
		nh_private.param<std::string>("calibration_file_path",calibration_file_path,"a.yaml");
		nh_private.param<bool>("is_show_image",is_show_image_,false);
		nh_private.param<int>("frame_rate",frame_rate_,30);
		ros::param::get("~resolution",resolution_);
	
		ROS_INFO("%s",calibration_file_path.c_str());
	
		if(!Loadintrinsics(calibration_file_path))
		{
			pub_ = it.advertise("/image_raw", 1);
			is_rectify_ = false;
		}
		else
		{
			pub_ = it.advertise("/image_rectified", 1);
			new_camera_instrinsics_ = getOptimalNewCameraMatrix(camera_instrinsics_,distortion_coefficients_,imgSize_,1.0);
			is_rectify_ = true;
		}
		camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/camera_info", 1);
		if(is_rectify_)
			timer_ = nh.createTimer(ros::Duration(0.2), &ImageTalker::timerCallback, this);
	}
	
	void timerCallback(const ros::TimerEvent& event)
	{
		this->pubCameraInfo();
	}
	
	void run()
	{
		cv::VideoCapture cap(camera_id_);
		
		if(!cap.isOpened())
		{
			ROS_ERROR("can not open video device\n");
			return;
		}
		
		cap.set(CV_CAP_PROP_FRAME_WIDTH, resolution_[0]);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, resolution_[1]);
		
		cv::Mat frame,src;
		sensor_msgs::ImagePtr msg;

		ros::Rate loop_rate(frame_rate_);
		
		while (ros::ok())
		{
			cap >> frame;
			//cv::flip(src,frame,-1); //将摄像头图像翻转一次再发布

			if(!frame.empty())
			{
				if(is_rectify_)
				{
					cv::undistort(frame, src, camera_instrinsics_, distortion_coefficients_,new_camera_instrinsics_);
					msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
					if(is_show_image_) 
					{
						cv::namedWindow("image_rectified",cv::WINDOW_NORMAL); 
						cv::imshow("image_rectified",src); cv::waitKey(1);
					}
				}
				else
				{
					msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
					if(is_show_image_) 
					{
						cv::namedWindow("image_raw",cv::WINDOW_NORMAL); 
						cv::imshow("image_raw",frame); cv::waitKey(1);
					}
				}
				msg->header.frame_id="camera";
				pub_.publish(msg);
			}
			
			loop_rate.sleep();
			ros::spinOnce();
		}
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
	bool is_rectify_;
	image_transport::Publisher pub_ ;
	ros::Publisher camera_info_pub_;
	ros::Timer timer_;
	
	cv::Mat new_camera_instrinsics_;
	cv::Mat camera_instrinsics_;
	cv::Mat distortion_coefficients_;
	std::string distModel_;
	sensor_msgs::CameraInfo camera_info_msg_;
	cv::Size imgSize_;
	std::vector<int> resolution_;
	int camera_id_;
	int frame_rate_;
	bool is_show_image_;
	
};


int main(int argc, char** argv)
{
	if(argc <2) 
	{
		ROS_ERROR("argv[1]: camera number is empty\n");
		return 1;
	}

	ros::init(argc, argv, _NODE_NAME_);
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	int camera_id = atoi(argv[1]);
	
	ImageTalker image_talker;
	image_talker.init(nh,nh_private,camera_id);
	image_talker.run();
	return 0;
}
