#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <pg_driver/Image.h>

class ImageSub
{
public:
  ImageSub(ros::NodeHandle n)
    : n_(n), it_(n)
  {
    sub1_ = n_.subscribe("camera/image1", 1, &ImageSub::imageCallback1, this);
    sub2_ = n_.subscribe("camera/image2", 1, &ImageSub::imageCallback2, this);

    pub1_ = it_.advertise("transfrom/ros_img1", 1);
    pub2_ = it_.advertise("transfrom/ros_img2", 1);
  }

  virtual ~ImageSub() {}

  void imageCallback1(const pg_driver::ImageConstPtr& msg)
  {
    try
      {
	std::vector<uchar> array = msg->data;
	cv::Mat img = cv::Mat(msg->rows, msg->cols, msg->type, array.data()).clone();
	sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	pub1_.publish(msg_out);
	ROS_INFO_STREAM("New image 1");
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("Could not convert to 'bgr8'.");
      }
  }

  void imageCallback2(const pg_driver::ImageConstPtr& msg)
  {
    try
      {
	std::vector<uchar> array = msg->data;
	cv::Mat img = cv::Mat(msg->rows, msg->cols, msg->type, array.data()).clone();
	sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	pub2_.publish(msg_out);
	ROS_INFO_STREAM("New image 2");
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("Could not convert to 'bgr8'.");
      }
  }


private:
  ros::NodeHandle n_;
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;

  image_transport::ImageTransport it_;
  image_transport::Publisher pub1_;
  image_transport::Publisher pub2_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_sub");
  ros::NodeHandle nh;

  ImageSub imageSub(nh);
  ros::spin();

  return 0;
}
