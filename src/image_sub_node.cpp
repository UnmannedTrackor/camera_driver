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
        sub_ = n_.subscribe("camera/image", 1, &ImageSub::imageCallback, this);
        pub_ = it_.advertise("transfrom/ros_img", 1);
    }

    virtual ~ImageSub() {}

    void imageCallback(const pg_driver::ImageConstPtr& msg)
    {
      try
        {
            std::vector<uchar> array = msg->data;
            cv::Mat img = cv::Mat(msg->rows, msg->cols, msg->type, array.data()).clone();
            sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            pub_.publish(msg_out);
            ROS_INFO_STREAM("New image");
        }
      catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert to 'bgr8'.");
        }
    }


private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;

    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_sub");
    ros::NodeHandle nh;

    ImageSub imageSub(nh);
    ros::spin();

    return 0;
}
