#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <ros/console.h>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    sensor_msgs::Image a =*msg;
    double times =a.header.stamp.toSec();
    ROS_INFO_STREAM("THE DIFF IS :" << ros::Time::now().toSec() - times);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera_array/cam0/image_raw", 1, imageCallback);
  ros::spin();
}
