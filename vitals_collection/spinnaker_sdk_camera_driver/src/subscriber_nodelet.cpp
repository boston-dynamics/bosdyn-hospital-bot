//
// Created by pushyami on 1/15/19.
//

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "spinnaker_sdk_camera_driver/subscriber_nodelet.h"


//This is a test nodelet for  measuring nodelet performance
namespace acquisition
{
    void subscriber_nodelet::chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        NODELET_INFO_STREAM("diff time is "<< ros::Time::now().toSec() - msg->header.stamp.toSec());
    }

    void subscriber_nodelet::onInit()
    {

        NODELET_INFO("Initializing nodelet");
        ros::NodeHandle& node = getNodeHandle();
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        ROS_INFO_STREAM("in sub nodelet");
        it_.reset(new image_transport::ImageTransport(node));
        image_sub_ = it_->subscribe("camera_array/cam0/image_raw",1, &subscriber_nodelet::chatterCallback, this) ;
    }



}

PLUGINLIB_EXPORT_CLASS(acquisition::subscriber_nodelet, nodelet::Nodelet)
