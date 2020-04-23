//
// Created by auv on 1/10/19.
//

#include "ros/ros.h"
#include "spinnaker_sdk_camera_driver/std_include.h"
#include <nodelet/loader.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/Image.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "subscriber_nodelet");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "acquisition/subscriber_nodelet", remap, nargv);

    return 0;
}