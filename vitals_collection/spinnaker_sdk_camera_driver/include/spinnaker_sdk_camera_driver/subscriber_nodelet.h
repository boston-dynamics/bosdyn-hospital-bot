//
// Created by pushyami on 1/15/19.
//

#ifndef SPINNAKER_SDK_CAMERA_DRIVER_SUBSCRIBER_NODELET_H
#define SPINNAKER_SDK_CAMERA_DRIVER_SUBSCRIBER_NODELET_H

#endif //SPINNAKER_SDK_CAMERA_DRIVER_SUBSCRIBER_NODELET_H
#include "spinnaker_sdk_camera_driver/std_include.h"
#include <nodelet/nodelet.h>
#include "capture.h"
namespace acquisition {
    class subscriber_nodelet: public nodelet::Nodelet
    {

    public:
        subscriber_nodelet(){}
        ~subscriber_nodelet(){}
        virtual void onInit();
        void chatterCallback(const sensor_msgs::Image::ConstPtr& msg);
        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Subscriber image_sub_;

    };

}