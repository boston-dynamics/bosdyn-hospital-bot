//
// Created by pushyami on 1/10/19.
//

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "spinnaker_sdk_camera_driver/capture_nodelet.h"

namespace acquisition
{

    void capture_nodelet::onInit()
    {
        //spinners
        //ros::AsyncSpinner spinner(0); // Use max cores possible for mt
        //spinner.start();

        NODELET_INFO("Initializing nodelet");
        //acquisition::Capture cobj(getNodeHandle(), getPrivateNodeHandle());
        inst_.reset(new Capture(getNodeHandle(), getPrivateNodeHandle()));
        inst_->init_array();
        pubThread_.reset(new boost::thread(boost::bind(&acquisition::Capture::run, inst_)));
        // cobj.run();
        //NODELET_INFO("Initializing nodelet");
    }



}

PLUGINLIB_EXPORT_CLASS(acquisition::capture_nodelet, nodelet::Nodelet)
