//
// Created by pushyami on 1/10/19.
//

#ifndef SPINNAKER_SDK_CAMERA_DRIVER_CAPTURE_NODELET_H
#define SPINNAKER_SDK_CAMERA_DRIVER_CAPTURE_NODELET_H

#endif //SPINNAKER_SDK_CAMERA_DRIVER_CAPTURE_NODELET_H

#include <nodelet/nodelet.h>
#include "capture.h"
namespace acquisition {
    class capture_nodelet: public nodelet::Nodelet
    {

    public:
        capture_nodelet(){}
        ~capture_nodelet(){
            if (pubThread_) {
                pubThread_->interrupt();
                pubThread_->join();
            }
        }
        virtual void onInit();

        boost::shared_ptr<Capture> inst_;
        std::shared_ptr<boost::thread> pubThread_;

    };

}

