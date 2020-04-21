#include "spinnaker_sdk_camera_driver/capture.h"
#include <nodelet/loader.h>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

int main(int argc, char** argv) {

    int result = 0;

    // Initializing the ros node
    ros::init(argc, argv, "acquisition_node");

    // This is code based nodelet loading, the preferred nodelet launching is done through roslaunch
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "acquisition/capture_nodelet", remap, nargv);

    return 0;

}
//
// Created by auv on 1/10/19.
//

