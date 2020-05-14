#include "spinnaker_sdk_camera_driver/capture.h"

#include "ros/console.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

int main(int argc, char** argv) {
    
    int result = 0;

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();
    }
    
    // Initializing the ros node
    ros::init(argc, argv, "acquisition_node");
    //spinners
    ros::AsyncSpinner spinner(1);
    spinner.start();

    acquisition::Capture cobj;
    cobj.init_array();

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
      ros::console::notifyLoggerLevelsChanged();
    }

    cobj.run();
    
    return 0;
        
}
