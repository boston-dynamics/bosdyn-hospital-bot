#ifndef CAMERA_HEADER
#define CAMERA_HEADER

#include "std_include.h"
#include "serialization.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;
using namespace std;

namespace acquisition {

    class Camera {

    public:

        ~Camera();
        Camera(CameraPtr);

        void init();
        void deinit();
        void begin_acquisition();
        void end_acquisition();

        ImagePtr grab_frame();
        Mat grab_mat_frame();
        string get_time_stamp();
        int get_frame_id();

        void setEnumValue(string, string);
        void setIntValue(string, int);
        void setFloatValue(string, float);
        void setBoolValue(string, bool);

        void trigger();
        
        void setISPEnable();
        void setFREnable();
        void setPixelFormat(gcstring formatPic);
        void exposureTest();
        void setResolutionPixels(int width, int height);
        void setBufferSize(int numBuf);
        void adcBitDepth(gcstring bitDep);
        void targetGreyValueTest();

        // void set_acquisition_mode_continuous();
        // void set_frame_rate(float);
        // void set_horizontal_binning(int);
        // void set_vertical_binning(int);
        // void set_horizontal_decimation(int);
        // void set_vertical_decimation(int);

        // void setTrigDelay(float delay);
        // void setTrigSelectorFrame();
        // void setTrigMode();
        // void setTriggerOverlapOff();

        string get_id() { return string(pCam_->GetUniqueID()); }
        void make_master() { MASTER_ = true; ROS_DEBUG_STREAM( "camera " << get_id() << " set as master"); }
        bool is_master() { return MASTER_; }
        void set_color(bool flag) { COLOR_ = flag; }
        
    private:

        Mat convert_to_mat(ImagePtr);
        
        CameraPtr pCam_;
        int64_t timestamp_;
        int frameID_;
        int lastFrameID_;

        bool COLOR_;
        bool MASTER_;
        uint64_t GET_NEXT_IMAGE_TIMEOUT_;

    };

}

#endif
