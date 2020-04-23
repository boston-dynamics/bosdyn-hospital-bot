#ifndef UTILS_HEADER
#define UTILS_HEADER

#include "std_include.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

// Serial number of camera in order 1 through 5 with 1 being master
string CAMERA_SNs[] = {"17011867", "17011857", "17011862", "17011871", "17011869"};

int AcquireImages(CameraList camList, int numCameras, int numImgsToCapture);

int PrintDeviceInfo(INodeMap & nodeMap);

int initializeCameras(CameraList camList, int numCameras, int numImgsToCapture);

CameraList sort_cam_list(CameraList camList);

std::string today_date()


#endif
