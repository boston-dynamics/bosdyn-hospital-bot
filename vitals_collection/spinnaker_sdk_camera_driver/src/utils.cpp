#include "spinnaker_sdk_camera_driver/utils.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

// This function acquires and saves 10 images from a device.  
int AcquireImages(CameraList camList, int numCameras, int numImgsToCapture) {

    int result = 0;
	
    cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;

    try {

        // Creating camera directories
        for (int cam=0; cam < numCameras; cam++) {
            ostringstream ss;
            ss<<"cam"<<cam;
            if (mkdir(ss.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0) {
                cout<<"Failed to create directory "<<ss.str()<<"! Data will be written into pre existing directory if it exists..." << endl;
            }
        }
		
        // Retrieve, convert, and save images
        const int skip_num = 20;
        const unsigned int k_numImages = numImgsToCapture; // 10;
        time_t result = time(NULL);

        for (int imageCnt =0; imageCnt < k_numImages ; imageCnt++) {
                
            try {
                //
                // Retrieve next received image
                //
                // *** NOTES ***
                // Capturing an image houses images on the camera buffer. Trying
                // to capture an image that does not exist will hang the camera.
                //
                // *** LATER ***
                // Once an image from the buffer is saved and/or no longer 
                // needed, the image must be released in order to keep the 
                // buffer from filling up.
                CameraPtr  pCams[numCameras];
                ImagePtr   pResultImages[numCameras];

                double t1;
                
                // Get all images from camera buffers
                t1 = omp_get_wtime();
                for (unsigned int i = 0; i < numCameras; i++) {

                    pCams[i] = NULL;
                    pResultImages[i] = NULL;
                                     
                    pCams[i] = camList.GetByIndex(i);
             
                    pResultImages[i] = pCams[i]->GetNextImage();
                               
                    // Check if the Image is complete
                    if (pResultImages[i]->IsIncomplete()) {
					
                        cout << "Image incomplete with image status " << pResultImages[i]->GetImageStatus() << "..." << endl << endl;
                        return -1;
                                
                    }
                    // cout<<"frameID:"<< pResultImages[i]->GetFrameID()<<endl;

                }
                LOG(INFO)<<"Time taken to get frames: "<<omp_get_wtime()-t1;

                
                t1 = omp_get_wtime();
                for (unsigned int i = 0; i < numCameras; i++) {
                                
                    ImagePtr pResultImage = pResultImages[i];

                    // skip the first 20 images
                    // if(imageCnt > skip_num)
                    // {
				   
                    // Print image information; height and width recorded in pixels

                    size_t width = pResultImage->GetWidth();
                    size_t height = pResultImage->GetHeight();
		  
                    LOG(INFO) << "Grabbed image " << imageCnt << ", width = " << width << ", height = " << height << ", timestamp: " << int(result) << "-" << pResultImage->GetTimeStamp() << ", cam = " << i << ", frameID = "<< pResultImage->GetFrameID() << endl;

                    // Extracting timestamp info
                    double t, it, ft;
                    t = double(pResultImage->GetTimeStamp())/1000000000;
                    ft = modf(t, &it);
		  
                    // Create a unique filename
                    ostringstream filename;

                    char num[10];
                    sprintf(num, "%06d", imageCnt);    
		  
                    // filename<<"cam"<<i<<"/"<< string(num) << "-" << int(result)+int(it) << int(ft*1000000000) << ".png";
                    filename<<"cam"<<i<<"/"<< int(result)+int(it) << int(ft*1000000000) << ".jpg";

                    if (imageCnt < skip_num) {
                        pResultImage->Save("dump.jpg");
                        LOG(INFO) << "Skipping frame...";
                    } else {
                        pResultImage->Save(filename.str().c_str());
                        LOG(INFO) << "Image saved at " << filename.str() << endl;
                    }
							
                    pResultImage->Release();

                }
                LOG(INFO)<<"Time taken to save images: "<<omp_get_wtime()-t1;

            }
                    
            catch (Spinnaker::Exception &e) {
                cout << "Error: " << e.what() << endl;
                result = -1;
            }
                
        }
	
    }
    
    catch (Spinnaker::Exception &e) {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
	
    return result;

}

std::string today_date()
{
    char out[9];
    std::time_t t=std::time(NULL);
    std::strftime(out, sizeof(out), "%Y%m%d", std::localtime(&t));
    std::cout<<out;
    std::String td(out);
    return td;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap & nodeMap) {

    int result = 0;
	
    cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

    try {
        
            FeatureList_t features;
            CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
            if (IsAvailable(category) && IsReadable(category)) {

                category->GetFeatures(features);

                FeatureList_t::const_iterator it;
                for (it = features.begin(); it != features.end(); ++it) {

                    CNodePtr pfeatureNode = *it;
                    cout << pfeatureNode->GetName() << " : ";
                    CValuePtr pValue = (CValuePtr)pfeatureNode;
                    cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                    cout << endl;
                }
                
            }
            else {
                cout << "Device control information not available." << endl;
            }
	}
    
    catch (Spinnaker::Exception &e) {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
	
    return result;

}


// This function acts as the body of the example; please see NodeMapInfo example 
// for more in-depth comments on setting up cameras.
int initializeCameras(CameraList camList, int numCameras, int numImgsToCapture) {

    int result = 0;

    // Set camera1 frame rate
    CameraPtr pCam = NULL;
        
    // Set cameras 1 to 4 to continuous
    //for (unsigned int i = 0; i < numCameras; i++)
    for (int i = numCameras-1 ; i >=0 ; i--) {
                        
        CameraPtr pCam = NULL;
        // Select camera
        pCam = camList.GetByIndex(i);

        cout << endl << "Running example for camera " << i << "..." << endl;

        try {

            // Retrieve TL device nodemap and print device information
            INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
		
            // result = PrintDeviceInfo(nodeMapTLDevice);
		
            // Initialize camera
            pCam->Init();
		
            // Retrieve GenICam nodemap
            INodeMap & nodeMap = pCam->GetNodeMap();

            // Retrieve enumeration node from nodemap
            CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
            if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) {

                cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
                return -1;
	    }
		
            // Retrieve entry node from enumeration node
            CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
            if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous)) {

                cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
                return -1;
	    }
		
            // Retrieve integer value from entry node
            int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
		
            // Set integer value from entry node as new value of enumeration node
            ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
		
            LOG(INFO) << "Acquisition mode set to continuous for camera: " <<i<<" "<< endl;

            pCam->BeginAcquisition();
            LOG(INFO) << "Acquiring images for camera: " <<i<<" "<< endl;

            LOG(INFO) << "Frame rate: " << pCam->AcquisitionFrameRate.GetValue();

            // LOG(INFO) << "Trigger: " << pCam->TriggerSource.GetValue();
            
            //  cout << "Camera " << i << " linkspeed is " << pCam->DeviceLinkSpeed << endl;

            
			
	}

        catch (Spinnaker::Exception &e) {

            cout << "Error: " << e.what() << endl;
            result = -1;
	}		
		
    }
	
    sleep(2);
    cout << "Sleep complete " << endl;

    // Acquire images for all the cameras
    result = result | AcquireImages(camList, numCameras, numImgsToCapture);
        
    for (int i = 0; i < numCameras; i++) {

        CameraPtr pCam = NULL;
        // Select camera
        pCam = camList.GetByIndex(i);
                
        // End acquisition
        //
        // *** NOTES ***
        // Ending acquisition appropriately helps ensure that devices clean up
        // properly and do not need to be power-cycled to maintain integrity.
        //
        cout << "Camera "<<i<<": "<<i<<"- End Acquisition "<<endl;
        // cout<<" "<<pCam->GetNumImagesInUse()<<" "<<endl; 
        pCam->EndAcquisition();

        cout << "Camera "<<i<<": "<<i<<"- Deinit "<<endl;
        // Deinitialize camera
        pCam->DeInit();
	    
    }
    
    cout << "All cameras deinitialized!!!"<<endl;
	
    return result;
}

CameraList sort_cam_list(CameraList camList) {

    CameraList camListcopy;
    CameraList camListnew;
    CameraPtr pCam;
    string pCamID;
    unsigned int numCameras = camList.GetSize();
              
    //  Sort as per camera ids defined in CAMERA_SNs      
    for (int camno=0; camno<sizeof(CAMERA_SNs)/sizeof(CAMERA_SNs[0]);camno++) {

        camListcopy = CameraList(camList);
		
        for (int acamno=numCameras-1; acamno>=0;acamno--) {

            pCam = camList.GetByIndex(acamno);
            pCamID = pCam->GetUniqueID();

            if (pCamID == CAMERA_SNs[camno]) {

                cout << "Camera serial is:"<<pCamID<< endl;
            }
            else {
                camListcopy.RemoveByIndex(acamno);
            }
            
        }
        camListnew.Append(camListcopy);
    }		

    camList = camListnew; // Sorted camlist

    cout << "camList[0]: " << camList.GetByIndex(0)->GetUniqueID() << endl;
    cout << "camList[1]: " << camList.GetByIndex(1)->GetUniqueID() << endl;

    camListcopy.Clear();
    camListnew.Clear();
    pCam = NULL;

    return camList;
    
}
