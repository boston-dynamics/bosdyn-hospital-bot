#include "spinnaker_sdk_camera_driver/camera.h"

acquisition::Camera::~Camera() {

    pCam_ = NULL;
    timestamp_ = 0;

}

acquisition::Camera::Camera(CameraPtr pCam) {

    pCam_ = pCam;

    if (pCam_->IsInitialized()) {
        ROS_WARN_STREAM("Camera already initialized. Deinitializing...");
        pCam_->EndAcquisition();
        pCam_->DeInit();
    }

    lastFrameID_ = -1;
    frameID_ = -1;
    MASTER_ = false;
    timestamp_ = 0;
    GET_NEXT_IMAGE_TIMEOUT_ = 2000;
    
}

void acquisition::Camera::init() {

    pCam_->Init();
    
}

void acquisition::Camera::deinit() {

    pCam_->DeInit();

}

ImagePtr acquisition::Camera::grab_frame() {

    ImagePtr pResultImage = pCam_->GetNextImage(GET_NEXT_IMAGE_TIMEOUT_);
    // Check if the Image is complete

    if (pResultImage->IsIncomplete()) {
        
        ROS_WARN_STREAM("Image incomplete with image status " << pResultImage->GetImageStatus() << "!");

    } else {

        timestamp_ = pResultImage->GetTimeStamp();
    
        if (frameID_ >= 0) {
            lastFrameID_ = frameID_;
            frameID_ = pResultImage->GetFrameID();
            ROS_WARN_STREAM_COND(frameID_ > lastFrameID_ + 1,"Frames are being skipped!");
        } else {
            frameID_ = pResultImage->GetFrameID();
            ROS_ASSERT_MSG(frameID_ == 0 ,"First frame ID was not zero! Might cause sync issues later...");
        }

    }

    ROS_DEBUG_STREAM("Grabbed frame from camera " << get_id() << " with timestamp " << timestamp_*1000);
    return pResultImage;    
}

// Returns last timestamp
string acquisition::Camera::get_time_stamp() {

    stringstream ss;
    ss<<timestamp_*1000;
    return ss.str();

}

int acquisition::Camera::get_frame_id() {

    return frameID_;

}

Mat acquisition::Camera::grab_mat_frame() {

    ImagePtr pResultImage = grab_frame();
    return convert_to_mat(pResultImage);

}

Mat acquisition::Camera::convert_to_mat(ImagePtr pImage) {

    ImagePtr convertedImage;
    if (COLOR_)
        convertedImage = pImage->Convert(PixelFormat_BGR8); //, NEAREST_NEIGHBOR);
    else
		convertedImage = pImage->Convert(PixelFormat_Mono8); //, NEAREST_NEIGHBOR);
    unsigned int XPadding = convertedImage->GetXPadding();
    unsigned int YPadding = convertedImage->GetYPadding();
    unsigned int rowsize = convertedImage->GetWidth();
    unsigned int colsize = convertedImage->GetHeight();
    //image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.
    Mat img;
    if (COLOR_)
        img = Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
    else
        img = Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, convertedImage->GetData(), convertedImage->GetStride());
    return img.clone();
    // return img;
    
}

void acquisition::Camera::begin_acquisition() {

    ROS_DEBUG_STREAM("Begin Acquisition...");
    pCam_->BeginAcquisition();
    
}

void acquisition::Camera::end_acquisition() {

    if (pCam_->GetNumImagesInUse())
        ROS_WARN_STREAM("Some images still currently in use! Use image->Release() before deinitializing.");
        
    ROS_DEBUG_STREAM("End Acquisition...");
    pCam_->EndAcquisition();    
    
}

void acquisition::Camera::setEnumValue(string setting, string value) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    // Retrieve enumeration node from nodemap
    CEnumerationPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr))
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << value << " (enum retrieval). Aborting...");

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrValue = ptr->GetEntryByName(value.c_str());
    if (!IsAvailable(ptrValue) || !IsReadable(ptrValue))
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << value << " (entry retrieval). Aborting...");
		
    // retrieve value from entry node
    int64_t valueToSet = ptrValue->GetValue();
		
    // Set value from entry node as new value of enumeration node
    ptr->SetIntValue(valueToSet);    

    ROS_DEBUG_STREAM(setting << " set to " << value);
    
}

void acquisition::Camera::setIntValue(string setting, int val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CIntegerPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting...");
    }
    ptr->SetValue(val);

    ROS_DEBUG_STREAM(setting << " set to " << val);
    
}

void acquisition::Camera::setFloatValue(string setting, float val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CFloatPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting...");
    }
    ptr->SetValue(val);

    ROS_DEBUG_STREAM(setting << " set to " << val);
    
}

void acquisition::Camera::setBoolValue(string setting, bool val) {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CBooleanPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting...");
    }
    if (val) ptr->SetValue("True");
		else ptr->SetValue("False");

    ROS_DEBUG_STREAM(setting << " set to " << val);
    
}



void acquisition::Camera::setResolutionPixels(int width, int height) {
    CIntegerPtr ptrHeight=pCam_->GetNodeMap().GetNode("Height");
    CIntegerPtr ptrWidth=pCam_->GetNodeMap().GetNode("Width");
    if (!IsAvailable(ptrWidth) || !IsWritable(ptrWidth)){
        ROS_FATAL_STREAM("Unable to set width" << "). Aborting...");
        return ; 
    }
    int64_t widthMax = ptrWidth->GetMax();
    if(widthMax<width)
        width=widthMax;
    ptrWidth->SetValue(width);
    ROS_DEBUG_STREAM("Set Width"<<width);

    if (!IsAvailable(ptrHeight) || !IsWritable(ptrHeight)){
        ROS_FATAL_STREAM("Unable to set height" << "). Aborting...");
        return ; 
    }
    int64_t heightMax = ptrHeight->GetMax();
    if(heightMax<height)
        height=heightMax;

    ROS_DEBUG_STREAM("Set Height"<<height);
    ptrHeight->SetValue(height);                                                                                                                                 
}

void acquisition::Camera::adcBitDepth(gcstring bitDep) {
    CEnumerationPtr ptrADC = pCam_->GetNodeMap().GetNode("AdcBitDepth");
    if (!IsAvailable(ptrADC) || !IsWritable(ptrADC)){
        ROS_FATAL_STREAM("Unable to set ADC Bit " <<  "). Aborting...");
        return ;
    }

    CEnumEntryPtr ptrADCA = ptrADC->GetEntryByName(bitDep);
    int currDepth=ptrADCA->GetValue();
    ptrADC->SetIntValue(currDepth);
    ROS_DEBUG_STREAM("BPS: "<<ptrADC->GetIntValue());

}

void acquisition::Camera::setBufferSize(int numBuf) {

    INodeMap & sNodeMap = pCam_->GetTLStreamNodeMap();
    CIntegerPtr StreamNode = sNodeMap.GetNode("StreamDefaultBufferCount");
    int64_t bufferCount = StreamNode->GetValue();
    if (!IsAvailable(StreamNode) || !IsWritable(StreamNode)){
        ROS_FATAL_STREAM("Unable to set StreamMode " << "). Aborting...");
        return;
    }
    StreamNode->SetValue(numBuf);
    ROS_DEBUG_STREAM("Set Buf "<<numBuf<<endl);
}

void acquisition::Camera::setISPEnable() {
    CBooleanPtr ptrISPEn=pCam_->GetNodeMap().GetNode("IspEnable");
    if (!IsAvailable(ptrISPEn) || !IsWritable(ptrISPEn)){
        ROS_FATAL_STREAM("Unable to set ISP Enable (node retrieval; camera " << "). Aborting...");
        return;
    }
    ptrISPEn->SetValue("True");
}

void acquisition::Camera::setFREnable() {
    CBooleanPtr ptrAcquisitionFrameRateEnable=pCam_->GetNodeMap().GetNode("AcquisitionFrameRateEnable");
    if (!IsAvailable(ptrAcquisitionFrameRateEnable) || !IsWritable(ptrAcquisitionFrameRateEnable)){
        ROS_FATAL_STREAM("Unable to set frameRateEnable (node retrieval; camera " << "). Aborting...");
        return;
    }

    ptrAcquisitionFrameRateEnable->SetValue("True");
}

void acquisition::Camera::setPixelFormat(gcstring formatPic) {
    CEnumerationPtr ptrPixelFormat = pCam_->GetNodeMap().GetNode(formatPic);
    if ( !IsWritable(ptrPixelFormat)){
        ROS_FATAL_STREAM("Unable to set Pixel Format " <<  "). Aborting...");
        return ;
    }
    CEnumEntryPtr ptrPixelEnt = ptrPixelFormat->GetEntryByName("RGB8Packed");
    if (!IsAvailable(ptrPixelEnt) || !IsReadable(ptrPixelEnt)){
        ROS_FATAL_STREAM("Unable to set RGBPoint"  << "). Aborting...");
        return ;
    }                                                                                                                                        
    int64_t colorNum = ptrPixelEnt->GetValue();
                                                                                                                                                
    ptrPixelFormat->SetIntValue(colorNum);
    ROS_DEBUG_STREAM( "Camera " << " set pixel format");
}

void acquisition::Camera::trigger() {

    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CCommandPtr ptr = nodeMap.GetNode("TriggerSoftware");
    if (!IsAvailable(ptr) || !IsWritable(ptr))
        ROS_FATAL_STREAM("Unable to execute trigger. Aborting...");

    ROS_DEBUG_STREAM("Executing software trigger...");
    ptr->Execute();
    
}


void acquisition::Camera::targetGreyValueTest() {
    CFloatPtr ptrExpTest =pCam_->GetNodeMap().GetNode("AutoExposureTargetGreyValue");
    //CFloatPtr ptrExpTest=pCam_->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExpTest) || !IsReadable(ptrExpTest)){
        ROS_FATAL_STREAM("Unable to set exposure " << "). Aborting..." << endl << endl);
        return ;
    }
    ptrExpTest->SetValue(90.0);
    ROS_INFO_STREAM("target grey mode Time: "<<ptrExpTest->GetValue()<<endl);

}

void acquisition::Camera::exposureTest() {
    CFloatPtr ptrExpTest=pCam_->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExpTest) || !IsReadable(ptrExpTest)){
        ROS_FATAL_STREAM("Unable to set exposure " << "). Aborting..." << endl << endl);
        return ;
    }
    float expTime=ptrExpTest->GetValue();
    ROS_DEBUG_STREAM("Exposure Time: "<<expTime<<endl);

}