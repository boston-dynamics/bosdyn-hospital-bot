/**
 * @maskro_flir_cam_node.cpp
 * @author  Marcel Stüttgen <stuettgen@fh-aachen.de>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * https://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * Ros based driver / tools for FLIR A325sc infrared camera
 * Ros node to publish raw and calculated/colored images
 */

#include <maskor_gige_cam/maskor_flir_cam_node.h>


FlirNode::FlirNode()
    : it_(nh_)
{
    showImages = false;

    _enabled = true;

    //new instance of gige camera
    cam_.reset(new maskor::GigeCam());

    //init image pointers
    RawImagePtr_.reset(new cv::Mat());
    ThermalImagePtr_.reset(new cv::Mat());
    TemperatureImagePtr_.reset(new cv::Mat());

    //set up the ROS publishers
    RawImagePub_ = it_.advertise("flir_camera/thermal_image_raw",1);
    ThermalImagePub_ = it_.advertise("flir_camera/thermal_image_palette",1);
    TemperatureImagePub_ = it_.advertise("flir_camera/temperature_image",1);
    TemperatureImagePub2_ = nh_.advertise<std_msgs::Float32MultiArray>("flir_camera/temperature_image2",1);

    //bind dynamic reconfigure callback
    ReconfigSvr_.setCallback(boost::bind(&FlirNode::_configCb, this, _1, _2));

    Enable_ = nh_.advertiseService("flir_camera/thermal_stream_enable", &FlirNode::_onEnable, this);
}


FlirNode::~FlirNode()
{

}

void FlirNode::receiveAndPublish()
{
    //get raw image from gige cam
    RawImagePtr_ = cam_->getImage();

    if (RawImagePtr_!=NULL)
    {

        //calculate the thermal image
        _thermalImageProc(*RawImagePtr_,*ThermalImagePtr_);

        //calculate the temperature image
        _temperatureImageProc(*RawImagePtr_,*TemperatureImagePtr_);

        //calculate the temperature image
        _temperatureImageProc(*RawImagePtr_, temperatures);

        //show images in separate openCV windows
        if(showImages)
        {
            _showImages();
        }

        //publish to ros

        // -- old publishing --
        auto h = std_msgs::Header(); // New Header
        h.stamp = ros::Time::now();
        h.frame_id = "a325sc";
        ImagePtr_ = cv_bridge::CvImage(h, "mono16", *RawImagePtr_ ).toImageMsg();
        RawImagePub_.publish(ImagePtr_);

        ImagePtr_ = cv_bridge::CvImage(h, "32FC1", *TemperatureImagePtr_ ).toImageMsg();
        TemperatureImagePub_.publish(ImagePtr_);

        ImagePtr_ = cv_bridge::CvImage(h, "bgr8", *ThermalImagePtr_ ).toImageMsg();
        ThermalImagePub_.publish(ImagePtr_);

        TemperatureImagePub2_.publish(temperatures);
    }

}


void FlirNode::_showImages(){

    cv::imshow("thermal_image_raw", *RawImagePtr_);
    cv::imshow("thermal_image_palette", *ThermalImagePtr_);

    cv::waitKey(60);

}


void FlirNode::_thermalImageProc(cv::Mat &in, cv::Mat &out) {

    //create empty image with 8 bit unsigned int and 1 channel
    out.create(cv::Size(in.cols,in.rows),CV_8UC1);

    unsigned short minpix = 0xFFFF;
    unsigned short maxpix = 0;
    
    //calculate pixel value ranges for auto color scaling
    for( int y = 0; y < in.rows; y++ )
    {
      for( int x = 0; x < in.cols ; x++ )
      {
        if (in.at<unsigned short>(y,x) < minpix) minpix = in.at<unsigned short>(y,x);
        if (in.at<unsigned short>(y,x) > maxpix) maxpix = in.at<unsigned short>(y,x);
      }
    }
    
    //calculate thermal image with autoscaling
    float span = (float)(maxpix - minpix + 1);
    for( int y = 0; y < in.rows; y++ )
    {
      for( int x = 0; x < in.cols ; x++ )
      {
        out.at<uchar>(y,x) = cv::saturate_cast<uchar>(((in.at<unsigned short>(y,x) - minpix) / span) * 0xFF);
      }
    }

    //apply jet colormap
    cv::applyColorMap(out, out, cv::COLORMAP_JET);
}


void FlirNode::_temperatureImageProc(cv::Mat &in, cv::Mat &out) {

    //create empty image with 32 bit float and 1 channel
    out.create(cv::Size(in.cols,in.rows),CV_32FC1);

    //calculate temperature image
    for( int y = 0; y < in.rows; y++ )
    {
        for( int x = 0; x < in.cols ; x++ )
        {
            //            std::cout << "in: " << in.at<unsigned short>(y,x) << std::endl;
            //            std::cout << "out: " << _pixel2temp(in.at<unsigned short>(y,x)) << std::endl;

            out.at<float>(y,x) = _pixel2temp(in.at<unsigned short>(y,x));
        }
    }

    //std::cout << out << std::endl;
}


void FlirNode::_temperatureImageProc(cv::Mat &in, std_msgs::Float32MultiArray &array) {

    array.data.clear();

    //calculate temperature image
    for( int y = 0; y < in.rows; y++ )
    {
        for( int x = 0; x < in.cols ; x++ )
        {
            unsigned short pixelValue = in.at<unsigned short>(y,x);
            array.data.push_back((pixelValue / 10.0f) - 273.15f);

        }
    }

}

bool FlirNode::_onEnable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if (req.data) {
    cam_->acquisitionStart();
  } else {
    cam_->acquisitionStop();
  }
  _enabled = req.data;
  res.success = true;
  return true;
}


void FlirNode::_configCb(maskor_gige_camera::FlirConfig &config, uint32_t level)
{ 
    // LEVEL:
    // RECONFIGURE_CLOSE = 3  # Parameters that need a sensor to be stopped completely when changed
    // RECONFIGURE_STOP = 1  # Parameters that need a sensor to stop streaming when changed
    // RECONFIGURE_RUNNING = 0 # Parameters that can be changed while a sensor is streaming

//    ROS_INFO("DynamicReconfigure request\n");// : %f %f %i %i %i %s %i %s %f %i",
//    ROS_INFO("Level: %i",level);
//               config.groups.angles.min_ang,
//               config.groups.angles.max_ang,
//               (int)config.intensity,
//               config.cluster,
//               config.skip,
//               config.port.c_str(),
//               (int)config.calibrate_time,
//               config.frame_id.c_str(),
//               config.time_offset,
//               (int)config.allow_unsafe_settings);


    //if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP) cam_->acquisitionStop();

    if (config.Start)       { cam_->acquisitionStart(); }
    if (config.Stop)        { cam_->acquisitionStop(); }
    if (config.AutoFocus)   { cam_->autoFocus(); }
    

//    cam_->setIRFormat(config.IRFormat);
//    cam_->setPixelFormat(config.PixelFormat);
//    cam_->setTestImage(config.TestImage);
//    cam_->setAcquisitionMode(config.AcquisitionMode);
//    cam_->setAcquisitionFrameCount(config.AcquisitionFrameCount);
//    cam_->setSize(config.Width, config.Height);
//    cam_->setOffset(config.OffsetX, config.OffsetY);

//    if (config.SensorSetDefaults)       { cam_->SensorSetDefaults(); }
//    if (config.SensorSetFactoryDefaults){ cam_->SensorSetFactoryDefaults(); }
//    if (config.GetSensorInfo)           { cam_->getSensorInfo(); }

    //if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP) cam_->acquisitionStart();
}


double FlirNode::_pixel2temp(unsigned short pixelValue)
{
    return (float)(pixelValue / 100.0f) - 273.15f;
}




int main(int argc, char** argv) {

    ros::init(argc, argv, "maskor_flir_cam_node");
    FlirNode fn;

    while(ros::ok())
    {
        fn.receiveAndPublish();
        ros::spinOnce();
    }

    return 0;
}
