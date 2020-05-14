/**
 * @maskro_flir_cam_node.h
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

#ifndef MASKOR_FLIR_CAM_NODE_H
#define MASKOR_FLIR_CAM_NODE_H

//GIGE CAM
#include <maskor_gige_cam/maskor_gige_cam.h>

//OTHER
#include <boost/scoped_ptr.hpp>

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <maskor_gige_cam/FlirConfig.h>
#include <driver_base/SensorLevels.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/SetBool.h>

class FlirNode
{
public:
    FlirNode();
    ~FlirNode();

    //mainloop
    void receiveAndPublish();

private:
    // ROS
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher RawImagePub_;
    image_transport::Publisher ThermalImagePub_;
    image_transport::Publisher TemperatureImagePub_;
    ros::Publisher TemperatureImagePub2_;
    sensor_msgs::ImagePtr ImagePtr_;
    ros::ServiceServer Enable_;
    bool _enabled;

    // FLIR Camera
    boost::scoped_ptr <maskor::GigeCam> cam_;

    //OpenCV Images
    boost::shared_ptr<cv::Mat> RawImagePtr_;
    boost::shared_ptr<cv::Mat> ThermalImagePtr_;
    boost::shared_ptr<cv::Mat> TemperatureImagePtr_;

    //dynamic reconfigure
    dynamic_reconfigure::Server<maskor_gige_camera::FlirConfig> ReconfigSvr_;

    //internal helper functions
    double _pixel2temp(unsigned short pixelValue);
    void _thermalImageProc(cv::Mat &in, cv::Mat &out);
    void _temperatureImageProc(cv::Mat &in, cv::Mat &out);
    void _temperatureImageProc(cv::Mat &in, std_msgs::Float32MultiArray &array);
    void _showImages();
    void _configCb(maskor_gige_camera::FlirConfig &config, uint32_t level);
    bool _onEnable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    //show images in separate openCV windows?
    bool showImages;

    std_msgs::Float32MultiArray temperatures;
};

#endif //MASKOR_FLIR_CAM_NODE_H
