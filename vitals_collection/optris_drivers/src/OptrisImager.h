#ifndef _OPTRISIMAGER_H_
#define _OPTRISIMAGER_H_

#include "libirimager/IRDevice.h"
#include "libirimager/IRImager.h"
#include "libirimager/IRImagerClient.h"

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/TimeReference.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "optris_drivers/AutoFlag.h"
#include "optris_drivers/TemperatureRange.h"
#include "optris_drivers/Temperature.h"
#include "optris_drivers/Flag.h"

namespace optris_drivers
{

/**
 * @class OptrisImager
 * @brief Node management class
 * @author Stefan May (Technische Hochschule Nürnberg Georg Simon Ohm)
 */
class OptrisImager : public evo::IRImagerClient
{
public:

  /**
   * Constructor
   * @param[in] dev UVC device instance
   * @param[in] params device parameters
   */
  OptrisImager(evo::IRDevice* dev, evo::IRDeviceParams params);

  /**
   * Destructor
   */
  virtual ~OptrisImager();

  /**
   * Blocking run method (calls ros::spin())
   */
  void run();

  /**
   * Timer callback for image grabbing timer
   * @param[in] event timer event
   */
  void onTimer(const ros::TimerEvent& event);

  /**
   * Raw frame callback (not needed in Linux implementations, since timer instance takes care of data acquisition => method is never called).
   * @param[in] data raw data
   * @param[in] size size of raw data
   */
  virtual void onRawFrame(unsigned char* data, int size) {};

  /**
   * Thermal frame callback
   * @param[in] image thermal image (for information about the format see IRImager class of libirimager)
   * @param[in] w width of image
   * @param[in] h height of image
   * @param[in] meta Metadata frame
   * @param[in] arg user defined data (passed via process method)
   */
  virtual void onThermalFrame(unsigned short* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg);

  /**
   * Callback method from image processing library (called at configured frame rate in xml file)
   * @param[in] image RGB image, if BISPECTRAL technology is available
   * @param[in] w image width
   * @param[in] h image height
   * @param[in] timestamp the frame's timestamp
   * @param[in] arg user defined data (passed via process method)
   */
  virtual void onVisibleFrame(unsigned char* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg);

  /**
   * Flag state change callback
   * @param[in] flagstate flag state
   * @param[in] arg user defined data (passed via process method)
   */
  virtual void onFlagStateChange(evo::EnumFlagState flagstate, void* arg);

  /**
    * Callback method for synchronizing data. This is the very last method to be called for each raw data set.
    * @param[in] arg user arguments (passed to process method of IRImager class)
    */
  virtual void onProcessExit(void* arg);

  /**
   * ROS service callback
   */
  bool onAutoFlag(AutoFlag::Request &req, AutoFlag::Response &res);

  /**
   * ROS service callback
   */
  bool onForceFlag(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * ROS service callback
   */
  bool onEnable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  /**
   * ROS service callback
   */
  bool onSetTemperatureRange(TemperatureRange::Request &req, TemperatureRange::Response &res);

private:

  void _temperatureImageProc(const sensor_msgs::Image& in, sensor_msgs::Image& out);

  void _temperatureImageProc(const sensor_msgs::Image& in, std_msgs::Float32MultiArray &array);

  double _pixel2temp(unsigned short pixelValue);

  evo::IRImager _imager;

  evo::IRDevice* _dev;

  unsigned int _img_cnt;

  unsigned char* _bufferRaw;

  sensor_msgs::Image _thermal_image;

  sensor_msgs::Image _temperature_image;

  std_msgs::Float32MultiArray _temperatures;

  sensor_msgs::Image _visible_image;

  sensor_msgs::TimeReference _device_timer;

  optris_drivers::Temperature _internal_temperature;

  image_transport::Publisher _thermal_pub;

  image_transport::Publisher _temperature_image_pub;

  ros::Publisher _temperature_array_pub;

  image_transport::Publisher _visible_pub;

  ros::Publisher _temp_pub;

  ros::Publisher _timer_pub;

  ros::Publisher _flag_pub;

  ros::ServiceServer _sAuto;

  ros::ServiceServer _sForce;

  ros::ServiceServer _sTemp;

  ros::ServiceServer _sEnable;

  bool _enabled;
};

} //namespace

#endif // _OPTRISIMAGER_H_
