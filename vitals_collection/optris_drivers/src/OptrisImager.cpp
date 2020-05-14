#include "OptrisImager.h"

namespace optris_drivers
{

OptrisImager::OptrisImager(evo::IRDevice* dev, evo::IRDeviceParams params)
{
  _imager.init(&params, dev->getFrequency(), dev->getWidth(), dev->getHeight(), dev->controlledViaHID());
  _imager.setClient(this);

  _bufferRaw = new unsigned char[dev->getRawBufferSize()];

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  _thermal_pub = it.advertise("thermal_image_raw", 1);
  _thermal_image.header.frame_id = "optris_thermal";
  _thermal_image.height          = _imager.getHeight();
  _thermal_image.width           = _imager.getWidth();
  _thermal_image.encoding        = "mono16";
  _thermal_image.step            = _thermal_image.width * 2;
  _thermal_image.data.resize(_thermal_image.height * _thermal_image.step);

  _temperature_image_pub = it.advertise("temperature_image", 1);
  _temperature_image.header.frame_id = "optris_thermal";
  _temperature_image.height          = _imager.getHeight();
  _temperature_image.width           = _imager.getWidth();
  _temperature_image.encoding        = "32FC1";
  _temperature_image.step            = _temperature_image.width * 4;
  _temperature_image.data.resize(_temperature_image.height * _temperature_image.step);

  _temperature_array_pub = n.advertise<std_msgs::Float32MultiArray>("temperature_image2", 1);
  _temperatures.data.reserve(_imager.getHeight() * _imager.getWidth());

  if(_imager.hasBispectralTechnology())
  {
    _visible_pub = it.advertise("visible_image_raw", 1);
    _visible_image.header.frame_id = "optris_visible";
    _visible_image.height          = _imager.getVisibleHeight();
    _visible_image.width           = _imager.getVisibleWidth();
    _visible_image.encoding        = "yuv422";
    _visible_image.step            = _visible_image.width * 2;
    _visible_image.data.resize(_visible_image.height * _visible_image.step);
  }

  // advertise the camera internal timer
  _timer_pub= n.advertise<sensor_msgs::TimeReference>("optris_timer", 1 );
  _device_timer.header.frame_id=_thermal_image.header.frame_id;

  _sAuto  = n.advertiseService("auto_flag",  &OptrisImager::onAutoFlag, this);
  _sForce = n.advertiseService("force_flag", &OptrisImager::onForceFlag, this);
  _sTemp  = n.advertiseService("temperature_range", &OptrisImager::onSetTemperatureRange, this);

  _sEnable = n.advertiseService("thermal_stream_enable", &OptrisImager::onEnable, this);

  // advertise all of the camera's temperatures in a single custom message
  _temp_pub = n.advertise<Temperature> ("internal_temperature", 1);
  _internal_temperature.header.frame_id=_thermal_image.header.frame_id;

  _flag_pub = n.advertise<Flag>("flag_state", 1);

  _img_cnt = 0;

  _dev = dev;

  _enabled = true;
}

OptrisImager::~OptrisImager()
{
  delete [] _bufferRaw;
}

void OptrisImager::run()
{
  _dev->startStreaming();

  ros::NodeHandle n;
  float max_framerate = _imager.getMaxFramerate();
  if (max_framerate < 1e-3) {
      ROS_WARN("Invalid max framerate; using default value");
      max_framerate = 100.0;
  }
  ROS_INFO_STREAM("Max framerate " << max_framerate);
  ros::Timer timer = n.createTimer(ros::Duration(1.0/max_framerate), &OptrisImager::onTimer, this);
  ros::spin();

  _dev->stopStreaming();
}

void OptrisImager::onTimer(const ros::TimerEvent& event)
{
  int retval = _dev->getFrame(_bufferRaw);
  if(retval==evo::IRIMAGER_SUCCESS)
  {
    _imager.process(_bufferRaw);
  }
  if(retval==evo::IRIMAGER_DISCONNECTED)
  {
    ros::shutdown();
  }
}

void OptrisImager::onThermalFrame(unsigned short* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg)
{
  memcpy(&_thermal_image.data[0], image, w * h * sizeof(*image));

  _thermal_image.header.seq = ++_img_cnt;
  _thermal_image.header.stamp = ros::Time::now();
  _thermal_pub.publish(_thermal_image);

  _temperatureImageProc(_thermal_image, _temperature_image);
  _temperature_image.header = _thermal_image.header;
  _temperature_image_pub.publish(_temperature_image);

  _temperatureImageProc(_thermal_image, _temperatures);
  _temperature_array_pub.publish(_temperatures);

  _device_timer.header = _thermal_image.header;
  _device_timer.time_ref.fromNSec(meta.timestamp);

  _internal_temperature.header = _thermal_image.header;

  _internal_temperature.temperature_flag = _imager.getTempFlag();
  _internal_temperature.temperature_box  = _imager.getTempBox();
  _internal_temperature.temperature_chip = _imager.getTempChip();

  _timer_pub.publish(_device_timer);
  _temp_pub.publish(_internal_temperature);
}

void OptrisImager::onVisibleFrame(unsigned char* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg)
{
  if(_visible_pub.getNumSubscribers()==0) return;

  memcpy(&_visible_image.data[0], image, 2 * w * h * sizeof(*image));

  _visible_image.header.seq   = _img_cnt;
  _visible_image.header.stamp = ros::Time::now();
  _visible_pub.publish(_visible_image);
}

void OptrisImager::onFlagStateChange(evo::EnumFlagState flagstate, void* arg)
{
  optris_drivers::Flag flag;
  flag.flag_state      = flagstate;
  flag.header          = _thermal_image.header;
  _flag_pub.publish(flag);
}

void OptrisImager::onProcessExit(void* arg)
{

}

bool OptrisImager::onAutoFlag(AutoFlag::Request &req, AutoFlag::Response &res)
{
  _imager.setAutoFlag(req.autoFlag);
  res.isAutoFlagActive = _imager.getAutoFlag();
  return true;
}

bool OptrisImager::onEnable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if (req.data) {
    _dev->startStreaming();
  } else {
    _dev->stopStreaming();
  }
  _enabled = req.data;
  res.success = true;
  return true;
}

bool OptrisImager::onForceFlag(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  _imager.forceFlagEvent();
  return true;
}

bool OptrisImager::onSetTemperatureRange(TemperatureRange::Request &req, TemperatureRange::Response &res)
{
  bool validParam = _imager.setTempRange(req.temperatureRangeMin, req.temperatureRangeMax);

  if(validParam)
  {
    _imager.forceFlagEvent(1000.f);
  }

  res.success = validParam;

  return true;
}

void OptrisImager::_temperatureImageProc(const sensor_msgs::Image& in, sensor_msgs::Image& out) {
    const unsigned short* in_dat = (const unsigned short*) in.data.data();
    float* out_dat = (float*) out.data.data();
    //calculate temperature image
    for( int y = 0; y < in.height; y++ )
    {
        for( int x = 0; x < in.width ; x++ )
        {
            const int i = y * in.width + x;
            out_dat[i] = _pixel2temp(in_dat[i]);
        }
    }
}

void OptrisImager::_temperatureImageProc(const sensor_msgs::Image& in, std_msgs::Float32MultiArray &array) {
    const unsigned short* in_dat = (const unsigned short*) in.data.data();

    array.data.clear();

    //calculate temperature image
    for( int y = 0; y < in.height; y++ )
    {
        for( int x = 0; x < in.width ; x++ )
        {
            const int i = y * in.width + x;
            array.data.push_back(_pixel2temp(in_dat[i]));
        }
    }

}

double OptrisImager::_pixel2temp(unsigned short pixelValue)
{
    return (pixelValue - 1000.0) / 10.0;
}

}
