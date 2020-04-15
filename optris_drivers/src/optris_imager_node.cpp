/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2016
 *  Technische Hochschule Nürnberg Georg Simon Ohm
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Nuremberg Institute of Technology
 *     Georg Simon Ohm nor the authors names may be used to endorse
 *     or promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Stefan May
 *********************************************************************/

#include <sys/stat.h>
#include "OptrisImager.h"
#include "libirimager/IRLogger.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "optris_imager_node");

  // private node handle to support command line parameters for rosrun
  ros::NodeHandle n_("~");

  std::string xmlConfig = "";
  n_.getParam("xmlConfig", xmlConfig);

  // A specific configuration file for each imager device is needed (cf. config directory)
  struct stat s;
  if(stat(xmlConfig.c_str(), &s) != 0)
  {
    std::cerr << "usage: rosrun <package> <node> _xmlConfig:=<xmlConfig>" << std::endl;
    std::cerr << " verify that <xmlConfig> exists" << std::endl;
    return -1;
  }

  ros::NodeHandle n;

  evo::IRLogger::setVerbosity(evo::IRLOG_DEBUG, evo::IRLOG_OFF);

  // Read parameters from xml file
  evo::IRDeviceParams params;
  if(!evo::IRDeviceParamsReader::readXML(xmlConfig.c_str(), params))
    return -1;

  // Find valid device
  evo::IRDevice* dev = evo::IRDevice::IRCreateDevice(params);
  if(!dev)
  {
    std::cout << "Error: UVC device with serial " << params.serial << " could not be found" << std::endl;
    return -1;
  }

  // Give control to class instance
  optris_drivers::OptrisImager imager(dev, params);
  imager.run();

  return 0;
}
