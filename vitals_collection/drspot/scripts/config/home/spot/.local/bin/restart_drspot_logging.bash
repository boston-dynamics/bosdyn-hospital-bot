#!/bin/bash

. /etc/ros/env.sh
. /home/spot/drspot_ws/devel/setup.bash

/home/spot/.local/bin/stop_drspot_logging.bash

cd /log
rosbag record -o auto --split --size=500 -a -x \
       "(.*)/compressed(.*)|(.*)/theora|(.*)/theora/(.*)|(.*)camera_array(.*)|(.*)temperature_image2|(.*)thermal_image_palette(.*)|(.*)camera_info|(.*)debug_(.*)_tracking(.*)"
