#!/bin/bash

. /etc/ros/env.sh
. /home/spot/drspot_ws/devel/setup.bash

/home/spot/.local/bin/stop_drspot_logging.bash

cd /log
rosbag record -o auto --split --size=500 -a -x "(.*)/compressed(.*)|(.*)/theora|(.*)/theora/(.*)"
