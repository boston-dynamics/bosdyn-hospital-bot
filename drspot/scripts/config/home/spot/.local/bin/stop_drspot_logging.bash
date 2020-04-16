#!/bin/bash

. /etc/ros/env.sh
. /home/spot/drspot_ws/devel/setup.bash
pkill -SIGINT -f '/opt/ros/melodic/lib/rosbag/record.*auto'
