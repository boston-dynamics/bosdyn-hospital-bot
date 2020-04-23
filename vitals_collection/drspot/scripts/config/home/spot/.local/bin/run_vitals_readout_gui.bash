#!/bin/bash

. /etc/ros/env.sh
. /home/spot/drspot_ws/devel/setup.bash
rosrun rqt_gui rqt_gui --force-discover --standalone VitalsReadout
