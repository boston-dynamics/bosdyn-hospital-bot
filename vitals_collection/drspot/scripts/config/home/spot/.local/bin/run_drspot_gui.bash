#!/bin/bash

. /etc/ros/env.sh
. /home/spot/drspot_ws/devel/setup.bash
roscd drspot
rosrun rqt_gui rqt_gui --perspective-file ./resources/live_drspot.perspective
