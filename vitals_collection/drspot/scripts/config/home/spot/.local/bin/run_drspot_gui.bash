#!/bin/bash

. /etc/ros/env.sh
. /home/spot/drspot_ws/devel/setup.bash
rosrun rqt_gui rqt_gui --perspective-file /home/spot/drspot_ws/src/drspot/resources/live_drspot.perspective
