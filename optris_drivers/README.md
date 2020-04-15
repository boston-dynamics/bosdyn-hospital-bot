optris_drivers
==============

ROS drivers for Optris thermal imagers

**Installing IR Imager Direct SDK**

In order to successfully compile this package, the IR Imager Direct SDK for Optris cameras is needed.
You can download it from http://ftp.evocortex.com/


If you want to install the latest available version using rosdep, add a rosdep source by executing

``sudo sh -c 'echo "yaml https://raw.githubusercontent.com/evocortex/optris_drivers/master/libirimager.yaml " > /etc/ros/rosdep/sources.list.d/19-libirimager.list'``

in a Linux terminal. For further information regarding rosdep, see http://wiki.ros.org/rosdep
