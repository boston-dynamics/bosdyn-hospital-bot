#!/bin/bash

# Run this script on the client side (not SpotCORE) with superuser permissions.

# This script adds a TUN interface to allow VPN-over-SSH to the SpotCORE.
# Over a fast connection, this allows e.g. forwarding all ROS traffic through an SSH tunnel.
# After running this script on the client side and spotcore_tun0.sh on the SpotCORE,
# run the proper ssh command to access the SpotCORE, but add the argument `-w 0:0`.
# This will establish connection using the TUN interface.

ip tuntap add tun0 mode tun user $USER
ip address add 192.168.0.1/24 dev tun0
ip link set dev tun0 up
