#!/bin/bash
ip tuntap add tun0 mode tun user $USER
ip address add 192.168.0.1/24 dev tun0
ip link set dev tun0 up
