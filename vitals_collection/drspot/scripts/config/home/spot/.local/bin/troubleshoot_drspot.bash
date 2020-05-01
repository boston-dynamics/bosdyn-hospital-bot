#!/bin/bash

. /etc/ros/env.sh
. /home/spot/drspot_ws/devel/setup.bash

BASENAME='/log/troubleshoot'

n=
set -o noclobber
until
  outfile=$BASENAME${n:+-$n}.out
  { command exec 3> "$outfile"; } 2> /dev/null
do
  ((n++))
done

n=
set -o noclobber
until
  errfile=$BASENAME${n:+-$n}.err
  { command exec 4> "$errfile"; } 2> /dev/null
do
  ((n++))
done

n=
set -o noclobber
until
  outfolder=$BASENAME${n:+-$n}
  { mkdir "$outfolder"; } 2> /dev/null
do
  ((n++))
done

n=
set -o noclobber
until
  tarfile=$BASENAME${n:+-$n}.tar.gz
  { command exec 5> "$tarfile"; } 2> /dev/null
do
  ((n++))
done

printf 'Please save "%s", "%s", and "%s" for troubleshooting.\n' "$outfile" "$errfile" "$outfolder"
printf 'Attempting to compress all outputs to "%s" - that file is sufficient.\n' "$tarfile"

DISPLAY=:16001

function exec_and_log() {
    echo "********************************************************************************" >&3
    echo "********************************************************************************" >&4
    echo "executing $@" >&3
    echo "executing $@" >&4
    echo "--------------------------------------------------------------------------------" >&3
    echo "--------------------------------------------------------------------------------" >&4
    TIMEOUT=${TIMEOUT:-20}
    SIGNAL=${SIGNAL:-"TERM"}
    set -x
    timeout -s $SIGNAL $TIMEOUT $@ >&3 2>&4
    set +x
    echo "********************************************************************************" >&3
    echo "********************************************************************************" >&4
}

date >&3
date >&4

# Environment variables
exec_and_log "echo ${DRSPOT_THERMAL_NS}"
DRSPOT_THERMAL_NS=${DRSPOT_THERMAL_NS:-"optris"}

# ROS core functionality
exec_and_log "systemctl status roscore.service"
exec_and_log "systemctl status roslaunch.service"
exec_and_log "roswtf"

# ROS nodes
TIMEOUT=2
exec_and_log "rostopic hz /${DRSPOT_THERMAL_NS}/thermal_image_raw"
exec_and_log "rostopic hz /${DRSPOT_THERMAL_NS}/temperature_image"
exec_and_log "rostopic hz /${DRSPOT_THERMAL_NS}/thermal_image_palette"
unset TIMEOUT

# Devices
exec_and_log "df -h"
exec_and_log "ip addr"
exec_and_log "ifconfig"
exec_and_log "ip route"
exec_and_log "lsusb"
exec_and_log "/usr/bin/ir_find_serial"
exec_and_log "cat /sys/class/thermal/thermal_zone*/temp"
exec_and_log "cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq"
exec_and_log "cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_max_freq"

# VNC
exec_and_log "systemctl status vncserver@16001.service"
TIMEOUT=2
exec_and_log "nc localhost 21901"
TIMEOUT=10
SIGNAL="KILL"
exec_and_log "/home/spot/.local/bin/run_drspot_gui.bash"
unset TIMEOUT
unset SIGNAL

# Other debug files
exec_and_log "/home/spot/.local/bin/save_rosgraph.py ${outfolder}/rosgraph.dot"
exec_and_log "cp /var/log/*.log ${outfolder}/"
dmesg > ${outfolder}/dmesg.log

# Last step - can't use the files anymore
tar -cz -O $outfile $errfile $outfolder >&5
