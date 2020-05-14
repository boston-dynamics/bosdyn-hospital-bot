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

# Config files
exec_and_log "cat /etc/ros/env.sh"
exec_and_log "cat /etc/ros/params/spinnaker_sdk_camera_driver_params.yaml"
exec_and_log "cat /etc/ros/params/thermal_reference_params.yaml"
exec_and_log "cat /usr/sbin/roslaunch"

# ROS nodes
TIMEOUT=2
## Image rates
exec_and_log "rostopic hz /${DRSPOT_THERMAL_NS}/thermal_image_raw"
exec_and_log "rostopic hz /${DRSPOT_THERMAL_NS}/temperature_image"
exec_and_log "rostopic hz /${DRSPOT_THERMAL_NS}/thermal_image_palette"
exec_and_log "rostopic hz /debug_thermal_tracking"

for band in red nir narrow_nir; do
    exec_and_log "rostopic hz /camera_array/mono_${band}/image_raw"
    exec_and_log "rostopic hz /mono_${band}_cropped"
done
exec_and_log "rostopic hz /debug_mono_red_tracking"

## Small messages rates and contents
exec_and_log "rostopic echo /thermal_reference_temp"

exec_and_log "rostopic echo /skin_temp_roi"
exec_and_log "rostopic echo /mask_roi"
exec_and_log "rostopic echo /ir_tracking_status"
exec_and_log "rostopic echo /skin_temperature_frame_msmt"
exec_and_log "rostopic echo /respiratory_rate_frame_msmt"

for band in red nir narrow_nir; do
    exec_and_log "rostopic echo /mono_${band}_region"
    exec_and_log "rostopic echo /mono_${band}_tracking_status"
done
exec_and_log "rostopic echo /mono_red_roi"
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
timeout 20 dmesg > ${outfolder}/dmesg.log

# Last step - can't use the files anymore
timeout 20 tar -cz -O $outfile $errfile $outfolder >&5
