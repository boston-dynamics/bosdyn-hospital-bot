# Attributions

This code and these algorithms were developed in collaboration with:
- Edward W. Boyer <eboyer@bwh.harvard.edu>
- Peter Chai <pchai@mit.edu>
- Henwei Huang <henwei@mit.edu>
- Canchen Li <canchenl@mit.edu>
- Giovanni Traverso <cgt20@mit.edu>

The heart rate and SpO2 algorithm implemented here is based on
[https://www.nature.com/articles/srep38609](https://www.nature.com/articles/srep38609)

Also see:
- [Multiperson tracking and vitals](https://github.com/Frost-Lee/thermal_monitoring)

# Dependencies

```
pip uninstall opencv-python
pip install opencv-contrib-python
pip install mxnet-mkl insightface numpy scipy transitions transitions-gui
pip install yoctopuce # for the thermal reference
```

On Ubuntu 18.04:
```
sudo apt install -y ros-melodic-desktop-full ros-melodic-driver-base libunwind-dev libudev-dev
```

**Make sure you run the system once with internet access, so that the insightface model
gets downloaded and cached.**

Or, run:
```
python -c "import insightface;  insightface.model_zoo.get_model('retinaface_r50_v1')"
```

# Supported Hardware

Thermal IR camera options:
1. FLIR A325sc over ethernet
    1. Requires eBUS SDK
2. Optris PI400 over USB
    1. Requires libirimager

Thermal reference options:
1. Yoctopuce PT-100
    1. Requires yoctopuce Python module (python -m pip install yoctopuce)

Monochrome camera options:
1. FLIR Chameleon monochrome
    1. Requires Spinnaker SDK

# Operation

## Nominal live usage

```
roslaunch drspot vitals.launch
```

```
roscd drspot
rosrun rqt_gui rqt_gui --perspective-file ./resources/live_drspot.perspective
```

## Nominal playback usage

```
roslaunch drspot vitals.launch launch_drivers:=false
```

```
roscd drspot
rosrun rqt_gui rqt_gui --perspective-file ./resources/live_drspot.perspective
```

## Disable use of the thermal reference
This is also relevant for playback, as some of the datasets do not have thermal reference data.

Add the following argument to the `roslaunch drspot vitals.launch` call: `use_thermal_reference:=false`.

So, for nominal live usage:
```
roslaunch drspot vitals.launch use_thermal_reference:=false
```

And for nominal playback usage:
```
roslaunch drspot vitals.launch launch_drivers:=false use_thermal_reference:=false
```

## Manually set the IR camera data source and namespace

For Optris:
```
export DRSPOT_THERMAL_NS=optris
```

For FLIR:
```
export DRSPOT_THERMAL_NS=flir_camera
```

Then restart any running ROS nodes. On the SpotCORE, run `sudo systemctl restart roslaunch.service`.

## Namespace the vitals measurement nodes to discard their measurements

This is useful if you want to replay all topics from a bag, but also use e.g. the respiratory_rate node to plot the frequency-domain analysis.

```
roscd drspot
ROS_NAMESPACE=/${DRSPOT_THERMAL_NS} ./nodes/skin_temperature \
                                    /${DRSPOT_THERMAL_NS}/ir_tracking_status:=ir_tracking_status \
                                    /${DRSPOT_THERMAL_NS}/skin_temp_roi:=/skin_temp_roi
```

## Manually specify ROI for temperature measurements

### Skin temperature

Make sure the `ir_face_tracker` node is **not** running, and that the desired thermal camera publishers **are** running.

```
rostopic pub -r 10 /ir_tracking_status std_msgs/Bool "data: true" &
rosrun image_view2 image_view2 \
       image:=/${DRSPOT_THERMAL_NS}/thermal_image_palette \
       /${DRSPOT_THERMAL_NS}/thermal_image_palette/screenrectangle:=skin_temp_roi \
       __name:=skin_temp_roi &
roscd drspot && ./nodes/skin_temperature \
      temperature_image:=/${DRSPOT_THERMAL_NS}/temperature_image \
      thermal_image_raw:=/${DRSPOT_THERMAL_NS}/thermal_image_raw &
rostopic echo /skin_temperature_frame_msmt
```

Select a ROI in the live feed.

### Add another ROI with some other name, in another terminal.

```
export ROI_NAME=some_other_roi
export ROS_NAMESPACE=$ROI_NAME
rostopic pub -r 10 ir_tracking_status std_msgs/Bool "data: true" &
rosrun image_view2 image_view2 \
       image:=/${DRSPOT_THERMAL_NS}/thermal_image_palette \
       /${DRSPOT_THERMAL_NS}/thermal_image_palette/screenrectangle:=temp_roi \
       __name:=${ROI_NAME} &
roscd drspot && ./nodes/skin_temperature \
      __name:=${ROI_NAME}_temperature \
      skin_temp_roi:=temp_roi \
      temperature_image:=/${DRSPOT_THERMAL_NS}/temperature_image \
      thermal_image_raw:=/${DRSPOT_THERMAL_NS}/thermal_image_raw &
rostopic echo ${ROI_NAME}_temperature_frame_msmt
```

## Relevant rostopics

### Optris

```
/optris/{temperature_image,thermal_image_raw,thermal_image_palette,internal_temperature,flag_state}
```
