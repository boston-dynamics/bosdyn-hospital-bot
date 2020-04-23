# Attributions

This code and these algorithms were developed in collaboration with:
- Edward W. Boyer <eboyer@bwh.harvard.edu>
- Peter Chai <pchai@mit.edu>
- Henwei Huang <henwei@mit.edu>
- Canchen Li <canchenl@mit.edu>
- Giovanni Traverso <cgt20@mit.edu>

Also see:
- [Multiperson tracking and vitals](https://github.com/Frost-Lee/thermal_monitoring)

# Dependencies

```
pip uninstall opencv-python
pip install opencv-contrib-python
pip install mxnet-mkl insightface numpy scipy
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

# Operation

```
roslaunch drspot vitals.launch
```

```
roscd drspot
rosrun rqt_gui rqt_gui --perspective-file ./resources/live_drspot.perspective
```