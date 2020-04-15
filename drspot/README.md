# Dependencies

```
pip install mxnet-mkl insightface
```

Make sure you run the system once with internet access, so that the insightface model gets downloaded.

Or, run:
```
python -c "import insightface;  insightface.model_zoo.get_model('retinaface_r50_v1')"
```

# Supported Hardware

Tested with:
1. FLIR A325sc over ethernet
2. Optris PI400 over USB

# Operation

```
roslaunch drspot vitals.launch
```
