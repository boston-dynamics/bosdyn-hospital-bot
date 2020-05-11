#!/usr/bin/env python

from yoctopuce.yocto_api import YAPI, YRefParam
from yoctopuce.yocto_temperature import YTemperature
import time

YAPI.RegisterHub('usb')
r = YTemperature.FirstTemperature()
while 1:
    print(r.get_currentValue())
    time.sleep(1)
