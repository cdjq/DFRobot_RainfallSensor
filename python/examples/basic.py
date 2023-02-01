# -*- coding: utf-8 -*
'''!
  @file  basic.py
  @brief 本示例介绍了使用本模块测试一小时内降雨量的采集的方法
  @copyright   Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      [fary](feng.yang@dfrobot.com)
  @version     V1.0
  @date        2023-01-28
  @url         https://github.com/DFRobor/DFRobot_RainfallSensor
'''
from __future__ import print_function
import sys
import os
sys.path.append("../")
import time

from DFRobot_RainfallSensor import *

#sensor=DFRobot_RainfallSensor_UART()
sensor=DFRobot_RainfallSensor_I2C()

def setup():
  while (sensor.begin() == False):
    print("Sensor initialize failed!!")
    time.sleep(1)
  print("Sensor  initialize success!!")

  print("Version: "+ sensor.get_firmware_version())
  print("vid: %#x"%(sensor.vid))
  print("pid: %#x"%(sensor.pid))
  #设置风速计半径，单位为cm
  sensor.set_radius_anemometer(9)
  #设置雨量累加值，单位为mm
  sensor.set_rain_accumulated_value(0.2794)
def loop():
  #获取系统运行时间内的累计雨量
  rainfall=sensor.get_rainfall()
  #获取系统1小时内的累计雨量（函数参数可选1-24）
  one_hour_rainfall=sensor.get_rainfall_time(1)

  #获取原始数据，雨量的翻斗次数，单位 次
  rainfall_raw=sensor.get_raw_data()

  print("rainfall : %f mm"%(rainfall))
  print("The amount of rain in one hour: %f mm"%(one_hour_rainfall))
  print("Original value collected by rain gauge %d"%(rainfall_raw))
  print("--------------------------------------------------------------------")
  time.sleep(1)

if __name__ == "__main__":
  setup()
  while True:
    loop()