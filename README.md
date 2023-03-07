DFRobot_RainfallSensor
===========================
* [中文版](./README_CN.md)

本库为SEN0575 雨量传感器套件提供了Arduino IDE 和树莓派软件驱动以及示例代码，用户可根据此库通过软件操作来获取雨量传感器获取的24小时内的雨量信息、传感器的工作时间以及传感器工作时间内的累计雨量信息。

![产品实物图](./resources/images/SEN0575.png)


## Product Link (https://www.dfrobot.com/)
    SKU：SEN0575


## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)


## Summary
* 读取雨量计数据


## Installation

这里提供两种使用本库的方法：
1. 打开Arduino IDE,在状态栏中的Tools--->Manager Libraries 搜索"DFRobot_RainfallSensor"并安装本库.
2. 首先下载库文件,将其粘贴到\Arduino\libraries目录中,然后打开examples文件夹并在该文件夹中运行演示.
注意：本库需要配合DFRobot_RTU使用，确保安装了DFRobot_RTU后再使用本库


## Methods

```C++
  /**
   * @fn begin
   * @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
   * @return 返回通信结果
   * @retval true  Succeed
   * @retval false Failed
   */
  bool begin(void);

  /**
   * @fn getFirmwareVersion
   * @brief  get firmware version
   * @return  Return  firmware version
   */
  String getFirmwareVersion(void);

  /**
   * @fn getRainfall
   * @brief 获取累计雨量
   * @return float 累计雨量
   */
  float getRainfall(void);

  /**
   * @fn getRainfall
   * @brief 获取指定时间内的累计雨量
   * @param hour 指定时间(有效设置为1-24小时)
   * @return float 累计雨量
   */
  float getRainfall(uint8_t hour);

  /**
   * @fn getRawData
   * @brief Get the Rawdata object
   * @return 雨量的翻斗次数，单位 次
   */
  uint32_t getRawData();

  /**
   * @fn setRainAccumulatedValue
   * @brief Set the Rain Accumulated Value object
   * @param accumulatedValue 雨量累加值，单位为毫米
   * @return 返回 0 设置成功，其他值设置失败 
   */
  uint8_t setRainAccumulatedValue(float accumulatedValue = 0.2794);

  /**
   * @fn getSensorWorkingTime
   * @brief Obtain the sensor working time
   * @return 工作时间,单位小时
   */
  float getSensorWorkingTime();
```


## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :----:
Arduino Uno        |      √       |              |             |
Arduino MEGA2560   |      √       |              |             |
Arduino Leonardo   |      √       |              |             |
FireBeetle-ESP8266 |      √       |              |             |
FireBeetle-ESP32   |      √       |              |             |
FireBeetle-M0      |      √       |              |             |
Micro:bit          |      √(IIC） |   √(UART)    |             |


## History

- 2023/02/28 - Version 1.0.0 released.

## Credits

Written by fary(feng.yang@dfrobot.com), 2023. (Welcome to our [website](https://www.dfrobot.com/))

