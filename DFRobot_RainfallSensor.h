/*!
 * @file  DFRobot_RainfallSensor.h
 * @brief  Define infrastructure of DFRobot_RainfallSensor class
 * @details  该库实现了与Kit0192设备进行通信的所有功能，包括配置设备参数和读取设备数据
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2023-01-28
 * @url  https://github.com/DFRobot/DFRobot_RainfallSensor
 */
#ifndef __DFROBOT_RAINFALLSENSOR_H__
#define __DFROBOT_RAINFALLSENSOR_H__
#include <Arduino.h>
#include "DFRobot_RTU.h"
#include "Wire.h"
#define IIC_MODE  0
#define UART_MODE 1
class DFRobot_RainfallSensor{
public: 
  /**
   * @enum  eKit0192InputReg_t
   * @brief  设备的输入寄存器地址
   */
  typedef enum{
    eInputRegPidKit0192=0x0000,                 /**< Kit0192 存储PID的输入寄存器的地址 */
    eInputRegVidKit0192,                        /**< Kit0192 存储VID的输入寄存器的地址 */
    eInputRegRegAddrKit0192,                    /**< Kit0192 储存设备地址的寄存器 */
    eInputRegBaudKit0192,                       /**< Kit0192 储存串口波特率的寄存器 */
    eInputRegVerifyAndStopKit0192,              /**< Kit0192 存储串口奇偶检验位与停止位的输入寄存器的地址 */
    eInputRegVersionKit0192,                    /**< Kit0192 存储固件版本的输入寄存器地址 */
    eInputRegTimeRainFallLKit0192,              /**< Kit0192 存储设置时间内的累计雨量低16位 */
    eInputRegTimeRainFallHKit0192,              /**< Kit0192 存储设置时间内的累计雨量高16位 */
    eInputRegCumulativeRainFallLKit0192,        /**< Kit0192 存储开始工作后累计雨量低16位 */
    eInputRegCumulativeRainFallHKit0192,        /**< Kit0192 存储开始工作后累计雨量高16位 */
    eInputRegRawDataLKit0192,                   /**< Kit0192 存储原始数据低16位 */
    eInputRegRawDataHKit0192,                   /**< Kit0192 存储原始数据高16位 */
  }eKit0192InputReg_t;

  /**
   * @enum  eKit0192HoldingReg_t
   * @brief  设备的保持寄存器地址
   */
  typedef enum{
    eHoldingRegReserved0Kit0192=0x0000,         /**< Kit0192 该寄存器保留 */
    eHoldingRegReserved1Kit0192,                /**< Kit0192 该寄存器保留 */
    eHoldingRegReserved2Kit0192,                /**< Kit0192 该寄存器保留 */
    eHoldingRegReserved3Kit0192,                /**< Kit0192 该寄存器保留 */
    eHoldingRegReserved4Kit0192,                /**< Kit0192 该寄存器保留 */
    eHoldingRegReserved5Kit0192,                /**< Kit0192 该寄存器保留 */
    eHoldingRegRainHourKit0192,                 /**< Kit0192 设置计算累计雨量的时间 */
    eHoldingRegBaseRainFallKit0192,             /**< Kit0192 设置雨量累加值 */
  }eKit0192HoldingReg_t;


#define I2C_REG_PID                            0x00
#define I2C_REG_VID                            0x02
#define I2C_REG_VERSION                        0x0A
#define I2C_REG_TIME_RAINFALL                  0x0C
#define I2C_REG_CUMULATIVE_RAINFALL            0x10
#define I2C_REG_RAW_DATA                       0x14
#define I2C_REG_RAIN_HOUR                      0x24
#define I2C_REG_BASE_RAINFALL                  0x26


  /**
   * @fn DFRobot_RainfallSensor
   * @brief Construct a new dfrobot windspeedwinddirectionrainsensor object
   * @param mode 工作模式，IIC_MODE:0 ,UART_MODE:1
   */
  DFRobot_RainfallSensor(uint8_t mode);
  ~DFRobot_RainfallSensor(){};

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
   * @fn getRawdata
   * @brief Get the Rawdata object
   * @return 雨量的翻斗次数，单位 次
   */
  uint32_t getRawdata();

  /**
   * @fn setRainAccumulatedValue
   * @brief Set the Rain Accumulated Value object
   * @param accumulatedValue 雨量累加值，单位为毫米
   * @return 返回 0 设置成功，其他值设置失败 
   */
  uint8_t setRainAccumulatedValue(float accumulatedValue = 0.2794);
  uint32_t vid;
  uint32_t pid;
private:
  /**
   * @fn getVid
   * @brief  Get VID and PID
   * @return  Return  true:正确获取，false:获取数据失败或者获取数据错误
   */
  bool getPidVid(void);
  virtual uint8_t readRegister(uint8_t reg,void* pBuf, size_t size){return 0;};
  virtual uint16_t readRegister(uint16_t reg){return 0;};
  virtual uint8_t writeRegister(uint8_t reg,void* pBuf,size_t size){return 0;};
  virtual uint16_t writeRegister(uint16_t reg,uint16_t data){return 0;};
protected:

private:
  int    _dePin;
  uint8_t   _mode;
};
class DFRobot_RainfallSensor_UART:public DFRobot_RainfallSensor,public DFRobot_RTU{
public: 
  /**
   * @brief Construct a new dfrobot windspeedwinddirectionrainsensor uart object
   * 
   * @param s 需要使用的串口设备
   */
  DFRobot_RainfallSensor_UART(Stream *s);
  ~DFRobot_RainfallSensor_UART(){};
  /**
   * @fn begin
   * @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
   * @return 返回通信结果
   * @retval true  Succeed
   * @retval false Failed
   */
  bool begin(void);
private:
  /**
   * @brief 读输入寄存器
   * 
   * @param reg 输入寄存器地址
   * @return uint16_t 读到的数据
   */
  uint16_t readRegister(uint16_t reg);

  /**
   * @brief 写保持寄存器,写完后内部延时了12MS,传感器需要下写入后花费12Ms进行保存，此时不能进行通信
   * 
   * @param reg 保持寄存器地址
   * @param data 要写入的数据
   * @return uint16_t 写入结果,0表示成功，其他值表示失败
   */
  uint16_t writeRegister(uint16_t reg,uint16_t data);
private:
  uint8_t _deviceAddr;
};

class DFRobot_RainfallSensor_I2C:public DFRobot_RainfallSensor{
public: 
  /**
   * @brief Construct a new dfrobot windspeedwinddirectionrainsensor iic object
   * 
   * @param pWire 需要使用的I2C设备
   */
  DFRobot_RainfallSensor_I2C(TwoWire *pWire);
  ~DFRobot_RainfallSensor_I2C(){};
  /**
   * @fn begin
   * @brief 本函数将会尝试与从机设备进行通信,根据返回值判断通信是否成功
   * @return 返回通信结果
   * @retval true  Succeed
   * @retval false Failed
   */
  bool begin(void);
private:
  /**
   * @brief 读I2C寄存器
   * 
   * @param reg I2C寄存器地址
   * @param pBuf 缓存空间
   * @param size 读取长度
   * @return 读取长度
   */
  uint8_t readRegister(uint8_t reg,void* pBuf, size_t size);

  /**
   * @brief 写I2C寄存器,写完后内部延时了12MS,传感器需要下写入后花费12Ms进行保存，此时不能进行通信
   * 
   * @param reg I2C寄存器地址
   * @param pBuf 数据存储空间
   * @param size 读取长度
   * @return uint8_t 0表示成功，其他值表示失败
   */
  uint8_t writeRegister(uint8_t reg,void* pBuf,size_t size);
private:
  TwoWire* _pWire;
  uint8_t _deviceAddr;
};
#endif