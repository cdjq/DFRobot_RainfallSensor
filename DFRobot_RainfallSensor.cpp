/*!
 * @file  DFRobot_RainfallSensor.cpp
 * @brief  Define infrastructure of DFRobot_RainfallSensor class
 * @details  该库实现了与Kit0192设备进行通信的所有功能，包括配置设备参数和读取设备数据
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2023-01-28
 * @url  https://github.com/DFRobot/DFRobot_RainfallSensor
 */
#include "DFRobot_RainfallSensor.h"

DFRobot_RainfallSensor:: DFRobot_RainfallSensor(uint8_t mode){
  _mode = mode;
  pid=0;
  vid=0;
}

bool DFRobot_RainfallSensor::begin()
{
  return getPidVid();
}

String DFRobot_RainfallSensor::getFirmwareVersion()
{
  uint16_t version=0;
  if(_mode==IIC_MODE){
    uint8_t buff[2]={0};
    readRegister(I2C_REG_VERSION,(void*)buff,2);
    version=buff[0]|(((uint16_t)buff[1])<<8);
  }else{
    version =readRegister(eInputRegVersionKit0192);
  }
  return String(version>>12)+'.'+String(((version>>8)&0x0F))+'.'+String(((version>>4)&0x0F))+'.'+String((version&0x0F));
}

bool DFRobot_RainfallSensor::getPidVid()
{
  bool ret = false;
  if(_mode==IIC_MODE){
    uint8_t buff[4]={0};
    readRegister(I2C_REG_PID,(void*)buff,  4);
    pid = buff[0] | (((uint16_t)buff[1])<<8) | (((uint32_t)(buff[3]&0xC0))<<10);
    vid = buff[2] | (uint16_t)((buff[3]&0x3F)<<8);
  }else{
    pid =readRegister(eInputRegPidKit0192);
    vid =readRegister(eInputRegVidKit0192);
    pid = (vid&0xC000)<<2 | pid;
    vid = vid&0x3FFF;
  }
  if( (vid==0x3343) && (pid == 0x100C0) ){
    ret = true;
  }
  return ret;
}

float DFRobot_RainfallSensor::getRainfall()
{
  uint32_t rainfall=0;
  if(_mode==IIC_MODE){
    uint8_t buff[4]={0};
    readRegister(I2C_REG_CUMULATIVE_RAINFALL,(void*)buff,4);
    rainfall=buff[0]|(((uint32_t)buff[1])<<8)|(((uint32_t)buff[2])<<16)|(((uint32_t)buff[3])<<24);
  }else{
    rainfall =readRegister(eInputRegCumulativeRainFallHKit0192);
    rainfall =rainfall<<16 | readRegister(eInputRegCumulativeRainFallLKit0192);
  }
  return rainfall/10000.0;
}

float DFRobot_RainfallSensor::getRainfall(uint8_t hour)
{
  uint32_t rainfall=0;
  if(_mode==IIC_MODE){
    writeRegister(I2C_REG_RAIN_HOUR,(void*)&hour,1);
    uint8_t buff[4]={0};
    if(readRegister(I2C_REG_TIME_RAINFALL,(void*)buff,4)==0){
      return -1;
    }
    rainfall=buff[0]|(((uint32_t)buff[1])<<8)|(((uint32_t)buff[2])<<16)|(((uint32_t)buff[3])<<24);
  }else{
    writeRegister(eHoldingRegRainHourKit0192,hour);
    rainfall =readRegister(eInputRegTimeRainFallHKit0192);
    rainfall =rainfall<<16 | readRegister(eInputRegTimeRainFallLKit0192);
  }
  return rainfall/10000.0;
}

uint32_t DFRobot_RainfallSensor::getRawdata()
{
  uint32_t rawdata=0;
  if(_mode==IIC_MODE){
    uint8_t buff[4]={0};
    readRegister(I2C_REG_RAW_DATA,(void*)buff,4);
    rawdata=buff[0]|(((uint32_t)buff[1])<<8)|(((uint32_t)buff[2])<<16)|(((uint32_t)buff[3])<<24);
  }else{
    rawdata =readRegister(eInputRegRawDataHKit0192);
    rawdata =rawdata<<16 | readRegister(eInputRegRawDataLKit0192);
  }
  return rawdata;
}

uint8_t DFRobot_RainfallSensor::setRainAccumulatedValue(float value)
{
  uint8_t ret =0;
  uint16_t data = value*10000;
  if(_mode==IIC_MODE){
    uint8_t buff[2]={0};
    buff[0]=(data&0xFF);
    buff[1]=(data>>8);
    ret=writeRegister(I2C_REG_BASE_RAINFALL,(void*)buff,2);
  }else{
    ret=writeRegister(eHoldingRegBaseRainFallKit0192,data);
  }
  return ret;
}

DFRobot_RainfallSensor_UART::DFRobot_RainfallSensor_UART(Stream *s)
:DFRobot_RainfallSensor(UART_MODE),DFRobot_RTU(s)
{
  _deviceAddr=0xC0;
}
bool DFRobot_RainfallSensor_UART::begin(void)
{
  return DFRobot_RainfallSensor::begin();
}
uint16_t DFRobot_RainfallSensor_UART::readRegister(uint16_t reg)
{
  setTimeoutTimeMs(1000);
  return readInputRegister((uint8_t)_deviceAddr, (uint16_t)reg);
}

uint16_t DFRobot_RainfallSensor_UART::writeRegister(uint16_t reg,uint16_t data)
{
  uint16_t ret=writeHoldingRegister(_deviceAddr, reg, data);
  delay(30);
  return ret;
}

DFRobot_RainfallSensor_I2C::DFRobot_RainfallSensor_I2C(TwoWire *pWire)
:DFRobot_RainfallSensor(IIC_MODE),_pWire(pWire)
{
  _deviceAddr=0x1D;
}
bool DFRobot_RainfallSensor_I2C::begin(void)
{
  _pWire->begin();
  return DFRobot_RainfallSensor::begin();
}

uint8_t DFRobot_RainfallSensor_I2C::writeRegister(uint8_t reg,void* pBuf,size_t size)
{
  if(pBuf == NULL){
	  return 1;
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  _pWire->beginTransmission(_deviceAddr);
  _pWire->write(reg);
  for(uint16_t i = 0; i < size; i++){
    _pWire->write(_pBuf[i]);
  }
  _pWire->endTransmission();
  delay(100);
  return 0;
}

uint8_t DFRobot_RainfallSensor_I2C::readRegister(uint8_t reg,void* pBuf, size_t size)
{
  if(pBuf == NULL){
    return 0;
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  _pWire->beginTransmission(_deviceAddr);
  _pWire->write(reg);
  if(_pWire->endTransmission()!=0){
    return 0;
  }
  _pWire->requestFrom(_deviceAddr, (uint8_t)size );
  for(uint8_t i=0 ;i<size;i++){
    _pBuf[i] = _pWire->read();
  }
  return size;
}