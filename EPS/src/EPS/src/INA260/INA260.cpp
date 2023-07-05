/*
 * Title: INA260.cpp
 * Copyright (c) 2012, Adafruit Industries
 * Modified by: Aldo Aguilar-Nadalini 
 * Date: May 7th, 2019
 * Description: INA260 V/I Monitor library
 */

// Imports
#include "Arduino.h"
#include "INA260.h"

/**************************************************************************/
/*! 
    @brief  CONSTRUCTOR INA260 Class
*/
/**************************************************************************/
INA260::INA260(int mode)
{
  if (mode == 1){
    INA260_i2caddr = INA260_ADDRESS_1;
  } else if (mode == 2){
    INA260_i2caddr = INA260_ADDRESS_2;
  } else if (mode == 3){
    INA260_i2caddr = INA260_ADDRESS_3;
  } else if (mode == 4){
    INA260_i2caddr = INA260_ADDRESS_4;
  } else if (mode == 5){
    INA260_i2caddr = INA260_ADDRESS_5;
  } else if (mode == 6){
    INA260_i2caddr = INA260_ADDRESS_6;
  }
}

/**************************************************************************/
/*! 
    @brief  Set EPS internal I2C bus as the bus used by the INA260
*/
/**************************************************************************/
void INA260::setI2C(SoftwareWire i2c_bus)
{
  eps_wire = i2c_bus;                                 // I2C Bus
}

/**************************************************************************/
/*! 
    @brief  Write I2C
*/
/**************************************************************************/
bool INA260::wireWriteRegister (uint8_t reg, uint16_t value)
{
  int success = -1;

  // I2C Writing to INA260
  eps_wire.beginTransmission(INA260_i2caddr);
  eps_wire.write(reg);                       			// Register to write
  eps_wire.write((value >> 8) & 0xFF);       			// Upper 8-bits
  eps_wire.write(value & 0xFF);              			// Lower 8-bits
  success = eps_wire.endTransmission();

  if (success == 0){									            // If success == 0, the transmission was executed successfully
    return true;
  } else {
    return false;
  }
}

/**************************************************************************/
/*! 
    @brief  Read I2C
*/
/**************************************************************************/
void INA260::wireReadRegister(uint8_t reg, uint16_t *value)
{
  // I2C Reading from INA260
  eps_wire.beginTransmission(INA260_i2caddr);
  eps_wire.write(reg);                       			// Register to read
  eps_wire.endTransmission();
  
  delay(2); 											                // Max 12-bit conversion time is 1.1ms per sample (default mode)

  eps_wire.requestFrom(INA260_i2caddr, (uint8_t)2);  
  *value = ((eps_wire.read() << 8) | eps_wire.read());
}

/**************************************************************************/
/*! 
    @brief  Set de Configuration Register de INA260
*/
/**************************************************************************/
bool INA260::INA260SetConfig(void){
  bool state = false;
  
  // Definition of the INA260 Configuration register (Avg: 64; V_CT: 1.1ms; C_CT: 1.1ms; Mode: Continuous voltage & current)
  uint16_t config = INA260_CONFIG_AVG1 | INA260_CONFIG_AVG0 | INA260_CONFIG_VBUS_CT2 | INA260_CONFIG_CSH_CT2 |
                    INA260_CONFIG_MODE_2 | INA260_CONFIG_MODE_1 | INA260_CONFIG_MODE_0;

  state = wireWriteRegister(INA260_REG_CONFIG, config);
  return state;
}

/**************************************************************************/
/*! 
    @brief  Initialization of the INA260
*/
/**************************************************************************/
bool INA260::begin() {   
  bool state = false; 
  state = INA260SetConfig();
  return state;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw bus voltage (16-bit signed integer, range [-9584,9584])
*/
/**************************************************************************/
int16_t INA260::getBusVoltage_raw(void) {
  uint16_t value;
  wireReadRegister(INA260_REG_BUSVOLTAGE, &value);
  return (int16_t) value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw shunt current (16-bit signed integer, range [-10000,10000])
*/
/**************************************************************************/
int16_t INA260::getCurrent_raw(void) {
  uint16_t value;
  wireReadRegister(INA260_REG_CURRENT, &value);
  return (int16_t) value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw shunt voltage (16-bit signed integer, range [0,14975])
*/
/**************************************************************************/
int16_t INA260::getPower_raw(void) {
  uint16_t value;
  wireReadRegister(INA260_REG_POWER, &value);
  return (int16_t) value;
}

/**************************************************************************/
/*! 
    @brief  Gets the bus voltage in volts.
    INA260 Bus voltage register has 16 bits. Data returned by INA260
    ranges from -9584 to 9584. The measurement resolution is 1.25mV.
    To get value in mV: 9584*1.25mV = 11,980mV
    To get value in V: 9584*1.25mV/1000 = 11.98V
*/
/**************************************************************************/
float INA260::getBusVoltage_V(int channel) {
  int16_t value = getBusVoltage_raw();
  return value * 0.00125;
}

/**************************************************************************/
/*! 
    @brief  Gets the current value in mA.
    INA260 Shunt current register has 16 bits. Data returned by INA260
    ranges from -10000 to 10000. The measurement resolution is 1.25mA.
    To get value in mA: 10000*1.25mA = 12,500mA
*/
/**************************************************************************/
float INA260::getCurrent_mA(int channel) {
  int16_t value = getCurrent_raw();
  return value * 1.25;
}

/**************************************************************************/
/*! 
    @brief  Gets the power value in mW.
    INA260 Power register has 16 bits. Data returned by INA260
    ranges from 0 to 14975. The measurement resolution is 10mW.
    To get value in mW: 14975*10mW = 149,750mW
*/
/**************************************************************************/
float INA260::getPower_mW(void) {
  int16_t value = getPower_raw();
  return value * 10;
}