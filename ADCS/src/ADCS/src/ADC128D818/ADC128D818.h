/**
 * @brief ADC128D818 12-Bit, 8-Channel, ADC System Monitor w/ Temp Sensor, Internal/External Reference, & I2C Interface
 * @author bryanduxbury
 * https://github.com/bryanduxbury/adc128d818_driver
 * Modified by: J.P. Cahueque, D. Alvarez and A. Ixtecoc (Universidad del Valle de Guatemala)
 * Modification Date: 2018
 */
#ifndef __ADC128D818_H__
#define __ADC128D818_H__

#include "Arduino.h"
#include "../SoftwareWire/SoftwareWire.h"

enum ADC_REG {
    CONFIG_REG  = 0x00,
    INTERRUPT_STATUS_REG = 0x01,
    INTERRUPT_MASK_REG = 0x03,
    CONV_RATE_REG = 0x07,
    CHANNEL_DISABLE_REG = 0x08,
    ONE_SHOT_REG = 0x09,
    DEEP_SHUTDOWN_REG = 0x0A,
    ADV_CONFIG_REG = 0x0B,
    BUSY_STATUS_REG = 0x0C,
    CHANNEL_READINGS_REG = 0x20,
    LIMIT_REG = 0x2A,
    MANUFACTURER_ID_REG = 0x3E,
    REVISION_ID_REG = 0x3F,
    DUMMY_CMD = 0x70
};

enum reference_mode_t {
  INTERNAL_REF = 0,
  EXTERNAL_REF = 1
};

enum conv_mode_t {
  LOW_POWER,
  CONTINUOUS,
  ONE_SHOT
};

enum operation_mode_t {
  SINGLE_ENDED_WITH_TEMP = 0,
  SINGLE_ENDED = 1,
  DIFFERENTIAL = 2,
  MIXED = 3
};

enum ADC_ENABLE {
    ENABLE_IN0 = (uint8_t)~(0x01 <<0),
    ENABLE_IN1 = (uint8_t)~(0x01 <<1),
    ENABLE_IN2 = (uint8_t)~(0x01 <<2),
    ENABLE_IN3 = (uint8_t)~(0x01 <<3),
    ENABLE_IN4 = (uint8_t)~(0x01 <<4),
    ENABLE_IN5 = (uint8_t)~(0x01 <<5),
    ENABLE_IN6 = (uint8_t)~(0x01 <<6),
    ENABLE_IN7 = (uint8_t)~(0x01 <<7),
    ENABLE_TEMP = (uint8_t)~(0x01 <<7),
    ENABLE_ALL = (uint8_t)0x00
};

class ADC128D818 {
public:
  /**
   * @brief Constructor.
   *
   * @param address
   */
  ADC128D818(uint8_t address);

  /**
   * @brief setReference
   *
   * @param ref_voltage
   */
  void setReference(double ref_voltage);

  /**
   * @brief setReferenceMode
   *
   * @param reference_mode_t mode
   */
  void setReferenceMode(reference_mode_t mode);

  /**
   * @brief setOperationMode
   *
   * @param operation_mode_t mode
   */
  void setOperationMode(operation_mode_t mode);

  /**
   * @brief setDisabledMask
   *
   * @param disabled_mask
   */
  void setDisabledMask(uint8_t disabled_mask);

  /**
   * @brief ConversionMode
   *
   * @param conv_mode_t mode
   */
  void setConversionMode(conv_mode_t mode);

  /**
   * @brief begin
   *
   * @return byte
   */
  bool begin_shutdown();

  bool begin();


  /**
   * @brief testComm
   * 
   * @return bool, true if no errors.
   */
  bool testComm();

  /**
   * @brief read
   *
   * @param channel
   * @return uint16_t
   */
  uint16_t read(uint8_t channel);

  /**
   * @brief readConverted
   *
   * @param channel
   * @return double value
   */
  double readConverted(uint8_t channel);

  /**
   * @brief readTemperatureConverted
   *
   * @return double value
   */
  double readTemperatureConverted();

  /**
   * @brief setI2C
   */
  void setI2C(SoftwareWire i2c_bus);

private:
  uint8_t addr;
  uint8_t disabled_mask;
  double ref_v;
  SoftwareWire adcs_wire;

  reference_mode_t ref_mode;
  operation_mode_t op_mode;
  conv_mode_t conv_mode;

public:
  /**
   * @brief setRegisterAddress
   *
   * @param reg_addr
   * @return byte
   */
  byte setRegisterAddress(uint8_t reg_addr);
  void setRegister(uint8_t reg_addr, uint8_t value);
  uint8_t readCurrentRegister8();
};

#endif
