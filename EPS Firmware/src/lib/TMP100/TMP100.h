/*
 * Title: TMP100.h
 * Originally from: Alex Wende @ SparkFun Electronics
 * Available online at: https://github.com/sparkfun/SparkFun_TMP102_Arduino_Library
 * Modified by: Aldo Aguilar-Nadalini 
 * Date: December 16th, 2018
 * Description: TMP100 Battery Thermal Sensor library headers
 */

// Imports
#include "Arduino.h"
#include <SoftwareWire.h>

#define TMP100_ADDRESS_1               0x48				      // Connect ADD1 pin to GND and ADD0 pin to GND
#define TMP100_ADDRESS_2               0x4A				      // Connect ADD1 pin to GND and ADD0 pin to VCC
#define TMP100_ADDRESS_3               0x49             // Connect ADD0 pin to float (TMP100 TEST)

// General Constants //
///////////////////////
#define TMP100_TEMPERATURE_REGISTER    0x00
#define TMP100_CONFIG_REGISTER		   0x01
#define TMP100_TLOW_REGISTER		   0x02
#define TMP100_THIGH_REGISTER		   0x03
#define TMP100_I2C_TIMEOUT			   	500

class TMP100{
 public:
 
  TMP100(int mode);
  bool begin(void);
  void sleep(void);	                                    // Switch sensor to low power mode
  bool wakeup(void);	                                  // Wakeup and start running in normal power mode
  float readTempC(void);
  void setI2C(SoftwareWire i2c_bus);

 private:
  uint8_t TMP100_i2caddr;
  int16_t temp_data;
  SoftwareWire eps_wire;

  bool i2cReadBytes(uint8_t pointer, int16_t *value, uint8_t count);
  bool i2cWriteBytes (uint8_t pointer, uint8_t value);
  void openPointerRegister(byte pointerReg);            // Changes the pointer register
  byte readRegister(bool registerNumber);	              // reads 1 byte of from register
};