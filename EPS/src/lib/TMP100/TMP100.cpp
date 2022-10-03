/*
 * Title: TMP100.cpp
 * Originally from: Alex Wende @ SparkFun Electronics
 * Available online at: https://github.com/sparkfun/SparkFun_TMP102_Arduino_Library
 * Modified by: Aldo Aguilar-Nadalini 
 * Modification Date: December 16th, 2018
 * Description: TMP100 Thermal Sensor library
 */

// Imports
#include "Arduino.h"
#include <TMP100.h>
#include <SoftwareWire.h>

/**************************************************************************/
/*! 
    @brief  CONSTRUCTOR TMP100 Class
*/
/**************************************************************************/
TMP100::TMP100(int mode)
{
  if (mode == 0){
    TMP100_i2caddr = TMP100_ADDRESS_1;
  } else if (mode == 1){
    TMP100_i2caddr = TMP100_ADDRESS_2;
  } else if (mode == 3){
    TMP100_i2caddr = TMP100_ADDRESS_3;
  }
}

/**************************************************************************/
/*! 
    @brief  Set EPS internal I2C bus as the bus used by the TMP100
*/
/**************************************************************************/
void TMP100::setI2C(SoftwareWire i2c_bus)
{
  eps_wire = i2c_bus;
}

/**************************************************************************/
/*! 
    @brief  Write I2C
*/
/**************************************************************************/
bool TMP100::i2cWriteBytes (uint8_t pointer, uint8_t value)
{
  eps_wire.beginTransmission(TMP100_i2caddr);
  eps_wire.write(pointer);
  eps_wire.write(value);
  eps_wire.endTransmission();
  return true;
}

/**************************************************************************/
/*! 
    @brief  Read I2C
*/
/**************************************************************************/
bool TMP100::i2cReadBytes(uint8_t pointer, int16_t *value, uint8_t count)
{
  // Verification variables
  int transmission = -1;
  int reception = -1;

  int16_t timeout = TMP100_I2C_TIMEOUT;
  eps_wire.beginTransmission(TMP100_i2caddr);
  eps_wire.write(pointer);
  transmission = eps_wire.endTransmission();

  reception = eps_wire.requestFrom(TMP100_i2caddr, count);
  while ((eps_wire.available() < count) && timeout--)
  {
    delay(1);  
  }

  if (timeout)
  {
    *value = ((eps_wire.read() << 8) | eps_wire.read());
  }

  // Transmission check (If transmission is 0, write was success. If 2 bytes are received, read was success)
  if ((transmission == 0) && (reception == count)){
    return true;                                          // Transmission-reception success
  } else {
    return false;
  }
}

// Initializes I2C and verifies communication with the TMP100.
bool TMP100::begin()
{    
  int16_t value;
  if (i2cReadBytes(TMP100_TEMPERATURE_REGISTER, &value, 2))
  {
    return true; // If device is connected, return true
  }
  return false; // Otherwise return false
}

/*************************** LIBRARY TMP100 FUNCTIONS ***************************/

// Function to point to specific register
void TMP100::openPointerRegister(uint8_t pointerReg)
{ 
  eps_wire.beginTransmission(TMP100_i2caddr); // Connect to TMP100
  eps_wire.write(pointerReg);                 // Open specified register
  eps_wire.endTransmission();                 // Close communication with TMP100
}

// Function to read specific register
uint8_t TMP100::readRegister(bool registerNumber){  
  uint8_t registerByte[2];                    
  
  // Read current configuration register value
  eps_wire.requestFrom(TMP100_i2caddr, (uint8_t) 2);   // Read two bytes from TMP100
  registerByte[0] = (eps_wire.read());       // Read first byte
  registerByte[1] = (eps_wire.read());       // Read second byte
  
  return registerByte[registerNumber];
}

// Function to wake up TMP100
bool TMP100::wakeup(void)
{
  uint8_t registerByte;

  // Change pointer address to configuration register (1)
  openPointerRegister(TMP100_CONFIG_REGISTER);
  
  // Read current configuration register value
  registerByte = readRegister(0);
  registerByte &= 0xFE;                                       // Clear SD (bit 0 of first byte)

  // Set configuration registers
  eps_wire.beginTransmission(TMP100_i2caddr);
  eps_wire.write(TMP100_CONFIG_REGISTER);                     // Point to configuration register
  eps_wire.write(registerByte);                               // Write first byte
  int transmission = eps_wire.endTransmission();              // Close communication with TMP100

  // Succesful transmission
  if(transmission == 0){
    return true;
  } else {
    return false;
  }
}

// Function to shutdown up TMP100
void TMP100::sleep(void)
{
  uint8_t registerByte;

  // Change pointer address to configuration register (0x01)
  openPointerRegister(TMP100_CONFIG_REGISTER);
  
  // Read current configuration register value
  registerByte = readRegister(0);
  registerByte |= 0x01;                       // Set SD (bit 0 of first byte)

  // Set configuration register
  eps_wire.beginTransmission(TMP100_i2caddr);
  eps_wire.write(TMP100_CONFIG_REGISTER);         // Point to configuration register
  eps_wire.write(registerByte);            // Write first byte
  eps_wire.endTransmission();              // Close communication with TMP100
}

// Reads and returns specified
float TMP100::readTempC(void)
{
  uint8_t registerByte[2];  // Store the data from the register here
  int16_t digitalTemp;  // Temperature stored in TMP100 register
  
  // Read Temperature
  // Change pointer address to temperature register (0)
  openPointerRegister(TMP100_TEMPERATURE_REGISTER);
  // Read from temperature register
  registerByte[0] = readRegister(0);
  registerByte[1] = readRegister(1);

  // Bit 0 of second byte will always be 0 in 12-bit readings and 1 in 13-bit
  if(registerByte[1]&0x01)  // 13 bit mode
  {
  // Combine bytes to create a signed int
    digitalTemp = ((registerByte[0]) << 5) | (registerByte[1] >> 3);
  // Temperature data can be + or -, if it should be negative,
  // convert 13 bit to 16 bit and use the 2s compliment.
    if(digitalTemp > 0xFFF)
  {
      digitalTemp |= 0xE000;
    }
  }
  else  // 12 bit mode
  {
  // Combine bytes to create a signed int 
    digitalTemp = ((registerByte[0]) << 4) | (registerByte[1] >> 4);
  // Temperature data can be + or -, if it should be negative,
  // convert 12 bit to 16 bit and use the 2s compliment.
    if(digitalTemp > 0x7FF)
  {
      digitalTemp |= 0xF000;
    }
  }
  // Convert digital reading to analog temperature (1-bit is equal to 0.0625 C)
  return digitalTemp*0.0625;
}
