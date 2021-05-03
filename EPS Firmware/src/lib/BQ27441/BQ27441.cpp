/*
 * Title: BQ27441.cpp
 * Originally from: Jim Lindblom @ SparkFun Electronics (Copyright (c) 2016 SparkFun Electronics, Inc.)
 * Available online at: https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library
 * Modified by: Aldo Aguilar-Nadalini 
 * Date: December 16th, 2018
 * Description: BQ27441 Battery Gauge library
 */

// Imports
#include "Arduino.h"
#include <BQ27441.h>
#include <SoftwareWire.h>

// Global variables
bool _userConfigControl = false;
bool _sealFlag = false;

/**************************************************************************/
/*! 
    @brief  CONSTRUCTOR BQ27441 Class
*/
/**************************************************************************/
BQ27441::BQ27441()
{
  BQ27441_i2caddr = BQ27441_ADDRESS;
}

/**************************************************************************/
/*! 
    @brief  Set EPS internal I2C bus as the bus used by the BQ27441
*/
/**************************************************************************/
void BQ27441::setI2C(SoftwareWire i2c_bus)
{
  eps_wire = i2c_bus;
}

/**************************************************************************/
/*! 
    @brief  Read WORD I2C
*/
/**************************************************************************/
uint16_t BQ27441::readWord(uint16_t subAddress)
{
  uint8_t data[2];
  i2cReadBytes(subAddress, data, 2);
  return ((uint16_t) data[1] << 8) | data[0];
}

/**************************************************************************/
/*! 
    @brief  Write I2C
*/
/**************************************************************************/
bool BQ27441::i2cWriteBytes (uint16_t subAddress, uint8_t *source, uint8_t count)
{
  eps_wire.beginTransmission(BQ27441_i2caddr);
  eps_wire.write(subAddress);
  for (int i=0; i<count; i++)
  {
    eps_wire.write(source[i]);
  }
  eps_wire.endTransmission(true);

  return true;
}

/**************************************************************************/
/*! 
    @brief  Read I2C
*/
/**************************************************************************/
int16_t BQ27441::i2cReadBytes(uint16_t subAddress, uint8_t *value, uint8_t count)
{
  int16_t timeout = BQ27441_I2C_TIMEOUT;
  eps_wire.beginTransmission(BQ27441_i2caddr);
  eps_wire.write(subAddress);
  eps_wire.endTransmission(true);

  eps_wire.requestFrom(BQ27441_i2caddr, count);
  while ((eps_wire.available() < count) && timeout--)
  {
    delay(1);  
  }
  if (timeout)
  {
    for (int i = 0; i < count; i++)
    {
      value[i] = eps_wire.read();
    }
  }

  return timeout;
}

// Initializes I2C and verifies communication with the BQ27441.
bool BQ27441::begin() {    
  uint16_t deviceID = 0;
  deviceID = deviceType();            // Read deviceType from BQ27441
  if (deviceID == BQ27441_DEVICE_ID)
  {
    return true;                      // If device ID is valid, return true
  }
  return false;                       // Otherwise return false
}

// Read a 16-bit subcommand() from the BQ27441-G1A's control()
uint16_t BQ27441::readControlWord(uint16_t function)
{
  uint8_t subCommandMSB = (function >> 8);
  uint8_t subCommandLSB = (function & 0x00FF);
  uint8_t command[2] = {subCommandLSB, subCommandMSB};
  uint8_t data[2] = {0, 0};

  i2cWriteBytes((uint8_t) 0, command, 2);

  if (i2cReadBytes((uint8_t) 0, data, 2))
  {
    return ((uint16_t)data[1] << 8) | data[0];
  }

  return false;
}

// Read the device type - should be 0x0421
uint16_t BQ27441::deviceType(void)
{
  return readControlWord(BQ27441_CONTROL_DEVICE_TYPE);
}

// Set battery capacity
bool BQ27441::setCapacity(uint16_t capacity)
{
  // Write to STATE subclass (82) of BQ27441 extended memory.
  // Offset 0x0A (10)
  // Design capacity is a 2-byte piece of data - MSB first
  uint8_t capMSB = capacity >> 8;
  uint8_t capLSB = capacity & 0x00FF;
  uint8_t capacityData[2] = {capMSB, capLSB};
  return writeExtendedData(BQ27441_ID_STATE, 10, capacityData, 2);
}

// Write a specified number of bytes to extended data specifying a
// class ID, position offset.
bool BQ27441::writeExtendedData(uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len)
{
  if (len > 32)
    return false;

  if (!_userConfigControl) enterConfig(false);

  if (!blockDataControl())            // enable block data memory control
    return false;                     // Return false if enable fails
  if (!blockDataClass(classID))       // Write class ID using DataBlockClass()
    return false;

  blockDataOffset(offset / 32);       // Write 32-bit block offset (usually 0)
  computeBlockChecksum();             // Compute checksum going in
  uint8_t oldCsum = blockDataChecksum();

  // Write data bytes:
  for (int i = 0; i < len; i++)
  {
    // Write to offset, mod 32 if offset is greater than 32
    // The blockDataOffset above sets the 32-bit block
    writeBlockData((offset % 32) + i, data[i]);
  }

  // Write new checksum using BlockDataChecksum (0x60)
  uint8_t newCsum = computeBlockChecksum(); // Compute the new checksum
  writeBlockChecksum(newCsum);

  if (!_userConfigControl) exitConfig();

  return true;
}

// Enter configuration mode - set userControl if calling from an Arduino sketch
// and you want control over when to exitConfig
bool BQ27441::enterConfig(bool userControl)
{
  if (userControl) _userConfigControl = true;

  if (sealed())
  {
    _sealFlag = true;
    unseal();                         // Must be unsealed before making changes
  }

  if (executeControlWord(BQ27441_CONTROL_SET_CFGUPDATE))
  {
    int16_t timeout = BQ27441_I2C_TIMEOUT;
    while ((timeout--) && (!(Status() & BQ27441_FLAG_CFGUPMODE)))
      delay(1);

    if (timeout > 0)
      return true;
  }

  return false;
}

// Check if the BQ27441-G1A is sealed or not.
bool BQ27441::sealed(void)
{
  uint16_t stat = Status();
  return stat & BQ27441_STATUS_SS;
}

// UNseal the BQ27441-G1A
bool BQ27441::unseal(void)
{
  // To unseal the BQ27441, write the key to the control
  // command. Then immediately write the same key to control again.
  if (readControlWord(BQ27441_UNSEAL_KEY))
  {
    return readControlWord(BQ27441_UNSEAL_KEY);
  }
  return false;
}

// Execute a subcommand() from the BQ27441-G1A's control()
bool BQ27441::executeControlWord(uint16_t function)
{
  uint8_t subCommandMSB = (function >> 8);
  uint8_t subCommandLSB = (function & 0x00FF);
  uint8_t command[2] = {subCommandLSB, subCommandMSB};
  uint8_t data[2] = {0, 0};

  if (i2cWriteBytes((uint8_t) 0, command, 2))
    return true;

  return false;
}

// Issue a BlockDataControl() command to enable BlockData access
bool BQ27441::blockDataControl(void)
{
  uint8_t enableByte = 0x00;
  return i2cWriteBytes(BQ27441_EXTENDED_CONTROL, &enableByte, 1);
}

// Issue a DataClass() command to set the data class to be accessed
bool BQ27441::blockDataClass(uint8_t id)
{
  return i2cWriteBytes(BQ27441_EXTENDED_DATACLASS, &id, 1);
}

// Issue a DataBlock() command to set the data block to be accessed
bool BQ27441::blockDataOffset(uint8_t offset)
{
  return i2cWriteBytes(BQ27441_EXTENDED_DATABLOCK, &offset, 1);
}

// Read the current checksum using BlockDataCheckSum()
uint8_t BQ27441::blockDataChecksum(void)
{
  uint8_t csum;
  i2cReadBytes(BQ27441_EXTENDED_CHECKSUM, &csum, 1);
  return csum;
}

// Use BlockData() to write a byte to an offset of the loaded data
bool BQ27441::writeBlockData(uint8_t offset, uint8_t data)
{
  uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
  return i2cWriteBytes(address, &data, 1);
}

// Read all 32 bytes of the loaded extended data and compute a
// checksum based on the values.
uint8_t BQ27441::computeBlockChecksum(void)
{
  uint8_t data[32];
  i2cReadBytes(BQ27441_EXTENDED_BLOCKDATA, data, 32);

  uint8_t csum = 0;
  for (int i=0; i<32; i++)
  {
    csum += data[i];
  }
  csum = 255 - csum;

  return csum;
}

// Use the BlockDataCheckSum() command to write a checksum value
bool BQ27441::writeBlockChecksum(uint8_t csum)
{
  return i2cWriteBytes(BQ27441_EXTENDED_CHECKSUM, &csum, 1);
}

// Exit configuration mode with the option to perform a resimulation
bool BQ27441::exitConfig(bool resim)
{
  // There are two methods for exiting config mode:
  //    1. Execute the EXIT_CFGUPDATE command
  //    2. Execute the SOFT_RESET command
  // EXIT_CFGUPDATE exits config mode _without_ an OCV (open-circuit voltage)
  // measurement, and without resimulating to update unfiltered-SoC and SoC.
  // If a new OCV measurement or resimulation is desired, SOFT_RESET or
  // EXIT_RESIM should be used to exit config mode.
  if (resim)
  {
    if (softReset())
    {
      int16_t timeout = BQ27441_I2C_TIMEOUT;
      while ((timeout--) && ((flags() & BQ27441_FLAG_CFGUPMODE)))
        delay(1);
      if (timeout > 0)
      {
        if (_sealFlag) seal();        // Seal back up if we IC was sealed coming in
        return true;
      }
    }
    return false;
  }
  else
  {
    return executeControlWord(BQ27441_CONTROL_EXIT_CFGUPDATE);
  }
}

// Seal the BQ27441-G1A
bool BQ27441::seal(void)
{
  return readControlWord(BQ27441_CONTROL_SEALED);
}

// Read the flags() command
uint16_t BQ27441::flags(void)
{
  return readWord(BQ27441_COMMAND_FLAGS);
}

// Issue a soft-reset to the BQ27441-G1A
bool BQ27441::softReset(void)
{
  return executeControlWord(BQ27441_CONTROL_SOFT_RESET);
}

// Read the CONTROL_STATUS subcommand of control()
uint16_t BQ27441::Status(void)
{
  return readControlWord(BQ27441_CONTROL_STATUS);
}

// Reads and returns specified state of charge measurement
uint16_t BQ27441::soc(soc_measure type)
{
  uint16_t socRet = 0;
  switch (type)
  {
  case FILTERED:
    socRet = readWord(BQ27441_COMMAND_SOC);
    break;
  case UNFILTERED:
    socRet = readWord(BQ27441_COMMAND_SOC_UNFL);
    break;
  }

  return socRet;
}

// Reads and returns the battery voltage
uint16_t BQ27441::voltage(void)
{
  return readWord(BQ27441_COMMAND_VOLTAGE);
}

// Reads and returns the specified current measurement
int16_t BQ27441::current(current_measure type)
{
  int16_t current = 0;
  switch (type)
  {
  case AVG:
    current = (int16_t) readWord(BQ27441_COMMAND_AVG_CURRENT);
    break;
  case STBY:
    current = (int16_t) readWord(BQ27441_COMMAND_STDBY_CURRENT);
    break;
  case MAX:
    current = (int16_t) readWord(BQ27441_COMMAND_MAX_CURRENT);
    break;
  }

  return current;
}

// Reads and returns the specified capacity measurement
uint16_t BQ27441::capacity(capacity_measure type)
{
  uint16_t capacity = 0;
  switch (type)
  {
  case REMAIN:
    return readWord(BQ27441_COMMAND_REM_CAPACITY);
    break;
  case FULL:
    return readWord(BQ27441_COMMAND_FULL_CAPACITY);
    break;
  case AVAIL:
    capacity = readWord(BQ27441_COMMAND_NOM_CAPACITY);
    break;
  case AVAIL_FULL:
    capacity = readWord(BQ27441_COMMAND_AVAIL_CAPACITY);
    break;
  case REMAIN_F:
    capacity = readWord(BQ27441_COMMAND_REM_CAP_FIL);
    break;
  case REMAIN_UF:
    capacity = readWord(BQ27441_COMMAND_REM_CAP_UNFL);
    break;
  case FULL_F:
    capacity = readWord(BQ27441_COMMAND_FULL_CAP_FIL);
    break;
  case FULL_UF:
    capacity = readWord(BQ27441_COMMAND_FULL_CAP_UNFL);
    break;
  case DESIGN:
    capacity = readWord(BQ27441_EXTENDED_CAPACITY);
  }

  return capacity;
}

// Reads and returns measured average power
int16_t BQ27441::power(void)
{
  return (int16_t) readWord(BQ27441_COMMAND_AVG_POWER);
}

// Reads and returns specified state of health measurement
uint8_t BQ27441::soh(soh_measure type)
{
  uint16_t sohRaw = readWord(BQ27441_COMMAND_SOH);
  uint8_t sohStatus = sohRaw >> 8;
  uint8_t sohPercent = sohRaw & 0x00FF;

  if (type == PERCENT)
    return sohPercent;
  else
    return sohStatus;
}
