/**
 * @brief ADC128D818 12-Bit, 8-Channel, ADC System Monitor w/ Temp Sensor, Internal/External Reference, & I2C Interface
 * @author bryanduxbury
 * https://github.com/bryanduxbury/adc128d818_driver
 * Modified by: J.P. Cahueque, D. Alvarez and A. Ixtecoc (Universidad del Valle de Guatemala)
 * Modification Date: 2018
 */

#include "ADC128D818.h"

#define BUSY_BIT 0
#define NOT_READY_BIT 1

ADC128D818::ADC128D818(uint8_t address) {
  this->addr = address;

  /*
    External voltage reference of 3.3V
    SINGLE_ENDED: 8 channels to read
    ONE_SHOT: The One-Shot register is used to initiate a single conversion and
    comparison cycle when the device is in shutdown mode or deep shutdown mode,
    after which the device returns to the respective mode it was in. The obvious
    advantage of using this mode is lower power consumption because the device
    is operating in shutdown or deep shutdown mode. This register is not a data
    register, and it is the write operation that causes the one-shot conversion.
    The data written to this address is irrelevant and is not stored. A zero will
    always be read from this register.
  */
  ref_v = 3.3f;
  ref_mode = EXTERNAL_REF;
  op_mode = SINGLE_ENDED;
  conv_mode = CONTINUOUS;
  // Enable all except channel 7 and 8. Check table 8 Address 08h
  disabled_mask = 0xC0;
  /*
  ref_v = 2.56f;
  ref_mode = INTERNAL_REF;
  op_mode = SINGLE_ENDED_WITH_TEMP;
  conv_mode = CONTINUOUS;
  disabled_mask = ENABLE_IN0 & ENABLE_TEMP;
  */
}

void ADC128D818::setReference(double ref_voltage) {
  ref_v = ref_voltage;
}

void ADC128D818::setReferenceMode(reference_mode_t mode) {
  ref_mode = mode;
}

void ADC128D818::setOperationMode(operation_mode_t mode) {
  op_mode = mode;
}

void ADC128D818::setConversionMode(conv_mode_t mode) {
  conv_mode = mode;
}

void ADC128D818::setDisabledMask(uint8_t disabled_mask) {
  this->disabled_mask = disabled_mask;
}

bool ADC128D818::begin_shutdown() {
  // Read busy reg until it returns 0
  uint8_t ans;

  ans = setRegisterAddress(BUSY_STATUS_REG);
  if (ans != 0) {
    return false;
  }
  while (1) {
    //Serial.println("waiting for ready bit unset");
    if ((readCurrentRegister8() & (1 << NOT_READY_BIT)) == 0) {
      break;
    }
    delay(35);
  }

  //Serial.println("made it out");

  // Program advanced config reg
  setRegister(ADV_CONFIG_REG, ref_mode | (op_mode << 1));

  // Program conversion rate reg
  setRegister(CONV_RATE_REG, conv_mode);

  // Program enabled channels
  setRegister(CHANNEL_DISABLE_REG, disabled_mask);

  // Program limit regs
  // Currently noop!

  // Shutdown mode
  setRegister(CONFIG_REG, 0);
  setRegister(DEEP_SHUTDOWN_REG, 1);
  return true;
}

bool ADC128D818::begin() {
  // read busy reg until it returns 0
  setRegisterAddress(BUSY_STATUS_REG);
  while (1) {
    //Serial.println("waiting for ready bit unset");
    if ((readCurrentRegister8() & (1 << NOT_READY_BIT)) == 0) {
      break;
    }
    delay(35);
  }
  
  //Serial.println("made it out");

  // program advanced config reg
  setRegister(ADV_CONFIG_REG, ref_mode | (op_mode << 1));

  // program conversion rate reg
  setRegister(CONV_RATE_REG, conv_mode);

  // program enabled channels
  setRegister(CHANNEL_DISABLE_REG, disabled_mask);

  // program limit regs
  // currently noop!

  // set start bit in configuration (interrupts disabled)
  setRegister(CONFIG_REG, 1);

  return true;
}

bool ADC128D818::testComm() {
  uint8_t ans;
  adcs_wire.beginTransmission(addr);
  adcs_wire.write(DUMMY_CMD);
  ans = adcs_wire.endTransmission();

  if(ans == 0) {
    return true;
  }
  else return false;
}

uint16_t ADC128D818::read(uint8_t channel) {
  //Set the current channel address to be read
  setRegisterAddress(CHANNEL_READINGS_REG + channel);
  //Wait until the connection is stable
  adcs_wire.requestFrom(addr, (uint8_t)2);
  while (!adcs_wire.available()) {
    delay(1);
  }
  //Receive the values read by the channel
  uint8_t high_byte = adcs_wire.read();
  uint8_t low_byte = adcs_wire.read();

  //Get the value in only one variable
  uint16_t result = ((((uint16_t)high_byte) << 8) | ((uint16_t)low_byte));
  //Perform a bit shift to the value
  result = result >>4;
  return result;
}

double ADC128D818::readConverted(uint8_t channel) {
  //Make the conversion to get the real number
  return ((double)read(channel) )/ 4095.0 * ref_v;
}

double ADC128D818::readTemperatureConverted() {
  //Set the current channel address to be read
  setRegisterAddress(CHANNEL_READINGS_REG + 7);
  //Wait until the connection is stable
  adcs_wire.requestFrom(addr, (uint8_t)2);
  while (!adcs_wire.available()) {
    delay(1);
  }
  uint8_t high_byte = adcs_wire.read();
  uint8_t low_byte = adcs_wire.read();

  uint16_t result = ((((uint16_t)high_byte) << 8) | ((uint16_t)low_byte));
  result = result >> 7; // get only 9 bits

  if (result >> 8 == 0) {
    return (double)result / 2.0;
  } else {
    return -(double)(512 - result) / 2.0;
  }
}

/**************************************************************************/
/*! 
    @brief  Software I2C Setup
*/
/**************************************************************************/
void ADC128D818::setI2C(SoftwareWire i2c_bus)
{
  adcs_wire = i2c_bus;
}


//
// private methods
//

/*
returns:
0:success
1:data too long to fit in transmit buffer
2:received NACK on transmit of address
3:received NACK on transmit of data
4:other error
*/
byte ADC128D818::setRegisterAddress(uint8_t reg_addr) {
  adcs_wire.beginTransmission(addr);
  adcs_wire.write(reg_addr);
  return adcs_wire.endTransmission();
}

void ADC128D818::setRegister(uint8_t reg_addr, uint8_t value) {
  adcs_wire.beginTransmission(addr);
  adcs_wire.write(reg_addr);
  adcs_wire.write(value);
  adcs_wire.endTransmission();
}

uint8_t ADC128D818::readCurrentRegister8() {
  adcs_wire.requestFrom(addr, (uint8_t)1);
  while (!adcs_wire.available()) {
    delay(1);
  }
  return adcs_wire.read();
}
