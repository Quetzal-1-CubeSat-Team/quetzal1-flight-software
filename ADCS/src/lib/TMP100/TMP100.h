/*
 * Titulo: TMP100.h
 * Autor: Aldo Stefano Aguilar Nadalini 15170
 * Fecha: 16 de diciembre de 2018
 * Descripcion: Headers para programa que ejecuta funcionamiento de TMP100´s conectados a uC de Modulo EPS
 */

// Imports
#include "Arduino.h"
#include <SoftwareWire.h>

#define TMP100_ADDRESS_1            0x48				  // Conectar ADD1 a GND y ADD0 a GND
#define TMP100_ADDRESS_2            0x4A				  // Conectar ADD1 a GND y ADD0 a VCC
#define TMP100_ADDRESS_3            0x49          // Conectar ADD0 a float (TMP101 TEST)

// General Constants //
///////////////////////
#define TMP100_TEMPERATURE_REGISTER 0x00
#define TMP100_CONFIG_REGISTER		  0x01
#define TMP100_TLOW_REGISTER		    0x02
#define TMP100_THIGH_REGISTER		    0x03
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
  SoftwareWire adcs_wire;

  bool i2cReadBytes(uint8_t pointer, int16_t *value, uint8_t count);
  bool i2cWriteBytes (uint8_t pointer, uint8_t value);
  void openPointerRegister(byte pointerReg);            // Changes the pointer register
  byte readRegister(bool registerNumber);	              // reads 1 byte of from register
};