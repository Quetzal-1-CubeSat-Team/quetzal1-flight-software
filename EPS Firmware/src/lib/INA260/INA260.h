/*
 * Title: INA260.h
 * Copyright (c) 2012, Adafruit Industries
 * Modified by: Aldo Aguilar-Nadalini 
 * Date: May 7th, 2019
 * Description: Headers for the INA260 V/I Monitor library
 */

// Imports
#include "Arduino.h"
#include <SoftwareWire.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define INA260_ADDRESS_1                        0x40       // Connect A1 to GND and A0 to GND
    #define INA260_ADDRESS_2                        0x41       // Connect A1 to GND and A0 to VS
    #define INA260_ADDRESS_3                        0x42       // Connect A1 to GND and A0 to SDA
    #define INA260_ADDRESS_4                        0x44       // Connect A1 to GND and A0 to SCL
    #define INA260_ADDRESS_5                        0x44       // Connect A1 to VS and A0 to GND
    #define INA260_ADDRESS_6                        0x45       // Connect A1 to VS and A0 to VS

    #define INA260_READ                            (0x01)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER (R/W)
    -----------------------------------------------------------------------*/
    #define INA260_REG_CONFIG                      (0x00)
    /*---------------------------------------------------------------------*/
    #define INA260_CONFIG_RESET                    (0x8000)  // Reset Bit
  
    #define INA260_CONFIG_AVG2                     (0x0800)  // AVG Samples Bit 2 - See table 5 spec
    #define INA260_CONFIG_AVG1                     (0x0400)  // AVG Samples Bit 1 - See table 5 spec
    #define INA260_CONFIG_AVG0                     (0x0200)  // AVG Samples Bit 0 - See table 5 spec

    #define INA260_CONFIG_VBUS_CT2                 (0x0100)  // VBUS bit 2 Conversion time - See table 5 spec
    #define INA260_CONFIG_VBUS_CT1                 (0x0080)  // VBUS bit 1 Conversion time - See table 5 spec
    #define INA260_CONFIG_VBUS_CT0                 (0x0040)  // VBUS bit 0 Conversion time - See table 5 spec

    #define INA260_CONFIG_CSH_CT2                  (0x0020)  // Cshunt bit 2 Conversion time - See table 5 spec
    #define INA260_CONFIG_CSH_CT1                  (0x0010)  // Cshunt bit 1 Conversion time - See table 5 spec
    #define INA260_CONFIG_CSH_CT0                  (0x0008)  // Cshunt bit 0 Conversion time - See table 5 spec

    #define INA260_CONFIG_MODE_2                   (0x0004)  // Operating Mode bit 2 - See table 5 spec
    #define INA260_CONFIG_MODE_1                   (0x0002)  // Operating Mode bit 1 - See table 5 spec
    #define INA260_CONFIG_MODE_0                   (0x0001)  // Operating Mode bit 0 - See table 5 spec

/*=========================================================================*/

/*=========================================================================
    CURRENT REGISTER (R)
    -----------------------------------------------------------------------*/
    #define INA260_REG_CURRENT                     (0x01)
/*=========================================================================*/

/*=========================================================================
    BUS VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
    #define INA260_REG_BUSVOLTAGE                  (0x02)
/*=========================================================================*/

/*=========================================================================
    POWER VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
    #define INA260_REG_POWER                       (0x03)
/*=========================================================================*/

class INA260{
 public:
  INA260(int mode);
  bool begin();
  float getBusVoltage_V(int channel);
  float getCurrent_mA(int channel);
  float getPower_mW(void);
  void setI2C(SoftwareWire i2c_bus);

 private:
  uint8_t INA260_i2caddr;
  SoftwareWire eps_wire;

  bool wireWriteRegister(uint8_t reg, uint16_t value);
  void wireReadRegister(uint8_t reg, uint16_t *value);
  bool INA260SetConfig(void);
  int16_t getBusVoltage_raw(void);
  int16_t getCurrent_raw(void);
  int16_t getPower_raw(void);
};