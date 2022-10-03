/***************************************************************************
  This is the Software for the Attitude Determination and Control System
  (ADCS) for the Quetzal-1 satellite, Guatemalas first satellite.
  
  Written to work specifically with the ADCS Printed Circuit Board (PCB).
  See our github for more info!
  ------> https://github.com/Quetzal-1-CubeSat-Team

  Quetzal-1 was a 1U CubeSat developed by an engineering team from
  Universidad del Valle de Guatemala (UVG). The satellite was deployed
  from the International Space Station's (ISS) KiboCUBE module, on April
  28, 2020, and operated succesfully in space from the day of deployment
  to November of the same year. This amounted to 211 days of operation,
  which validated the performance of all systems on-board.
 ***************************************************************************/

#ifndef __ADCS_H__
#define __ADCS_H__

// I2C parameters
#define SLAVE_ADDR          0xAA
#define BNO_ADDR            0x55
#define TMP100_ADDR         0x48    // 0x48 = A0: LOW, A1: LOW.
#define ADC1_ADDR           0x1D    // 0x1D = A0: LOW, A1: LOW.
#define ADC2_ADDR           0x1F    // 0x1F = A0: HIGH, A1: LOW.

#define ADCS_READY          0x03

#define GYROSCOPES          0x05
#define MAGNETOMETERS       0x07
#define PHOTODIODES_1       0x09
#define PHOTODIODES_2       0x0A
#define TEMPERATURE         0X0B
#define TMP100_DATA         0X0C
#define SEND_TRANS_FLAGS    0x0D

// All data
#define COLLECT_DATA        0x0E
#define SEND_DATA           0x0F
#define RESET               0x11
#define ADM_RESET           0x12

// I2C answers
#define OK                  0x4B
#define NO                  0x4E
#define YES                 0x59

#endif
