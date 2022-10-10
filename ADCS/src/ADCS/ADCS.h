/***************************************************************************
  This is the Software for the Attitude Determination and Control System
  (ADCS) for the Quetzal-1 satellite, Guatemala's first satellite.

  Copyright (C) 2022  Universidad del Valle de Guatemala

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
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
