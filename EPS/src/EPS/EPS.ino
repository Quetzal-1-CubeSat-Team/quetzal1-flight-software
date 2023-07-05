/***************************************************************************
  This is the Software for the Electrical Power System (EPS) for the
  Quetzal-1 satellite, Guatemala's first satellite.

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

/* Written by Aldo-Aguilar Nadalini, with the support of Quetzal-1 team members */

// general libraries
#include <Wire.h>
#include <avr/power.h>
#include <avr/sleep.h>
// local libraries
#include "EPS.h"
#include "src/SoftwareWire/SoftwareWire.h"
#include "src/INA260/INA260.h"
#include "src/BQ27441/BQ27441.h"
#include "src/TMP100/TMP100.h"

//-------------------------------------------------------------------------/
//  SENSOR INITIALIZATION
//-------------------------------------------------------------------------/
SoftwareWire eps_wire(SDA_PIN, SCL_PIN);      // Internal EPS I2C bus, // SDA = 8 ; SCL = 9 (ATMEGA328P)

INA260 ina260_1(1);                           // Solar Channel Monitor (SCM)
INA260 ina260_2(2);                           // Main Bus Monitor (MBM)
INA260 ina260_3(5);                           // Secondary Bus Monitor (SBM)
BQ27441 bq27441_1;                            // Battery Gauge
TMP100 tmp100_1(3);                           // Battery Thermal Sensor

//-------------------------------------------------------------------------/
//  GLOBAL VARIABLES
//-------------------------------------------------------------------------/
bool collect_flag = false;                    // Flag to execute collect_data()
bool collect_soc_flag = false;                // Flag to execute collect_soc()
bool collect_FPB_flag = false;                // Flag to execute collect_FPB()
bool reset_flag = false;                      // Flag to execute resetFunc()
bool EPS_ready_flag = false;                  // Flag to indicate that EPS was configured correctly upon start-up
bool collected_data = false;                  // Flag to indicate if there is pending EPS data to be collected by OBC
byte wireCommand = 0;                         // Variable to store OBC commands received via the satellite's main I2C bus

// INA260 No.1
float busvoltage1_raw = 0;                    // To store raw data from INA260 No.1
float current_mA1_raw = 0;
uint8_t busvoltage1_temp = 0x00;              // To store processed data from INA260 No.1
uint16_t current_mA1_temp = 0x0000;
byte busvoltage1 = 0x00;                      // To transmit data from INA260 No.1
byte current_mA1[2] = {0};

// INA260 No.2
float busvoltage2_raw = 0;                    // To store raw data from INA260 No.2
float current_mA2_raw = 0;
uint8_t busvoltage2_temp = 0x00;              // To store processed data from INA260 No.2
uint16_t current_mA2_temp = 0x0000;
byte busvoltage2 = 0x00;                      // To transmit data from INA260 No.2
byte current_mA2[2] = {0};

// INA260 No.3
float busvoltage3_raw = 0;                    // To store raw data from INA260 No.3
float current_mA3_raw = 0;
uint8_t busvoltage3_temp = 0x00;              // To store processed data from INA260 No.3
uint16_t current_mA3_temp = 0x0000;
byte busvoltage3 = 0x00;                      // To transmit data from INA260 No.3
byte current_mA3[2] = {0};

// BQ27441 No.1
unsigned int state_charge_raw1 = 0;           // To store raw data from BQ27441 No.1
unsigned int battery_voltage_raw1 = 0;
int average_current_raw1 = 0;
unsigned int remaining_capacity_raw1 = 0;
int average_power_raw1 = 0;
unsigned int state_health_raw1 = 0;
uint8_t state_charge_temp1 = 0x00;            // To store processed data from BQ27441 No.1
uint8_t battery_voltage_temp1 = 0x00;          
uint16_t average_current_temp1 = 0x0000;
uint16_t remaining_capacity_temp1 = 0x00;
uint16_t average_power_temp1 = 0x0000;
uint8_t state_health_temp1 = 0x00;
byte state_charge = {0};                      // To transmit data from BQ27441 No.1
byte battery_voltage = {0};
byte average_current[2] = {0};
byte remaining_capacity[2] = {0};
byte average_power[2] = {0};
byte state_health = {0};

// TMP100 No.1
float temperature_data1_raw = 0;              // To store raw data from TMP100 No.1
uint8_t temperature_data1_temp = 0;           // To store processed data from TMP100 No.1
byte temperature_data1 = 0x00;                // To transmit data from TMP100 No.1

// Fault Protection Boards
float ADCS_current_raw = 0;                   // To store raw data from Fault Protection Boards (FPB)
float COMMS_current_raw = 0;
float PAYLOAD_current_raw = 0;
float HEATER_current_raw = 0;
uint16_t ADCS_current_temp = 0x0000;          // To store processed data from Fault Protection Boards (FPB)
uint16_t COMMS_current_temp = 0x0000;
uint16_t PAYLOAD_current_temp = 0x0000;
uint16_t HEATER_current_temp = 0x0000;
byte ADCS_current[2] = {0};                   // To transmit data from Fault Protection Boards (FPB)
byte COMMS_current[2] = {0};
byte PAYLOAD_current[2] = {0};
byte HEATER_current[2] = {0};

// Enable Flags (false - system OFF; true - system ON)
bool ADCS_enabled = false;
bool V5V0_enabled = false;
bool COMMS_enabled = false;
bool PAYLOAD_enabled = false;
bool HEATER_enabled = false;

// All EPS data byte array (sent to OBC)
byte bus_data[30] = {0};
// Communication Flags byte array
byte comm_flags[5] = {0};
// Transmission Flags byte array
byte trans_flags[5] = {0};
// Communication Flags (CF) byte (sent to OBC)
byte communication_flags = 0x00;
// Transmission Flags (TF) byte (sent to OBC)
byte transmission_flags = 0x00;
// FPB Fault Flags (FPBF) byte (sent to OBC)
byte fault_flags = 0x00;

// Mapping function for floats (For INA260 and TMP100)
// NOTE: The map() function from Arduino showed to be unreliable for the EPS data, so
//       these dedicated map functions were implemented instead
float map_INA_TMP(float raw, float lowLim, float highLim, float mapLow, float mapHigh){
  float m = (mapHigh - mapLow)/(highLim - lowLim);
  float b = mapHigh - m*highLim;
  float y = m*raw + b;
  return y;
}

// Mapping function for signed integers (For BQ27441)
float map_BQ_INT(int raw, float lowLim, float highLim, float mapLow, float mapHigh){
  float m = (mapHigh - mapLow)/(highLim - lowLim);
  float b = mapHigh - m*highLim;
  float y = m*raw + b;
  return y;
}

// Mapping function for unsigned integers (For BQ27441)
float map_BQ_UINT(unsigned int raw, float lowLim, float highLim, float mapLow, float mapHigh){
  float m = (mapHigh - mapLow)/(highLim - lowLim);
  float b = mapHigh - m*highLim;
  float y = m*raw + b;
  return y;
}

// Mapping function for unsigned integers (For FPBs)
float map_FPB(unsigned int raw, float lowLim, float highLim, float mapLow, float mapHigh){
  float m = (mapHigh - mapLow)/(highLim - lowLim);
  float b = mapHigh - m*highLim;
  float y = m*raw + b;
  return y;
}

// Function to collect data from all EPS sensors
void collect_data(){

    // Reset collected_data flag to indicate that there is currently no new EPS data to be retrieved by OBC
    collected_data = false;

    // Initialize Transmission Flags (if a sensor's flag switches from 1 to 0, the sensor failed to transmit data correctly)
    trans_flags[0] = 1;                       // INA260_1: Solar Channel Monitor (SCM) transmission flag
    trans_flags[1] = 1;                       // INA260_2: Main Bus Monitor (MBM) transmission flag
    trans_flags[2] = 1;                       // INA260_3: Secondary Bus Monitor (SBM) transmission flag
    trans_flags[3] = 1;                       // BQ27441_1: Battery Gauge transmission flag
    trans_flags[4] = 1;                       // TMP100_1: Battery Thermal Sensor transmission flag

    // Reset Transmission Flags (TF) and FPB Fault Flags (FPBF) bytes
    transmission_flags = 0x00;
    fault_flags = 0x00;

    // INA260 No.1 (SCM Block) *************************************************************
    int INA_TRY_COUNTER1 = 0;
    while(INA_TRY_COUNTER1 < 3){

      // If sensor was initialized correctly during setup(), execute data retrieval and transmission
      int INA260_CONFIGURED1 = false;
      if (bitRead(communication_flags, 0) == 1){
        INA260_CONFIGURED1 = ina260_1.begin();
      }

      if (INA260_CONFIGURED1){

        // SOLAR PANELS CHANNEL - INA260 [Expected range: 0 - 4.2 V; Measuring range: 0 - 4.5 V; Mapped to: 0 - 252 bits; Resolution: 17.86 mV/bit]
        busvoltage1_raw = ina260_1.getBusVoltage_V(SOLAR_PANELS_CHANNEL);                   // Channel No.1: SOLAR PANELS VOLTAGE (Sensor range: +/- 32.76V)
                                                                                            // Truncate negative voltages
        if (busvoltage1_raw >= 0 && busvoltage1_raw <= 4.5){                                // Check if voltage is in authorized range (0 - 4.5 V)
          busvoltage1_temp = (uint8_t) map_INA_TMP(busvoltage1_raw, 0, 4.5, 0, 252);        // Map 0 - 4.5 V to 0 - 252 bits (Reserved values: 253,254,255)
          busvoltage1 = ((byte) busvoltage1_temp) & 0xFF;                                   // Parse to byte for transmission
        } else {
          busvoltage1 = (byte) 0xFF;                                                        // RANGE ERROR (trans_flags[0] = 0 & busvoltage1 = 255)
          trans_flags[0] = 0;                                                               // Value out of authorized range 0 - 4.5 V. Reset SCM Transmission Flag                     
        }

        // SOLAR PANELS CHANNEL - INA260 [Expected range: 0 - 0.8 A; Measuring range: 0 - 2500 mA; Mapped to: 0 - 4092 bits; Resolution: 0.61 mA/bit]
        current_mA1_raw = ina260_1.getCurrent_mA(SOLAR_PANELS_CHANNEL) + 1.25;              // Channel No.1: SOLAR PANELS CURRENT (Sensor range: +/- 3276mA) (Offset = 1.25 for zero adjustment)
        if (current_mA1_raw >= 0){                                                          // Check if value is an authorized positive value
          if (current_mA1_raw >= 0 && current_mA1_raw <= 2500){                             // Check if current is in authorized range (0 - 2500 mA)
            current_mA1_temp = (uint16_t) map_INA_TMP(current_mA1_raw, 0, 2500, 0, 4092);   // Map 0 - 2500 mA to 0 - 4092 bits (Reserved values: 4093,4094,4095)
            current_mA1[0] = (byte) (current_mA1_temp >> 8) & 0xFF;                         // Parse to byte for transmission
            current_mA1[1] = ((byte) current_mA1_temp) & 0xFF;
          } else {
            current_mA1[0] = (byte) 0x0F;                                                   // RANGE ERROR (trans_flags[0] = 0 & current_mA1 = 4095)
            current_mA1[1] = (byte) 0xFF;
            trans_flags[0] = 0;                                                             // Value out of authorized range 0 - 2500 mA. Reset SCM Transmission Flag                    
          }
        } else {
          current_mA1[0] = (byte) 0x0F;                                                     // NEGATIVE ERROR (trans_flags[0] = 0 & current_mA1 = 4094)
          current_mA1[1] = (byte) 0xFE;
          trans_flags[0] = 0;                                                               // Value is unauthorized negative. Reset SCM Transmission Flag
        }

        break;
      } 

      INA_TRY_COUNTER1 += 1;
      if (INA_TRY_COUNTER1 == 3){
        busvoltage1 = (byte) 0xFD;                                                          // TRANSMISSION ERROR (trans_flags[0] = 0 & busvoltage1 = 253)
        current_mA1[0] = (byte) 0x0F;                                                       // TRANSMISSION ERROR (trans_flags[0] = 0 & current_mA1 = 4093)
        current_mA1[1] = (byte) 0xFD;
        trans_flags[0] = 0;                                                                 // INA260_1 did not send data back. Reset SCM Transmission Flag                                                         
      }
    }

    // INA260 No.2 (MBM Block) *************************************************************
    int INA_TRY_COUNTER2 = 0;
    while(INA_TRY_COUNTER2 < 3){

      // If sensor was initialized correctly during setup(), execute data retrieval and transmission
      int INA260_CONFIGURED2 = false;
      if (bitRead(communication_flags, 1) == 1){
        INA260_CONFIGURED2 = ina260_2.begin();
      }

      if (INA260_CONFIGURED2){

        // REGULATOR 3V3 CHANNEL - INA260 [Expected range: 2.75 - 4.3V; Measuring range: 0 - 4.5 V; Mapped to: 0 - 252 bits; Resolution: 17.86 mV/bit]
        busvoltage2_raw = ina260_2.getBusVoltage_V(REGULATOR_3V3_CHANNEL);                  // Channel No.2: REGULATOR 3V3 CHANNEL (Sensor range: +/- 32.76V)  
        if (busvoltage2_raw >= 0 && busvoltage2_raw <= 4.5){                                // Check if voltage is in authorized range (0 - 4.5 V)
          busvoltage2_temp = (uint8_t) map_INA_TMP(busvoltage2_raw, 0, 4.5, 0, 252);        // Map 0 - 4.5 V to 0 - 252 bits (Reserved values: 253,254,255)
          busvoltage2 = ((byte) busvoltage2_temp) & 0xFF;                                   // Parse to byte for transmission
        } else {
          busvoltage2 = (byte) 0xFF;                                                        // RANGE ERROR (trans_flags[1] = 0 & busvoltage2 = 255)
          trans_flags[1] = 0;                                                               // Value out of authorized range 0 - 4.5 V. Reset MBM Transmission Flag                     
        }

        // REGULATOR 3V3 CHANNEL - INA260 [Expected range: 0 - 2.0A; Measuring range: 0 - 2500 mA; Mapped to: 0 - 4092 bits; Resolution: 0.61 mA/bit]
        current_mA2_raw = ina260_2.getCurrent_mA(REGULATOR_3V3_CHANNEL) + 1.25;             // Channel No.2: REGULATOR 3V3 CHANNEL (Sensor range: +/- 3276mA) (Offset = 1.25 for zero adjustment)
        if (current_mA2_raw >= 0){                                                          // Check if value is an authorized positive value
          if (current_mA2_raw >= 0 && current_mA2_raw <= 2500){                             // Check if current is in authorized range (0 - 2500 mA)
            current_mA2_temp = (uint16_t) map_INA_TMP(current_mA2_raw, 0, 2500, 0, 4092);   // Map 0 - 2500 mA to 0 - 4092 bits (Reserved values: 4093,4094,4095)
            current_mA2[0] = (byte) (current_mA2_temp >> 8) & 0xFF;                         // Parse to byte for transmission
            current_mA2[1] = ((byte) current_mA2_temp) & 0xFF;
          } else {
            current_mA2[0] = (byte) 0x0F;                                                   // RANGE ERROR (trans_flags[1] = 0 & current_mA2 = 4095)
            current_mA2[1] = (byte) 0xFF;
            trans_flags[1] = 0;                                                             // Value out of authorized range 0 - 2500 mA. Reset MBM Transmission Flag                   
          }
        } else {
          current_mA2[0] = (byte) 0x0F;                                                     // NEGATIVE ERROR (trans_flags[1] = 0 & current_mA2 = 4094)
          current_mA2[1] = (byte) 0xFE;
          trans_flags[1] = 0;                                                               // Value is unauthorized negative. Reset MBM Transmission Flag
        }

        break;
      }

      INA_TRY_COUNTER2 += 1;
      if (INA_TRY_COUNTER2 == 3){                                                           
        busvoltage2 = (byte) 0xFD;                                                          // TRANSMISSION ERROR (trans_flags[1] = 0 & busvoltage2 = 253)
        current_mA2[0] = (byte) 0x0F;                                                       // TRANSMISSION ERROR (trans_flags[1] = 0 & current_mA2 = 4093)
        current_mA2[1] = (byte) 0xFD;
        trans_flags[1] = 0;                                                                 // INA260_2 did not send data back. Reset MBM Transmission Flag                                                          
      }
    }

    // INA260 No.3 (SBM Block) *************************************************************
    int INA_TRY_COUNTER3 = 0;
    while(INA_TRY_COUNTER3 < 3){

      // If sensor was initialized correctly during setup(), execute data retrieval and transmission
      int INA260_CONFIGURED3 = false;
      if (bitRead(communication_flags, 2) == 1){
        INA260_CONFIGURED3 = ina260_3.begin();
      }

      if (INA260_CONFIGURED3){

        // REGULATOR 5V0 CHANNEL - INA260 [Expected range: 2.75 - 4.3V; Measuring range: 0 - 4.5 V; Mapped to: 0 - 252 bits; Resolution: 17.86 mV/bit]
        busvoltage3_raw = ina260_3.getBusVoltage_V(REGULATOR_5V0_CHANNEL);                  // Channel No.3: REGULATOR 5V0 CHANNEL (Sensor range: +/- 32.76V)  
        if (busvoltage3_raw >= 0 && busvoltage3_raw <= 4.5){                                // Check if voltage is in authorized range (0 - 4.5 V)
          busvoltage3_temp = (uint8_t) map_INA_TMP(busvoltage3_raw, 0, 4.5, 0, 252);        // Map 0 - 4.5 V to 0 - 252 bits (Reserved values: 253,254,255)
          busvoltage3 = ((byte) busvoltage3_temp) & 0xFF;                                   // Parse to byte for transmission
        } else {
          busvoltage3 = (byte) 0xFF;                                                        // RANGE ERROR (trans_flags[2] = 0 & busvoltage3 = 255)
          trans_flags[2] = 0;                                                               // Value out of authorized range 0 - 4.5 V. Reset SBM Transmission Flag                     
        }

        // REGULATOR 5V0 CHANNEL - INA260 [Expected range: 0 - 2.0A; Measuring range: 0 - 2500 mA; Mapped to: 0 - 4092 bits; Resolution: 0.61 mA/bit]
        current_mA3_raw = ina260_3.getCurrent_mA(REGULATOR_5V0_CHANNEL) + 1.25;             // Channel No.3: REGULATOR 5V0 CHANNEL (Sensor range: +/- 3276mA) (Offset = 1.25 for zero adjustment)
        if (current_mA3_raw >= 0){                                                          // Check if value is an authorized positive value
          if (current_mA3_raw >= 0 && current_mA3_raw <= 2500){                             // Check if current is in authorized range (0 - 2500 mA)
            current_mA3_temp = (uint16_t) map_INA_TMP(current_mA3_raw, 0, 2500, 0, 4092);   // Map 0 - 2500 mA to 0 - 4092 bits (Reserved values: 4093,4094,4095)
            current_mA3[0] = (byte) (current_mA3_temp >> 8) & 0xFF;                         // Parse to byte for transmission
            current_mA3[1] = ((byte) current_mA3_temp) & 0xFF;
          } else {
            current_mA3[0] = (byte) 0x0F;                                                   // RANGE ERROR (trans_flags[2] = 0 & current_mA3 = 4095)
            current_mA3[1] = (byte) 0xFF;
            trans_flags[2] = 0;                                                             // Value out of authorized range 0 - 2500 mA. Reset SBM Transmission Flag                    
          }
        } else {
          current_mA3[0] = (byte) 0x0F;                                                     // NEGATIVE ERROR (trans_flags[2] = 0 & current_mA3 = 4094)
          current_mA3[1] = (byte) 0xFE;
          trans_flags[2] = 0;                                                               // Value is unauthorized negative. Reset SBM Transmission Flag
        }

        break;
      }

      INA_TRY_COUNTER3 += 1;
      if (INA_TRY_COUNTER3 == 3){                                                           
        busvoltage3 = (byte) 0xFD;                                                          // TRANSMISSION ERROR (trans_flags[2] = 0 & busvoltage3 = 253)
        current_mA3[0] = (byte) 0x0F;                                                       // TRANSMISSION ERROR (trans_flags[2] = 0 & current_mA3 = 4093)
        current_mA3[1] = (byte) 0xFD;
        trans_flags[2] = 0;                                                                 // INA260_3 did not send data back. Reset SBM Transmission Flag                                                           
      }
    }

    // BQ27441 No.1 (Battery Gauge Block) **************************************************
    int BQ_TRY_COUNTER1 = 0;
    while (BQ_TRY_COUNTER1 < 2){

      // If sensor was initialized correctly during setup(), execute data retrieval and transmission
      int BQ27441_1_CONFIGURED = false;
      if (bitRead(communication_flags, 3) == 1){
        BQ27441_1_CONFIGURED = bq27441_1.begin();
      }

      if (BQ27441_1_CONFIGURED){

        // BATTERY STATE OF CHARGE - BQ27441 [Expected range: 0 - 100%; Value transmitted directly]
        state_charge_raw1 = (unsigned int) bq27441_1.soc(FILTERED);                         // StateOfCharge() (Sensor range: 0 - 100%)
        state_charge_temp1 = (uint8_t) state_charge_raw1 & 0x00FF;                          // Mapping not necessary
        state_charge = ((byte) state_charge_temp1) & 0xFF;                                  // Parse to byte for transmission

        // BATTERY VOLTAGE - BQ27441 [Expected range: 2.75 - 4.3V; Measuring range: 2.5 - 4.5 V; Mapped to: 1 - 252 bits; Resolution: 7.97 mV/bit]
        battery_voltage_raw1 = (unsigned int) bq27441_1.voltage();                          // BatteryVoltage() (Sensor range: 0 - 6000 mV)
        if (battery_voltage_raw1 >= 0){                                                     // Check if value is an authorized positive value
          if (battery_voltage_raw1 >= 2500 && battery_voltage_raw1 <= 4500){                // Check if voltage is in authorized range (2.5 - 4.5 V)
            battery_voltage_temp1 = (uint8_t) map_BQ_UINT(battery_voltage_raw1, 2500, 4500, 1, 252);// Map 2.5 - 4.5 V to 1 - 252 bits (Reserved values: 253,254,255)
            battery_voltage = ((byte) battery_voltage_temp1) & 0xFF;                        // Parse to byte for transmission
          } else if (battery_voltage_raw1 == 0){
            battery_voltage_temp1 = (uint8_t) 0x00;                                         // Battery is disconnected (0 V)
            battery_voltage = ((byte) battery_voltage_temp1) & 0xFF;                        // Parse to byte for transmission
          } else {
            battery_voltage = (byte) 0xFF;                                                  // RANGE ERROR (trans_flags[3] = 0 & battery_voltage[0] = 255)
            trans_flags[3] = 0;                                                             // Value out of authorized range 2.5 - 4.5 V. Reset Battery Gauge Transmission Flag                      
          }
        } else {
          battery_voltage = (byte) 0xFE;                                                    // NEGATIVE ERROR (trans_flags[3] = 0 & battery_voltage[0] = 254)
          trans_flags[3] = 0;                                                               // Value is unauthorized negative. Reset Battery Gauge Transmission Flag
        }

        // AVERAGE BATTERY CURRENT - BQ27441 [Expected range: -2 - 2 A; Measuring range: -2.5 - 2.5 A; Mapped to: 0 - 4092 bits; Resolution: 1.22 mA/bit]
        // [Negative current = battery discharge; Positive current = battery charge]
        average_current_raw1 = (int) bq27441_1.current(AVG);                                // Battery Current (Sensor range: +/- ????mA)
        if (average_current_raw1 >= -2500 && average_current_raw1 <= 2500){                 // Check if current is in authorized range (-2.5 - 2.5 A)
          average_current_temp1 = (uint16_t) (map_BQ_INT(average_current_raw1, -2500, 2500, 0, 4092) + 0.61); // Map -2500 - 2500mA to 0 - 4092 bits (Reserved values: 4093,4094,4095). 0.61 mA offset to transmit real zero
          average_current[0] = (byte) (average_current_temp1 >> 8) & 0xFF;                  // Parse to byte for transmission
          average_current[1] = ((byte) average_current_temp1) & 0xFF;
        } else {
          average_current[0] = (byte) 0x0F;                                                 // RANGE ERROR (trans_flags[3] = 0 & average_current = 4095)
          average_current[1] = (byte) 0xFF;                                                 // RANGE ERROR (trans_flags[3] = 0 & average_current = 4095)
          trans_flags[3] = 0;                                                               // Value out of authorized range -2.5 - 2.5 A. Reset Battery Gauge Transmission Flag                     
        }

        // REMAINING BATTERY CAPACITY - BQ27441 [Expected range: 0 - 4000 mAh; Measuring range: 0 - 4000 mAh; Mapped to: 0 - 4092 bits; Resolution: 0.98 mAh/bit]
        remaining_capacity_raw1 = (unsigned int) bq27441_1.capacity(REMAIN);                // Battery Remaining capacity (Sensor range: +/- RM mAh)
        if (remaining_capacity_raw1 >= 0){                                                  // Check if value is an authorized positive value
          if (remaining_capacity_raw1 >= 0 && remaining_capacity_raw1 <= 4000){             // Check if remaining capacity is in authorized range (0 - 4000 mAh)
            remaining_capacity_temp1 = (uint16_t) map_BQ_UINT(remaining_capacity_raw1, 0, 4000, 0, 4092); // Map 0 - 4000 mAh to 0-4092 bits (Reserved values: 4093,4094,4095)
            remaining_capacity[0] = (byte) (remaining_capacity_temp1 >> 8) & 0xFF;          // Parse to byte for transmission
            remaining_capacity[1] = ((byte) remaining_capacity_temp1) & 0xFF;
          } else {
            remaining_capacity[0] = (byte) 0x0F;                                            // RANGE ERROR (trans_flags[3] = 0 & remaining_capacity = 4095)
            remaining_capacity[1] = (byte) 0xFF;
            trans_flags[3] = 0;                                                             // Value out of authorized range 0 - 4000 mAh. Reset Battery Gauge Transmission Flag                     
          }
        } else {
          remaining_capacity[0] = (byte) 0x0F;                                              // NEGATIVE ERROR (trans_flags[3] = 0 & remaining_capacity = 4094)
          remaining_capacity[1] = (byte) 0xFE;
          trans_flags[3] = 0;                                                               // Value is unauthorized negative. Reset Battery Gauge Transmission Flag
        }

        // BATTERY POWER - BQ27441 [Expected range: -8.4 - 8.4 W; Measuring range: -8.5 - 8.5 W; Mapped to: 0 - 4092 bits; Resolution: 4.15 mW/bit]
        // [Negative power = battery discharge; Positive power = battery charge]
        average_power_raw1 = (int) bq27441_1.power();                                       // Battery power (Sensor range: +/- ????mW)
        if (average_power_raw1 >= -8500 && average_power_raw1 <= 8500){                     // Check if power is in authorized range (-8.5 - 8.5 W)
          average_power_temp1 = (uint16_t) map_BQ_INT(average_power_raw1, -8500, 8500, 0, 4092); // Map -8500 - 8500 mW to 0 - 4092 bits (Reserved values: 4093,4094,4095)
          average_power[0] = (byte) (average_power_temp1 >> 8) & 0xFF;                      // Parse to byte for transmission
          average_power[1] = ((byte) average_power_temp1) & 0xFF;
        } else {
          average_power[0] = (byte) 0x0F;                                                   // RANGE ERROR (trans_flags[3] = 0 & average_power = 4095)
          average_power[1] = (byte) 0xFF;                                                   // RANGE ERROR (trans_flags[3] = 0 & average_power = 4095)
          trans_flags[3] = 0;                                                               // Value out of authorized range -8500 - 8500 mW. Reset Battery Gauge Transmission Flag                  
        }

        // BATTERY STATE OF HEALTH - BQ27441 [Expected range: 0 - 100%; Value transmitted directly]
        state_health_raw1 = (unsigned int) bq27441_1.soh(PERCENT);                          // StateOfHealth() (Sensor range: 0 - 100%)
        state_health_temp1 = (uint8_t) state_health_raw1 & 0x00FF;                          // Mapping not necessary
        state_health = ((byte) state_health_raw1) & 0xFF;                                   // Parse to byte for transmission

        break;
      }

      BQ_TRY_COUNTER1 += 1;
      if (BQ_TRY_COUNTER1 == 2){
        state_charge = (byte) 0xFD;                                                         // TRANSMISSION ERROR (trans_flags[3] = 0 & state_charge = 253)
        battery_voltage = (byte) 0xFD;                                                      // TRANSMISSION ERROR (trans_flags[3] = 0 & battery_voltage = 253)
        average_current[0] = (byte) 0x0F;                                                   // TRANSMISSION ERROR (trans_flags[3] = 0 & average_current = 4093)
        average_current[1] = (byte) 0xFD;
        remaining_capacity[0] = (byte) 0x0F;                                                // TRANSMISSION ERROR (trans_flags[3] = 0 & remaining_capacity = 4093)
        remaining_capacity[1] = (byte) 0xFD;
        average_power[0] = (byte) 0x0F;                                                     // TRANSMISSION ERROR (trans_flags[3] = 0 & average_power = 4093)
        average_power[1] = (byte) 0xFD;
        state_health = (byte) 0xFD;                                                         // TRANSMISSION ERROR (trans_flags[3] = 0 & state_health = 253)
        trans_flags[3] = 0;                                                                 // BQ27441_1 did not send data back. Reset Battery Gauge Transmission Flag
      }
    }

    // TMP100 No.1 (Battery Thermal Sensor Block) ******************************************
    int TMP1_TRY_COUNTER = 0;
    while (TMP1_TRY_COUNTER < 3){

      // If sensor was initialized correctly during setup(), execute data retrieval and transmission
      int TMP_1_CONFIGURED = false;
      if (bitRead(communication_flags, 4) == 1){
        TMP_1_CONFIGURED = tmp100_1.wakeup();
      }

      if (TMP_1_CONFIGURED){

        // BATTERY TEMPERATURE - TMP100 [Expected range: -15 - 60°C; Measuring range: -25 -> 70°C; Mapped to: 0 - 252 bits; Resolution: 0.38°C/bit; 0°C = 67]
        temperature_data1_raw = tmp100_1.readTempC();                                       // Battery Temperature (Sensor range: -55 -> 125°C)
        if (temperature_data1_raw >= -25 && temperature_data1_raw <= 70){                   // Check if temperature is in authorized range (-25 - 70°C)
          temperature_data1_temp = (uint8_t) map_INA_TMP(temperature_data1_raw, -25, 70, 0, 252); // Map -25 - 70°C to 0 - 252 bits (Reserved values: 253,254,255)
          temperature_data1 = ((byte) temperature_data1_temp) & 0xFF;                       // Parse to byte for transmission
        } else {
          temperature_data1 = (byte) 0xFF;                                                  // RANGE ERROR (trans_flags[4] = 0 & temperature_data1 = 255)
          trans_flags[4] = 0;                                                               // Value out of authorized range -25 - 70°C. Reset Battery Thermal Sensor Transmission Flag 
        }
        tmp100_1.sleep();                                                                   //Power OFF
        break;
      } 

      TMP1_TRY_COUNTER += 1;
      if (TMP1_TRY_COUNTER == 3){
        temperature_data1 = 0xFD;                                                           // TRANSMISSION ERROR (trans_flags[34] = 0 & temperature_data1 = 253)
        trans_flags[4] = 0;                                                                 // TMP100_1 did not send data back. Reset Battery Thermal Sensor Transmission Flag                                                
      }
    }

    // Fault Detection (Overcurrent) in pins that receive analogic signals from Fault Protection Boards (FPBs)
    // ATMEGA328P: analog pin A0, A1, A2, A3
    
    // ADCS Fault Protection Board
    ADCS_current_raw = 0;
    for (int count = 0; count < 10; count++){
      ADCS_current_raw += (float) map_FPB(analogRead(A1), 0, 1023, 0, 3.3);   
      delay(0.1);
    }
    ADCS_current_raw = ADCS_current_raw/10.0;                                               // Calculate average ADCS FPB output voltage (from 10 measurements)  
    if (ADCS_current_raw >= ADCS_OVERCURRENT_LIMIT){                                        // ADCS overcurrent limit set to 290 mA (equivalent to 0.377 V FPB output voltage)
      fault_flags = fault_flags | 0x01;
    }
    ADCS_current_raw = ADCS_current_raw * 770.00;                                           // Convert FPB output voltage to equivalent current (where 2.5 V = 2 A as per FPB design)
    if (ADCS_enabled && ADCS_current_raw < ADCS_SHORTCIRCUIT_LIMIT){                        // If ADCS is enabled and ADCS current is below 3 mA, set shortcircuit flag
      fault_flags = fault_flags | 0x10;
    }
    ADCS_current_temp = (uint16_t) ADCS_current_raw;
    ADCS_current[0] = (byte) (ADCS_current_temp >> 8) & 0xFF;                               // Parse to byte for transmission
    ADCS_current[1] = ((byte) ADCS_current_temp) & 0xFF;

    // COMMS Fault Protection Board
    COMMS_current_raw = 0;
    for (int count = 0; count < 10; count++){
      COMMS_current_raw += (float) map_FPB(analogRead(A0), 0, 1023, 0, 3.3);  
      delay(0.1);
    }
    COMMS_current_raw = COMMS_current_raw/10.0;                                             // Calculate average COMMS FPB output voltage (from 10 measurements)  
    if (COMMS_current_raw >= COMMS_OVERCURRENT_LIMIT){                                      // COMMS overcurrent limit set to 1000 mA (equivalent to 1.299 V FPB output voltage)
      fault_flags = fault_flags | 0x02;
    } 
    COMMS_current_raw = COMMS_current_raw * 770.00;                                         // Convert FPB output voltage to equivalent current (where 2.5 V = 2 A as per FPB design)
    if (COMMS_enabled && COMMS_current_raw < COMMS_SHORTCIRCUIT_LIMIT){                     // If COMMS is enabled and COMMS current is below 15 mA, set shortcircuit flag
      fault_flags = fault_flags | 0x20;
    }
    COMMS_current_temp = (uint16_t) COMMS_current_raw;
    COMMS_current[0] = (byte) (COMMS_current_temp >> 8) & 0xFF;                             // Parse to byte for transmission
    COMMS_current[1] = ((byte) COMMS_current_temp) & 0xFF;

    // PAYLOAD Fault Protection Board
    PAYLOAD_current_raw = 0;
    for (int count = 0; count < 10; count++){
      PAYLOAD_current_raw += (float) map_FPB(analogRead(A2), 0, 1023, 0, 3.3);
      delay(0.1);
    }
    PAYLOAD_current_raw = PAYLOAD_current_raw/10.0;                                         // Calculate average Payload FPB output voltage (from 10 measurements)  
    if (PAYLOAD_current_raw >= PAYLOAD_OVERCURRENT_LIMIT){                                  // Payload overcurrent limit set to 290 mA (equivalent to 0.377 V FPB output voltage)
      fault_flags = fault_flags | 0x04;
    } 
    PAYLOAD_current_raw = PAYLOAD_current_raw * 770.00;                                     // Convert FPB output voltage to equivalent current (where 2.5 V = 2 A as per FPB design)
    if (PAYLOAD_enabled && PAYLOAD_current_raw < PAYLOAD_SHORTCIRCUIT_LIMIT){               // If Payload is enabled and Payload current is below 15 mA, set shortcircuit flag
      fault_flags = fault_flags | 0x40;
    }
    PAYLOAD_current_temp = (uint16_t) PAYLOAD_current_raw;
    PAYLOAD_current[0] = (byte) (PAYLOAD_current_temp >> 8) & 0xFF;                         // Parse to byte for transmission
    PAYLOAD_current[1] = ((byte) PAYLOAD_current_temp) & 0xFF;

    // HEATER Fault Protection Board
    HEATER_current_raw = 0;
    for (int count = 0; count < 10; count++){
      HEATER_current_raw += (float) map_FPB(analogRead(A3), 0, 1023, 0, 3.3); 
      delay(0.1);
    }
    HEATER_current_raw = HEATER_current_raw/10.0;                                           // Calculate average Heater FPB output voltage (from 10 measurements)  
    if (HEATER_current_raw >= HEATER_OVERCURRENT_LIMIT){                                    // Heater overcurrent limit set to 450 mA (equivalent to 0.585 V FPB output voltage)
      fault_flags = fault_flags | 0x08;
    }
    HEATER_current_raw = HEATER_current_raw * 770.00;                                       // Convert FPB output voltage to equivalent current (where 2.5 V = 2 A as per FPB design)
    if (HEATER_enabled && HEATER_current_raw < HEATER_SHORTCIRCUIT_LIMIT){                  // If Heater is enabled and Heater current is below 15 mA, set shortcircuit flag
      fault_flags = fault_flags | 0x80;
    }
    HEATER_current_temp = (uint16_t) HEATER_current_raw;
    HEATER_current[0] = (byte) (HEATER_current_temp >> 8) & 0xFF;                           // Parse to byte for transmission
    HEATER_current[1] = ((byte) HEATER_current_temp) & 0xFF;

    // Configure Transmission Flags (TF) byte according to transmission flags array
    if (trans_flags[0] == 1){
      transmission_flags = transmission_flags | 0x01;                                       // All INA260 No.1 data is correct
    }
    if (trans_flags[1] == 1){
      transmission_flags = transmission_flags | 0x02;                                       // All INA260 No.2 data is correct
    }
    if (trans_flags[2] == 1){
      transmission_flags = transmission_flags | 0x04;                                       // All INA260 No.3 data is correct
    }
    if (trans_flags[3] == 1){
      transmission_flags = transmission_flags | 0x08;                                       // All BQ27441 data is correct
    }
    if (trans_flags[4] == 1){
      transmission_flags = transmission_flags | 0x10;                                       // All TMP100 data is correct
    }

    // Reset flag that triggered the collect_data() execution
    collect_flag = false;

    // Set collected_data flag indicating that there is new EPS data to be retrieved by OBC
    collected_data = true;
}

// Function to specifically retrieve battery state of charge (SOC) measured by Battery Gauge (BQ27441)
void collect_soc(){

  trans_flags[3] = 1;

  // Reset Battery Gauge Transmission Flag only
  transmission_flags = transmission_flags & 0xF7;

  //BQ27441
  int BQ_TRY_COUNTER1 = 0;
  while(BQ_TRY_COUNTER1 < 2){

    // If sensor was initialized correctly during setup(), execute data retrieval and transmission
    int BQ27441_1_CONFIGURED = false;
    if (bitRead(communication_flags, 3) == 1){
      BQ27441_1_CONFIGURED = bq27441_1.begin();
    }

    if (BQ27441_1_CONFIGURED){
      // BATTERY STATE OF CHARGE - BQ27441 [Expected range: 0 - 100%; Value transmitted directly]
      state_charge_raw1 = (unsigned int) bq27441_1.soc(FILTERED);                           // StateOfCharge() (Sensor range: 0 - 100%)
      state_charge_temp1 = (uint8_t) state_charge_raw1 & 0x00FF;                            // Mapping not necessary
      state_charge = ((byte) state_charge_temp1) & 0xFF;                                    // Parse to byte for transmission
      break;
    } 

    BQ_TRY_COUNTER1 += 1;
    if (BQ_TRY_COUNTER1 == 2){
      state_charge = (byte) 0xFD;                                                           // TRANSMISSION ERROR (trans_flags[3] = 0 & state_charge[0] = 253)
      trans_flags[3] = 0;                                                                   // BQ27441_1 did not send data back. Reset Battery Gauge Transmission Flag
    }
  }

  // Set Battery Gauge Transmission Flag
  if (trans_flags[3] == 1){
    transmission_flags = transmission_flags | 0x08;                                         // All BQ27441 data is correct
  }

  // Reset flag that triggered the collect_soc() execution
  collect_soc_flag = false;
}

// Function to specifically retrieve data from the Fault Protection Boards (FPBs)
void collect_FPB(){

  // Reset FPBF byte
  fault_flags = 0x00;

  // Fault Detection (Overcurrent) in pins that receive analogic signals from Fault Protection Boards (FPBs)
  // ATMEGA328P: analog pin A0, A1, A2, A3

  // ADCS Fault Protection Board
  ADCS_current_raw = 0;
  for (int count = 0; count < 10; count++){
    ADCS_current_raw += (float) map_FPB(analogRead(A1), 0, 1023, 0, 3.3);   
    delay(0.1);
  }
  ADCS_current_raw = ADCS_current_raw/10.0;                                                 // Calculate average ADCS FPB output voltage (from 10 measurements)  
  if (ADCS_current_raw >= ADCS_OVERCURRENT_LIMIT){                                          // ADCS overcurrent limit set to 290 mA (equivalent to 0.377 V FPB output voltage)
    fault_flags = fault_flags | 0x01;
  }
  ADCS_current_raw = ADCS_current_raw * 770.00;                                             // Convert FPB output voltage to equivalent current (where 2.5 V = 2 A as per FPB design)
  if (ADCS_enabled && ADCS_current_raw < ADCS_SHORTCIRCUIT_LIMIT){                          // If ADCS is enabled and ADCS current is below 3 mA, set shortcircuit flag
    fault_flags = fault_flags | 0x10;
  }
  ADCS_current_temp = (uint16_t) ADCS_current_raw;
  ADCS_current[0] = (byte) (ADCS_current_temp >> 8) & 0xFF;                                 // Parse to byte for transmission
  ADCS_current[1] = ((byte) ADCS_current_temp) & 0xFF;

  // COMMS Fault Protection Board
  COMMS_current_raw = 0;
  for (int count = 0; count < 10; count++){
    COMMS_current_raw += (float) map_FPB(analogRead(A0), 0, 1023, 0, 3.3);  
    delay(0.1);
  }
  COMMS_current_raw = COMMS_current_raw/10.0;                                               // Calculate average COMMS FPB output voltage (from 10 measurements)  
  if (COMMS_current_raw >= COMMS_OVERCURRENT_LIMIT){                                        // COMMS overcurrent limit set to 1000 mA (equivalent to 1.299 V FPB output voltage)
    fault_flags = fault_flags | 0x02;
  } 
  COMMS_current_raw = COMMS_current_raw * 770.00;                                           // Convert FPB output voltage to equivalent current (where 2.5 V = 2 A as per FPB design)
  if (COMMS_enabled && COMMS_current_raw < COMMS_SHORTCIRCUIT_LIMIT){                       // If COMMS is enabled and COMMS current is below 15 mA, set shortcircuit flag
    fault_flags = fault_flags | 0x20;
  }
  COMMS_current_temp = (uint16_t) COMMS_current_raw;
  COMMS_current[0] = (byte) (COMMS_current_temp >> 8) & 0xFF;                               // Parse to byte for transmission
  COMMS_current[1] = ((byte) COMMS_current_temp) & 0xFF;

  // PAYLOAD Fault Protection Board
  PAYLOAD_current_raw = 0;
  for (int count = 0; count < 10; count++){
    PAYLOAD_current_raw += (float) map_FPB(analogRead(A2), 0, 1023, 0, 3.3);
    delay(0.1);
  }
  PAYLOAD_current_raw = PAYLOAD_current_raw/10.0;                                           // Calculate average Payload FPB output voltage (from 10 measurements)  
  if (PAYLOAD_current_raw >= PAYLOAD_OVERCURRENT_LIMIT){                                    // Payload overcurrent limit set to 290 mA (equivalent to 0.377 V FPB output voltage)
    fault_flags = fault_flags | 0x04;
  } 
  PAYLOAD_current_raw = PAYLOAD_current_raw * 770.00;                                       // Convert FPB output voltage to equivalent current (where 2.5 V = 2 A as per FPB design)
  if (PAYLOAD_enabled && PAYLOAD_current_raw < PAYLOAD_SHORTCIRCUIT_LIMIT){                 // If Payload is enabled and Payload current is below 15 mA, set shortcircuit flag
    fault_flags = fault_flags | 0x40;
  }
  PAYLOAD_current_temp = (uint16_t) PAYLOAD_current_raw;
  PAYLOAD_current[0] = (byte) (PAYLOAD_current_temp >> 8) & 0xFF;                           // Parse to byte for transmission
  PAYLOAD_current[1] = ((byte) PAYLOAD_current_temp) & 0xFF;

  // HEATER Fault Protection Board
  HEATER_current_raw = 0;
  for (int count = 0; count < 10; count++){
    HEATER_current_raw += (float) map_FPB(analogRead(A3), 0, 1023, 0, 3.3); 
    delay(0.1);
  }
  HEATER_current_raw = HEATER_current_raw/10.0;                                             // Calculate average Heater FPB output voltage (from 10 measurements)  
  if (HEATER_current_raw >= HEATER_OVERCURRENT_LIMIT){                                      // Heater overcurrent limit set to 450 mA (equivalent to 0.585 V FPB output voltage)
    fault_flags = fault_flags | 0x08;
  }
  HEATER_current_raw = HEATER_current_raw * 770.00;                                         // Convert FPB output voltage to equivalent current (where 2.5 V = 2 A as per FPB design)
  if (HEATER_enabled && HEATER_current_raw < HEATER_SHORTCIRCUIT_LIMIT){                    // If Heater is enabled and Heater current is below 15 mA, set shortcircuit flag
    fault_flags = fault_flags | 0x80;
  }
  HEATER_current_temp = (uint16_t) HEATER_current_raw;
  HEATER_current[0] = (byte) (HEATER_current_temp >> 8) & 0xFF;                             // Parse to byte for transmission
  HEATER_current[1] = ((byte) HEATER_current_temp) & 0xFF;

  // Reset flag that triggered the collect_FPB() execution
  collect_FPB_flag = false;
}

// Function to send all EPS data to the OBC through the main I2C bus
void send_data(){

  // TMP100 Data
  bus_data[0] = temperature_data1;

  // BQ27441 Data
  bus_data[1] = state_charge;
  bus_data[2] = battery_voltage;
  bus_data[3] = average_current[0];
  bus_data[4] = average_current[1];
  bus_data[5] = remaining_capacity[0];
  bus_data[6] = remaining_capacity[1];
  bus_data[7] = average_power[0];
  bus_data[8] = average_power[1];
  bus_data[9] = state_health;
  
  // INA260s Data
  bus_data[10] = busvoltage1;
  bus_data[11] = current_mA1[0];
  bus_data[12] = current_mA1[1];
  bus_data[13] = busvoltage2;
  bus_data[14] = current_mA2[0];
  bus_data[15] = current_mA2[1];
  bus_data[16] = busvoltage3;
  bus_data[17] = current_mA3[0];
  bus_data[18] = current_mA3[1];

  // Fault Protection Boards
  bus_data[19] = ADCS_current[0];
  bus_data[20] = ADCS_current[1];
  bus_data[21] = COMMS_current[0];
  bus_data[22] = COMMS_current[1];
  bus_data[23] = PAYLOAD_current[0];
  bus_data[24] = PAYLOAD_current[1];
  bus_data[25] = HEATER_current[0];
  bus_data[26] = HEATER_current[1];

  // Fault Flags from Fault Protection Boards
  bus_data[27] = fault_flags;

  // Communication and Transmission Flags
  bus_data[28] = communication_flags;
  bus_data[29] = transmission_flags;

  // Sending data
  Wire.write(bus_data, 30);

  // Reset collected_data to indicate that there is no new EPS data to be retrieved by OBC
  collected_data = false;
}

// SOFTWARE RESET
void(* resetFunc) (void) = 0;

// SETUP
void setup(void){  

  // Initialize serial channel (for debugging in console)
  Serial.begin(9600);
  analogReference(DEFAULT);

  // Initialize analog input pins connected to the Fault Protection Boards' signals
  // Analog pins that receive signals from the INA169´s in the Fault Protection Boards (system overcurrent indicators)
  pinMode(A1, INPUT);                                                                       // ADCS FPB
  pinMode(A0, INPUT);                                                                       // COMMS FPB
  pinMode(A2, INPUT);                                                                       // PAYLOAD FPB
  pinMode(A3, INPUT);                                                                       // HEATER FPB

  // Initialize digital output pins to send active-high signals to enables/disable the satellite's systems (ATMEGA328P: pins 2-6)
  pinMode(ENABLE_PIN_ADCS, OUTPUT);
  pinMode(ENABLE_PIN_5V0, OUTPUT);
  pinMode(ENABLE_PIN_COMM, OUTPUT);
  pinMode(ENABLE_PIN_PAYLOAD, OUTPUT);
  pinMode(ENABLE_PIN_HEATER, OUTPUT);

  // Start with all systems disabled and powered off
  digitalWrite(ENABLE_PIN_ADCS, LOW);
  digitalWrite(ENABLE_PIN_5V0, LOW);
  digitalWrite(ENABLE_PIN_COMM, LOW);
  digitalWrite(ENABLE_PIN_PAYLOAD, LOW);
  digitalWrite(ENABLE_PIN_HEATER, LOW);

  // Configuration of the main I2C bus connected to the OBC
  Wire.begin(EPS_ADDRESS);                                                                  // Join the I2C bus with Slave ID (ATMEGA2560: A4 = SDA, A5 = SCL)
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Initialize EPS internal I2C bus connected to all EPS sensors (slaves)
  eps_wire.begin();

  // Connect slaves to the EPS I2C bus
  ina260_1.setI2C(eps_wire);
  ina260_2.setI2C(eps_wire);
  ina260_3.setI2C(eps_wire);
  bq27441_1.setI2C(eps_wire);
  tmp100_1.setI2C(eps_wire);
  
  // Initial configuration of the Solar Channel Monitor (INA260 No.1)
  int INA_TRY_COUNTER1 = 0;
  while (INA_TRY_COUNTER1 < 3){
    if (ina260_1.begin()){
        comm_flags[0] = 1;
        break;
    }
    INA_TRY_COUNTER1 += 1;
    if (INA_TRY_COUNTER1 == 3){
      comm_flags[0] = 0;
    }
  }

  // Initial configuration of the Main Bus Monitor (INA260 No.2)
  int INA_TRY_COUNTER2 = 0;
  while (INA_TRY_COUNTER2 < 3){
    if (ina260_2.begin()){
        comm_flags[1] = 1;
        break;
    }
    INA_TRY_COUNTER2 += 1;
    if (INA_TRY_COUNTER2 == 3){
      comm_flags[1] = 0;
    }
  }

  // Initial configuration of the Secondary Bus Monitor (INA260 No.3)
  int INA_TRY_COUNTER3 = 0;
  while (INA_TRY_COUNTER3 < 3){
    if (ina260_3.begin()){
        comm_flags[2] = 1;
        break;
    }
    INA_TRY_COUNTER3 += 1;
    if (INA_TRY_COUNTER3 == 3){
      comm_flags[2] = 0;
    }
  }

  // Initial configuration of the Battery Gauge (BQ27441)
  int BQ_TRY_COUNTER1 = 0;
  while (BQ_TRY_COUNTER1 < 3){
    if (bq27441_1.begin()){
        bq27441_1.setCapacity(4000);
        comm_flags[3] = 1;
        break;
    }
    BQ_TRY_COUNTER1 += 1;
    if (BQ_TRY_COUNTER1 == 3){
      comm_flags[3] = 0;
    }
  }
                         
  // Initial configuration of the Battery Thermal Sensor (TMP100)
  int TMP1_TRY_COUNTER = 0;
  while (TMP1_TRY_COUNTER < 3){
    if (tmp100_1.wakeup()){
        comm_flags[4] = 1;
        tmp100_1.sleep();
        break;
    }
    TMP1_TRY_COUNTER += 1;
    if (TMP1_TRY_COUNTER == 3){
      comm_flags[4] = 0;
    }
  }

  // Set the Communications Flags (CF) byte according to the communication flags array
  if (comm_flags[0] == 1){
    communication_flags = communication_flags | 0x01;                                       // INA260 No.1 configured successfully upon start-up
  }
  if (comm_flags[1] == 1){
    communication_flags = communication_flags | 0x02;                                       // INA260 No.2 configured successfully upon start-up
  }
  if (comm_flags[2] == 1){
    communication_flags = communication_flags | 0x04;                                       // INA260 No.3 configured successfully upon start-up
  }
  if (comm_flags[3] == 1){
    communication_flags = communication_flags | 0x08;                                       // BQ27441 configured successfully upon start-up
  }
  if (comm_flags[4] == 1){
    communication_flags = communication_flags | 0x10;                                       // TMP100 configured successfully upon start-up
  }

  int comm_flags_int = (int) communication_flags;

  // Set EPS_ready_flag to indicate that the EPS system is ready to operate
  if (comm_flags_int != 0){
    EPS_ready_flag = true;
    //Serial.println("QUETZAL-1 :: EPS ONLINE");
  } else {
    EPS_ready_flag = false;
    //Serial.println("EPS FAILURE");
  }
}

// LOOP
void loop(void){

  //Serial.println("LOOP");

  // If OBC requested full data collection
  if(collect_flag){
    collect_data();
  }

  // If OBC requested SOC data collection
  if(collect_soc_flag){
    collect_soc();
  }

  // If OBC requested FPB data collection
  if (collect_FPB_flag){
    collect_FPB();
  }

  // If OBC requested and EPS software reset to re-execute setup()
  if (reset_flag){
    reset_flag = false;
    delay(1);
    resetFunc();
  }

  // Delay for loop stability
  delay(10);
}

// Function to receive and store I2C command from OBC
void receiveEvent(int howMany) {
  wireCommand = Wire.read();
}

// Function to return data or execute action according to OBC I2C commands
void requestEvent() {

  switch (wireCommand) {

    // EPS Commands

    // Return EPS_ready_flag
    case EPS_READY:
      if (EPS_ready_flag){
        Wire.write(OK);
      } else {
        Wire.write(0xFD);
      }
      delay(1);
      break;

    // Enable/disable ADCS system
    case ADCS_ENABLE:
      digitalWrite(ENABLE_PIN_ADCS, HIGH);
      digitalWrite(ENABLE_PIN_5V0, HIGH);
      ADCS_enabled = true;
      V5V0_enabled = true;
      Wire.write(OK);
      break;
    case ADCS_DISABLE:
      digitalWrite(ENABLE_PIN_ADCS, LOW);
      digitalWrite(ENABLE_PIN_5V0, LOW);
      ADCS_enabled = false;
      V5V0_enabled = false;
      Wire.write(OK);
      break;

    // Enable/disable COMMS system
    case COMM_ENABLE:
      digitalWrite(ENABLE_PIN_COMM, HIGH);
      COMMS_enabled = true;
      Wire.write(OK);
      break;
    case COMM_DISABLE:
      digitalWrite(ENABLE_PIN_COMM, LOW);
      COMMS_enabled = false;
      Wire.write(OK);
      break;

    // Enable/disable Payload system
    case PAYLOAD_ENABLE:
      digitalWrite(ENABLE_PIN_PAYLOAD, HIGH);
      PAYLOAD_enabled = true;
      V5V0_enabled = true;
      Wire.write(OK);
      break;
    case PAYLOAD_DISABLE:
      digitalWrite(ENABLE_PIN_PAYLOAD, LOW);
      PAYLOAD_enabled = false;
      V5V0_enabled = false;
      Wire.write(OK);
      break;

    // Enable/disable battery heater
    case HEATER_ENABLE:
      digitalWrite(ENABLE_PIN_HEATER, HIGH);
      HEATER_enabled = true;
      Wire.write(OK);
      break;
    case HEATER_DISABLE:
      digitalWrite(ENABLE_PIN_HEATER, LOW);
      HEATER_enabled = false;
      Wire.write(OK);
      break;

    // Send battery temperature
    case TEMPERATURE_DATA:
      Wire.write(temperature_data1);
      break;

    // Send Battery Gauge (BQ27441) data
    case STATE_OF_CHARGE_COLLECT:
      collect_soc_flag = true;
      Wire.write(OK);
      break;
    case STATE_OF_CHARGE_SEND:
      Wire.write(state_charge);
      break;
    case BATTERY_VOLTAGE:
      Wire.write(battery_voltage);
      break;
    case AVERAGE_CURRENT:
      Wire.write(average_current,2);
      break;
    case REMAINING_CAPACITY:
      Wire.write(remaining_capacity,2);
      break;
    case AVERAGE_POWER:
      Wire.write(average_power,2);
      break;
    case STATE_OF_HEALTH:
      Wire.write(state_health);
      break;

    // Send monitors' (INA260) data
    case VOLTAGE_CH1:
      Wire.write(busvoltage1);
      break;
    case CURRENT_CH1:
      Wire.write(current_mA1,2);
      break;
    case VOLTAGE_CH2:
      Wire.write(busvoltage2);
      break;
    case CURRENT_CH2:
      Wire.write(current_mA2,2);
      break;
    case VOLTAGE_CH3:
      Wire.write(busvoltage3);
      break;
    case CURRENT_CH3:
      Wire.write(current_mA3,2);
      break;

    // Send FPB data
    case FPB_COLLECT:
      collect_FPB_flag = true;
      Wire.write(OK);
      break;
    case SEND_FAULT_FLAGS:
      Wire.write(fault_flags);
      break;
    case SEND_FPB_CURRENTS:
      Wire.write(ADCS_current[0]);
      Wire.write(ADCS_current[1]);
      Wire.write(COMMS_current[0]);
      Wire.write(COMMS_current[1]);
      Wire.write(PAYLOAD_current[0]);
      Wire.write(PAYLOAD_current[1]);
      Wire.write(HEATER_current[0]);
      Wire.write(HEATER_current[1]);
      break;
    case SEND_ADCS_CURRENT:
      Wire.write(ADCS_current[0]);
      Wire.write(ADCS_current[1]);
      break;
    case SEND_COMMS_CURRENT:
      Wire.write(COMMS_current[0]);
      Wire.write(COMMS_current[1]);
      break;
    case SEND_PAYLOAD_CURRENT:
      Wire.write(PAYLOAD_current[0]);
      Wire.write(PAYLOAD_current[1]);
      break;
    case SEND_HEATER_CURRENT:
      Wire.write(HEATER_current[0]);
      Wire.write(HEATER_current[1]);
      break;

    // Send CF and TF bytes
    case SEND_COMM_FLAGS:
      Wire.write(communication_flags);
      break;
    case SEND_TRANS_FLAGS:
      Wire.write(transmission_flags);
      break;

    // Direct-action commands
    case COLLECT_DATA:
      collect_flag = true;
      Wire.write(OK);
      break;
    case COLLECT_READY:
      if (collected_data){
        Wire.write(OK);
      } else {
        Wire.write(0xFD);
      }
      break;
    case SEND_DATA:
      send_data();
      break;
    case RESET_EPS:
      reset_flag = true;
      Wire.write(OK);
      delay(1);
      break;
  }
}
