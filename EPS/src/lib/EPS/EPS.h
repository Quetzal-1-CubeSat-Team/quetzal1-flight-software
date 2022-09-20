#ifndef __EPS_H__
#define __EPS_H__

// Electrical Power System (EPS) address
#define EPS_ADDRESS			  0x99
#define SDA_PIN               8
#define SCL_PIN               9

// PIN Names
#define ENABLE_PIN_ADCS       4
#define ENABLE_PIN_5V0        2
#define ENABLE_PIN_COMM       3
#define ENABLE_PIN_PAYLOAD    5
#define ENABLE_PIN_HEATER     6

// Overcurrent limits for Fault Protection Boards (in volts; 2.5 V = 2 A as per FPB design)
#define ADCS_OVERCURRENT_LIMIT     0.377
#define COMMS_OVERCURRENT_LIMIT    1.299
#define PAYLOAD_OVERCURRENT_LIMIT  0.377
#define HEATER_OVERCURRENT_LIMIT   0.585

// Shortcircuit limits for Fault Protection Boards (in milliamps)
#define ADCS_SHORTCIRCUIT_LIMIT    3
#define COMMS_SHORTCIRCUIT_LIMIT   15
#define PAYLOAD_SHORTCIRCUIT_LIMIT 15
#define HEATER_SHORTCIRCUIT_LIMIT  15

// INA260 Channel Numbers
#define SOLAR_PANELS_CHANNEL          1
#define REGULATOR_3V3_CHANNEL         2
#define REGULATOR_5V0_CHANNEL         3

// EPS
#define EPS_READY             0x05

// ADCS
#define ADCS_ENABLE           0x06
#define ADCS_DISABLE          0x07

// COMMS
#define COMM_ENABLE           0x09
#define COMM_DISABLE          0x0A

// PAYLOAD
#define PAYLOAD_ENABLE        0x0B
#define PAYLOAD_DISABLE       0x0C

// Heater
#define HEATER_ENABLE         0x0D
#define HEATER_DISABLE        0x0E

// Temperature Data
#define TEMPERATURE_DATA      0x0F

// BQ27441 Data
#define STATE_OF_CHARGE_COLLECT       0x11
#define STATE_OF_CHARGE_SEND       	  0x12
#define BATTERY_VOLTAGE       0x13
#define AVERAGE_CURRENT       0x14
#define REMAINING_CAPACITY    0x15
#define AVERAGE_POWER         0x16
#define STATE_OF_HEALTH       0x17

// INA260 Data
#define VOLTAGE_CH1           0x18
#define CURRENT_CH1           0x19
#define VOLTAGE_CH2           0x1A
#define CURRENT_CH2           0x1B
#define VOLTAGE_CH3           0x1C
#define CURRENT_CH3           0x1D

// Fault Flags
#define FPB_COLLECT           0x1E
#define SEND_FAULT_FLAGS      0x1F

// FPB Data
#define SEND_FPB_CURRENTS     0x20
#define SEND_ADCS_CURRENT     0x21
#define SEND_COMMS_CURRENT    0x22
#define SEND_PAYLOAD_CURRENT  0x23
#define SEND_HEATER_CURRENT   0x24

// CF and TF byte commands
#define SEND_COMM_FLAGS		  0x25
#define SEND_TRANS_FLAGS	  0x26

// All Data
#define COLLECT_DATA          0x27
#define COLLECT_READY         0x28
#define SEND_DATA             0x29
#define RESET_EPS	          0x2A

// OK from ATMEGA328P
#define OK                    0x4B

#endif