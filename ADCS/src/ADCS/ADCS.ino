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

// general libraries
#include <avr/power.h>
#include <avr/sleep.h>
#include <Wire.h>
// local libraries
#include "ADCS.h"
#include "src/LowPower/LowPower.h"
#include "src/SoftwareWire/SoftwareWire.h"
#include "src/Adafruit_Sensor/Adafruit_Sensor.h"
#include "src/Adafruit_BNO055/Adafruit_BNO055.h"
#include "src/ADC128D818/ADC128D818.h"
#include "src/TMP100/TMP100.h"

//-------------------------------------------------------------------------/
//  CONSTANTS
//-------------------------------------------------------------------------/
#define ADM_RESET_PIN   3

//-------------------------------------------------------------------------/
//  SENSOR INITIALIZATION
//-------------------------------------------------------------------------/
SoftwareWire adcs_wire(A2,A3);              // Software wire configuration
Adafruit_BNO055 bno = Adafruit_BNO055(55);  // IMU
ADC128D818 adc1(ADC1_ADDR);                 // ADC 1
ADC128D818 adc2(ADC2_ADDR);                 // ADC2
TMP100 tmp100(0);                           // Temp sensor

//-------------------------------------------------------------------------/
//  GLOBAL VARIABLES
//-------------------------------------------------------------------------/
byte wireCommand = 0;               // The command received from the On Board Computer

byte adcs_ready_flag = 0;           // Signals all sensors initialized correctly
bool collect_flag = false;          // Allows collecting data
bool checkcomm_flag = false;        // Check comm with sensors on ADCS board
bool reset_flag = false;            // Soft reset the ADCS board
bool adm_reset_flag = false;        // Reset the ADM GPIO Expander

float temp_gyro[3] = {0};           // Stores the gyro data in deg/s
byte gyro[3] = {0};                 // Stores the converted gyro data (1 byte per axis)

float temp_magneto_float[3] = {0};  // Stores the magnetometer data uTeslas
uint16_t temp_magneto[3] = {0};     // Stores the converted gyro data (2 bytes per axis)
byte magneto[6] = {0};              // Stores separated high and low bytes for each axis

float temp_adc1pd[6] = {0};         // Stores the voltage read from each channel of the ADC1 (in volts)
byte adc1pd[6] = {0};               // Stores the converted values from the ADC1 (1 byte per channel)  
float temp_adc2pd[6] = {0};         // Stores the voltage read from each channel of the ADC2 (in volts)
byte adc2pd[6] = {0};               // Stores the converted values from the ADC2 (1 byte per channel)  

int8_t imu_temperature = 0;         // Stores the temperature read from the BNO055 IMU
int16_t temp_tmp100 = 0;            // Stores the temperature read from the TMP100 sensor
byte tmp100_temperature[2] = {0};   // Stores the separated high and low bytes read from the TMP100 sensor

// Communication flags
byte comm_flags = 0x00;             // [TMP100_comm, adc2_comm, adc1_comm, imu_comm]

// Counter for communication iterations
uint8_t count_bno = 0;
uint8_t count_adc1 = 0;
uint8_t count_adc2 = 0;
uint8_t count_tmp100 = 0;

//-------------------------------------------------------------------------/
//  HELPER FUNCTIONS FOR INITIALIZATION
//-------------------------------------------------------------------------/

/**************************************************************************/
/*! 
    @brief  Power saving configuration for unused modules
*/
/**************************************************************************/
void powerSaving() {
  power_adc_disable();
  power_spi_disable();
  //power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
}

/**************************************************************************/
/*! 
    @brief  Attempt to initialize the BNO055 IMU
*/
/**************************************************************************/
void bno_init() {
  count_bno = 0;
  while (count_bno < 3) {
    if (bno.begin()) {
      bno.setExtCrystalUse(true);
      bno.enterSuspendMode();      
      comm_flags |= 0x01;
      break;
    }
    count_bno++;
  }
  if(count_bno == 3) {
    comm_flags &= 0xFE;
  }
}

/**************************************************************************/
/*! 
    @brief  Attempt to initialize the Analog to Digital Converter (ADC) #1
*/
/**************************************************************************/
void adc1_init() {
  count_adc1 = 0;
  while (count_adc1 < 3) {
    if (adc1.begin()) {
      comm_flags |= 0x02;
      break;
    }
    count_adc1++;
  }
  if(count_adc1 == 3) {
    comm_flags &= 0xFD;
  }
}

/**************************************************************************/
/*! 
    @brief  Attempt to initialize the Analog to Digital Converter (ADC) #2
*/
/**************************************************************************/
void adc2_init() {
  count_adc2 = 0;
  while (count_adc2 < 3) {
    if (adc2.begin()) {
      comm_flags |= 0x04;
      break;
    }
    count_adc2++;
  }
  if(count_adc2 == 3) {
    comm_flags &= 0xFB;
  }
}

/**************************************************************************/
/*! 
    @brief  Attempt to initialize the TMP100 temperature sensor
*/
/**************************************************************************/
void tmp100_init() {
  count_adc2 = 0;
  while (count_tmp100 < 3) {
    if (tmp100.wakeup()){
      tmp100.sleep();
      comm_flags |= 0x08;
      break;
    }
    count_tmp100++;
  }
  if (count_tmp100 == 3) {
    comm_flags &= 0xF7;
  }
}

/**************************************************************************/
/*! 
    @brief  Retry init for BNO055 IMU
*/
/**************************************************************************/
void bno_checkComm() {
  if (!comm_flags & (1<<0)) {
    bno_init();
  }
}

/**************************************************************************/
/*! 
    @brief  Retry init for ADC #1
*/
/**************************************************************************/
void adc1_checkComm() {
  if (!comm_flags & (1<<1)) {
    adc1_init();
  }
}

/**************************************************************************/
/*! 
    @brief  Retry init for ADC #2
*/
/**************************************************************************/
void adc2_checkComm() {
  if (!comm_flags & (1<<2)) {
    adc2_init();
  }
}

/**************************************************************************/
/*! 
    @brief  Retry init for TMP100
*/
/**************************************************************************/
void tmp100_checkComm() {
  if (!comm_flags & (1<<3)) {
    tmp100_init();
  }
}

/**************************************************************************/
/*! 
    @brief  Retries init for all sensors
*/
/**************************************************************************/
void checkComm() {
  bno_checkComm();
  adc1_checkComm();
  adc2_checkComm();
  tmp100_checkComm();
}

//-------------------------------------------------------------------------/
//  FUNCTIONS TO COLLECT AND SEND DATA TO OBC
//-------------------------------------------------------------------------/

/**************************************************************************/
/*! 
    @brief  Reset all variables relating to sensor data
*/
/**************************************************************************/
void clean_data() {
  for (uint8_t i = 0; i<3; i++) {
    temp_gyro[i] = 0;
    gyro[i] = 0;
    temp_magneto_float[i] = 0;
    temp_magneto[i] = 0;
  }  
  for (uint8_t i = 0; i<6; i++) {
    magneto[i] = 0;
    temp_adc1pd[i] = 0;
    adc1pd[i] = 0;
    temp_adc2pd[i] = 0;
    adc2pd[i] = 0;
  }   

  imu_temperature = 0;
  temp_tmp100 = 0;
  tmp100_temperature[0] = 0;
  tmp100_temperature[1] = 0;
  comm_flags = 0;
}


/**************************************************************************/
/*! 
    @brief Send adcs_ready_flag
*/
/**************************************************************************/
void adcs_ready() {
  Wire.write(adcs_ready_flag);
}

/**************************************************************************/
/*! 
    @brief Send gyroscope data
*/
/**************************************************************************/
void send_gyroscopes() {
  Wire.write(gyro, 3);
}

/**************************************************************************/
/*! 
    @brief Send magnetometer data
*/
/**************************************************************************/
void send_magnetometers() {
  Wire.write(magneto, 6);
}

/**************************************************************************/
/*! 
    @brief Send photodiode data for ADC #1
*/
/**************************************************************************/
void send_photodiodes1() {
  Wire.write(adc1pd, 6);
}

/**************************************************************************/
/*! 
    @brief Send photodiode data for ADC #2
*/
/**************************************************************************/
void send_photodiodes2() {
  Wire.write(adc2pd, 6);
}

/**************************************************************************/
/*! 
    @brief Send temperature from the IMU sensor
*/
/**************************************************************************/
void send_temperature() {
  Wire.write(imu_temperature);
}

/**************************************************************************/
/*! 
    @brief Send temperature from the TMP100 sensor
*/
/**************************************************************************/
void send_tmp100() {
  Wire.write(tmp100_temperature, 2);
}

/**************************************************************************/
/*! 
    @brief Send comm flag byte
*/
/**************************************************************************/
void send_commflags() {
  Wire.write(comm_flags);
}

/**************************************************************************/
/*! 
    @brief Set the collect_flag, respond with OK
*/
/**************************************************************************/
void collect_response() {
  collect_flag = true;
  Wire.write(OK);
}

/**************************************************************************/
/*! 
    @brief Collect data from the IMU, TMP100 and ADCs
*/
/**************************************************************************/
void collect_data() {
  // Reset all sensor variables
  clean_data();

  // BNO055 data
  count_bno = 0;
  while (count_bno <= 3) {
    // Tries to communicate with the device 3 times.
    if (bno.testComm()) {
      bno.enterNormalMode();
      delay(60);
      // Collect gyroscope data
      imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      temp_gyro[0] = gyroscope.x();
      temp_gyro[1] = gyroscope.y();
      temp_gyro[2] = gyroscope.z();

      // Float to byte conversion.
      // ERRATA - September 2022: This should be changed to limit to 100 dps,
      //                          since the conversion formula below is set to
      //                          work for ranges [-100, 100] dps.
      for (int8_t i = 0; i < 3; i++) {
        if (temp_gyro[i] < -125) temp_gyro[i] == -125;
        if (temp_gyro[i] >  125) temp_gyro[i] ==  125;
      }

      // Save each gyroscope axis into 1 byte
      for (int8_t i = 0; i < 3; i++) {
        gyro[i] = 1.275 * temp_gyro[i] + 127.5;
      }

      // Collect magnetometer data
      imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      temp_magneto_float[0] = magnetometer.x();
      temp_magneto_float[1] = magnetometer.y();
      temp_magneto_float[2] = magnetometer.z();

      // Float to byte conversion.
      // X and Y axes limited to max range of +/- 1300 uT
      // see Table 0-2 for Magnetic field range (https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
      for (int8_t i = 0; i < 4; i++) {
        if (temp_magneto_float[i] < -1300) temp_magneto_float[i] == -1300;
        if (temp_magneto_float[i] >  1300) temp_magneto_float[i] ==  1300;
      }

      // Z axis limited to max range of +/- 2500 uT
      // see Table 0-2 for Magnetic field range (https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
      for (int8_t i = 4; i < 6; i++) {
        if (temp_magneto_float[i] < -2500) temp_magneto_float[i] == -2500;
        if (temp_magneto_float[i] >  2500) temp_magneto_float[i] ==  2500;
      }

      // Save each magnetometer axis into 2 bytes
      temp_magneto[0] = (8192/325) * temp_magneto_float[0] + 65536/2;
      temp_magneto[1] = (8192/325) * temp_magneto_float[1] + 65536/2;
      temp_magneto[2] = (8192/625) * temp_magneto_float[2] + 65536/2;

      // Separate high and low bytes of each magnetometer axis
      for (int8_t i = 0; i < 3; i++) {
        magneto[2 * i] = temp_magneto[i] >> 8;
        magneto[2 * i + 1] = temp_magneto[i];
      }     
  
      imu_temperature = bno.getTemp();  // read temperature from IMU (1 byte, signed)
      bno.enterSuspendMode();           // suspend the IMU to conserve power
      comm_flags |= 0x01;               // set the IMU flag to 1
      break;
    }
    else {
      comm_flags &= 0xFE;               // set IMU flag to 0
      count_bno++;                      // increase the retry counter
    }
  }

  // ADC1 data
  count_adc1 = 0;
  while (count_adc1 <= 3) {
    // Tries to communicate with the device 3 times.
    if (adc1.testComm()) {
      // readConverted returns a double between 0 and 3.3 volts
      for (int8_t i = 0; i < 6; i++) {
        temp_adc1pd[i] = adc1.readConverted(i);
      }
      // convert each channel to 1 byte
      for (int8_t i = 0; i < 6; i++) {
        adc1pd[i] = 77.27 * temp_adc1pd[i];
      }

      comm_flags |= 0x02; // set the ADC1 flag to 1
      break;
    }
    else {
      comm_flags &= 0xFD; // set the ADC1 flag to 0
      count_adc1++;       // increase the retry counter
    }
  }

  // ADC2 data
  count_adc2 = 0;
  while (count_adc2 <= 3) {
    // Tries to communicate with the device 3 times.
    if (adc2.testComm()) {
      // readConverted returns a double between 0 and 3.3 volts
      for (int8_t i = 0; i < 6; i++) {
        temp_adc2pd[i] = adc2.readConverted(i);
      }
      // convert each channel to 1 byte
      for (int8_t i = 0; i < 6; i++) {
        adc2pd[i] = 77.27 * temp_adc2pd[i];
      }

      comm_flags |= 0x04; // set the ADC2 flag to 1
      break;
    }
    else {
      comm_flags &= 0xFB; // set the ADC2 flag to 0
      count_adc2++;       // increase the retry counter
    }
  }

  // TMP100 data
  count_tmp100 = 0;
  while (count_tmp100 < 3) {
    // Tries to communicate with the device 3 times.
    if (tmp100.wakeup()){
      temp_tmp100 = tmp100.readTempC(); // read the temperature (2 bytes, signed)
      tmp100.sleep();                   // sleep the sensor after reading

      // separate the high and low bytes
      tmp100_temperature[0] = temp_tmp100 >> 8;
      tmp100_temperature[1] = temp_tmp100;
      
      comm_flags |= 0x08; // set the TMP100 flag to 1
      break;
    }
    else {
      comm_flags &= 0xF7; // set the TMP100 flag to 0
      count_tmp100++;     // increase the retry counter
    }
  }
}

/**************************************************************************/
/*! 
    @brief Send complete data package
*/
/**************************************************************************/
void send_data() {
  send_gyroscopes();
  send_magnetometers();
  send_photodiodes1();
  send_photodiodes2(); 
  send_temperature();
  send_tmp100();   
  send_commflags();

  checkcomm_flag = true;
}

/**************************************************************************/
/*! 
    @brief Set the reset flag, to be checked in the main loop to perform a
    soft reset of this microcontroller
*/
/**************************************************************************/
void reset() {
  reset_flag = true;
  Wire.write(OK);
}

/**************************************************************************/
/*! 
    @brief Set the ADM reset flag, to be check in the main loop to perform
    a reset of the ADM GPIO Expander
*/
/**************************************************************************/
void adm_reset() {
  adm_reset_flag = true;
  Wire.write(OK);
}

/**************************************************************************/
/*! 
    @brief Soft reset, return program counter to initial position
*/
/**************************************************************************/
void(* resetFunc) (void) = 0;


//-------------------------------------------------------------------------/
//  MAIN ARDUINO FUNCTIONS
//-------------------------------------------------------------------------/

/**************************************************************************/
/*! 
    @brief Arduino setup function, configure GPIO and I2C buses,
    initialize flag values
*/
/**************************************************************************/
void setup() {
  // disable unnecessary modules
  powerSaving();

  // ADM is enable HIGH
  pinMode(ADM_RESET_PIN, OUTPUT);
  digitalWrite(ADM_RESET_PIN, HIGH);

  // Add components to the software wire bus
  bno.setI2C(adcs_wire);
  adc1.setI2C(adcs_wire);
  adc2.setI2C(adcs_wire);
  tmp100.setI2C(adcs_wire);

  // Init sensors
  bno_init();
  adc1_init();
  adc2_init();
  tmp100_init();  

  // OBC I2C Bus configuration
  Wire.begin(SLAVE_ADDR);         // join i2c bus with Slave ID
  Wire.onReceive(receiveEvent);   // Register a receive from master event
  Wire.onRequest(requestEvent);

  adcs_ready_flag = OK;
}

/**************************************************************************/
/*! 
    @brief Arduino loop, checks flags, executes if necessary, then sleeps
*/
/**************************************************************************/
void loop() {
  // reset the GPIO expander on the ADM PCB
  if (adm_reset_flag) {
    adm_reset_flag = false;
    digitalWrite(ADM_RESET_PIN, LOW);
    delayMicroseconds(1);
    digitalWrite(ADM_RESET_PIN, HIGH);
  }

  // soft reset this microcontroller
  if (reset_flag) {
    reset_flag = false;
    delay(1);
    resetFunc();
  }

  // collect data from the ADCS sensors
  if (collect_flag) {
    collect_data();
    collect_flag = false;     // Reset collect_flag status
  }

  // check comm with the ADCS sensors
  if (checkcomm_flag) {
    checkComm();
    checkcomm_flag = false;   // Reset checkcomm_flag
  }
  
  // go to highest power saving mode
  // will be woken up by an I2C command from the OBC
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

  // wait a while after power up from sleep to execute main loop again
  delay(15);
}

//-------------------------------------------------------------------------/
//  I2C BUS EVENT HANDLERS
//-------------------------------------------------------------------------/

/**************************************************************************/
/*! 
    @brief Handle for I2C onReceive. Handles behaviour when something is
    received
*/
/**************************************************************************/
void receiveEvent(int howMany) {
  wireCommand = Wire.read();
}

/**************************************************************************/
/*! 
    @brief Handle for I2C onRequest. Handles behaviour depending on what
    was received
*/
/**************************************************************************/
void requestEvent() {
  switch (wireCommand) {
    case ADCS_READY:
      adcs_ready();
      break;
    case GYROSCOPES:
      send_gyroscopes();
      break;
    case MAGNETOMETERS:
      send_magnetometers();
      break;
    case PHOTODIODES_1:
      send_photodiodes1();
      break;
    case PHOTODIODES_2:
      send_photodiodes2();
      break;
    case TEMPERATURE:
      send_temperature();
      break;
    case TMP100_DATA:
      send_tmp100();
      break;
    case SEND_TRANS_FLAGS:
      send_commflags();
      break;    
    case COLLECT_DATA:
      collect_response();
      break;
    case SEND_DATA:
      send_data();
      break;
    case RESET:
      reset();
      break;
    case ADM_RESET:
      adm_reset();
      break;
    default:
      break;
  }
  wireCommand = 0;
}
