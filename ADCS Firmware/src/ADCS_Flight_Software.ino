/* Libraries */
#include "ADCS.h"
#include <avr/power.h>
#include <avr/sleep.h>
#include "LowPower.h"
#include "SoftwareWire.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include "ADC128D818.h"
#include "TMP100.h"

#define ADM_RESET_PIN   3

// Software wire configuration
SoftwareWire adcs_wire(A2,A3);

// BNO055 object declaration
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ADC1 object declaration.
ADC128D818 adc1(ADC1_ADDR);

// ADC2 object declaration.
ADC128D818 adc2(ADC2_ADDR);

// TMP100 object declaration
TMP100 tmp100(0);

byte wireCommand = 0;

byte adcs_ready_flag = 0;
bool collect_flag = false;    // Allows collecting data
bool checkcomm_flag = false;
bool reset_flag = false;
bool adm_reset_flag = false;

float temp_gyro[3] = {0};
byte gyro[3] = {0};

float temp_magneto_float[3] = {0};
uint16_t temp_magneto[3] = {0};
byte magneto[6] = {0};

float temp_adc1pd[6] = {0};
byte adc1pd[6] = {0};
float temp_adc2pd[6] = {0};
byte adc2pd[6] = {0};

int8_t imu_temperature = 0;
int16_t temp_tmp100 = 0;
byte tmp100_temperature[2] = {0};

// Communication flags
byte comm_flags = 0x00; // [TMP100_comm, adc1_comm, adc2_comm, imu_comm]

// Counter for communication iterations
uint8_t count_bno = 0;
uint8_t count_adc1 = 0;
uint8_t count_adc2 = 0;
uint8_t count_tmp100 = 0;


void powerSaving() {
  power_adc_disable();
  power_spi_disable();
  //power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
}


void bno_init() {
  // BNO055 Initialization
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


void adc1_init() {
  // ADC1 Initialization
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


void adc2_init() {
  // ADC2 Initialization
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


void tmp100_init() {
  // TMP100 Initialization
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


void bno_checkComm() {
  if (!comm_flags & (1<<0)) {
    bno_init();
  }
}


void adc1_checkComm() {
  if (!comm_flags & (1<<1)) {
    adc1_init();
  }
}


void adc2_checkComm() {
  if (!comm_flags & (1<<2)) {
    adc2_init();
  }
}


void tmp100_checkComm() {
  if (!comm_flags & (1<<3)) {
    tmp100_init();
  }
}


void checkComm() {
  bno_checkComm();
  adc1_checkComm();
  adc2_checkComm();
  tmp100_checkComm();
}


/************************* SEND DATA ****************************************/
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


// Send adcs_ready_flag
void adcs_ready() {
  Wire.write(adcs_ready_flag);
}

// Send gyroscope data
void send_gyroscopes() {
  Wire.write(gyro, 3);
}

// Send magnetometers data
void send_magnetometers() {
  Wire.write(magneto, 6);
}

// Send ADC1 photodiodes data
void send_photodiodes1() {
  Wire.write(adc1pd, 6);
}

// Send ADC2 photodiodes data
void send_photodiodes2() {
  Wire.write(adc2pd, 6);
}

// Send temperature data
void send_temperature() {
  Wire.write(imu_temperature);
}

// Send tmp100 data
void send_tmp100() {
  Wire.write(tmp100_temperature, 2);
}

void send_commflags() {
  Wire.write(comm_flags);
}

void collect_response() {
  collect_flag = true;
  Wire.write(OK);
}

/*********************** DATA COLLECT FROM IMU, TMP100 AND ADCS *******************/
void collect_data() {
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

      // Float to byte conversion. Limits in -31.42 and 31.42
      for (int8_t i = 0; i < 3; i++) {
        if (temp_gyro[i] < -125) temp_gyro[i] == -125;
        if (temp_gyro[i] >  125) temp_gyro[i] ==  125;
      }

      for (int8_t i = 0; i < 3; i++) {
        gyro[i] = 1.275 * temp_gyro[i] + 127.5;
      }

      // Collect magnetometer data
      imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      temp_magneto_float[0] = magnetometer.x();
      temp_magneto_float[1] = magnetometer.y();
      temp_magneto_float[2] = magnetometer.z();

      // Float to byte conversion. Limits in -100 and 100
      for (int8_t i = 0; i < 4; i++) {
        if (temp_magneto_float[i] < -1300) temp_magneto_float[i] == -1300;
        if (temp_magneto_float[i] >  1300) temp_magneto_float[i] ==  1300;
      }

      for (int8_t i = 4; i < 6; i++) {
        if (temp_magneto_float[i] < -2500) temp_magneto_float[i] == -2500;
        if (temp_magneto_float[i] >  2500) temp_magneto_float[i] ==  2500;
      }

      temp_magneto[0] = (8192/325) * temp_magneto_float[0] + 65536/2;
      temp_magneto[1] = (8192/325) * temp_magneto_float[1] + 65536/2;
      temp_magneto[2] = (8192/625) * temp_magneto_float[2] + 65536/2;

      for (int8_t i = 0; i < 3; i++) {
        magneto[2 * i] = temp_magneto[i] >> 8;
        magneto[2 * i + 1] = temp_magneto[i];
      }     
  
      imu_temperature = bno.getTemp();
      bno.enterSuspendMode();
      comm_flags |= 0x01;
      break;
    }
    else {
      comm_flags &= 0xFE;
      count_bno++;
    }
  }

  /* 
  * Photodiodes float to byte conversion.
  * Initiate a single conversion and comparison cycle when the device is in shutdown mode or deep shutdown mode, 
  * after which the device returns to the respective mode that it was in. This register is not a data register, 
  * and it is the write operation that causes the one-shot conversion. The data written to this address is 
  * irrelevant and is not stored. A zero will always be read from this register.
  */

  // ADC1 data
  count_adc1 = 0;
  while (count_adc1 <= 3) {
    // Tries to communicate with the device 3 times.
    if (adc1.testComm()) {
      //readConverted returns a double.
      for (int8_t i = 0; i < 6; i++) {
        temp_adc1pd[i] = adc1.readConverted(i);
      }
      for (int8_t i = 0; i < 6; i++) {
        adc1pd[i] = 77.27 * temp_adc1pd[i];
      }

      comm_flags |= 0x02;
      break;
    }
    else {
      comm_flags &= 0xFD;
      count_adc1++;
    }
  }

  // ADC2 data
  count_adc2 = 0;
  while (count_adc2 <= 3) {
    // Tries to communicate with the device 3 times.
    if (adc2.testComm()) {
      //readConverted returns a double.
      for (int8_t i = 0; i < 6; i++) {
        temp_adc2pd[i] = adc2.readConverted(i);
      }
      for (int8_t i = 0; i < 6; i++) {
        adc2pd[i] = 77.27 * temp_adc2pd[i];
      }

      comm_flags |= 0x04;
      break;
    }
    else {
      comm_flags &= 0xFB;
      count_adc2++;
    }
  }

  // TMP100 data
  count_tmp100 = 0;
  while (count_tmp100 < 3) {
    if (tmp100.wakeup()){
      temp_tmp100 = tmp100.readTempC();
      tmp100.sleep();

      tmp100_temperature[0] = temp_tmp100 >> 8;
      tmp100_temperature[1] = temp_tmp100;
      
      comm_flags |= 0x08;
      break;
    }
    else {
      comm_flags &= 0xF7;
      count_tmp100++;
    }
  }
}

/*******************************************************************************/

// Send complete data package
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

void reset() {
  reset_flag = true;
  Wire.write(OK);
}

void adm_reset() {
  adm_reset_flag = true;
  Wire.write(OK);
}

void(* resetFunc) (void) = 0;


void setup() {
  powerSaving();

  pinMode(ADM_RESET_PIN, OUTPUT);
  digitalWrite(ADM_RESET_PIN, HIGH);

  // Add components to the software wire bus
  bno.setI2C(adcs_wire);
  adc1.setI2C(adcs_wire);
  adc2.setI2C(adcs_wire);
  tmp100.setI2C(adcs_wire);

  //Init sensors
  bno_init();
  adc1_init();
  adc2_init();
  tmp100_init();  

  // OBC I2C Bus configuration
  Wire.begin(SLAVE_ADDR);         // join i2c bus with Slave ID
  Wire.onReceive(receiveEvent);   // Register a recieve from master event
  Wire.onRequest(requestEvent);

  adcs_ready_flag = OK;
}


void loop() {
  if (adm_reset_flag) {
    adm_reset_flag = false;
    digitalWrite(ADM_RESET_PIN, LOW);
    delayMicroseconds(1);
    digitalWrite(ADM_RESET_PIN, HIGH);
  }

  if (reset_flag) {
    reset_flag = false;
    delay(1);
    resetFunc();
  }

  if (collect_flag) {
    collect_data();
    collect_flag = false;     // Reset collect_flag status
  }

  if (checkcomm_flag) {
    checkComm();
    checkcomm_flag = false;   // Reset checkcomm_flag
  }
  
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  delay(15);
}


void receiveEvent(int howMany) {
  wireCommand = Wire.read();
}


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
