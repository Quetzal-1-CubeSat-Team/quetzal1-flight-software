#include "Wire.h"
#include "ADC128D818.h"

ADC128D818 adc(0x1D);

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  adc.setReferenceMode(EXTERNAL_REF);
  adc.setReference(2.048);
  adc.begin();
}

void loop() {
  // IN0-IN6 ...
  for (int i = 0; i < 7; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(adc.readConverted(i));
    Serial.println();
  }
  // ... and the internal temp sensor
  Serial.print("Temp: ");
  Serial.print(adc.readTemperatureConverted());
  Serial.println(" deg C");

  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}