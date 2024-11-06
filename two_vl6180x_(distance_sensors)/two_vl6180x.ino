#include <Adafruit_VL6180X.h>

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 7
#define SHT_LOX2 6

// objects for the VL6180X
Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();

/*
    Reset both sensors by controlling their shutdown pins.
    Initialize each sensor separately by toggling their shutdown pins and setting their I2C address.
*/
void setID() {
  // Step 1: Reset both sensors
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // Step 2: Initialize LOX1 (sensor 1)
  digitalWrite(SHT_LOX1, HIGH);  // Enable sensor 1
  digitalWrite(SHT_LOX2, LOW);  // Disable sensor 3

  delay(10);  // Give time for the sensor to boot

  if (!lox1.begin()) {
    Serial.println(F("Failed to boot first VL6180X"));
    while (1);
  }
  lox1.setAddress(LOX1_ADDRESS);  // Change I2C address for LOX1
  delay(10);

  // Step 3: Initialize LOX2 (sensor 2)
  digitalWrite(SHT_LOX2, HIGH);  // Enable LOX2
  delay(10);  // Give time for the sensor to boot

  if (!lox2.begin()) {
    Serial.println(F("Failed to boot second VL6180X"));
    while (1);
  }
  lox2.setAddress(LOX2_ADDRESS);  // Change I2C address for LOX2
  delay(10);
}

uint8_t readSensor(Adafruit_VL6180X &vl) {
  uint8_t range = vl.readRange();
  Serial.print(range);
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    return range;
  } else {
    Serial.print(" Error: ");
    Serial.println(status);
    return 9999;
  }
}

void read_sensors() {
  uint8_t range1 = readSensor(lox1);
  uint8_t range2 = readSensor(lox2);
  Serial.print(String("Sensor 1: ") + range1 + String(", Sensor 2: ") + range2);
  Serial.print('\n');

}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println("Initializing sensors...");
  setID();
}

void loop() {
  read_sensors();
  delay(100);  // Read every 100 ms
}


