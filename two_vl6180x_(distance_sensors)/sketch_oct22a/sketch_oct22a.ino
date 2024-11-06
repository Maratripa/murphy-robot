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
    Reset both sensors by setting both XSHUT pins low for delay(10),
    then set both XSHUT high to bring out of reset.
    Initialize both sensors with lox.begin(new_i2c_address).
*/
void setID() {
  // reset both sensors
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  // bring both sensors out of reset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // initialize LOX1
  if (!lox1.begin()) {
    Serial.println(F("Failed to boot first VL6180X"));
    while (1);
  }
  lox1.setAddress(LOX1_ADDRESS);
  delay(10);

  // initialize LOX2
  if (!lox2.begin()) {
    Serial.println(F("Failed to boot second VL6180X"));
    while (1);
  }
  lox2.setAddress(LOX2_ADDRESS);
}

void readSensor(Adafruit_VL6180X &vl) {
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    Serial.print(" Range: ");
    Serial.println(range);
  } else {
    Serial.print(" Error: ");
    Serial.println(status);
  }
}

void read_sensors() {
  Serial.print("Sensor 1");
  readSensor(lox1);
  Serial.print("Sensor 2");
  readSensor(lox2);
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
