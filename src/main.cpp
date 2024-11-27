#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <AS5600.h>
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_VL6180X.h>

#include "pid.h"

// 35 Y 34 son solo input
#define SDA_2 19
#define SCL_2 18

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define LOX1_SHT 7
#define LOX2_SHT 6

#define HOME_DIRECTION -1
#define HOME_SPEED 50
#define HOME_SLOW 25

#define MICROSTEPS 32

#define SSTEP1 32
#define SDIR1 33
#define REDUCTION1 3.0
#define ENDSTOP1 2

#define SSTEP2 27
#define SDIR2 14
#define REDUCTION2 80/17
#define ENDSTOP2 4

#define SSTEP3 25
#define SDIR3 26
#define REDUCTION3 1.0

#define tx2 17
#define rx2 16

AccelStepper stepper1(AccelStepper::DRIVER, SSTEP1, SDIR1);
AS5600 encoder1(&Wire);
double target1 = 0;
volatile bool homingComplete1 = false;
PID pid1(20.0, 0.002, 0.5);

AccelStepper stepper2(AccelStepper::DRIVER, SSTEP2, SDIR2);
AS5600 encoder2(&Wire1);
double target2 = 0;
volatile bool homingComplete2 = false;
PID pid2(20.0, 0.002, 0.5);

AccelStepper stepperz(AccelStepper::DRIVER, SSTEP3, SDIR3);
double targetz = 0;
PID pidz(20.0, 0.002, 0.5);

Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();

char buf[128];
int bufPos = 0;
int recived = 0;
int to_clean = 0;

int32_t lastPosition;

void readSerialInput();
void parseSerialInput();
void homeMotors();
void homingInterrupt1();
void homingInterrupt2();
void setupTOFSensors();
uint8_t readSensor(Adafruit_VL6180X &vl);
void setupFullRutine(void);
void setupMotorTest(void);
void loopFullRutine(void);
void loopMotorTest(void);

void setup() {
  Serial2.begin(115200, SERIAL_8N1,rx2,tx2); // raspberry comunication
  Serial.begin(9600);
  stepper2.stop();
  stepper2.setMaxSpeed(200 * MICROSTEPS);

  /*stepper1.stop();
  stepper2.stop();
  

  Wire.begin();
  Wire1.begin(SDA_2, SCL_2);

  pinMode(LOX1_SHT, OUTPUT);
  pinMode(LOX2_SHT, OUTPUT);
  Serial2.println("Initializing sensors...");
  // setupTOFSensors();

  encoder1.begin(30);
  encoder1.setDirection(AS5600_CLOCK_WISE);

  encoder2.begin(31);
  encoder2.setDirection(AS5600_CLOCK_WISE);

  Serial2.println(encoder1.getAddress());

  pinMode(ENDSTOP1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP1), homingInterrupt1, RISING);

  pinMode(ENDSTOP2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP2), homingInterrupt2, RISING);

  int b = encoder1.isConnected();
  int c = encoder2.isConnected();
  Serial2.print("Connected: ");
  Serial2.print(b);
  Serial2.print(" | ");
  Serial2.println(c);

  stepper1.setMaxSpeed(200 * MICROSTEPS);
  stepper2.setMaxSpeed(200 * MICROSTEPS);
  stepperz.setMaxSpeed(200 * MICROSTEPS);
  stepper1.setSpeed(-3200);
  stepper1.runSpeed();
  delay(500);
  delay(2000);

  homeMotors();

  pid1.setTolerance(1.8);
  pid1.setPoint(0);

  pid2.setTolerance(1.8);
  pid2.setPoint(0);

  pidz.setTolerance(1);
  pidz.setPoint(30);

  //lastPosition = encoder.resetCumulativePosition(0);

  delay(5000);*/
}

void loop() { 
  readSerialInput();
  if (atoi(buf) == 1){
    stepper1.setSpeed(1600);
    
  } else if (atoi(buf) == 2){
    stepper1.setSpeed(-1600);
  } else {
    stepper1.setSpeed(0);
  }
  stepper1.runSpeed();

  /*static uint32_t lastTime = 0; 

  int32_t pos1 = encoder1.getCumulativePosition();
  double pos_deg1 = pos1 * AS5600_RAW_TO_DEGREES;
  double signal1 = pid1.update(pos_deg1);

  int32_t pos2 = encoder2.getCumulativePosition();
  double pos_deg2 = pos2 * AS5600_RAW_TO_DEGREES;
  double signal2 = pid2.update(pos_deg2);

  uint8_t posz = readSensor(lox1);
  double signalz = pidz.update(posz);

  if (signal1 != 0) {
    stepper1.setSpeed(-signal1 * MICROSTEPS);
    stepper1.runSpeed();
  } else {
    stepper1.setSpeed(0);
  }

  if (signal2 != 0) {
    stepper2.setSpeed(-signal2 * MICROSTEPS);
    stepper2.runSpeed();
  } else {
    stepper2.setSpeed(0);
  }

  if (signalz != 0) {
    stepperz.setSpeed(-signalz * MICROSTEPS);
    stepperz.runSpeed();
  } else {
    stepperz.setSpeed(0);
  }
    
  // if (millis() - lastTime >= 100 && signal1 != 0) {
  //   lastTime = millis();
  //   Serial2.print(pos_deg1);
  //   Serial2.print("\t");
  //   Serial2.print(encoder1.rawAngle() * AS5600_RAW_TO_DEGREES);
  //   Serial2.print("\n");
  // }*/
}

void homingInterrupt1() {
  if (homingComplete1) return;
  homingComplete1 = true;
  stepper1.stop();
}

void homingInterrupt2() {
  if (homingComplete2) return;
  homingComplete2 = true;
  stepper2.stop();
}

void parseSerialInput() {
  char *ptr;

  target1 = strtod(buf, &ptr) * REDUCTION1;

  ptr++;

  target2 = strtod(ptr, &ptr) * REDUCTION2;

  ptr++;

  targetz = strtod(ptr, &ptr) * REDUCTION3;

  pid1.setPoint(target1);
  pid2.setPoint(target2);
  pidz.setPoint(targetz);
}

void homeMotors() {
  Serial2.println("Starting coarse homing 1...");
  homingComplete1 = false;
  stepper1.setSpeed(HOME_DIRECTION * HOME_SPEED * MICROSTEPS / 2);

  while (!homingComplete1) {
    stepper1.runSpeed();
  }

  stepper1.stop();
  Serial2.println("Coarse homing 1 complete");
  
  Serial2.println("Retracting...");
  stepper1.move(-HOME_DIRECTION*30*MICROSTEPS);
  stepper1.setAcceleration(200);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }

  homingComplete1 = false;
  Serial2.println("Starting fine homing 1...");
  stepper1.setSpeed(HOME_DIRECTION * HOME_SLOW * MICROSTEPS / 2);

  while (!homingComplete1) {
    stepper1.runSpeed();
  }

  stepper1.stop();
  Serial2.println("Fine homing 1 complete");

  stepper1.setCurrentPosition(0);
  encoder1.resetCumulativePosition(0);

  detachInterrupt(digitalPinToInterrupt(ENDSTOP1));

  Serial2.println("Starting coarse homing 2...");
  homingComplete2 = false;
  stepper2.setSpeed(HOME_DIRECTION * HOME_SPEED * MICROSTEPS / 2);

  while (!homingComplete2) {
    stepper2.runSpeed();
  }

  stepper2.stop();
  Serial2.println("Coarse homing 2 complete");

  Serial2.println("Retracting...");
  stepper2.move(-HOME_DIRECTION*30*MICROSTEPS);
  stepper2.setAcceleration(200);
  while (stepper2.distanceToGo() != 0) {
    stepper2.run();
  }

  homingComplete2 = false;
  Serial2.println("Starting fine homing 2...");
  stepper2.setSpeed(HOME_DIRECTION* HOME_SLOW * MICROSTEPS / 2);

  while (!homingComplete2) {
    stepper2.runSpeed();
  }

  stepper2.stop();
  Serial2.println("Fine homing 2 complete");

  stepper2.setCurrentPosition(0);
  encoder2.resetCumulativePosition(0);

  detachInterrupt(digitalPinToInterrupt(ENDSTOP2));
}

void readSerialInput() {
  
  while (Serial2.available()>0) {
    buf[bufPos] = Serial2.read();
    if (buf[bufPos] == ';') {
      if (to_clean == 0){
        memset(buf,0,sizeof(buf));
        to_clean = 1;
      };
      for (int i = 0 ; i <= bufPos; i++){ 
        Serial2.write(buf[i]);
        };
      buf[bufPos] = '\0';
      bufPos = 0;
      parseSerialInput();
    } 
    else {
      bufPos++;
    }
  }
}

void setupTOFSensors() {
  digitalWrite(LOX1_SHT, LOW);
  digitalWrite(LOX2_SHT, LOW);
  delay(10);

  digitalWrite(LOX1_SHT, HIGH);
  digitalWrite(LOX2_SHT, HIGH);
  delay(10);

  digitalWrite(LOX1_SHT, HIGH);
  digitalWrite(LOX2_SHT, LOW);
  delay(10);

  if (!lox1.begin()) {
    Serial2.println("Failed to boot first VL6180X");
    while (1);
  }

  lox1.setAddress(LOX1_ADDRESS);
  delay(10);

  digitalWrite(LOX2_SHT, HIGH);
  delay(10);

  if (!lox2.begin()) {
    Serial2.println("Failed to boot second VL6180X");
  }

  lox2.setAddress(LOX2_ADDRESS);
  delay(10);
}

uint8_t readSensor(Adafruit_VL6180X &vl) {
  uint8_t range = vl.readRange();
  
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    return range;
  } else {
    return 255;
  }
}
