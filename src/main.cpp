#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <AS5600.h>

#include "pid.h"

#define HOME_DIRECTION -1
#define HOME_SPEED 50
#define HOME_SLOW 25

#define MICROSTEPS 32

void readSerialInput();
void parseSerialInput();
void homeMotor();
void homingInterrupt();

AccelStepper stepper1(AccelStepper::DRIVER, 52, 53);
AS5600 encoder1;
int target1 = 0;
int prevTarget1 = 0;
bool changed1 = false;
volatile bool homingComplete1 = false;

PID pid1(20.0, 0.002, 0.5);


char buf[128];
int bufPos = 0;

int32_t lastPosition;

void setup() {
  stepper1.stop();
  Serial.begin(9600);

  Wire.begin();

  encoder1.begin(30);
  encoder1.setDirection(AS5600_CLOCK_WISE);

  Serial.println(encoder1.getAddress());

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), homingInterrupt, RISING);

  int b = encoder1.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  delay(1000);
  stepper1.setMaxSpeed(200 * MICROSTEPS);
  
  delay(2000);
  homeMotor();

  pid1.setTolerance(1.8);
  pid1.setPoint(0);

  //lastPosition = encoder.resetCumulativePosition(0);

  delay(5000);
}

void loop() {
  readSerialInput();

  static uint32_t lastTime = 0; 

  int32_t pos = encoder1.getCumulativePosition();
  double pos_deg = pos * AS5600_RAW_TO_DEGREES;

  double signal = pid1.update(pos_deg);

  if (signal != 0) {
    stepper1.setSpeed(-signal * MICROSTEPS);
    stepper1.runSpeed();
  } else {
    stepper1.setSpeed(0);
  }
    
  if (millis() - lastTime >= 100 && signal != 0) {
    lastTime = millis();
    Serial.print(pos_deg);
    Serial.print("\t");
    Serial.print(encoder1.rawAngle() * AS5600_RAW_TO_DEGREES);
    Serial.print("\n");
  }

}

void homingInterrupt() {
  if (homingComplete1) return;
  Serial.println("CLICKUP");
  homingComplete1 = true;
  stepper1.stop();
}

void parseSerialInput() {
  prevTarget1 = target1;
  target1 = strtod(buf, NULL);

  pid1.setPoint((double)target1);
}

void homeMotor() {
  Serial.println("Starting coarse homing...");
  homingComplete1 = false;
  stepper1.setSpeed(HOME_DIRECTION * HOME_SPEED * MICROSTEPS / 2);

  while (!homingComplete1) {
    stepper1.runSpeed();
  }

  stepper1.stop();
  Serial.println("Coarse homing complete");
  
  Serial.println("Retracting...");
  stepper1.move(-HOME_DIRECTION*30*MICROSTEPS);
  stepper1.setAcceleration(200);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }

  homingComplete1 = false;
  Serial.println("Starting fine homing...");
  stepper1.setSpeed(HOME_DIRECTION * HOME_SLOW * MICROSTEPS / 2);

  while (!homingComplete1) {
    stepper1.runSpeed();
  }

  stepper1.stop();
  Serial.println("Fine homing complete");

  stepper1.setCurrentPosition(0);
  encoder1.resetCumulativePosition(0);

  detachInterrupt(digitalPinToInterrupt(2));
}

void readSerialInput() {
  while (Serial.available()) {
    buf[bufPos] = Serial.read();

    if (buf[bufPos] == '\n') {
      buf[bufPos] = '\0';
      bufPos = 0;
      parseSerialInput();
    } else {
      bufPos++;
    }
  }
}