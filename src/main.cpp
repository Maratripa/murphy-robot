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

AccelStepper stepper(AccelStepper::DRIVER, 52, 53);
int smolTarget = 0;
int prevSmolTarget = 0;
bool smolChanged = false;

PID pid(20.0, 0.002, 1.0);

AS5600 encoder;
double encoderOffset = 0.0;

char buf[128];
int bufPos = 0;

volatile bool homingComplete = false;

int32_t lastPosition;

void setup() {
  stepper.stop();
  Serial.begin(9600);

  Wire.begin();

  encoder.begin(31);
  encoder.setDirection(AS5600_CLOCK_WISE);

  Serial.println(encoder.getAddress());

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), homingInterrupt, RISING);

  int b = encoder.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  delay(1000);
  stepper.setMaxSpeed(200 * MICROSTEPS);
  
  delay(2000);
  homeMotor();

  pid.setTolerance(1.8);
  pid.setPoint(0);

  //lastPosition = encoder.resetCumulativePosition(0);

  delay(5000);
}

void loop() {
  readSerialInput();

  static uint32_t lastTime = 0; 

  int32_t pos = encoder.getCumulativePosition();
  double pos_deg = pos * AS5600_RAW_TO_DEGREES;

  double signal = pid.update(pos_deg);

  if (signal != 0) {
    stepper.setSpeed(-signal * MICROSTEPS);
    stepper.runSpeed();
  } else {
    stepper.setSpeed(0);
  }
    
  if (millis() - lastTime >= 100 && signal != 0) {
    lastTime = millis();
    Serial.print(pos_deg);
    Serial.print("\t");
    Serial.print(encoder.rawAngle() * AS5600_RAW_TO_DEGREES);
    Serial.print("\n");
  }

}

void homingInterrupt() {
  if (homingComplete) return;
  Serial.println("CLICKUP");
  homingComplete = true;
  stepper.stop();
}

void parseSerialInput() {
  prevSmolTarget = smolTarget;
  smolTarget = strtod(buf, NULL);

  pid.setPoint((double)smolTarget);
}

void homeMotor() {
  Serial.println("Starting coarse homing...");
  homingComplete = false;
  stepper.setSpeed(HOME_DIRECTION * HOME_SPEED * MICROSTEPS / 2);

  while (!homingComplete) {
    stepper.runSpeed();
  }

  stepper.stop();
  Serial.println("Coarse homing complete");
  
  Serial.println("Retracting...");
  stepper.move(-HOME_DIRECTION*30*MICROSTEPS);
  stepper.setAcceleration(200);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  homingComplete = false;
  Serial.println("Starting fine homing...");
  stepper.setSpeed(HOME_DIRECTION * HOME_SLOW * MICROSTEPS / 2);

  while (!homingComplete) {
    stepper.runSpeed();
  }

  stepper.stop();
  Serial.println("Fine homing complete");

  stepper.setCurrentPosition(0);
  encoder.resetCumulativePosition(0);

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