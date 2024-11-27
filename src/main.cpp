#include <Arduino.h>
#include <AccelStepper.h>

#define MICROSTEPS 32

#define SSTEP1 48
#define SDIR1 49

AccelStepper stepper(AccelStepper::DRIVER, SSTEP1, SDIR1);

void setup() {
  stepper.setMaxSpeed(200 * MICROSTEPS);
  stepper.setAcceleration(200);
}

void loop() {
  stepper.moveTo(50*MICROSTEPS);
  stepper.setSpeed(100*MICROSTEPS);
  stepper.runSpeedToPosition();
}