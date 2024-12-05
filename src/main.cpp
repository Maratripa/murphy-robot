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
#define SDA_2 18
#define SCL_2 19

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define LOX1_SHT 7
#define LOX2_SHT 6

#define HOME_DIRECTION 1
#define HOME_SPEED 50
#define HOME_SLOW 25

#define MICROSTEPS 16

// Brazo
#define SSTEP1 27
#define SDIR1 14
#define REDUCTION1 3.0
#define ENDSTOP1 5

// Rotacion base
#define SSTEP2 26
#define SDIR2 25
#define REDUCTION2 4.70588235294117 // 80/17
#define ENDSTOP2 4

#define SSTEP3 32
#define SDIR3 33
#define REDUCTION3 1.0

#define tx2 17
#define rx2 16

AccelStepper stepper1(AccelStepper::DRIVER, SSTEP1, SDIR1);
AS5600 encoder1(&Wire);
int32_t pos_deg1 = 0;
double target1 = 0;
bool homingComplete1 = false;
PID pid1(20.0, 0.002, 0.5);

AccelStepper stepper2(AccelStepper::DRIVER, SSTEP2, SDIR2);
AS5600 encoder2(&Wire1);
int32_t pos_deg2 = 0;
double target2 = 0;
bool homingComplete2 = false;
PID pid2(20.0, 0.002, 0.5);

AccelStepper stepperz(AccelStepper::DRIVER, SSTEP3, SDIR3);
uint8_t currentZ = 0;
double targetz = 0;
int lastSensorTime = 0;
int arrivedz = 0;
PID pidz(3000.0, 1, 5);

Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();

char buf[128];
int bufPos = 0;
int recived = 0;
int to_clean = 0;

int motorResponseSent = 1;
bool newMessage = false;

int32_t lastPosition;

void readSerialInput();
void parseSerialInput();
void homeMotors();
void homeMotor1(void);
void homeMotor2(void);

void homingInterrupt1();
void homingInterrupt2();
void readEncoders();
void setupTOFSensors();
uint8_t readSensor(Adafruit_VL6180X &vl, uint8_t num);
uint8_t readSensorFast(Adafruit_VL6180X &vl, uint8_t num);

void setupFullRutine(void);
void setupMotorTest(void);
void loopFullRutine(void);
void loopMotorTest(void);

void setup() {
  Serial2.begin(115200, SERIAL_8N1,rx2,tx2); // raspberry comunication
  Serial.begin(9600);

  Wire.begin();
  Wire1.begin(SDA_2, SCL_2);

  stepper1.stop();
  stepper2.stop();

  stepper1.setMaxSpeed(200 * MICROSTEPS * REDUCTION1);
  stepper2.setMaxSpeed(200 * MICROSTEPS * REDUCTION2);
  stepperz.setMaxSpeed(1000 * MICROSTEPS);

  setupTOFSensors();

  pinMode(ENDSTOP1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP1), homingInterrupt1, RISING);

  pinMode(ENDSTOP2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP2), homingInterrupt2, RISING);

  encoder1.begin();
  encoder1.setDirection(AS5600_COUNTERCLOCK_WISE);

  encoder2.begin();
  encoder2.setDirection(AS5600_COUNTERCLOCK_WISE);

  int b = encoder1.isConnected();
  int c = encoder2.isConnected();
  Serial.println("Connected: ");
  Serial.println(b);
  Serial.println(" | ");
  Serial.println(c);

  currentZ = readSensor(lox1,10);

  //homeMotors();
  // encoder1.resetCumulativePosition(0);
  // encoder2.resetCumulativePosition(0);
  // stepper1.setCurrentPosition(0);
  // stepper2.setCurrentPosition(0);




  pid1.setTolerance(1.8 / REDUCTION1);
  pid1.setPoint(0);

  pid2.setTolerance(1.8 / REDUCTION2);
  pid2.setPoint(0);

  pidz.setTolerance(2);
  pidz.setPoint(126);




}

void loop() { 
  readSerialInput();
  //readEncoders();
 
  



 int32_t pos1 = encoder1.getCumulativePosition();
  double pos_deg1 = pos1 * AS5600_RAW_TO_DEGREES / REDUCTION1;
  double signal1 = pid1.update(pos_deg1);

  if (signal1 != 0) {
     stepper1.setSpeed(-signal1 * MICROSTEPS * REDUCTION1);
     stepper1.runSpeed();
   } else {
     stepper1.setSpeed(0);
   }

  int32_t pos2 = encoder2.getCumulativePosition();
  double pos_deg2 = pos2 * AS5600_RAW_TO_DEGREES / REDUCTION2;
  double signal2 = pid2.update(pos_deg2);

  if (signal2 != 0) {
     stepper2.setSpeed(signal2 * MICROSTEPS * REDUCTION2);
     stepper2.runSpeed();
   } else {
     stepper2.setSpeed(0);
   }

  

 
  if (signal1 == 0 && signal2 == 0) {
    if (currentZ - targetz < 8 && currentZ - targetz > 0 ) {
        currentZ = readSensorFast(lox1,20);
    } else {
        currentZ =readSensor(lox1,1);
    }
    double signalz = pidz.update(currentZ);

    if (signalz != 0 && arrivedz == 0) {
       stepperz.setSpeed(10 * signalz * MICROSTEPS);
       stepperz.runSpeed();
    }  else {
        stepperz.setSpeed(0);
        arrivedz = 1;

         if (motorResponseSent == 0) {
    Serial2.write("G;");
    motorResponseSent = 1;
  }

    }
  } else {
      stepperz.setSpeed(0);

  }  
  
}

void homingInterrupt1() {
  if (homingComplete1) return;
  homingComplete1 = true;
  // stepper1.stop();
}

void homingInterrupt2() {
  if (homingComplete2) return;
  homingComplete2 = true;
  // stepper2.stop();
}       


void parseSerialInput() {
  char *ptr = buf;
  if (*ptr == ',') ptr++; 

  switch (*ptr) {
    case 'm': {
      motorResponseSent = 0;
      ptr++;
      if (*ptr == ',') ptr++;
      target1 = strtod(ptr, &ptr); 
      if (*ptr == ',') ptr++;
      target2 = strtod(ptr, &ptr);
      if (*ptr == ',') ptr++;
      targetz = strtod(ptr, &ptr);
      arrivedz = 0;

      //2. Agregar pid y probar cada motor por separado
      pid1.setPoint(target1);
      pid2.setPoint(target2);

      if(targetz > 126) {
        targetz = 126;
      }
      pidz.setPoint(targetz); // quitar offset de sensor de distancia
      //3. Probar todo junto

      Serial.print("Moviendo motor 1 a ");
      Serial.print(target1);
      Serial.print(", motor 2 a ");
      Serial.print(target2);
      Serial.print(", y motor 3 a ");
      Serial.println(targetz);
      break;
    }
    case 's': {
      Serial.println("Stopping");
      
      break;
    }
    case 'h': {
      Serial.println("Homing");
      homeMotors();
      break;
    }
    case 'r': {
      ptr++;
      if (*ptr == ',') ptr++; 
      if (strncmp(ptr, "z1", 2) == 0) {
        ptr += 2;
        if (*ptr == ',') ptr++; 
        int num_samples = atoi(ptr); 

        char response[32];
        uint8_t z1 = readSensor(lox1,num_samples);
        snprintf(response, sizeof(response), "z1,%d;", z1);
        Serial2.write(response);

      } else if (strncmp(ptr, "z2", 2) == 0) {
        ptr += 2;
        if (*ptr == ',') ptr++; 
        int num_samples = atoi(ptr); 
        
        char response[32];
        uint8_t z2 = readSensor(lox2,num_samples);
        snprintf(response, sizeof(response), "z2,%d;", z2);
        Serial2.write(response);
      } else {
        Serial.println("Comando r no reconocido");
      }
      break;
    }
    default:
      Serial.println("Comando no reconocido");
      break;
  }
}

void readSerialInput() {
  memset(buf, 0, sizeof(buf));
  bufPos = 0;
  while (Serial2.available()) {
    char incomingByte = Serial2.read();
    buf[bufPos] = incomingByte;
    bufPos++;

    if (incomingByte == ';') {
      buf[bufPos] = '\0';
      parseSerialInput();
      memset(buf, 0, sizeof(buf));
      bufPos = 0;
    }
  }
}

void homeMotor1() {
  Serial.println("Starting coarse homing 1...");
  homingComplete1 = false;
  stepper1.setSpeed(HOME_DIRECTION * HOME_SPEED * MICROSTEPS / 2);

  while (!homingComplete1) {
    stepper1.runSpeed();
  }

  stepper1.stop();
  Serial.println("Coarse homing 1 complete");
  
  Serial.println("Retracting...");
  stepper1.move(-HOME_DIRECTION*30*MICROSTEPS);
  stepper1.setAcceleration(200);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }

  homingComplete1 = false;
  Serial.println("Starting fine homing 1...");
  stepper1.setSpeed(HOME_DIRECTION * HOME_SLOW * MICROSTEPS / 2);

  while (!homingComplete1) {
    stepper1.runSpeed();
  }

  stepper1.stop();

  stepper1.setCurrentPosition(0);
 // encoder1.resetCumulativePosition(0);

  detachInterrupt(digitalPinToInterrupt(ENDSTOP1)); }

  void homeMotor2() {
  Serial.println("Starting coarse homing 2...");
  homingComplete2 = false;
  stepper2.setSpeed(-HOME_DIRECTION * HOME_SPEED * MICROSTEPS / 2);

  while (!homingComplete2) {
    stepper2.runSpeed();
  }

  stepper2.stop();
  Serial.println("Coarse homing 2 complete");
  
  Serial.println("Retracting...");
  stepper2.move(HOME_DIRECTION*30*MICROSTEPS);
  stepper2.setAcceleration(200);
  while (stepper2.distanceToGo() != 0) {
    stepper2.run();
  }

  homingComplete2 = false;
  Serial.println("Starting fine homing 2...");
  stepper2.setSpeed(-HOME_DIRECTION * HOME_SLOW * MICROSTEPS / 2);

  while (!homingComplete1) {
    stepper2.runSpeed();
  }

  stepper2.stop();

  stepper2.setCurrentPosition(0);
 // encoder1.resetCumulativePosition(0);

  detachInterrupt(digitalPinToInterrupt(ENDSTOP2)); }


void homeMotors() {
  Serial.println("Starting coarse homing 1...");
  homingComplete1 = false;
  stepper1.setSpeed(HOME_DIRECTION * HOME_SPEED * MICROSTEPS / 2);

  while (!homingComplete1) {
    stepper1.runSpeed();
  }

  stepper1.stop();
  Serial.println("Coarse homing 1 complete");
  
  Serial.println("Retracting...");
  stepper1.move(-HOME_DIRECTION*30*MICROSTEPS);
  stepper1.setAcceleration(200);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }

  homingComplete1 = false;
  Serial.println("Starting fine homing 1...");
  stepper1.setSpeed(HOME_DIRECTION * HOME_SLOW * MICROSTEPS / 2);

  while (!homingComplete1) {
    stepper1.runSpeed();
  }

  stepper1.stop();
  Serial.println("Fine homing 1 complete");


  delay(500);

  stepper1.setCurrentPosition(0);
  encoder1.resetCumulativePosition(0);

  detachInterrupt(digitalPinToInterrupt(ENDSTOP1));

  Serial.println("Starting coarse homing 2...");
  homingComplete2 = false;
  stepper2.setSpeed(-HOME_DIRECTION * HOME_SPEED * MICROSTEPS / 2);

  while (!homingComplete2) {
    stepper2.runSpeed();
  }

  stepper2.stop();
  Serial.println("Coarse homing 2 complete");

  Serial.println("Retracting...");
  stepper2.move(HOME_DIRECTION*30*MICROSTEPS);
  stepper2.setAcceleration(200);
  while (stepper2.distanceToGo() != 0) {
    stepper2.run();
  }

  homingComplete2 = false;
  Serial.println("Starting fine homing 2...");
  stepper2.setSpeed(-HOME_DIRECTION* HOME_SLOW * MICROSTEPS / 2);

  while (!homingComplete2) {
    stepper2.runSpeed();
  }

  stepper2.stop();
  Serial.println("Fine homing 2 complete");


  delay(500);

  stepper2.setCurrentPosition(0);
  encoder2.resetCumulativePosition(0);

  detachInterrupt(digitalPinToInterrupt(ENDSTOP2));

  Serial2.write("G;");

}

int lastTime = 0;
void readEncoders() {
    pos_deg1 = encoder1.getCumulativePosition() * AS5600_RAW_TO_DEGREES;

  pos_deg2 = encoder2.getCumulativePosition() * AS5600_RAW_TO_DEGREES;

  // if (millis() - lastTime >= 200) {
  //   Serial.print("Encoder 2 ");
  //   Serial.println(pos_deg2);
  //   lastTime = millis();
  // }

   if (millis() - lastTime >= 200) {
    Serial.print("Encoder 1 ");
    Serial.println(pos_deg1);
    Serial.print("Encoder 2 ");
    Serial.println(pos_deg2);
    lastTime = millis();
  }

  // pos_deg2 = encoder2.getCumulativePosition();
  // Serial.print("Encoder 2 ");
  // Serial.println(pos_deg2);
}

void setupTOFSensors() {
 

  if (!lox1.begin(&Wire)) {
    Serial.println("Failed to boot first VL6180X");
    while (1);
  }

  if (!lox2.begin(&Wire1)) {
    Serial.println("Failed to boot second VL6180X");
  }

}

uint8_t readSensor(Adafruit_VL6180X &vl,uint8_t num) {

  if (millis() - lastSensorTime < 1000) {
    return currentZ;
   }

  lastSensorTime = millis();


  uint16_t suma = 0;

  for (int i = 0; i < num; i++) {
    suma += vl.readRange(); 
      }

  float promedio = (float)suma / num;

  int entero = int(promedio);
  int decimal = int((promedio - entero)* 10);

  //   char response1[32];
  // snprintf(response1, sizeof(response1), "entero,%d;", entero);

  // char response[32];
  // snprintf(response, sizeof(response), "decimal,%d;", decimal);

  // Serial.print(response1);
  // Serial.print(response);

  if (decimal >= 3) {
    entero++;
  }

  return entero;
  
  // uint8_t status = vl.readRangeStatus();

  // if (status == VL6180X_ERROR_NONE) {
  //   return range;
  // } else {
  //   return 255;
  // }
}

uint8_t readSensorFast(Adafruit_VL6180X &vl,uint8_t num) {

  if (millis() - lastSensorTime < 500) {
    return currentZ;
   }

  lastSensorTime = millis();


  uint16_t suma = 0;

  for (int i = 0; i < num; i++) {
    suma += vl.readRange(); 
      }

  float promedio = (float)suma / num;

  int entero = int(promedio);
  int decimal = int((promedio - entero)* 10);

  if (decimal >= 3) {
    entero++;
  }

  return entero;
}
