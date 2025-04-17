#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

#define stepsPerRevolution 200  // based on the Stepper's specifications
#define SERVOMIN 150  // Min pulse length out of 4096
#define SERVOMAX 600  // Max pulse length out of 4096

#define numSteppers 7

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// 32 microsteps
const double STEPS_PER_M = 63648.0; // steps per revolution / pitch circumference
const long MIN_STEPS = 0;
const long MAX_STEPS[7] = {15680, 12736, 5088, 180, 180, 180, 180}; 

float speed  = 1; // speed in rotations per second
int delayValue = 2000;
int position = 400;     // Motor position in steps
int currPosition = 0;

long current[numSteppers] = {0,0,0,0,0,0,0}; //array of current stepper positions. {X,Y,Z,rot1,rot2,rot3,grip}
long target[numSteppers] = {0,0,0,0,0,0,0}; //array of desired stepper positions. {X,Y,Z,rot1,rot2,rot3,grip}

// Begin Stepper Configuration

struct StepperInfo {
  short stepPin;
  short dirPin;
  int minSteps;
  int maxSteps;
  int maxSpeed;
  double stepsPerM;
  double maxAcceleration;
};

const StepperInfo stepperInfo[numSteppers] = {
  // First DOF
  {
    49, // stepPin
    48, // dirPin
    -1000, // minSteps
    1000, // maxSteps
    400, // maxSpeed
    2508, // stepsPerM
    200 // maxAcceleration
  },

  // Crossbar DOF
  {
    45, // stepPin
    44, // dirPin
    -1000, // minSteps
    1000, // maxSteps
    200, // maxSpeed
    2508, // stepsPerM
    200 // maxAcceleration
  },
  
  // Vertical DOF
  {
    53, // stepPin
    52, // dirPin
    0, // minSteps
    1000, // maxSteps
    400, // maxSpeed
    5016, // stepsPerM
    200 // maxAcceleration
  },

  // Wrist
  {
    41, // stepPin
    40, // dirPin
    -2000, // minSteps
    2000, // maxSteps
    1000, // maxSpeed
    1000, // stepsPerRadian
    1000 // maxAcceleration
  },

  // Diff 1
  {
    33, // stepPin
    32, // dirPin
    -1000, // minSteps
    1000, // maxSteps
    75, // maxSpeed
    350, // stepsPerRadian
    200 // maxAcceleration
  },

  // Diff 2
  {
    11, // stepPin
    10, // dirPin
    -1000, // minSteps
    1000, // maxSteps
    75, // maxSpeed
    350, // stepsPerRadian
    200 // maxAcceleration
  },

  // Gripper
  {
    9, // stepPin
    8, // dirPin
    -10000, // minSteps
    10000, // maxSteps
    1000, // maxSpeed
    1000, // stepsPerRadian
    1000 // maxAcceleration
  },
  
};

unsigned long lastStep = 0;

AccelStepper singleSteppers[numSteppers];

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

AccelStepper cutter;

void setup() {
  Serial.begin(115200); // Start serial communication

  Serial.println("initializing steppers ...");
  for (int i = 0; i < numSteppers; i++) {
    singleSteppers[i] = AccelStepper(AccelStepper::DRIVER, stepperInfo[i].stepPin, stepperInfo[i].dirPin);
    singleSteppers[i].setMaxSpeed(stepperInfo[i].maxSpeed);
    singleSteppers[i].setAcceleration(stepperInfo[i].maxAcceleration);
//    steppers.addStepper(singleSteppers[i]);
    
  }
  cutter = AccelStepper(AccelStepper::HALF4WIRE, 24, 22, 25, 23);
  cutter.setMaxSpeed(2000);
  cutter.setAcceleration(1000);
  
  Serial.println("meow");
  delay(1000);
}

void loop() {

  // Read the raspi's message
  int getCommand = 0;
  if (Serial.available()) {
    String str = Serial.readStringUntil('\n');
    char* targetXYZ = str.c_str(); // in form: xxxxx,yyyyy,zzzzz,rot11111,rot2222,rot333,gripyyyyy
    //targetXYZ * pch;     // (an attempt to) split raspi's message by commas into an array
    if (strcmp("get", targetXYZ) == 0) {
      getCommand = 1;
    } else if (strcmp("cut", targetXYZ) == 0) {
        cutter.moveTo(160);
    } else if (strcmp("uncut", targetXYZ) == 0) {
        cutter.moveTo(0);
    } else {
      char* pch = strtok(targetXYZ, ",");
      int index = 0;
      while(pch != NULL){
//        Serial.println(pch);
        if(strcmp("x", pch) == 0) {
          target[index] = singleSteppers[index].currentPosition();
          singleSteppers[index].stop();
//          Serial.print(index);
//          Serial.print(": ");
//          Serial.print(target[index]);
//          Serial.println("");
        } else {
          target[index] = long(stepperInfo[index].stepsPerM * atof(pch));   //convert cstring to double
          target[index] = min(target[index], stepperInfo[index].maxSteps);   // ensure target position is inbetween MAX_STEPS and MIN_STEPS
          target[index] = max(target[index], stepperInfo[index].minSteps);
          singleSteppers[index].moveTo(target[index]);
        }
        pch = strtok(NULL, ",");
        index++;
      }
//      steppers.moveTo(target);
    }
//      Serial.println("New target (m*63648)");
//      char s[100];
//      sprintf(s, "%ld, %ld, %ld", target[0], target[1], target[2]);
//      Serial.println(s);
  }
  for (int i = 0; i < numSteppers; i++) {
    singleSteppers[i].run();
  }

  cutter.run();
//  if (cutter.currentPosition() >= 5) {
//    cutter.moveTo(-5);
//  }
//  if (cutter.currentPosition() <= -5) {
//    cutter.moveTo(5);
//  }

  if (getCommand == 1) {
    for(int i=0;i<numSteppers;i++){
       Serial.print(((double) singleSteppers[i].currentPosition())/stepperInfo[i].stepsPerM);
      if (i != numSteppers - 1) {
        Serial.print(",");
      }
    }
    Serial.println("");
  }
  
}
