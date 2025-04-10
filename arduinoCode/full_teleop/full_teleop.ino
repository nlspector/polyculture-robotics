#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

#define stepsPerRevolution 200  // based on the Stepper's specifications
#define SERVOMIN 150  // Min pulse length out of 4096
#define SERVOMAX 600  // Max pulse length out of 4096

#define numSteppers 5

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// 32 microsteps
const double STEPS_PER_M = 63648.0; // steps per revolution / pitch circumference
const long MIN_STEPS = 0;
const long MAX_STEPS[7] = {15680, 12736, 5088, 180, 180, 180, 180}; 

float speed  = 1; // speed in rotations per second
int delayValue = 2000;
int position = 400;     // Motor position in steps
int currPosition = 0;

long current[numSteppers] = {0,0,0,0,0}; //array of current stepper positions. {X,Y,Z,rot1,rot2,rot3,grip}
long target[numSteppers] = {0,0,0,0,0}; //array of desired stepper positions. {X,Y,Z,rot1,rot2,rot3,grip}

// Begin Stepper Configuration

struct StepperInfo {
  short stepPin;
  short dirPin;
  int minSteps;
  int maxSteps;
  int maxSpeed;
  double stepsPerM;
};

const StepperInfo stepperInfo[numSteppers] = {
  // First DOF
  {
    22, // stepPin
    23, // dirPin
    0, // minSteps
    1000, // maxSteps
    100, // maxSpeed
    1000 // stepsPerM
  },

  // Crossbar DOF
  {
    38, // stepPin
    39, // dirPin
    0, // minSteps
    1000, // maxSteps
    100, // maxSpeed
    1000 // stepsPerM
  },
  
  // Vertical DOF
  {
    3, // stepPin
    4, // dirPin
    0, // minSteps
    1000, // maxSteps
    100, // maxSpeed
    1000 // stepsPerM
  },

  {
    8, // stepPin
    9, // dirPin
    0, // minSteps
    1000, // maxSteps
    100, // maxSpeed
    1000 // stepsPerM
  },
  
  {
    10, // stepPin
    11, // dirPin
    0, // minSteps
    1000, // maxSteps
    100, // maxSpeed
    1000 // stepsPerM
  }
};

unsigned long lastStep = 0;

AccelStepper singleSteppers[numSteppers];

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

// Helper function to map angle (0-180) to PCA9685 PWM pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Function to move servo to a target angle at a specified speed
void moveServoToAngle(int channel, int currentAngle, int targetAngle, int speed) {
  int step = (targetAngle > currentAngle) ? 1 : -1; // Determine the direction of movement
  if (targetAngle == currentAngle) { return; }
  int delayTime = 1000 / speed;  // Speed control: higher speed means smaller delay
  
  // Move servo in small steps towards target angle
  for (int angle = currentAngle; angle != targetAngle; angle += step) {
    pwm.setPWM(channel, 0, angleToPulse(angle));
    delay(delayTime); // Control speed by delaying between steps
  }
  // Set the final angle to ensure it reaches target precisely
  pwm.setPWM(channel, 0, angleToPulse(targetAngle));
}

void setup() {
  Serial.begin(115200); // Start serial communication

  Serial.println("initializing steppers ...");
  for (int i = 0; i < numSteppers; i++) {
    singleSteppers[i] = AccelStepper(AccelStepper::DRIVER, stepperInfo[i].stepPin, stepperInfo[i].dirPin);
    singleSteppers[i].setMaxSpeed(stepperInfo[i].maxSpeed);
    steppers.addStepper(singleSteppers[i]);
    
  }

//  pwm.begin();
//  pwm.setPWMFreq(60); // Analog servos run at ~60Hz

  // Set servos to a 0
  //moveServoToAngle(1,90,180,20);
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
    } else {
      char* pch = strtok(targetXYZ, ",");
      int index = 0;
      while(pch != NULL){
        Serial.println(pch);
        if(strcmp("x", pch) == 0) {
          target[index] = singleSteppers[index].currentPosition();
          singleSteppers[index].stop();
          Serial.print(index);
          Serial.print(": ");
          Serial.print(target[index]);
          Serial.println("");
        } else {
          target[index] = long(stepperInfo[index].stepsPerM * atof(pch));   //convert cstring to double
          target[index] = min(target[index], stepperInfo[index].maxSteps);   // ensure target position is inbetween MAX_STEPS and MIN_STEPS
          target[index] = max(target[index], stepperInfo[index].minSteps);
        }
        pch = strtok(NULL, ",");
        index++;
      }
      steppers.moveTo(target);
    }
//      Serial.println("New target (m*63648)");
//      char s[100];
//      sprintf(s, "%ld, %ld, %ld", target[0], target[1], target[2]);
//      Serial.println(s);
  }
  steppers.run();

//  //Move the steppers one at a time
//  for(int i=0;i<3;i++){
//    //determine rotation direction
//    if(target[i] == current[i]) continue;
//    else if (target[i] < current[i]){
//      digitalWrite(dirPins[i],LOW);
//      current[i] -= 1;
//    } else {
//      digitalWrite(dirPins[i],HIGH);
//      current[i] += 1;
//    }
////    Serial.println(current[i]);
//  }
//
//  for(int i=0;i<3;i++) {
//    //move the stepper
//    if(target[i] != current[i])
//      digitalWrite(stepPins[i], HIGH);
//  }
//  delayMicroseconds(15);
//  
//  for(int i=0;i<3;i++) {
//    //move the stepper
//    if(target[i] != current[i])
//      digitalWrite(stepPins[i], LOW);
//  }
//  unsigned long elapsedTime = micros() - lastStep;
//  if (elapsedTime < delayValue) {
//    delayMicroseconds(delayValue - elapsedTime);
//  }
//  lastStep = micros();
  

  // move the servos one at a time
//  for(int i=0;i<4;i++){
//    // TODO
//    if (target[i+3] != current[i+3]) {
//      int step = (target[i+3] > current[i+3]) ? 1 : -1;
////      pwm.setPWM(i, 0, angleToPulse(current[i+3]));
//      current[i+3] += step;
////      Serial.print("setting ");
////      Serial.print(i);
////      Serial.print(" to ");
////      Serial.println(current[i+3]);
//      continue;
//    }
//  }
  if (getCommand == 1) {
    for(int i=0;i<numSteppers;i++){
       Serial.print(((double) singleSteppers[i].currentPosition())/STEPS_PER_M);
//      if (i == 0) {
//       Serial.print(((double) stepper1.currentPosition())/STEPS_PER_M);
//      } else if (i == 1) {
//       Serial.print(((double) stepper2.currentPosition())/STEPS_PER_M);
//      }
//      else if (i == 2) {
//       Serial.print(((double) -stepper3.currentPosition())/STEPS_PER_M);
//      } else {
//        Serial.print(((double) current[i])/STEPS_PER_M);
//      }
      if (i != numSteppers - 1) {
        Serial.print(",");
      }
    }
    Serial.println("");
  }
  
}
