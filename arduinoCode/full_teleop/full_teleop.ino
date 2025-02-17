#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

#define stepPin1 2
#define dirPin1 3
#define stepPin2 4
#define dirPin2 5
#define stepPin3 6
#define dirPin3 7
#define stepsPerRevolution 200  // based on the Stepper's specifications
#define SERVOMIN 150  // Min pulse length out of 4096
#define SERVOMAX 600  // Max pulse length out of 4096

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// 32 microsteps
const double STEPS_PER_M = 63648.0; // steps per revolution / pitch circumference
const long MIN_STEPS = 0;
const long MAX_STEPS[7] = {15680, 12736, 5088, 180, 180, 180, 180}; 

float speed  = 1; // speed in rotations per second
int delayValue = 2000;
int position = 400;     // Motor position in steps
int currPosition = 0;

long current[7] = {0,0,0,0,0,0,0}; //array of current stepper positions. {X,Y,Z,rot1,rot2,rot3,grip}
long target[7] = {0,0,0,0,0,0,0}; //array of desired stepper positions. {X,Y,Z,rot1,rot2,rot3,grip}
int stepPins[3] = {stepPin1,stepPin2,stepPin3};
int dirPins[3] = {dirPin1,dirPin2,dirPin3};

unsigned long lastStep = 0;

AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);
AccelStepper stepper3(AccelStepper::DRIVER, stepPin3, dirPin3);

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
  // Set pin modes for each motor  
//  pinMode(stepPin1, OUTPUT);
//  pinMode(dirPin1, OUTPUT);
//  pinMode(stepPin2, OUTPUT);
//  pinMode(dirPin2, OUTPUT);
//  pinMode(stepPin3, OUTPUT);
//  pinMode(dirPin3, OUTPUT);

 // Configure each stepper
  stepper1.setMaxSpeed(4000);
  stepper2.setMaxSpeed(4000);
  stepper3.setMaxSpeed(4000);
 
  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);

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
//        Serial.println(pch);
        if(strcmp("x", pch) == 0) {
          target[index]=current[index];
        } else {
          target[index] = long(STEPS_PER_M * atof(pch));   //convert cstring to double
          target[index] = min(target[index], MAX_STEPS[index]);   // ensure target position is inbetween MAX_STEPS and MIN_STEPS
          target[index] = max(target[index], MIN_STEPS);
        }
        pch = strtok(NULL, ",");
        index++;
      }
    }
//      Serial.println("New target (m*63648)");
//      char s[100];
//      sprintf(s, "%ld, %ld, %ld", target[0], target[1], target[2]);
//      Serial.println(s);
  }

  // position 3 is backwards
  long positions[3] = {target[0], target[1], -target[2]};
  steppers.moveTo(positions);
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
  for(int i=0;i<4;i++){
    // TODO
    if (target[i+3] != current[i+3]) {
      int step = (target[i+3] > current[i+3]) ? 1 : -1;
      pwm.setPWM(i, 0, angleToPulse(current[i+3]));
      current[i+3] += step;
//      Serial.print("setting ");
//      Serial.print(i);
//      Serial.print(" to ");
//      Serial.println(current[i+3]);
      continue;
    }
  }
  if (getCommand == 1) {
    for(int i=0;i<7;i++){
      if (i == 0) {
       Serial.print(((double) stepper1.currentPosition())/STEPS_PER_M);
      } else if (i == 1) {
       Serial.print(((double) stepper2.currentPosition())/STEPS_PER_M);
      }
      else if (i == 2) {
       Serial.print(((double) stepper3.currentPosition())/STEPS_PER_M);
      } else {
        Serial.print(((double) current[i])/STEPS_PER_M);
      }
      if (i != 6) {
        Serial.print(",");
      }
    }
    Serial.println("");
  }
  
}
