#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define stepPin1 2
#define dirPin1 3
#define stepPin2 4
#define dirPin2 5
#define stepPin3 6
#define dirPin3 7
#define stepsPerRevolution 1000  // based on the Stepper's specifications
#define SERVOMIN 150  // Min pulse length out of 4096
#define SERVOMAX 600  // Max pulse length out of 4096

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const double STEPS_PER_M = 1989; // steps per revolution / pitch circumference
const int MIN_STEPS = 0;
const int MAX_STEPS[7] = {490, 398, 159, 180, 180, 180, 180}; 

float speed  = 1; // speed in rotations per second
int delayValue = round((1/speed) * stepsPerRevolution);    // delay between each step
int position = 400;     // Motor position in steps
int currPosition = 0;

int current[7] = {0,0,0,0,0,0,0}; //array of current stepper positions. {X,Y,Z,rot1,rot2,rot3,grip}
int target[7] = {0,0,0,0,0,0,0}; //array of desired stepper positions. {X,Y,Z,rot1,rot2,rot3,grip}
int stepPins[3] = {stepPin1,stepPin2,stepPin3};
int dirPins[3] = {dirPin1,dirPin2,dirPin3};


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
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);

  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60Hz

  // Set servos to a 0
  //moveServoToAngle(1,90,180,20);
  Serial.println("meow");
  delay(1000);
}

void loop() {
  // Read the raspi's message
  if (Serial.available()) {
    String str = Serial.readStringUntil('\n');
    char* targetXYZ = str.c_str(); // in form: xxxxx,yyyyy,zzzzz,rot11111,rot2222,rot333,gripyyyyy
    //targetXYZ * pch;     // (an attempt to) split raspi's message by commas into an array
    char* pch = strtok(targetXYZ, ",");
    int index = 0;
    while(pch != NULL){
      Serial.println(pch);
      if(strcmp("x", pch) == 0) {
        target[index]=current[index];
      } else {
        target[index] = int(STEPS_PER_M * atof(pch));   //convert cstring to double
      target[index] = min(target[index], MAX_STEPS[index]);   // ensure target position is inbetween MAX_STEPS and MIN_STEPS
      target[index] = max(target[index], MIN_STEPS);
      }
      pch = strtok(NULL, ",");
      index++;
    }
    
  //    Serial.println("New target (m*1989)");
  //    char s[5];
  //    sprintf(s, "%d, %d, %d", target[0], target[1], target[2]);
  //    Serial.println(s);
  }
  

  //Move the steppers one at a time
  for(int i=0;i<3;i++){
    //determine rotation direction
    if(target[i] == current[i]) continue;
    else if (target[i] < current[i]){
      digitalWrite(dirPins[i],LOW);
      current[i] -= 1;
    } else {
      digitalWrite(dirPins[i],HIGH);
      current[i] += 1;
    }
//    Serial.println(current[i]);
  }

  for(int i=0;i<3;i++) {
    //move the stepper
    if(target[i] != current[i])
      digitalWrite(stepPins[i], HIGH);
  }
  delayMicroseconds(delayValue);
  
  for(int i=0;i<3;i++) {
    //move the stepper
    if(target[i] != current[i])
      digitalWrite(stepPins[i], LOW);
  }
  delayMicroseconds(delayValue);

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
}
