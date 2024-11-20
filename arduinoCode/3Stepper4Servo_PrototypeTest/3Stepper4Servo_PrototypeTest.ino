/*
This program is designed to run all 3 stepper motors and 4 servos at the same time,
allowing for testing the Polyculture robot's first prototype full mechanization.
This program will just run each actuator back & forth between two set points.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Define stepper motor values
#define stepPin1 2
#define dirPin1 3
#define stepPin2 4
#define dirPin2 5
#define stepPin3 6
#define dirPin3 7
#define stepsPerRevolution 200  // Steps per revolution for the stepper motor
int delayValue = 5000; // Starting speed in microseconds
int position = 400;    // Desired motor position in # of steps

//Define servo motor values
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_MIN 92  // Min pulse length out of 4096. Calibrated to be * 92 *.
#define SERVO_MAX 680  // Max pulse length out of 4096. Calibrated to be * 680 *.
#define NUM_SERVOS 4
#define SERVO_DELAY 1000  // delay in miliseconds
int servoPins[NUM_SERVOS] = {0,1,2,3};

// ----- Define servo motor functions ----- //
// Helper function to map angle (0-180) to PCA9685 PWM pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 210, SERVO_MIN, SERVO_MAX);
}

// Function to move servo to a target angle at a specified speed
void moveServoToAngle(int channel, int currentAngle, int targetAngle, int speed) {

  int step = (targetAngle > currentAngle) ? 1 : -1; // Determine the direction of movement
  int delayTime = 1000 / speed;  // Speed control: higher speed means smaller delay
  
  // Move servo in small steps towards target angle
  for (int angle = currentAngle; angle != targetAngle; angle += step) {
    pwm.setPWM(channel, 0, angleToPulse(angle));
    delay(delayTime); // Control speed by delaying between steps
  }
  // Set the final angle to ensure it reaches target precisely
  pwm.setPWM(channel, 0, angleToPulse(targetAngle));
}

// Function to set angles for three servos with different speeds
void setServoAnglesWithSpeed(int angle1, int speed1, int angle2, int speed2, int angle3, int speed3) {
  // Example current angles; can be updated dynamically if you track servo positions
  static int currentAngle1 = 90;
  static int currentAngle2 = 90;
  static int currentAngle3 = 90;

  moveServoToAngle(0, currentAngle1, angle1, speed1);  // Move Servo 1
  moveServoToAngle(1, currentAngle2, angle2, speed2);  // Move Servo 2
  moveServoToAngle(2, currentAngle3, angle3, speed3);  // Move Servo 3

  // Update current angles
  currentAngle1 = angle1;
  currentAngle2 = angle2;
  currentAngle3 = angle3;
}

void setup() {
  Serial.begin(9600); //begin serial communication

  // Setup steppers
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  // Set initial direction for each stepper based on the position variable
  digitalWrite(dirPin1, position > 0 ? HIGH : LOW);
  digitalWrite(dirPin2, position > 0 ? HIGH : LOW);
  digitalWrite(dirPin3, position > 0 ? HIGH : LOW);

  delay(1000); // Wait one second

  //Move each stepper one full revolution
  Serial.println("Moving all steppers through one full revolution...");
  for (int x = 0; x < abs(stepsPerRevolution); x++) {
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(delayValue);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(delayValue);
    Serial.println(x);
  }
  Serial.println("hello!");
  Serial.println("Steppers initiated.");
  Serial.println("hello");
  // Initiate servos 
  for(int i = 0; i < NUM_SERVOS; i++){
    pwm.setPWM(servoPins[i],0,SERVO_MIN);
    Serial.print("Servo "); Serial.print(i); Serial.print(" has ben initiated."); Serial.println("");
  }
  delay(500); //wait half a second
  Serial.println("Moving all servos through one full revolution...");
  pwm.setPWM(0,0,angleToPulse(0));
  pwm.setPWM(1,0,angleToPulse(0));
  pwm.setPWM(2,0,angleToPulse(0));
  pwm.setPWM(3,0,angleToPulse(0));
}

void loop() {
  // Will run steppers back and forth through a full revolution and the servos
  // back and forth through 0 deg to 90 deg

  // ---- Move steppers ---- //
  // Rotate steppers in one direction
  Serial.println("Moving steppers...");
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, HIGH);
  digitalWrite(dirPin3, HIGH);
  for (int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(delayValue);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(delayValue);
  }
  delay(250);  //wait 1/4 of a second

  // Rotate stepppers in opposite direction
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  digitalWrite(dirPin3, LOW);
  for (int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(delayValue);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(delayValue);
  }
  delay(250); //wait 1/4 of a second

  
  // ---- Move Servos ---- //
  Serial.println("Moving servos...");
  moveServoToAngle(0, 0, 90, 100);
  moveServoToAngle(1, 0, 90, 100);
  moveServoToAngle(2, 0, 90, 100);
  moveServoToAngle(3, 0, 90, 100);
  delay(5000); //wait 5 seconds
  moveServoToAngle(0, 90, 0, 100);
  moveServoToAngle(1, 90, 0, 100);
  moveServoToAngle(2, 90, 0, 100);
  moveServoToAngle(3, 90, 0, 100);

}
