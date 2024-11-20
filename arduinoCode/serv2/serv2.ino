#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150  // Min pulse length out of 4096
#define SERVOMAX 600  // Max pulse length out of 4096

// Helper function to map angle (0-180) to PCA9685 PWM pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
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
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz
}

void loop() {
  // Example usage: Set angles with speeds for each servo
  setServoAnglesWithSpeed(90, 10, 45, 5, 135, 20); // Set servo angles with different speeds
  delay(1000); // Wait 1 second
}
