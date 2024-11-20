#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 92  // Min pulse length out of 4096. Calibrated to be * 92 *.
#define SERVO_MAX 680  // Max pulse length out of 4096. Calibrated to be * 680 *.
#define NUM_SERVOS 4
#define SERVO_DELAY 1000  // delay in miliseconds

int servoPins[NUM_SERVOS] = {0,1,2,3};


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
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz
  
  for(int i = 0; i < NUM_SERVOS; i++){
    pwm.setPWM(servoPins[i],0,SERVO_MIN);
    Serial.print("Servo "); Serial.print(i); Serial.print(" has ben initiated"); Serial.println("");
  }
}

void loop() {
  // Example usage: Set angles with speeds for each servo
  //setServoAnglesWithSpeed(90, 10, 45, 5, 135, 20); // Set servo angles with different speeds
  delay(1000); // Wait 1 second
  //pwm.setPWM(0,0,SERVO_MAX);
  //pwm.setPWM(1,0,SERVO_MAX);
  //pwm.setPWM(2,0,SERVO_MAX);
  //pwm.setPWM(3,0,SERVO_MAX);
  moveServoToAngle(0, 0, 180, 100);
  moveServoToAngle(1, 0, 180, 100);
  moveServoToAngle(2, 0, 180, 100);
  moveServoToAngle(3, 0, 180, 100);

  Serial.println("Moved to 180 degrees");
  delay(5000);
  //pwm.setPWM(0,0,SERVO_MIN);
  //pwm.setPWM(1,0,SERVO_MIN);
  //pwm.setPWM(2,0,SERVO_MIN);
  //pwm.setPWM(3,0,SERVO_MIN);
  pwm.setPWM(0,0,angleToPulse(0));
  pwm.setPWM(1,0,angleToPulse(0));
  pwm.setPWM(2,0,angleToPulse(0));
  pwm.setPWM(3,0,angleToPulse(0));
  Serial.println("Moved to 0 degrees");
}
