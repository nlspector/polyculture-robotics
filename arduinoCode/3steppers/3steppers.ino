// Define pins for each stepper motor
#define stepPin1 2
#define dirPin1 3
#define stepPin2 4
#define dirPin2 5
#define stepPin3 6
#define dirPin3 7

#define stepsPerRevolution 200  // Steps per revolution for the stepper motor

int delayValue = 5000; // Starting speed in microseconds
int position = 400;    // Motor position in steps

void setup() {
  Serial.begin(9600); // Start serial communication
  
  // Set pin modes for each motor
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);

  delay(1000);

  // Set initial direction for each motor based on the position variable
  digitalWrite(dirPin1, position > 0 ? HIGH : LOW);
  digitalWrite(dirPin2, position > 0 ? HIGH : LOW);
  digitalWrite(dirPin3, position > 0 ? HIGH : LOW);

  // Move each motor one revolution
  for (int x = 0; x < abs(stepsPerRevolution); x++) {
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(delayValue);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(delayValue);
  }
}

void loop() {
  // Rotate motors in one direction
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
  delay(250);

  // Rotate motors in the opposite direction
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
  delay(250);
}

