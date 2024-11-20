#define stepPin 2
#define dirPin 5

int delayValue = 875; // Starting speed in microseconds
int position = 0;     // Motor position in steps
int stepsPerRevolution = 360; // Change this based on the motor's steps per revolution

void setup() {
  pinMode(stepPin, OUTPUT); 
  pinMode(dirPin, OUTPUT);
  Serial.begin(9600); // Start serial communication
  Serial.println("Enter speed in microseconds (delay between steps):");
}

void loop() {
  // Check if new speed is entered through Serial
  if (Serial.available() > 0) {
    int newSpeed = Serial.parseInt();
    if (newSpeed > 0) {
      delayValue = newSpeed;
      Serial.print("Speed set to: ");
      Serial.println(delayValue);
    }
  }

  // Rotate clockwise
  digitalWrite(dirPin, HIGH);
  for (int x = 0; x < 800; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayValue);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayValue);
    position++; // Update position
  }

  // Rotate counterclockwise
  digitalWrite(dirPin, LOW);
  for (int x = 0; x < 1600; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayValue);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayValue);
    position--; // Update position
  }

  // Print the position
  Serial.print("Current Position: ");
  Serial.println(position);
}


