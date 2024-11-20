#define stepPin 2
#define dirPin 3
#define stepsPerRevolution 200  // based on the Stepper's specifications

const int STEPS_PER_M = 1989; // steps per revolution / pitch circumference
const int MIN_STEPS = 0;
const int MAX_STEPS = 490; 

int delayValue = 5000; // Starting speed in microseconds
int position = 400;     // Motor position in steps
int currPosition = 0;
int targetPosition = 0;

void setup() {
  Serial.begin(9600); // Start serial communication
  
  pinMode(stepPin, OUTPUT); 
  pinMode(dirPin, OUTPUT);

  delay(1000);
  
  if (position > 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  Serial.println("initializing stepper...");
  // for (int x = 0; x < abs(stepsPerRevolution); x++) {
  //   digitalWrite(stepPin, HIGH);
  //   delayMicroseconds(delayValue);
  //   digitalWrite(stepPin, LOW);
  //   delayMicroseconds(delayValue);

  // }

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    targetPosition = STEPS_PER_M * atof(Serial.readString().c_str());
    targetPosition = min(targetPosition, MAX_STEPS);
    targetPosition = max(targetPosition, MIN_STEPS);

    Serial.println("New target");

  }
  if (targetPosition == currPosition) return;
  else if (targetPosition < currPosition) {
    digitalWrite(dirPin,LOW);
    currPosition -= 1;
  } else {
    digitalWrite(dirPin,HIGH);
    currPosition += 1;
  }
  Serial.println(currPosition);
  
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(delayValue);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayValue);
  // digitalWrite(dirPin, HIGH);
  // Serial.println("Spinning clockwise...");
  // for (int x = 0; x < position; x++) {
  //   
  // }
  // delay(250);
  // Serial.println("Spinning counter-clockwise...");
  // for (int x = 0; x < position; x++) {
  //   digitalWrite(stepPin, HIGH);
  //   delayMicroseconds(delayValue);
  //   digitalWrite(stepPin, LOW);
  //   delayMicroseconds(delayValue);

  // }
  // delay(250);
}



