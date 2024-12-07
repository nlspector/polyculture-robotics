#define stepPin1 2
#define dirPin1 3
#define stepPin2 4
#define dirPin2 5
#define stepPin3 6
#define dirPin3 7
#define stepsPerRevolution 200  // based on the Stepper's specifications

const int STEPS_PER_M = 1989; // steps per revolution / pitch circumference
const int MIN_STEPS = 0;
const int MAX_STEPS = 490; 

int delayValue = 5000; // Starting speed in microseconds
int position = 400;     // Motor position in steps
int currPosition = 0;

double current[3] = {0,0,0}; //array of current stepper positions. {X,Y,Z}
double target[3]; //array of desired stepper positions. {X,Y,Z}
int stepPins[3] = {2,4,6};
int dirPins[3] = {2,5,7};}

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

  delay(1000);
}

void loop() {
  // Read the raspi's message
  if (Serial.available()) {
    targetXYZ = Serial.readString().c_str();   // in form: xxxxx,yyyyy,zzzzz
    targetXYZ * pch;     // (an attempt to) split raspi's message by commas into an array
    pch = strtok(targetXYZ, ",");
    index = 0;
    while(pch != NULL){
      target[index] = atof(pch);   //convert cstring to double
      target[index] = min(target[index], MAX_STEPS));   // ensure target position is inbetween MAX_STEPS and MIN_STEPS
      target[index] = max(target[index], MIN_STEPS));
      index++;
    }
    
    Serial.println("New target");

  }

  //Move the steppers one at a time
  for(i=0;i<3;i++){
    //determine rotation direction
    if(target[i] == current[i]) return;
    else if (target[i] < current[i]){
      digitalWrite(dirPins[i],LOW);
      current[i] -= 1;
    } else {
      digitalWrite(dirPins[i],HIGH);
      current[i] += 1;
    }
    Serial.println(current[i]);

    //move the stepper
    digitalWrite(stepPins[i], HIGH);
    delayMicroseconds(delayValue);
    digitalWrite(stepPins[i], LOW);
    delayMicroseconds(delayValue);
  }
}



