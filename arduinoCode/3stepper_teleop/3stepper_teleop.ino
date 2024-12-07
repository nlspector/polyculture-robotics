#define stepPin1 2
#define dirPin1 3
#define stepPin2 4
#define dirPin2 5
#define stepPin3 6
#define dirPin3 7
#define stepsPerRevolution 200  // based on the Stepper's specifications

const double STEPS_PER_M = 1989; // steps per revolution / pitch circumference
const int MIN_STEPS = 0;
const int MAX_STEPS = 490; 

int delayValue = 5000; // Starting speed in microseconds
int position = 400;     // Motor position in steps
int currPosition = 0;

int current[3] = {0,0,0}; //array of current stepper positions. {X,Y,Z}
int target[3]; //array of desired stepper positions. {X,Y,Z}
int stepPins[3] = {stepPin1,stepPin2,stepPin3};
int dirPins[3] = {dirPin1,dirPin2,dirPin3};

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
    String str = Serial.readStringUntil('\n');
    char* targetXYZ = str.c_str(); // in form: xxxxx,yyyyy,zzzzz
    //targetXYZ * pch;     // (an attempt to) split raspi's message by commas into an array
    char* pch = strtok(targetXYZ, ",");
    int index = 0;
    while(pch != NULL){
      if(strcmp("x", pch) == 0) {
        target[index]=current[index];
      } else {
        target[index] = int(STEPS_PER_M * atof(pch));   //convert cstring to double
//      target[index] = min(target[index], MAX_STEPS);   // ensure target position is inbetween MAX_STEPS and MIN_STEPS
//      target[index] = max(target[index], MIN_STEPS);
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

    //move the stepper
    digitalWrite(stepPins[i], HIGH);
    delayMicroseconds(delayValue);
    digitalWrite(stepPins[i], LOW);
    delayMicroseconds(delayValue);
  }
}
