/*
Jordan Shaw
Added Secondary Motor
Timer to test and control wheel movements with forward, stop, left and right
*/

// Motor One
int enablePin = 11;
int in1Pin = 10;
int in2Pin = 9;

// Motor Two
int enablePin2 = 6;
int in3Pin = 2;
int in4Pin = 3;

int switchPin = 7;
int potPin = 0;

const int onTime1 = 0; // in ms
const int offTime1 = 1000; // in ms

const int onTime2 = 1001; // in ms
const int offTime2 = 2000; // in ms

const int onTime3 = 2001; // in ms
const int offTime3 = 3000; // in ms

const int onTime4 = 3001; // in ms
const int offTime4 = 4000; // in ms

const int onTime5 = 4001; // in ms
const int offTime5 = 5000; // in ms

boolean currentlyOn = false;
boolean forwardOn = false;
boolean backwardOn = false;
boolean leftOn = false;
boolean rightOn = false;

// will store last time LED was updated
unsigned long previousMillis = 0;
unsigned long startTime;

// interval at which to blink (milliseconds)
const long interval = 1000;

void setup()
{

  Serial.begin(9600);

  startTime = millis();
  
  //  Motor One
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  //Motor Two
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  
  pinMode(switchPin, INPUT_PULLUP);
}

void loop()
{
  int speed = analogRead(potPin) / 4;
  boolean reverse = digitalRead(switchPin);
//  setMotor(speed, reverse);

  unsigned long currentMillis = millis();
  
  if (currentMillis >= startTime + onTime1 && currentMillis <= startTime + offTime1){ // Switch resistor off
    drive_forward(speed, reverse);
    currentlyOn=false;
    Serial.println("1");
  } else if(currentMillis >= startTime + onTime2 && currentMillis <= startTime + offTime2) {
    drive_backward(speed, reverse);  
    currentlyOn=true;
    Serial.println("2");
  } else if (currentMillis >= startTime + onTime3 && currentMillis <= startTime + offTime3) {
    turn_left(speed, reverse);  
    Serial.println("3");
  } else if (currentMillis >= startTime + onTime4 && currentMillis <= startTime + offTime4) {
    turn_right(speed, reverse);  
    Serial.println("4");
  } else if (currentMillis >= startTime + onTime5 && currentMillis <= startTime + offTime5) {
    motor_stop(speed, reverse);
    Serial.println("5");
  } else {
    // Reset timer
    startTime=millis();  
  }
}

void setMotor(int speed, boolean reverse)
{
//  Motor One
  analogWrite(enablePin, speed);
  digitalWrite(in1Pin, ! reverse);
  digitalWrite(in2Pin, reverse);

//  Motor Two
  analogWrite(enablePin2, speed);
  digitalWrite(in3Pin, ! reverse);
  digitalWrite(in4Pin, reverse);
}


// Stearing The Motor
void motor_stop(int speed, boolean reverse){
  analogWrite(enablePin, speed);
  digitalWrite(in1Pin, LOW); 
  digitalWrite(in2Pin, LOW); 

  analogWrite(enablePin2, speed);
  digitalWrite(in3Pin, LOW); 
  digitalWrite(in4Pin, LOW);
}

void drive_forward(int speed, boolean reverse){
  analogWrite(enablePin, speed);
  digitalWrite(in1Pin, HIGH); 
  digitalWrite(in2Pin, LOW); 

  analogWrite(enablePin2, speed);
  digitalWrite(in3Pin, HIGH); 
  digitalWrite(in4Pin, LOW); 
}

void drive_backward(int speed, boolean reverse){
  analogWrite(enablePin, speed);
  digitalWrite(in1Pin, LOW); 
  digitalWrite(in2Pin, HIGH); 

  analogWrite(enablePin2, speed);
  digitalWrite(in3Pin, LOW); 
  digitalWrite(in4Pin, HIGH); 
}

void turn_left(int speed, boolean reverse){
  analogWrite(enablePin, speed);
  digitalWrite(in1Pin, LOW); 
  digitalWrite(in2Pin, HIGH); 

  analogWrite(enablePin2, speed);
  digitalWrite(in3Pin, HIGH); 
  digitalWrite(in4Pin, LOW);
}

void turn_right(int speed, boolean reverse){
  analogWrite(enablePin, speed);
  digitalWrite(in1Pin, HIGH); 
  digitalWrite(in2Pin, LOW); 

  analogWrite(enablePin2, speed);
  digitalWrite(in3Pin, LOW); 
  digitalWrite(in4Pin, HIGH); 
}
