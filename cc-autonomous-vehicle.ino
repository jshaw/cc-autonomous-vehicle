/*
Adafruit Arduino - Lesson 15. Bi-directional Motor
Jordan Shaw
Added Secondary Motor
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

// will store last time LED was updated
unsigned long previousMillis = 0;

// interval at which to blink (milliseconds)
const long interval = 1000;

void setup()
{

  Serial.begin(9600);
  
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

//  if(currentMillis - previousMillis >= interval) {
//  }
  drive_forward(speed, reverse);
  delay(1000);
  motor_stop(speed, reverse);
  Serial.println("1");
  
  drive_backward(speed, reverse);
  delay(1000);
  motor_stop(speed, reverse);
  Serial.println("2");
  
  turn_left(speed, reverse);
  delay(1000);
  motor_stop(speed, reverse);
  Serial.println("3");
  
  turn_right(speed, reverse);
  delay(1000);
  motor_stop(speed, reverse);
  Serial.println("4"); 
  
  motor_stop( speed, reverse);
  delay(1000);
  motor_stop(speed, reverse);
  Serial.println("5");
  
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
  delay(25);
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
