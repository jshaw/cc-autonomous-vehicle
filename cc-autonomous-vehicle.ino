/*
Jordan Shaw
Added Secondary Motor
Timer to test and control wheel movements with forward, stop, left and right
*/

//Setting up the QTR Sensor
#include <QTRSensors.h>

#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

#define KP .2
#define KD 5
#define M1_DEFAULT_SPEED 50
#define M2_DEFAULT_SPEED 50
#define M1_MAX_SPEED 70
#define M2_MAX_SPEED 70
#define DEBUG 0 // set to 1 if serial debug output needed

//For the start button
#define START_BUTTON_PIN 8  // emitter is controlled by digital pin 2
//boolean toStart = false;
//int onOff = 0;
int motorState = LOW;
int motorStateButton;
int motorPrevious = LOW;
long time = 0;
long debounce = 200;

int lastError = 0;
int last_proportional = 0;
int integral = 0;


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

//  Print the line position every...
const long lineInterval = 1000;
unsigned long linePreviousMillis = 0;

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

  // For start Button
  pinMode(START_BUTTON_PIN, INPUT);

  // Collaboration of the QTR Sensor
  
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  
}

void loop()
{

  unsigned long currentMillis = millis();

  motorStateButton = digitalRead(START_BUTTON_PIN);
//  onOff = digitalRead(START_BUTTON_PIN);
  if(stateButton == LOW && motorPrevious == HIGH && millis() - time > debounce) {
    if(motorState == LOW){
      motorState = HIGH; 
    } else {
       motorState = LOW; 
    }
    time = millis();
  }
  motorPrevious = motorStateButton ;

  //  TODO: ADD ON / OFF Button Logic
//  onOff = digitalRead(START_BUTTON_PIN);
//
//  Serial.print("onOff: ");
//  Serial.println(onOff);
//
//  // Added turn on and off button for the car
//  if(onOff == 0){
//    // Button is pushed
//    Serial.print("HIGH: ");
//    Serial.println(onOff);
//    toStart = !toStart;
//  } else {
//    Serial.print("LOW: ");
//    Serial.println(onOff);
//  }

  // Wait before the start button is pushed to start driving
//  if (toStart == false){
  if (motorState == LOW){
    Serial.println("DO NOTHING!!!");
    digitalWrite(in1Pin, LOW); 
    digitalWrite(in2Pin, LOW); 
    digitalWrite(in3Pin, LOW); 
    digitalWrite(in4Pin, LOW); 
    return;  
  }

  Serial.println("TO START NOW");
  
  int speed = analogRead(potPin) / 4;
  boolean reverse = digitalRead(switchPin);
  //  setMotor(speed, reverse);
  
  // Get averaged line position
  unsigned int position = qtra.readLine(sensorValues);
  int error = position - 2000;

  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  int leftMotorSpeed = M1_DEFAULT_SPEED + motorSpeed;
  int rightMotorSpeed = M2_DEFAULT_SPEED - motorSpeed;

  // set motor speeds using the two motor speed variables above
  set_motors(leftMotorSpeed, rightMotorSpeed);

//  if(currentMillis - linePreviousMillis >= lineInterval) {
//    linePreviousMillis = currentMillis;
//    // comment this line out if you are using raw values
//    Serial.print("Line Position: ");
//    Serial.println(position);
//  }
  
//  if (currentMillis >= startTime + onTime1 && currentMillis <= startTime + offTime1){ // Switch resistor off
//    drive_forward(speed, reverse);
//    currentlyOn=false;
//    Serial.println("1");
//  } else if(currentMillis >= startTime + onTime2 && currentMillis <= startTime + offTime2) {
//    drive_backward(speed, reverse);  
//    currentlyOn=true;
//    Serial.println("2");
//  } else if (currentMillis >= startTime + onTime3 && currentMillis <= startTime + offTime3) {
//    turn_left(speed, reverse);  
//    Serial.println("3");
//  } else if (currentMillis >= startTime + onTime4 && currentMillis <= startTime + offTime4) {
//    turn_right(speed, reverse);  
//    Serial.println("4");
//  } else if (currentMillis >= startTime + onTime5 && currentMillis <= startTime + offTime5) {
//    motor_stop(speed, reverse);
//    Serial.println("5");
//  } else {
//    // Reset timer
//    startTime=millis();  
//  }
}

void set_motors(int motor1speed, int motor2speed)
{

  unsigned long currentMillis = millis();
  
  if(currentMillis - linePreviousMillis >= lineInterval) {
    linePreviousMillis = currentMillis;
    
    Serial.print("Motor 1 Speed: ");
    Serial.print(motor1speed);
    
    Serial.print("\t Motor 2 Speed: ");
    Serial.println(motor2speed);

    digitalWrite(in1Pin, HIGH); 
    digitalWrite(in2Pin, LOW); 
  }

  
  if (motor1speed > M1_MAX_SPEED ) motor1speed = M1_MAX_SPEED; // limit top speed
  if (motor2speed > M2_MAX_SPEED ) motor2speed = M2_MAX_SPEED; // limit top speed
  if (motor1speed < 0) motor1speed = 0; // keep motor above 0
  if (motor2speed < 0) motor2speed = 0; // keep motor speed above 0
//  motor1.setSpeed(motor1speed);     // set motor speed
//  motor2.setSpeed(motor2speed);     // set motor speed
//  motor1.run(FORWARD);  
//  motor2.run(FORWARD);
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
