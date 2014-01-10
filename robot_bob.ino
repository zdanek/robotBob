#include <Wire.h>
#include <Adafruit_MotorShield.h>



#ifndef PRODUCTION
#define DEBUG(MSG) Serial.println(MSG)
#define DEBUGN(MSG) Serial.print(MSG)
#define DEBUGHEX(LONG) Serial.println(LONG, HEX)
#else
#define DEBUGHEX(LONG) 
#define DEBUG(MSG) 
#define DEBUGN(MSG)
#endif


const int pingPin = 7;

const int frontLightInPin = A0;
const int leftLightInPin = A4;
const int rightLightInPin = A5;

const int rightSpeed = 170;
const int leftSpeed = 200;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *rMotor = AFMS.getMotor(1);
Adafruit_DCMotor *lMotor = AFMS.getMotor(4);



void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  
  AFMS.begin();  // create with the default frequency 1.6KHz

    lMotor->run(RELEASE);
    rMotor->run(RELEASE);
}

void loop()
{
    
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  cm = microsecondsToCentimeters(duration);
  
  //cm = 50;
  
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  
  long mainLight = readLight(frontLightInPin);
  long leftLight = readLight(leftLightInPin);
  long rightLight = readLight(rightLightInPin);
  
  DEBUGN("Front light ");
  DEBUG(mainLight);
  DEBUGN("Left light ");
  DEBUG(leftLight);
  DEBUGN("Right light ");
  DEBUG(rightLight);
  DEBUGN("Dist ");
  DEBUGN(cm);
  DEBUG("cm");
  
  
  goBob(cm, mainLight, leftLight, rightLight);

  delay(2000);
}

void goBob(long distanceCm, long mainLight, long leftLight, long rightLight) {
    if (distanceCm > 30) {
        DEBUG("go");
        
        if (mainLight < leftLight && mainLight < rightLight) {
            DEBUG("FWD^^^^^^");
            go();
        } else {
            if (leftLight < rightLight) {
                DEBUG("LEFT <----");
                twistLeft();
            } else {
                DEBUG("RIGHT ---->");
                twistRight();
            }
        }
    } else {
        DEBUG("swing");
        swing();
    }
}

void go() {
  rMotor->run(FORWARD);
  lMotor->run(FORWARD);
  lMotor->setSpeed(200);
  rMotor->setSpeed(200);
}

void swing() {
}

void twistLeft() {
  lMotor->run(RELEASE);
  rMotor->run(FORWARD);
  rMotor->setSpeed(100);
}

void twistRight() {
  rMotor->run(RELEASE);
  lMotor->run(FORWARD);
  lMotor->setSpeed(100);
}

long readLight(int analogInPin) {
    long sensorValue = analogRead(analogInPin);            
  // map it to the range of the analog out:
//    long outputValue = map(sensorValue, 0, 1023, 0, 255);  

    return sensorValue;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}


