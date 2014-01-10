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

const int rightSpeed = 220;
const int leftSpeed = 250;

const int bobStopDist = 10;
const int swingDist = 20;

const int frontLightStopValue = 190;

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

  delay(200);
}

void goBob(long distanceCm, long mainLight, long leftLight, long rightLight) {
    
    if (mainLight < frontLightStopValue || leftLight < frontLightStopValue || rightLight < frontLightStopValue) {
        bobStop();
        return;
    }
    
    if (distanceCm > swingDist) {
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
        if (distanceCm < bobStopDist) {
            DEBUG("stop");
            bobStop();
        } else {
            DEBUG("swing");
            swing();
        }
    }
}

void go() {
  rMotor->run(FORWARD);
  lMotor->run(FORWARD);
  lMotor->setSpeed(leftSpeed);
  rMotor->setSpeed(rightSpeed);
}

void swing() {
    return;
  rMotor->run(FORWARD);
  lMotor->run(BACKWARD);
  lMotor->setSpeed(120);
  rMotor->setSpeed(120);
  delay(10);

}

void bobStop() {
  rMotor->run(RELEASE);
  lMotor->run(RELEASE);
//  delay(100);
}

void twistLeft() {
  lMotor->run(RELEASE);
  rMotor->run(FORWARD);
  rMotor->setSpeed(100);
  //delay(100);
//  go();
}

void twistRight() {
  rMotor->run(RELEASE);
  lMotor->run(FORWARD);
  lMotor->setSpeed(100);
//  delay(100);
  //go();
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


