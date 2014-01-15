#include <Wire.h>
#include <Adafruit_MotorShield.h>


//#define PRODUCTION
#define ENABLE_RC

#ifdef ENABLE_RC
#include <IRremote.h>
#endif


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

const int rightSpeed = 230;
const int leftSpeed = 250;

const int bobStopDist = 20;
const int swingDist = 0;

const int frontLightStopValue = 220;

const int IR_RECV_PIN = 4;
const long PILOT_FWD = 0x122458A7;
const long PILOT_BCK = 0x1224D827;
const long PILOT_LFT = 0x1224B847;
const long PILOT_RGT = 0x1224B04F;
const long PILOT_PWR = 0x122430CF;
const long PILOT_FAV = 0x1224E41B;
const long PILOT_EXT = 0x122448B7;

#ifdef ENABLE_RC
IRrecv irrecv(IR_RECV_PIN);
decode_results results;
#endif

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *rMotor = AFMS.getMotor(1);
Adafruit_DCMotor *lMotor = AFMS.getMotor(4);



void setup() {
  // initialize serial communication:
#ifndef PRODUCTION
  
  Serial.begin(9600);
#endif  

#ifdef ENABLE_RC
  irrecv.enableIRIn(); // Start the receiver
#endif

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
  
  
  long mainLight = readLight(frontLightInPin);
  long leftLight = readLight(leftLightInPin);
  long rightLight = readLight(rightLightInPin);
  #ifdef DUMMY
  DEBUGN("Front light ");
  DEBUG(mainLight);
  DEBUGN("Left light ");
  DEBUG(leftLight);
  DEBUGN("Right light ");
  DEBUG(rightLight);
  DEBUGN("Dist ");
  DEBUGN(cm);
  DEBUG("cm");
  #endif
  
  //goBob(cm, mainLight, leftLight, rightLight);

    stopIfObstacle(cm);

  receiveCommands();

  delay(200);
}

void receiveCommands() {
#ifdef ENABLE_RC
    if (receiveRC()) {
        DEBUG("Received code");
        DEBUGHEX(results.value);
        dispatchRCCode(results.value);
     }
     receiveResume();
#endif
}

void stopIfObstacle(long distanceCm) {
    if (distanceCm < bobStopDist) {
        bobStop();
    }
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

void back() {
  rMotor->run(BACKWARD);
  lMotor->run(BACKWARD);
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
  rMotor->setSpeed(0);
  lMotor->setSpeed(0);
//  delay(100);
}

void twistLeft() {
  lMotor->run(BACKWARD);
  rMotor->run(FORWARD);
  rMotor->setSpeed(150);
  lMotor->setSpeed(150);
  //delay(100);
//  go();
}

void twistRight() {
  rMotor->run(BACKWARD);
  lMotor->run(FORWARD);
  lMotor->setSpeed(150);
  rMotor->setSpeed(150);
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

boolean receiveRC() {
#ifdef ENABLE_RC
  return irrecv.decode(&results);
#else
  //results.value = scaryCode;
//  delay(3000);
  return false;
#endif
}

void receiveResume() {    
#ifdef ENABLE_RC
    irrecv.resume();
#endif
}

void dispatchRCCode(unsigned long code) {
  switch(code) {
  case PILOT_FWD: 
    go();
    break;
  case PILOT_BCK: 
      back();
      break;
  case PILOT_LFT:
      twistLeft();
      break;
  case PILOT_RGT:
      twistRight();
      break;
  case PILOT_EXT:
    bobStop();
    break;

  default:
    DEBUG("Unknown code");
    DEBUGHEX(code);

  }
}


