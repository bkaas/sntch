#include <PID_v1.h>
#include <SoftwareSerial.h>

#include <avr/power.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "main.h"
#include "support.h"

SoftwareSerial UltrasonicBus(txrxPin, txrxPin);
SoftwareSerial blue(8, 7); //RX, TX

//must be defined....
extern double Output = 0;

extern int minThrottle = 1700;
extern bool ultra = 0;
extern bool infrared = 0;
extern double dist = 0;
extern double p = 2.8, i = 0.0, d = 0.0; //p=2.8, i=0.81
extern int midVal[3] = {1591,1520,1518}; // {1491, 1522, 1561}; // pitch roll yaw midVal[1] = 1500; int midVal[0] = 1500; int midVal[2] = 1500;
extern double setPoint = 100;
char blueval;

//Specify the links and initial tuning parameters
PID myPID(&dist, &Output, &setPoint, p, i, d, DIRECT);


int trimStep = 2;

// IR sensors
byte irPinN = 11; //12 ;
byte irPinE = 9; //11;
byte irPinS = A5; //9;
byte irPinW = 12; //A5;
extern byte frontRedLED = 13;

// Roll and Pitch movement stuff
bool state[4] = {1,1,1,1};  //north, east, south, west in that order
unsigned long previousMillis = 0;

void setup() {

  myPID.SetMode(AUTOMATIC);

  Serial.begin(115200);
  while (!Serial && !blue.available());
  UltrasonicBus.begin(9600);
  while (!Serial && !UltrasonicBus.available());
  blue.begin(9600);

  // set sensor pins to detect objects
  pinMode(irPinN, INPUT);
  pinMode(irPinE, INPUT);
  pinMode(irPinS, INPUT);
  pinMode(irPinW, INPUT);

  pinMode(frontRedLED, OUTPUT);
  

}

void loop()  {
  unsigned long currentMillis = millis();
  
  if ( ultra ) {
    UltrasonicBus.listen();
    if (UltrasonicBus.isListening()) {

      dist = doRange(srfAddress2, UltrasonicBus);
      myPID.Compute();
      float thrLevel = minThrottle + map(Output, 0, 255, 0, 1999 - minThrottle);

      Serial.print("t" + String(int(thrLevel)));
    }
  }
  
  if ( infrared && dist > 10 ) {
    
    /****PITCH/ROLL****/
    state[0] = digitalRead(irPinN);
    state[2] = digitalRead(irPinS);
    state[1] = digitalRead(irPinE);
    state[3] = digitalRead(irPinW);

    if (!state[0] && (currentMillis - previousMillis) >= 250) {
      Serial.print('x');
      previousMillis = currentMillis;
    }

    if (!state[2] && (currentMillis - previousMillis) >= 250) {
      Serial.print('y');
      previousMillis = currentMillis;
    }

    if (!state[1] && (currentMillis - previousMillis) >= 250) {
      Serial.print('z');
      previousMillis = currentMillis;
    }

    if (!state[3] && (currentMillis - previousMillis) >= 250) {
      Serial.print('u');
      previousMillis = currentMillis;
    }
    
  }

  blue.listen();
  if (blue.isListening()) {
    delay(50); //does this really need to be this long? 50 sort of works?
    blueval = blue.read();
  }

  blueInterpret(blueval, blue, trimStep);
  
}
