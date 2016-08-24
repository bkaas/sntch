// Not sure what these are for
//#include <avr/power.h>
//#include <avr/io.h>
//#include <avr/interrupt.h>

#include "main.h"
#include "support.h"

SoftwareSerial UltrasonicBus(txrxPin, txrxPin);
SoftwareSerial blue(8, 7); //RX, TX

// Sensor enables, initially off
bool ultra = 0;
bool infrared = 0;

// stores chars read from bluetooth
char blueval;

// quad height as read by the ultrasonic sensor
double height;

// PID
double OutputPID;
double setPoint = 100;
double p = 2.8, i = 0.0, d = 0.0; //p=2.8, i=0.81
PID myPID(&height, &OutputPID, &setPoint, p, i, d, DIRECT);

// Initial minimum throttle value, and initial pitch/roll/yaw mid points
int minThrottle = 1700;
int midVal[3] = {1591,1520,1518}; // pitch roll yaw


// IR sensors:

// Pins
byte irPinN = 11;
byte irPinE = 9;
byte irPinS = A5;
byte irPinW = 12;

// Pin states
bool state[4] = {1,1,1,1};    //north, east, south, west in that order

// Minimum height (cm) for operation
int irMinHeight = 10;

// A triggered IR sensor must will not send an action to the quad if itself or another IR
// sensor has sent an action within minTriggerTime span (ms)
int minTriggerTime = 250;
unsigned long previousMillis = 0; // counter

// IR sensors end.


// Front red LEDs pin, used as indicators
byte frontRedLED = 13;



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

      height = doRange(srfAddress2, UltrasonicBus);
      myPID.Compute();
      float thrLevel = minThrottle + map(OutputPID, 0, 255, 0, 1999 - minThrottle);

      Serial.print("t" + String(int(thrLevel)));
    }
  }
  
  if ( infrared && height > irMinHeight ) {
    
    /****PITCH/ROLL****/
    state[0] = digitalRead(irPinN);
    state[2] = digitalRead(irPinS);
    state[1] = digitalRead(irPinE);
    state[3] = digitalRead(irPinW);

    if (!state[0] && (currentMillis - previousMillis) >= minTriggerTime) {
      Serial.print('x');
      previousMillis = currentMillis;
    }

    if (!state[2] && (currentMillis - previousMillis) >= minTriggerTime) {
      Serial.print('y');
      previousMillis = currentMillis;
    }

    if (!state[1] && (currentMillis - previousMillis) >= minTriggerTime) {
      Serial.print('z');
      previousMillis = currentMillis;
    }

    if (!state[3] && (currentMillis - previousMillis) >= minTriggerTime) {
      Serial.print('u');
      previousMillis = currentMillis;
    }
    
  }

  blue.listen();
  if (blue.isListening()) {
    delay(50); //does this really need to be this long? 50 sort of works?
    blueval = blue.read();
  }

  blueInterpret(blueval, blue);
  
}
