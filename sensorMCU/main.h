#include "Arduino.h"
#include <PID_v1.h>
#include <SoftwareSerial.h>

// Not sure what this is for
//#define F_CPU 16000000

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

#define txrxPin A0        // Ultrasonic coms pin
#define srfAddress2 0x02  // Ultrasonic sensor's address
#define getRange 0x54     // Byte used to get range from SRF01 in cm


/*************** Global variables used in both c files *****************/

//PID
extern double setPoint;
extern double p, i, d;

//toggle sesnors via BLE
extern bool ultra;
extern bool infrared;

extern int midVal[3]; // pitch roll yaw
extern int minThrottle;

extern byte frontRedLED; // = 13;

//extern int adjustment = 100;
