#include "Arduino.h"
#include <PID_v1.h>

#define F_CPU 16000000

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

#define txrxPin A0
#define srfAddress2 0x02
#define getRange 0x54                                        // Byte used to get range from SRF01 in cm 


//PID:
//Define Variables we'll be connecting to
extern double Output;
extern double dist;
extern double setPoint;

extern double p, i, d;

//extern char blueval;
extern bool ultra;
extern bool infrared;

//extern float thrLevel = 1201;
extern int midVal[3]; // pitch roll yaw midVal[1] = 1500; int midVal[0] = 1500; int midVal[2] = 1500;
extern int minThrottle;
//extern int ptchLevel; int rlLevel;


//Hermionie: N = 12, E = 11, S = 9, W = A5
//OldFuckful: N = 11, E = 9, S = A5, W = 12

// IR sensors
extern byte irPinN; // = 11;
extern byte irPinE; // = 9;
extern byte irPinS; // = A5;
extern byte irPinW; // = 12;
extern byte frontRedLED; // = 13;

//extern int adjustment = 100;
