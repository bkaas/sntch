//#include "Arduino.h"
#include <SoftwareSerial.h>

int doRange(byte Address,SoftwareSerial bus);
void SRF01_Cmd(byte Address, byte cmd, SoftwareSerial bus);
void blueInterpret(char blueval, SoftwareSerial bus, int trimstep);
