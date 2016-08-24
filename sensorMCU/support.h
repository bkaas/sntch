#include <SoftwareSerial.h>  //not sure if this needs to be included since it's already included in main.h

int doRange(byte Address,SoftwareSerial bus);
void SRF01_Cmd(byte Address, byte cmd, SoftwareSerial bus);
void blueInterpret(char blueval, SoftwareSerial bus);
