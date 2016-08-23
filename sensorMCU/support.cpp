#include "main.h"
#include "support.h"


int doRange(byte Address, SoftwareSerial bus) {
  SRF01_Cmd(Address, getRange, bus);                              // Calls a function to get range from SRF01
  while (bus.available() < 2);                     // Waits to get good data
  byte highByte = bus.read();                      // Get high byte
  byte lowByte = bus.read();                       // Get low byte
  int distance = ((highByte << 8) + lowByte);                    // Put them together
  return distance;
}

void SRF01_Cmd(byte Address, byte cmd, SoftwareSerial bus) {                     // Function to send commands to the SRF01
  pinMode(txrxPin, OUTPUT);                                  // Set pin to output and send break by sending pin low, waiting 2ms and sending it high again for 1ms
  digitalWrite(txrxPin, LOW);
  delay(2);
  digitalWrite(txrxPin, HIGH);
  delay(1);
  bus.write(Address);                              // Send the address of the SRF01
  bus.write(cmd);                                  // Send commnd byte to SRF01
  pinMode(txrxPin, INPUT);                                   // Make input ready for Rx
  int availableJunk = bus.available();             // Filter out the junk data
  for (int x = 0; x < availableJunk; x++) {
    byte junk = bus.read();
  }
}

void blueInterpret(char blueval, SoftwareSerial bus, int trimStep) {

  uint8_t c;
  String tmpStr;
//  int tmpInt;

  switch (blueval) {

     /*** ARM ***/
    case 'a':
      Serial.print(blueval);
      //delay(20); //if we delay above, do we need delay here?
//      thrLevel = 1201;    
      break;

    case 'u': //toggle ultrasonic measurements, with ramp up to minimize initial error by Dan
      Serial.print('t'+ String(minThrottle+50));
      delay(30);
      
      ultra ^= 1;
      digitalWrite(frontRedLED, 1);
      delay(15);
      digitalWrite(frontRedLED, 0);
      break;

    case 'f':
      infrared ^= 1;
      digitalWrite(frontRedLED, infrared);
      break;

    //Set minimum throttle on sensor circuit and quad
    case 'm':
      for (int i = 0; i < 4; i++) {
        c = bus.read();
        tmpStr += (char)c;
      }
      minThrottle = tmpStr.toInt();
      Serial.print("m" + tmpStr);
      tmpStr = "";
      delay(20);
      digitalWrite(frontRedLED, 1);
      delay(15);
      digitalWrite(frontRedLED, 0);
      break;

    //EEPROM Clear
    case 'q': 
      Serial.print('e');
      delay(20); //if we delay above, do we need delay here?
      digitalWrite(frontRedLED, 1);
      delay(15);
      digitalWrite(frontRedLED, 0);
      break;

   /**** IMPORTANT: resets the middle pitch and roll value after calibration ****/ 
   case 'x':
      Serial.print('s');
      break;

    case 'h': //height, changes setpoint
      for (int i = 0; i < 4; i++) {
        c = bus.read();
        tmpStr += (char)c;
      }
      setPoint = tmpStr.toInt();
      tmpStr = "";
      break;

    case 'p': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = bus.read();
        tmpStr += (char)c;
      }
      p = double(tmpStr.toInt());
      p = p / 1000.0;
      tmpStr = "";
      break;

    case 'i': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = bus.read();
        tmpStr += (char)c;
      }
      i = double(tmpStr.toInt());
      i = i / 1000.0;
      tmpStr = "";
      break;

    case 'd': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = bus.read();
        tmpStr += (char)c;
      }
      d = double(tmpStr.toInt());
      d = d / 1000.0;
      tmpStr = "";
      break;



    /*** Send Manual Throttle values  ***/
    case 't':
      for (int i = 0; i < 4; i++) {
        c = bus.read();
        tmpStr += (char)c;
      }
      Serial.print("t" + tmpStr);
      tmpStr = "";
      break;

    case 'w':   //down roll midVal (west)
      midVal[1] -= trimStep;
      midVal[1] = MAX(midVal[1], 1370);  //1370 trim min from multiwii
      Serial.print("r" + String(midVal[1]));
      delay(20);
      digitalWrite(frontRedLED, 1);
      delay(15);
      digitalWrite(frontRedLED, 0);
      break;

    case 'e':  //up roll midVal (east)
      midVal[1] += trimStep;
      midVal[1] = MIN(midVal[1], 1585);  //1585 trim max from multiwii
      Serial.print("r" + String(midVal[1]));
      delay(20);
      digitalWrite(frontRedLED, 1);
      delay(15);
      digitalWrite(frontRedLED, 0);
      break;

    case 's':  //down pitch midVal (south)
      midVal[0] -= trimStep;
      midVal[0] = MAX(midVal[0], 1370);
      Serial.print("p" + String(midVal[0]));
      delay(20);
      digitalWrite(frontRedLED, 1);
      delay(15);
      digitalWrite(frontRedLED, 0);
      break;

    case 'n':  //up pitch midVal (north)
      midVal[0] += trimStep;
      midVal[0] = MIN(midVal[0], 1585);
      Serial.print("p" + String(midVal[0]));
      delay(20);
      digitalWrite(frontRedLED, 1);
      delay(15);
      digitalWrite(frontRedLED, 0);
      break;

    case 'l': //down yaw midVal (left)
      midVal[2] -= trimStep;
      midVal[2] = MAX(midVal[2], 1370);
      Serial.print("y" + String(midVal[2]));
      delay(20);
      break;

    case 'r': //up yaw midVal (right)
      midVal[2] += trimStep;
      midVal[2] = MIN(midVal[2], 1585);
      Serial.print("y" + String(midVal[2]));
      delay(20);
      break;

    case 'z':  //reset roll and pitch
      midVal[0] = 1500;
      midVal[1] = 1500;
      midVal[2] = 1500;
      Serial.print("p" + String(midVal[0]));
      delay(20);
      Serial.print("r" + String(midVal[1]));
      delay(20);
      Serial.print("y" + String(midVal[2]));
      delay(20);
      digitalWrite(frontRedLED, 1);
      delay(15);
      digitalWrite(frontRedLED, 0);
      break;





    
//    // does nothing, adjustment set on quad side to 100
//    // changing the adjustment via BLE may be desireable moving forward
//    case 'j': //(ad)justment
//      for (int i = 0; i < 4; i++) {
//        c = bus.read();
//        tmpStr += (char)c;
//      }
//      adjustment = tmpStr.toInt();
//      tmpStr = "";
//      break;
//
//    // does nothing
//    case 'o': //
//      for (int i = 0; i < 4; i++) {
//        c = bus.read();
//        tmpStr += (char)c;
//      }
//      dip = tmpStr.toInt();
//      tmpStr = "";
//      break;
//
//   // does nothing
//   case 'g': //fraction of dip to compensate
//      for (int i = 0; i < 4; i++) {
//        c = bus.read();
//        tmpStr += (char)c;
//      }
//      compFrac = double(tmpStr.toInt());
//      compFrac = compFrac / 1000.0;
//      tmpStr = "";
//      digitalWrite(frontRedLED, 1);
//      delay(15);
//      digitalWrite(frontRedLED, 0);
//      break;


  }
  
}
