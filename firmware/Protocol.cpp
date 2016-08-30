#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "Output.h"
#include "MultiWii.h"
#include "Serial.h"
#include "Protocol.h"
#include "RX.h"
#include <EEPROM.h>

#define MAX(a, b) ((a > b) ? a : b)
#define MIN(a, b) ((a < b) ? a : b)
String thrLevel;
String rlLevel;
String ptchLevel;
String yawLevel;
String tmpString;
int adjustment = 100;
int faster = 0;

int minThrottle = 1700;
int tmpInt;
int midPitch = rcData[PITCH];
int midRoll = rcData[ROLL];

//********************Roll and Pitch***************//
bool state[4] = {0,0,0,0};  //north, east, south, west in that order
int timeout=0, timeout1=0;
int count = 0;


static uint8_t CURRENTPORT = 0;
//
#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];
static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

void evaluateOtherData(uint8_t sr);
#ifndef SUPPRESS_ALL_SERIAL_MSP
void evaluateCommand();
#endif


//Check incoming data if it matches the MSP protocol.
void serialCom() {
  //  CURRENTPORT = 0;
  //  uint8_t c,n;
  //  uint8_t cc = SerialAvailable(CURRENTPORT);
  //  while( cc-- ){
  //    uint8_t bytesTXBuff = SerialUsedTXBuff(CURRENTPORT); // indicates the number of occupied bytes in TX buffer
  //    if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return;  // ensure there is enough free TX buffer to go further (50 bytes margin)......TX_BUFFER_SIZE 128
  //    c = SerialRead(CURRENTPORT);
  //    if(c == 'a'){
  //      f.ARMED ^= 1;
  //      conf.throttleIn = 1350;
  //    }
  //  }
  unsigned long currentMillis = millis();
  uint8_t c, n;
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state[UART_NUMBER];// = IDLE;

  for (n = 0; n < UART_NUMBER; n++) {
#if !defined(PROMINI)
    CURRENTPORT = n;
#endif
#define GPS_COND
#if defined(GPS_SERIAL)
#if defined(GPS_PROMINI)
#define GPS_COND
#else
#undef GPS_COND
#define GPS_COND  && (GPS_SERIAL != CURRENTPORT)
#endif
#endif
#define SPEK_COND
#if defined(SPEKTRUM) && (UART_NUMBER > 1)
#define SPEK_COND && (SPEK_SERIAL_PORT != CURRENTPORT)
#endif
#define SBUS_COND
#if defined(SBUS) && (UART_NUMBER > 1)
#define SBUS_COND && (SBUS_SERIAL_PORT != CURRENTPORT)
#endif

    uint8_t cc = SerialAvailable(CURRENTPORT);

    while (cc-- GPS_COND SPEK_COND SBUS_COND) {
      uint8_t bytesTXBuff = SerialUsedTXBuff(CURRENTPORT); // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      c = SerialRead(CURRENTPORT);

      switch (c) {

        /****ARM****/
        case 'a':
          f.ARMED ^= 1;
          conf.throttleIn = 1100;
          break;

        case 'q':
          for (int i = 0 ; i < EEPROM.length() ; i++) {
            EEPROM.write(i, 0);
          }
          break;
        /*****SETVALS******/
        case 's':
            midPitch = rcData[PITCH];
            midRoll = rcData[ROLL];
            conf.pitchIn = midPitch;
            conf.rollIn = midRoll;
          break;
        /*****THROTTLE*****/
        case 't':
          if (f.ARMED == 1) {
            for (int i = 0; i < 4; i++) {
              c = SerialRead(CURRENTPORT);
              thrLevel += (char)c;
            }
            tmpInt = MAX(minThrottle, thrLevel.toInt());
            conf.throttleIn = tmpInt;
            thrLevel = "";
          }
          break;

        /******ROLL*****/
        case 'r':
          if (f.ARMED == 1) {
            for (int i = 0; i < 4; i++) {
              c = SerialRead(CURRENTPORT);
              rlLevel += (char)c;
            }
            conf.rollIn = rlLevel.toInt();
            rlLevel = "";
          }
          break;

        /******PITCH*****/
        case 'p':
          if (f.ARMED == 1) {
            for (int i = 0; i < 4; i++) {
              c = SerialRead(CURRENTPORT);
              ptchLevel += (char)c;
            }
            conf.pitchIn = ptchLevel.toInt();
            ptchLevel = "";
          }
          break;

        /*******YAW*******/
//        case 'y':
//          if (f.ARMED == 1) {
//            for (int i = 0; i < 4; i++) {
//              c = SerialRead(CURRENTPORT);
//              ptchLevel += (char)c;
//            }
//            conf.yawIn = yawLevel.toInt();
//            yawLevel = "";
//          }
//          break;
        /**********MINNIERTHROTTLE***********/
        case 'm':
          if (f.ARMED == 1) {
            for (int i = 0; i < 4; i++) {
              c = SerialRead(CURRENTPORT);
              tmpString += (char)c;
            }
            minThrottle=tmpString.toInt();
            tmpString = "";
          }
          break;
        /************* PITCH/ROLL ****************/
        case 'x':
            state[0]=1;
            count=0;
//            faster++;
          break;

        case 'y':
            state[2]=1;
            count=0;
//            faster++;
          break;
        
        case 'z':
            state[1]=1;
            count=0;
//            count2=0;
//            faster++;
          break;

        case 'u':
            state[3]=1;
            count=0;
//            count2=0;
//            faster++;
          break;
      }
    } //while receiving data
      
      /****PITCH/ROLL****/
        conf.pitchIn = midPitch + (state[2] - state[0])*(adjustment);
        conf.rollIn = midRoll + (state[3] - state[1])*(adjustment);
//        conf.throttleIn = MIN( (conf.throttleIn + 75*(state[0] || state[1] || state[2] || state[3]) ), 1999);
        
        if(count > 100){
          state[0] = 0;
          state[1] = 0;
          state[2] = 0;
          state[3] = 0;
          count = 0;
//          conf.throttleIn = conf.throttleIn + 25;
        }

//        if(count2 > 100){
//          state[1] = 0;
//          state[3] = 0;
//          count2 = 0;
//        }
        
        count++;
        
  } //for n < UART_NUMBER
} //serialCom()









///************************************** MultiWii Serial Protocol *******************************************************/
//// Multiwii Serial Protocol 0
//#define MSP_VERSION              0
//
////to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
//#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
//#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
//#define MSP_RAW_IMU              102   //out message         9 DOF
//#define MSP_SERVO                103   //out message         8 servos
//#define MSP_MOTOR                104   //out message         8 motors
//#define MSP_RC                   105   //out message         8 rc chan and more
//#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
//#define MSP_COMP_GPS             107   //out message         distance home, direction home
//#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
//#define MSP_ALTITUDE             109   //out message         altitude, variometer
//#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
//#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
//#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
//#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
//#define MSP_MISC                 114   //out message         powermeter trig
//#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI
//#define MSP_BOXNAMES             116   //out message         the aux switch names
//#define MSP_PIDNAMES             117   //out message         the PID names
//#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
//#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
//#define MSP_SERVO_CONF           120   //out message         Servo settings
//
//#define MSP_SET_RAW_RC           200   //in message          8 rc chan
//#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
//#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
//#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
//#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
//#define MSP_ACC_CALIBRATION      205   //in message          no param
//#define MSP_MAG_CALIBRATION      206   //in message          no param
//#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
//#define MSP_RESET_CONF           208   //in message          no param
//#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
//#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
//#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
//#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
//#define MSP_SET_MOTOR            214   //in message          PropBalance function
//
//#define MSP_BIND                 240   //in message          no param
//
//#define MSP_EEPROM_WRITE         250   //in message          no param
//
//#define MSP_DEBUGMSG             253   //out message         debug string buffer
//#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4





//uint8_t read8()  {
//  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT] & 0xff;
//}
//uint16_t read16() {
//  uint16_t t = read8();
//  t+= (uint16_t)read8()<<8;
//  return t;
//}
//uint32_t read32() {
//  uint32_t t = read16();
//  t+= (uint32_t)read16()<<16;
//  return t;
//}

