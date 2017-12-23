
#ifndef _MBOT_H_
#define _MBOT_H_

/*

  mCore-Board I/O mapping 
  ----------------------------------------------------------------------------------
  Arduino mCore-Board     Define  Connector
  ----------------------------------------------------------------------------------
  D00(RXD)                Wireless Module
  D01(TXD)                Wireless Module
  D02   Infrared Receiver
  D03   Infrared Transmitter
  D04   MOTOR_02          M2(DIR2) 
  D05   MOTOR_02          M2(PWM2) 
  D06   MOTOR_01          M1(PWM1) 
  D07   MOTOR_01          M1(DIR1) 
  D08   Buzzer            PORT_06 
  D09   LineFollower      PORT_02   RJ25-P2(J2):5   
  D10   LineFollower      PORT_02   RJ25-P2(J2):6 
  D11   (Servo_01)        PORT_01   RJ25-P1(J1):5  ICSPI:4 D11/MOSI   
  D12   (Servo_02)        PORT_01   RJ25-P1(J1):6  ICSPI:1 D12/MISC 
  D13   RGB_LED                                    ICSPI:2 D13/SCK    
  A0    (LED_Matrix)      PORT_04   RJ25-P4(J4):5
  A1    (LED_Matrix)      PORT_04   RJ25-P4(J4):6
  A2                      PORT_03   RJ25-P3(J3):5
  A3    Ultrasonic Sensor PORT_03   RJ25-P3(J3):6
  A4                      CN_SDA    RJ25-P1~4(J1~4):1
  A5                      CN_SCL    RJ25-P1~4(J1~4):2
  A6    Light Sensor      PORT_06 
  A7    BUTTON
  ----------------------------------------------------------------------------------


  mCore-Board Connector 
  ----------------------------------------------
  Connector    Pin Descruption
  ----------------------------------------------
  RJ25-P1(J1) (1)SCL(2)SDA(3)GND(4)5V(5)11(6)12
  RJ25-P2(J2) (1)SCL(2)SDA(3)GND(4)5V(5)09(6)10
  RJ25-P3(J3) (1)SCL(2)SDA(3)GND(4)5V(5)A2(6)A3
  RJ25-P4(J4) (1)SCL(2)SDA(3)GND(4)5V(5)A0(6)A1
  ----------------------------------------------



  class MeDCMotor
      MeDCMotor(MEPORT port);
      MeDCMotor(uint8_t pwmPin,uint8_t dirPin);
      void run(int speed);
      void goStop();
      
*/
//#include "mCore.h"
#include <MeMCore.h>



//************************
//*      DEBUG           *
//************************
//#define DEBUG
#define COM_DEBUG
 
 
 
 /*
 **********************************************************
 *                                                        *
 *                   GLOBAL CONSTS                        *
 *                                                        *
 **********************************************************
 */ 
enum
{
  MODE_A,
  MODE_B,
  MODE_C,
  MODE_D,
  MODE_E,
  MODE_F,
};



/*
 **********************************************************
 *                                                        *
 *                   USE / UNUSE                          *
 *                                                        *
 **********************************************************
 */ 
//************************
//*    TARGET BOARD      *
//************************

#define BOARD_MCORE_V10         1
#define BOARD_ARDUINO_NANO_V10  2


//#define TARGET_BOARD    BOARD_MCORE_V10
#define TARGET_BOARD    BOARD_ARDUINO_NANO_V10 

/*
#if(TARGET_BOARD == BOARD_MCORE_V10)
  #define EN_SERVO
#elif(TARGET_BOARD == BOARD_ARDUINO_NANO_V10)

#endif
*/

//-- Robo Library
//#include <Otto.h>
//#include <Robo.h>
//Robo robo;  //This is Robo!!


//************************
//*      DEVICE          *
//************************
#define EN_ULTRA_SONIC
#define EN_LINE_FOLLOWER
#define EN_IR_RECEIVER
#define EN_MOTOR
//#define EN_SERVO
#define EN_RGBLED
#define EN_BUZZER


//************************
//*      COMMAND         *
//************************
// Serial Command via Bluetooth 
#define EN_BLUETOOTH
#define EN_SCMD



/*
 **********************************************************
 *                                                        *
 *                    PIN I/O                             *
 *                                                        *
 **********************************************************
 */ 
/*
  Bluetooth Module HC-05
  ----------------------
  Pin 01  VCC 
  Pin 02  GND
  Pin 03  TX
  Pin 04  RX (need 1K resistor for Arduino 5V TTL)
*/
#ifdef EN_ULTRA_SONIC
  #define PIN_BT_RX       11//12  // to Bluetooth TX pin3
  #define PIN_BT_TX       12//13  // to Bluetooth RX pin4
#endif



extern void welcome();

//#ifdef EN_MOTOR
void goForward();
void goBackward();
void TurnLeft();
void TurnRight();
void TurnLeftFast();
void TurnRightFast();

extern void receiveUnrecognized(const char *command);
extern void receiveStop(const char *command);
extern void recieveBuzzer(const char *command);  
extern void receiveMovement(const char *command);
extern void move(int moveId);
extern void requestBattery(const char *command);
extern void requestProgramId(const char *command);
extern void sendAck();
extern void sendFinalAck();
  
  
#ifdef EN_BUZZER
  #define NTD1 294 // note_D4
  #define NTD2 330 // note_E4
  #define NTD3 350 // note_F4
  #define NTD4 393
  #define NTD5 441
  #define NTD6 495
  #define NTD7 556
  #define NTDL1 147
  #define NTDL2 165
  #define NTDL3 175
  #define NTDL4 196
  #define NTDL5 221
  #define NTDL6 248
  #define NTDL7 278
  #define NTDH1 589
  #define NTDH2 661
  #define NTDH3 700
  #define NTDH4 786
  #define NTDH5 882
  #define NTDH6 990
  #define NTDH7 112
#endif // EN_BUZZER
  
#endif // _MBOT_H_