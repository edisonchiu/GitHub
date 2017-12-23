
/*************************************************************************
 * File Name          : mbot_firmware.ino
 * Author             : Edison Chiu, Ander, Mark Yan
 * Updated            : Edison Chiu
 * Version            : V06.01.107
 * Date               : 01/03/2017
 *                      12/23/2017 update for new mBot Library support
 * Description        : Firmware for Makeblock Electronic modules with Scratch.  
 * License            : CC-BY-SA 3.0
 * Copyright (C) 2013 - 2016 Maker Works Technology Co., Ltd. All right reserved.
 * http://www.makeblock.cc/
 **************************************************************************/
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
      void stop();
      
*/
//#include "mCore.h"
#include <MeMCore.h>



/*
 **************************
 *      USE / UNUSE        
 **************************
 */
#define EN_ULTRA_SONIC
#define EN_LINE_FOLLOWER
#define EN_IR_RECEIVER
#define EN_MOTOR
#define EN_SERVO
#define EN_RGBLED
#define EN_BUZZER



#ifdef EN_RGBLED
MeRGBLed rgb;
#endif


#ifdef EN_ULTRA_SONIC
//MeUltrasonic ultr(PORT_3);
MeUltrasonicSensor ultr(PORT_3);
#endif


#ifdef EN_LINE_FOLLOWER
MeLineFollower line(PORT_2);
#endif


#ifdef EN_IR_RECEIVER
MeIR ir;
#endif


#ifdef EN_BUZZER
MeBuzzer buzzer;
#endif


#ifdef EN_MOTOR
MeDCMotor MotorL(M1);
MeDCMotor MotorR(M2);
#endif


#ifdef EN_SERVO
  #define SERVO_CNT       2
  #define ANGLE_M        90
  Servo servo[SERVO_CNT];
  byte servo_port[] = { 11, 12 };
#endif // EN_SERVO


int moveSpeed = 200;
int minSpeed = 48;
int factor = 23;


#ifdef EN_BUZZER
  #define NTD1 294
  #define NTD2 330
  #define NTD3 350
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


#define RUN_F 0x01
#define RUN_B 0x01<<1
#define RUN_L 0x01<<2
#define RUN_R 0x01<<3
#define STOP 0
uint8_t motor_sta = STOP;
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
 **************************
 *      SERVO    
 **************************
 */
#ifdef EN_SERVO
void servo_setup() {
    // port attach 
    for(int i=0;i<SERVO_CNT;i++) {    
        servo[i].attach(servo_port[i]);
        servo[i].write(90);
        delay(1000);
        servo[i].detach();
    }
}


// server sg90 Operating speed: 0.1 s/60 degree 
void servo_shake_head(int servo_no) {
    servo[servo_no].attach(servo_port[servo_no]);
    servo[servo_no+1].attach(servo_port[servo_no+1]);
    
    int r= 100;
    for(int i=0;i<3;i++) {
        servo[servo_no].write(ANGLE_M + r);
        servo[servo_no+1].write(ANGLE_M - r);
        delay(160);
        servo[servo_no].write(ANGLE_M - r);
        servo[servo_no+1].write(ANGLE_M + r);
        delay(160); 
    }
    servo[servo_no].write(ANGLE_M);
    servo[servo_no+1].write(ANGLE_M);
    delay(100); 
    servo[servo_no].detach();
    servo[servo_no+1].detach();
    //delay(10); 
}
#endif // EN_SERVO



/*
 **************************
 *      ULTRA SONIC    
 **************************
 */
#ifdef EN_ULTRA_SONIC
double ultrasonic_distanceCm(uint16_t maxCm)
{
    long distance = ultrasonic_measure(maxCm * 55 + 200);
    return (double)distance / 58.0;
}

long ultrasonic_measure(unsigned long timeout)
{
    int _pin = A3; // A2 A3
    long duration;
    // MePort::dWrite2(LOW);
    // delayMicroseconds(2);
    // MePort::dWrite2(HIGH);
    // delayMicroseconds(10);
    // MePort::dWrite2(LOW);
    // pinMode(s2, INPUT);
    // duration = pulseIn(s2, HIGH, timeout);
    digitalWrite(_pin,LOW);
    delayMicroseconds(2);
    digitalWrite(_pin,HIGH);
    delayMicroseconds(10);
    digitalWrite(_pin,LOW);
    pinMode(_pin,INPUT);
    duration = pulseIn(_pin,HIGH,timeout);
    return duration;
}
#endif // EN_ULTRA_SONIC



/*
 **************************************
 *  Play_Alarm()
 **************************************
 */ 
//#define PORT_BUZZER 8
void Play_Alarm()
{

  
  int i,j;
  for (i = 0; i < 10; i++) {

    /*  警車警報訊號：
     *  低頻頻率六五○赫茲至七五○赫茲，高頻頻率一四五○赫茲至一五五○赫茲，
     *  由低頻升至高頻時間○．二三秒，再由高頻降至低頻為○．一秒
     */
    for(j=650;j<1450;j+=5) {
      //tone(PORT_BUZZER,j);
        //buzzer.tone(j, 230/((1450-650)/5));
        buzzer.tone(j, 230/((1450-650)/5));
      //delay(230/((1450-650)/5));
    }
    //tone(PORT_BUZZER,650); 
    //delay(400); 
    //noTone(PORT_BUZZER); 
    //tone(PORT_BUZZER,1450); 
    delay(50); //100
    //noTone(PORT_BUZZER); 
    buzzer.noTone();
        
        
    #ifdef EN_RGBLED    
    if(i%2) {
      // ser red
      // rgb.setColor(20, 0, 0);
      rgb.setColorAt(0,20,0,0);//RGB LED1 set red
      rgb.setColorAt(1,0,0,20);//RGB LED2 set blue
      rgb.show();
    } else {
      // ser blue
      //rgb.setColor(0, 0, 20);
      rgb.setColorAt(0,0,0,20);//RGB LED2 set blue
      rgb.setColorAt(1,20,0,0);//RGB LED2 set red
      rgb.show(); 
    }
    #endif // EN_RGBLED
  
  }
  
  #ifdef EN_RGBLED  
    // set mode A: All WHITE 
    rgb.setColor(10, 10, 10);
    rgb.show(); 
  #endif 
}


/*
 **************************************
 *  welcome()
 **************************************
 */ 
void welcome()
{
  //rgb.clear();
  //rgb.setColorAt(0,10, 0, 0);
  //rgb.setColorAt(1,10, 0, 0);
  
  
  //---- LED:red , tone:NTD1 -----
  #ifdef EN_RGBLED  
    rgb.setColor(10, 0, 0); // set LED:red
    rgb.show(); 
  #endif 
  
  #ifdef EN_BUZZER
  buzzer.tone(NTD1, 300); 
  delay(300);
  #endif 
  //------------------------------
  
  
  //---- LED:green , tone:NTD2 ---
  #ifdef EN_RGBLED  
    rgb.setColor(0, 10, 0); // set LED:green
    rgb.show();
  #endif    
  
  #ifdef EN_BUZZER
  buzzer.tone(NTD2, 300);
  delay(300);
  #endif 
  //------------------------------
  
  
  //---- LED:blue , tone:NTD3 ---
  #ifdef EN_RGBLED  
  rgb.setColor(0, 0, 10); // set LED:blue
  rgb.show(); 
  #endif  
  
  #ifdef EN_BUZZER
  buzzer.tone(NTD3, 300);
  delay(300);
  #endif
  //------------------------------
  
  
  //---- LED:clear and set default , noTone ---
  #ifdef EN_RGBLED
  rgb.clear();
  #endif 
  
  #ifdef EN_BUZZER
  buzzer.noTone();
  #endif
  
  #ifdef EN_RGBLED
  // set mode A: All WHITE 
  rgb.setColor(10, 10, 10);
  rgb.show(); 
  #endif 
  //--------------------------------------------
}


/*
 **************************************
 *  setup()
 **************************************
 */ 
uint8_t mode = MODE_A;
void setup()
{
  Stop();
  
  
  // RGBLED
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  delay(300);
  digitalWrite(13,LOW);
  //if((rgb.getPort() != port) || rgb.getSlot() != slot)
  {
    //rgb.reset(port,slot);
    //rgb.reset(13);
     
  }
  rgb.setpin(13);
  //rgb.reset(13);
  rgb.setNumber(2);//mCore default 2 RGB led
  //rgb.show();
  //rgb.setColorAt(0,20,0,0);//RGB LED1 set red
  //rgb.setColorAt(1,0,20,0);//RGB LED2 set green
  //rgb.show();              //show all led's change.  
  //delay(1000);
  // RGBLED
  
  ultr.setpin(A3);
  
  welcome();
  
  
  //Serial.begin(115200);
  Serial.begin(9600);
  
  randomSeed(analogRead(0));
  Stop();
  ir.begin();
  
  Serial.print("Version: ");
  //Serial.println(mVersion);
  
  ///
  servo_setup();
  ///
}


/*
 **************************************
 *  loop()
 **************************************
 */ 
void loop()
{
  while(1)
  {
    
    //get_ir_command();
    
    serial_proc();
    //usonic_proc();
    
    switch (mode)
    {
      case MODE_A:
        modeA_proc();
        break;
      case MODE_B:
        modeB_proc();
        break;
      case MODE_C:
        modeC_proc();
        break;
      case MODE_E:
        modeE_proc();
        break;
      case MODE_F:
        modeF_proc();
        break;
    }
  }
}


/*
 **************************************
 *  serial_proc()
 **************************************
 */ 
char serial_data;
int uart_buf[3];  //串口接收数据缓存
void serial_proc()
{
  
  if(Serial.available()>0){
    //isAvailable = true;
    serial_data = Serial.read();
    uart_buf[0] = serial_data;
  }
  switch(uart_buf[0])
  {
    case '1':
      Play_Alarm();
      Serial.println("  ## Received: 1");
      break;
    case '2':
      servo_shake_head(0);
      Serial.println("  ## Received: 2");
      break;
    case 's':
      /*  4       5 6 7
          H       L L H       Off
          H       H H L       Forward 
          L       H H H       Backward
          L       H H L       Right
          H       H H H       Left
          IN1 IN2     IN3 IN4
          
          
      */
      motor_sta = STOP;
      Serial.println("  ## Stop:");
      break;
    case 'w':
      /*
        ----------------------------
        Arduino      L298
        ----------------------------
        PIN_4        IN1
                    ~IN2
        PIN_5        ENA
        PIN_6        ENB
        PIN_7        IN3
                    ~IN4
        ----------------------------
        
        
        ----------------------------
        mBot        L298
        ----------------------------
        Forward     Right
        Reverse     Left
        Right       Reverse
        Left        Forward
        ----------------------------
        
        
        L298 Truth Table
        ----------------------------
        Input   Output
        ----------------------------
        IN1 IN2 IN3 IN4 Description
        L   L   L   L   Off
        H   L   H   L   Forward
        L   H   L   H   Reverse
        H   H   H   H   Off  
        ----------------------------
      */
      // 4:DIR_H 5:PWM_H 6:PWM_H 7:DIR_L
      // PWM_H DIR_H DIR_L PWM_H
      //       H L   H L
      motor_sta = RUN_F;
      Serial.println("  ## Forward:"); 
      break;
    case 'z':
      // 4:DIR_L 5:PWM_H 6:PWM_H 7:DIR_H
      motor_sta = RUN_B;
      Serial.println("  ## Backward:");
      break;
    case 'd':
      // 4:DIR_L 5:H 6:H 7:DIR_L
      motor_sta = RUN_R;
      Serial.println("  ## Right:");
      break;
    case 'a':
      // 4:DIR_L 5:H 6:H 7:DIR_L
      motor_sta = RUN_L;
      Serial.println("  ## Left:");
      break;
  }
  
  
  
  uart_buf[0] = 0;
}


/*
 ****************************************************************************
    get_ir_command()
    
    MeInfraredReceiver.h
    D:\Program Files (x86)\Arduino\libraries\makeblock\src\MeInfraredReceiver.h
 ****************************************************************************
 */ 
void get_ir_command()
{
  static long time = millis();
  if (ir.decode())
  {
    uint32_t value = ir.value;
    time = millis();
    switch (value >> 16 & 0xff)
    {
      // 0x45 IR_BUTTON_POWER,IR_BUTTON_A
      case IR_BUTTON_A:  
        buzzer.tone(NTD1, 300);
        moveSpeed = 220;
        mode = MODE_A;
        rgb.setColor(10, 10, 10);
        rgb.show(); 
        break;
        
      // 0x46  
      case IR_BUTTON_B:
        buzzer.tone(NTD1, 300);
        moveSpeed = 200;
        mode = MODE_B;
        rgb.setColor(0, 10, 0);
        rgb.show(); 
        break;
        
      // 0x47 IR_BUTTON_C,IR_BUTTON_MENU
      case IR_BUTTON_C:
        buzzer.tone(NTD1, 300);
        mode = MODE_C;
        moveSpeed = 120;
        rgb.setColor(0, 0, 10);
        rgb.show(); 
        break;
        
      // 0x44 IR_BUTTON_D,IR_BUTTON_TEST
      case IR_BUTTON_D:
        buzzer.tone(NTD1, 300);
        servo_shake_head(0);
        break;
       
      // 0x44 IR_BUTTON_E,IR_BUTTON_RETURN
      case IR_BUTTON_E:
        buzzer.tone(NTD1, 300);
        //mode = MODE_E; 
        Play_Alarm();
        break;
        
      // 0x0D IR_BUTTON_F,IR_BUTTON_CLR
      case IR_BUTTON_F:
        buzzer.tone(NTD1, 300);
   
        break;
        
      // 0x40 IR_BUTTON_PLUS,IR_BUTTON_UP
      case IR_BUTTON_PLUS:
        motor_sta = RUN_F;
        //               Forward();
        break;
        
      // 0x19 IR_BUTTON_MINUS,IR_BUTTON_DOWN
      case IR_BUTTON_MINUS:
        motor_sta = RUN_B;
        //               Backward();
        break;
        
      // 0x09 IR_BUTTON_NEXT,IR_BUTTON_RIGHT
      case IR_BUTTON_NEXT:
        motor_sta = RUN_R;
        //               TurnRight();
        break;
        
      // 0x07 IR_BUTTON_PREVIOUS, IR_BUTTON_LEFT
      case IR_BUTTON_PREVIOUS:
        motor_sta = RUN_L;
        //               TurnLeft();
        break;
        
      // 0x15 IR_BUTTON_SETTING,IR_BUTTON_PLAY
      case IR_BUTTON_SETTING:
        break;
        
      // 0x4A IR_BUTTON_9  
      case IR_BUTTON_9:
        buzzer.tone(NTD1, 300); 
        ChangeSpeed(factor * 9 + minSpeed);
        break;
      // 0x52 IR_BUTTON_8
      case IR_BUTTON_8:
        buzzer.tone(NTD1, 300);
        ChangeSpeed(factor * 8 + minSpeed);
        break;
      // 0x42 IR_BUTTON_7
      case IR_BUTTON_7:
        buzzer.tone(NTD1, 300);
        ChangeSpeed(factor * 7 + minSpeed);
        break;
      // 0x5A IR_BUTTON_6
      case IR_BUTTON_6:
        buzzer.tone(NTD1, 300);
        ChangeSpeed(factor * 6 + minSpeed);
        break;
      // 0x1C IR_BUTTON_5
      case IR_BUTTON_5:
        buzzer.tone(NTD1, 300);
        ChangeSpeed(factor * 5 + minSpeed);
        break;
      // 0x08 IR_BUTTON_4
      case IR_BUTTON_4:
        buzzer.tone(NTD1, 300);
        ChangeSpeed(factor * 4 + minSpeed);
        break;
      // 0x5E IR_BUTTON_3
      case IR_BUTTON_3:
        buzzer.tone(NTD1, 300);
        ChangeSpeed(factor * 3 + minSpeed);
        break;
      // 0x18 IR_BUTTON_2
      case IR_BUTTON_2:
        buzzer.tone(NTD1, 300);
        ChangeSpeed(factor * 2 + minSpeed);
        break; 
      // 0x0C IR_BUTTON_1
      case IR_BUTTON_1:
        buzzer.tone(NTD1, 300);
        ChangeSpeed(factor * 1 + minSpeed);
        break;
      // 0x16 IR_BUTTON_0
      case IR_BUTTON_0:
        buzzer.tone(NTD1, 300);
        break;
    }
  }
  else if (millis() - time > 120)
  {
    motor_sta = STOP;
    time = millis();
  }
}
void Forward()
{
  MotorL.run(-moveSpeed);
  MotorR.run(moveSpeed);
}
void Backward()
{
  MotorL.run(moveSpeed);
  MotorR.run(-moveSpeed);
}
void TurnLeft()
{
  MotorL.run(70 + moveSpeed / 5);
  MotorR.run(70 + moveSpeed / 5);
}
void TurnRight()
{
  MotorL.run(-(70 + moveSpeed / 5));
  MotorR.run(-(70 + moveSpeed / 5));
}
void TurnLeftFast()
{
  MotorL.run(255);
  MotorR.run(255);
}
void TurnRightFast()
{
  MotorL.run(-255);
  MotorR.run(-255);
}
void Stop()
{
  MotorL.run(0);
  MotorR.run(0);
}
void ChangeSpeed(int spd)
{
  moveSpeed = spd;
}


/*
 **************************************
 *  usonic_proc()
 **************************************
 */ 
void usonic_proc()
{
    uint8_t d;
    d = ultr.distanceCm(50);
    //d = ultr.distanceCm();
    //d = ultrasonic_distanceCm(50);
    //if (d < 20) {
     //   servo_shake_head(0);
    //}//
    /**/
    // dbg
    Serial.print("Distance : ");
    Serial.print(d);
    Serial.println(" cm");
    // dbg
    
    delay(100);// the minimal measure interval is 100 milliseconds
}


/*
 **************************************
 *  modeA_proc()
 **************************************
 */ 
void modeA_proc()
{
  switch (motor_sta)
  {
    case RUN_F:
      Forward();
      break;
    case RUN_B:
      Backward();
      break;
    case RUN_L:
      TurnLeftFast();
      break;
    case RUN_R:
      TurnRightFast();
      break;
    case STOP:
      Stop();
      break;
  }

}


/*
 **************************************
 *  modeB_proc()
 **************************************
 */ 
void modeB_proc()
{
  uint8_t d = ultr.distanceCm(50);
  static long time = millis();
  randomSeed(analogRead(6));
  uint8_t randNumber = random(2);
  if (d > 25 || d == 0)Forward();
  else if (d > 20) {  // >10
    switch (randNumber)
    {
      case 0:
        TurnLeft();
        break;
      case 1:
        TurnRight();
        break;
    }
  }
  else
  {
    Backward();
  }
  delay(100);
}


/*
 **************************************
 *  modeC_proc()
 **************************************
 */
void modeC_proc()
{
  uint8_t val;
  val = line.readSensors();
  if(moveSpeed >230)moveSpeed=230;
  switch (val)
  {
    case S1_IN_S2_IN:
      Forward();
      break;

    case S1_IN_S2_OUT:
      TurnLeft();
      break;

    case S1_OUT_S2_IN:
      TurnRight();
      break;

    case S1_OUT_S2_OUT:
      Backward();
      break;
  }
//  delay(50);
}


/*
 **************************************
 *  modeE_proc()
 **************************************
 */
int led_state=0;
void modeE_proc()
{
    if(led_state%2) {
      // ser red
      // rgb.setColor(20, 0, 0);
      rgb.setColorAt(0,20,0,0);//RGB LED1 set red
      rgb.setColorAt(1,0,0,20);//RGB LED2 set blue
      rgb.show();
      
        //tone(PORT_BUZZER,650); 
        //delay(400); 
        //noTone(PORT_BUZZER); 
        buzzer.tone(650, 400);
    } else {
      // ser blue
      //rgb.setColor(0, 0, 20);
      rgb.setColorAt(0,0,0,20);//RGB LED2 set blue
      rgb.setColorAt(1,20,0,0);//RGB LED2 set red
      rgb.show(); 
      
        buzzer.tone(900, 400);
        //tone(PORT_BUZZER,900); 
        //delay(600); 
        //noTone(PORT_BUZZER); 
    }
    //delay(100);
    led_state++;
    if(led_state==2)
        led_state=0;
}

/*
 **************************************
 *  modeF_proc()
 **************************************
 */
void modeF_proc()
{

}




