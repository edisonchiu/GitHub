
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


#include "mBot.h"



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



#ifdef EN_MOTOR // EN_MOTOR

  #define RUN_F 0x01
  #define RUN_B 0x01<<1
  #define RUN_L 0x01<<2
  #define RUN_R 0x01<<3
  #define STOP 0
  
  MeDCMotor MotorL(M1);
  MeDCMotor MotorR(M2);
  
#endif //--------- EN_MOTOR



#ifdef EN_SERVO // EN_SERVO
  #define SERVO_CNT       2
  #define ANGLE_M        90
  Servo servo[SERVO_CNT];
  byte servo_port[] = { 11, 12 };
#endif //--------- EN_SERVO



/*
 **********************************************************
 *
 *              GLOBAL OBJECT DEFINE
 *
 **********************************************************
 */ 
 
// SerialCommand
#ifdef EN_SCMD  
  
  //-- Library to manage serial commands
  // path = C:\Users\...\Documents\Arduino\libraries\SerialCommand\SerialCommand.h
  #include <SerialCommand.h>
  SerialCommand sCmd;  //The SerialCommand object
#endif // EN_SCMD


// Bluetooth Module 
#ifdef EN_BLUETOOTH
  // SoftwareSerial
  #include <SoftwareSerial.h>
  SoftwareSerial BTSerial(PIN_BT_RX, PIN_BT_TX); 
#endif // EN_BLUETOOTH



/*
 **********************************************************
 *                                                        *
 *                 GLOBAL VARIABLES                       *
 *                                                        *
 **********************************************************
 */ 
#ifdef EN_SCMD  
  const char programID[]="Otto_todo"; //Each program will have a ID
  const char name_fac='$'; //Factory name
  const char name_fir='#'; //First name
   
  //-- Movement parameters
  int T=1000;              //Initial duration of movement
  int moveId=0;            //Number of movement
  int moveSize=15;         //Asociated with the height of some movements
#endif // EN_SCMD

 
int moveSpeed = 200;
int minSpeed = 48;
int factor = 23;


uint8_t motor_sta = STOP;



 
/*
 **********************************************************
 *                                                        *
 *                       SETUP                            *
 *                                                        *
 **********************************************************
 */ 
uint8_t mode = MODE_A;
void setup()
{
  
  //Serial.begin(115200);
  Serial.begin(9600);
  
  #ifdef EN_SCMD 
  scmd_init();
  #endif 
  
  
  #ifdef EN_MOTOR
  goStop();
  #endif 
  
  
  #ifdef EN_RGBLED
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
  #endif // EN_RGBLED
  
  
  #ifdef EN_ULTRA_SONIC
  ultr.setpin(A3);
  #endif
  
  
  welcome();
  

  
  randomSeed(analogRead(0));
  
  
  #ifdef EN_MOTOR
  goStop();
  #endif
  
  
  #ifdef EN_IR_RECEIVER
  ir.begin();
  #endif
  
  
  Serial.print("Version: ");
  //Serial.println(mVersion);
  
  
  #ifdef EN_SERVO
  servo_setup();
  #endif
}


/*
  serial transmitter code test code
  
  #include <SoftwareSerial.h>
  SoftwareSerial BTSerial(PIN_BT_RX, PIN_BT_TX); 
  
  BTSerial.begin(9600);
  
  void loop {
    ...
    uart_test_output(&BTSerial);
    ...
  }
*/
/*
static long cnt=0;
void uart_test_output(SoftwareSerial *pSoftSerial){
  int randNumber = random(0,1000); 
  pSoftSerial->print(cnt++);
  pSoftSerial->print("\t");
  pSoftSerial->print(randNumber);
  pSoftSerial->println("");
  delay(2000);
}
*/


/*
 **********************************************************
 *                                                        *
 *                       LOOP                             *
 *                                                        *
 **********************************************************
 */ 
void loop()
{
  while(1)
  { 
    #ifdef EN_IR_RECEIVER
    get_ir_command();
    #endif
    
    #ifdef EN_SCMD 
    //scmd_proc();
    #endif
    
    
    serial_proc();
    
    
    #ifdef EN_ULTRA_SONIC
    usonic_proc();
    #endif
    
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
      Serial.println(F("Received: 1"));
      break;
    case '2':
      #ifdef EN_SERVO
      servo_shake_head(0);
      #endif
      Serial.println(F("Received: 2"));
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
      Serial.println("Stop:");
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
      #ifdef COM_DEBUG
      Serial.println(F("Forward:")); 
      #endif
      break;
    case 'z':
      // 4:DIR_L 5:PWM_H 6:PWM_H 7:DIR_H
      motor_sta = RUN_B;
      #ifdef COM_DEBUG
      Serial.println(F("Backward:"));
      #endif
      break;
    case 'd':
      // 4:DIR_L 5:H 6:H 7:DIR_L
      motor_sta = RUN_R;
      #ifdef COM_DEBUG
      Serial.println(F("Right:"));
      #endif
      break;
    case 'a':
      // 4:DIR_L 5:H 6:H 7:DIR_L
      motor_sta = RUN_L;
      #ifdef COM_DEBUG
      Serial.println(F("Left:"));
      #endif
      break;
  }
  
  uart_buf[0] = 0;
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


#ifdef EN_IR_RECEIVER
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
        #ifdef EN_SERVO
        servo_shake_head(0);
        #endif 
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
#endif // EN_IR_RECEIVER



#ifdef EN_MOTOR
void goForward()
{
  #if(TARGET_BOARD == BOARD_MCORE_V10)
    MotorL.run(-moveSpeed);
    MotorR.run(moveSpeed);
  #elif(TARGET_BOARD == BOARD_ARDUINO_NANO_V10)
    MotorL.run(moveSpeed);
    MotorR.run(moveSpeed);
  #endif
}
void goBackward()
{
  #if(TARGET_BOARD == BOARD_MCORE_V10)
    MotorL.run(moveSpeed);
    MotorR.run(-moveSpeed);
  #elif(TARGET_BOARD == BOARD_ARDUINO_NANO_V10)
    MotorL.run(-moveSpeed);
    MotorR.run(-moveSpeed);
  #endif

}
void TurnLeft()
{
  #if(TARGET_BOARD == BOARD_MCORE_V10)
    MotorL.run(70 + moveSpeed / 5);
    MotorR.run(70 + moveSpeed / 5);
  #elif(TARGET_BOARD == BOARD_ARDUINO_NANO_V10)
    MotorL.run(70 + moveSpeed / 5);
    MotorR.run(-(70 + moveSpeed / 5));
  #endif

}
void TurnRight()
{
  #if(TARGET_BOARD == BOARD_MCORE_V10)
    MotorL.run(70 + moveSpeed / 5);
    MotorR.run(70 + moveSpeed / 5);
  #elif(TARGET_BOARD == BOARD_ARDUINO_NANO_V10)
    MotorL.run(-(70 + moveSpeed / 5));
    MotorR.run(70 + moveSpeed / 5);
  #endif
}
void TurnLeftFast()
{
  #if(TARGET_BOARD == BOARD_MCORE_V10)
    MotorL.run(255);
    MotorR.run(255);
  #elif(TARGET_BOARD == BOARD_ARDUINO_NANO_V10)
    MotorL.run(255);
    MotorR.run(-255);
  #endif
}
void TurnRightFast()
{
  #if(TARGET_BOARD == BOARD_MCORE_V10)
    MotorL.run(-255);
    MotorR.run(-255);
  #elif(TARGET_BOARD == BOARD_ARDUINO_NANO_V10)
    MotorL.run(-255);
    MotorR.run(255);
  #endif
}
void goStop()
{
  MotorL.run(0);
  MotorR.run(0);
}
void ChangeSpeed(int spd)
{
  moveSpeed = spd;
}
#endif // EN_MOTOR




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
      goForward();
      break;
    case RUN_B:
      goBackward();
      break;
    case RUN_L:
      TurnLeftFast();
      break;
    case RUN_R:
      TurnRightFast();
      break;
    case STOP:
      goStop();
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
  if (d > 25 || d == 0) goForward();
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
    goBackward();
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
      goForward();
      break;

    case S1_IN_S2_OUT:
      TurnLeft();
      break;

    case S1_OUT_S2_IN:
      TurnRight();
      break;

    case S1_OUT_S2_OUT:
      goBackward();
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



/*
 **************************************************
 *                                                *
 *                   ULTRA SONIC                  *
 *                                                *
 **************************************************
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
    #if(0) //def COM_DEBUG
    Serial.print("Distance : ");
    Serial.print(d);
    Serial.println(" cm");
    #endif // COM_DEBUG
    // dbg
    
    delay(100);// the minimal measure interval is 100 milliseconds
}
#endif // EN_ULTRA_SONIC



/*
 **************************************************
 *                                                *
 *                    BUZZER                      *
 *                                                *
 **************************************************
 */
/*
 **************************************
 *  Play_Alarm()
 **************************************
 */ 
#ifdef EN_BUZZER
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
#endif // EN_BUZZER



/*
 **************************************************
 *                                                *
 *                    SERVO                       *
 *                                                *
 **************************************************
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
 **********************************************************
 *
 *              SERIAL COMMAND
 *
 **********************************************************
 */ 
#ifdef EN_SCMD  
/*
 **************************************
 *  scmd_init()
 **************************************
 */ 
void scmd_init() {
  BTSerial.begin(9600);   ///
  sCmd.setSoftwareSerial(&BTSerial);
  
  // Setup callbacks for SerialCommand commands 
  sCmd.addCommand("S", receiveStop);      //  sendAck & sendFinalAck
  sCmd.addCommand("M", receiveMovement);  //  sendAck & sendFinalAck
  
  //sCmd.addCommand("L", receiveLED);       //  sendAck & sendFinalAck
  sCmd.addCommand("T", recieveBuzzer);      //  sendAck & sendFinalAck
  //sCmd.addCommand("M", receiveMovement);  //  sendAck & sendFinalAck
  //sCmd.addCommand("H", receiveGesture);   //  sendAck & sendFinalAck
  //sCmd.addCommand("K", receiveSing);      //  sendAck & sendFinalAck
  //sCmd.addCommand("C", receiveTrims);     //  sendAck & sendFinalAck
  //sCmd.addCommand("G", receiveServo);     //  sendAck & sendFinalAck
  //sCmd.addCommand("R", receiveName);      //  sendAck & sendFinalAck
  //sCmd.addCommand("E", requestName);
  //sCmd.addCommand("D", requestDistance);
  //sCmd.addCommand("N", requestNoise);
  //sCmd.addCommand("B", requestBattery);
  //sCmd.addCommand("I", requestProgramId);   
  
  //sCmd.addDefaultHandler(receiveStop);
  sCmd.setDefaultHandler(receiveUnrecognized);  
}

/*
 **************************************
 *  scmd_proc()
 **************************************
 */ 
void scmd_proc()
{
  byte c;
  if (BTSerial.available() > 0) 
  { 
    //Serial.println("sCmd.readAvailable()");;
    sCmd.readSerial();
    
    //If Otto is moving yet
    //if (Otto.getRestState()==false) {
    //    move(moveId);
    //}
  } 
}

/*
 **********************************************************
 *
 *              CALLBACK FUNCTION
 *
 **********************************************************
 */ 
 
/*
 **************************************
 *  receiveUnrecognized()
 **************************************
 */ 

// This gets set as the default handler, and gets called when no other command matches.
void receiveUnrecognized(const char *command) {
  Serial.println("receiveUnrecognized()... What?");
}

//-- Function to receive Stop command.
//void receiveStop(){ old
void receiveStop(const char *command){
  Serial.println(F("receivegoStop()..."));
  sendAck();
  //myL9110.gogoStop();
  sendFinalAck();

}

//-- Function to receive buzzer commands
void recieveBuzzer(const char *command){
    
  #ifdef COM_DEBUG
  Serial.println(F("recieveBuzzer()"));
  #endif // COM_DEBUG
      
  //sendAck & stop if necessary
  sendAck();

  welcome();

  sendFinalAck();

}


//-- Function to receive movement commands
void receiveMovement(const char *command){

    #ifdef COM_DEBUG
    Serial.print(F("receiveMovement()"));
    #endif
    
    sendAck();

    //if (Otto.getRestState()==true){
    //    Otto.setRestState(false);
    //}

    //Definition of Movement Bluetooth commands
    // M  MoveID  T   MoveSize  
    // M 1      1000                GoForward    
    
    
    char *arg; 
    arg = sCmd.next(); 
    if (arg != NULL) {moveId=atoi(arg);}
    else {
      //Otto.putMouth(xMouth);
      //delay(2000);
      //Otto.clearMouth();
      moveId=0; //stop
    }
    
    arg = sCmd.next(); 
    if (arg != NULL) {T=atoi(arg);}
    else{
      T=1000;
    }

    arg = sCmd.next(); 
    if (arg != NULL) {moveSize=atoi(arg);}
    else{
      moveSize =15;
    }
}


//-- Function to execute the right movement according the movement command received.
void move(int moveId){

  bool manualMode = false;
  
  #ifdef COM_DEBUG
  Serial.print(F("move()"));
  Serial.println(moveId,HEX);
  #endif

  switch (moveId) {
    case 0:
      //Otto.home();
      //myL9110.gogoStop();
      break;
    case 1: // "M 1 1000" GoForward
      //Otto.walk(1,T,1);
      //myL9110.goForward();
      break;
    case 2: // "M 2 1000" GoBackward
      //Otto.walk(1,T,-1);
      //myL9110.goBackward();
      break;
    case 3: // "M 3 1000"
      //Otto.turn(1,T,1);
      //myL9110.turnLeft();
      break;
    case 4: // "M 4 1000""
      //Otto.turn(1,T,-1);
      //myL9110.turnRight();
      break;
    case 5: // "M 5 1000 30" 
      //Otto.updown(1,T,moveSize);
      break;

    default:
        manualMode = true;
      break;
  }

  if(!manualMode){
    sendFinalAck();
  }
       
}


//-- Function to send battery voltage percent
void requestBattery(const char *command){

    //Otto.home();  //stop if necessary

    //The first read of the batery is often a wrong reading, so we will discard this value. 
    //double batteryLevel = Otto.getBatteryLevel();

    BTSerial.print(F("&&"));
    BTSerial.print(F("B "));
    //BTSerial.print(batteryLevel);
    BTSerial.println(F("%%"));
    BTSerial.flush();
}


//-- Function to send program ID
void requestProgramId(const char *command){

  //Otto.home();   //stop if necessary

  BTSerial.print(F("&&"));
  BTSerial.print(F("I "));
  //BTSerial.print(programID);
  BTSerial.println(F("%%"));
  BTSerial.flush();
}


//-- Function to send Ack comand (A)
// "&&A%%"
void sendAck(){

  delay(30);

  BTSerial.print(F("&&"));
  BTSerial.print(F("A"));
  BTSerial.println(F("%%"));
  BTSerial.flush();
  
  Serial.print(F("&&"));
  Serial.print(F("A"));
  Serial.println(F("%%"));
  Serial.flush();
}


//-- Function to send final Ack comand (F)
// "&&F%%"
void sendFinalAck(){

  delay(30);

  BTSerial.print(F("&&"));
  BTSerial.print(F("F"));
  BTSerial.println(F("%%"));
  BTSerial.flush();
  
  Serial.print(F("&&"));
  Serial.print(F("F"));
  Serial.println(F("%%"));
  Serial.flush();
}

#endif // EN_SCMD  
  
  
  



