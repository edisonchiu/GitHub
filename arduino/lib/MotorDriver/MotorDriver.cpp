
/*
 **********************************************************
 *                                                        *
 *                Class of Motor Driver                   *
 *                                                        *
 **********************************************************
 */ 
 /*
      2017/12/25    @edisonchiu
                    initial
      2017/12/26    add L298,L9110      
                    
 */


/*
  L298
  ---------------------
  PIN_M0_EN   // 01 ENA
  PIN_M0_I1   // 02 IN1
  PIN_M0_I2   // 03 IN2
  PIN_M1_I2   // 04 IN3
  PIN_M1_I1   // 05 IN4
  PIN_M1_EN   // 06 ENB
  
  
  L9110
  -------------------------------------------------------------------
  // wired connections
  #define L9110_A_1A     8  // D10 --> Motor A Input A --> MOTOR A +
  #define L9110_A_1B     9  // D11 --> Motor A Input B --> MOTOR A -
  #define L9110_B_1A     10 // D10 --> Motor B Input A --> MOTOR B +
  #define L9110_B_1B     11 // D11 --> Motor B Input B --> MOTOR B -
*/


/*
  (L298) Dual Channel Motor Driver Module Connector
  L9110
  HG7881_Motor_Driver_Example - Arduino sketch
   
  This example shows how to drive a motor with using HG7881 (L9110) Dual
  Channel Motor Driver Module.  For simplicity, this example shows how to
  drive a single motor.  Both channels work the same way.
   
  This example is meant to illustrate how to operate the motor driver
  and is not intended to be elegant, efficient or useful.
   
  Connections:
   
    Arduino digital output D10 to motor driver input B-IA.
    Arduino digital output D11 to motor driver input B-IB.
    Motor driver VCC to operating voltage 5V.
    Motor driver GND to common ground.
    Motor driver MOTOR B screw terminals to a small motor.
     
  Related Banana Robotics items:
   
    BR010038 HG7881 (L9110) Dual Channel Motor Driver Module
    https://www.BananaRobotics.com/shop/HG7881-(L9110)-Dual-Channel-Motor-Driver-Module
 
  https://www.BananaRobotics.com
*/


#include "MotorDriver.h"

/*
  HG7881 (L9110) Dual Channel Motor Driver Module Connector
  
  
  Motor Control Interface
  ----------------------------
  Pin	      Description
  ----------------------------
  MOT_A1B   Motor A Input B (IB)
  MOT_A1A   Motor A Input A (IA)
  VCC       Operating Voltage 2.5-12V   
  GND       Ground
  MOT_B1B   Motor B Input B (IB)
  MOT_B1A   Motor B Input A (IA)
  ----------------------------
  
  
  Truth Table
  ----------------------------
  Input	  Output
  ----------------------------
  IA	IB	OA	OB	Description
  L	  L	  L	  L	  Off
  H	  L	  H	  L	  Forward
  L	  H	  L	  H	  Reverse
  H	  H	  H	  H	  Off  
  ----------------------------
  IA: Input A
  IB: Input B
  OA: Output A
  OB: Output B
*/

  
/*
  (L298) Dual Channel Motor Driver Module Connector
    
  Motor Control Interface
  -------------------------------
  Pin	      Description
  ---------------------------------------------
  01 ENA    Enable Bridge A Motor (PWM)
  02 IN1    Motor A Input 1
  03 IN2    Motor A Input 2    
  04 IN3    Motor B Input 1    
  05 IN4    Motor B Input 2 
  06 ENB    Enable Bridge B Motor (PWM)
  
  12V       Operating Voltage 2.5-12V 
  GND       Ground
  5V        5V(Input / Output) *select by Jump
  
  OUT1      Motor A  Output 1
  OUT2      Motor A  Output 2
  OUT3      Motor B  Output 1
  OUT4      Motor B  Output 1
  ---------------------------------------------
  
  
  Truth Table
  ----------------------------
  Input	  Output
  ----------------------------
  IN1	IN2	IN3	IN4	Description
  L	  L	  L	  L	  Off
  H	  L	  H	  L	  Forward
  L	  H	  L	  H	  Reverse
  H	  H	  H	  H	  Off  
  ----------------------------
  IA: Input A
  IB: Input B
  OA: Output A
  OB: Output B
  
  
  
  NA	  IN1	      IN2	    功能
  HIGH	HIGH	    LOW	    馬達正轉
  HIGH	LOW	      HIGH	  馬達反轉
  HIGH	IN1=IN2	  IN1=IN2	馬達快速停止
  LOW	  ingored	  ignored	馬達慢速停止
*/


// MOTOR CONTROL
// FRONT  M[PORT_MOTOR_IN2] M[PORT_MOTOR_IN4]
// BACK   M[PORT_MOTOR_IN1] M[PORT_MOTOR_IN3]

// define for easy keyword
//#define DW(port,state)  { digitalWrite(port,state); } // Digital Write
//#define AW(port,state)  { analogWrite(port,state); }  // Analog Write



/*
 **********************************************************
 *                                                        *
 *                          MACRO                         *
 *                                                        *
 **********************************************************
 */ 

// macro for I/O setting
#define FWD    0   // FORWARD   
#define REV    1   // REVERSE


//---- Set Speed ---------------------- 
// mot : motor index  0~1
// dir : direction    FWD=0,REV=1
//       speed :      0~255

#if(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L298) //-----// MOTOR_DRIVER_L298


  #define MSPD(mot,dir) { \
    int speed = ((mot==0)?m_speed_L:m_speed_R); \
    digitalWrite((mot==0)?m_pin_m0_dir1:m_pin_m1_dir1,(dir==REV) ? HIGH:LOW); \
    digitalWrite((mot==0)?m_pin_m0_dir2:m_pin_m1_dir2,(dir==REV) ? LOW:HIGH); \
    analogWrite ((mot==0)?m_pin_m0_pwm:m_pin_m1_pwm,speed); }
    
  #define MSTOP(mot) { \
    digitalWrite((mot==0)?m_pin_m0_dir1:m_pin_m1_dir1,LOW); \
    digitalWrite((mot==0)?m_pin_m0_dir2:m_pin_m1_dir2,LOW); \
    digitalWrite((mot==0)?m_pin_m0_pwm:m_pin_m1_pwm,LOW); }
    
    
#elif(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L298_DIR2) // MOTOR_DRIVER_L298_DIR2


  #define MSPD(mot,dir) { \
    int speed = ((mot==0)?m_speed_L:m_speed_R); \
    digitalWrite((mot==0)?m_pin_m0_dir:m_pin_m1_dir,(dir==REV)?HIGH:LOW); \
    analogWrite ((mot==0)?m_pin_m0_pwm:m_pin_m1_pwm,speed); }
    
  #define MSTOP(mot) { \
    digitalWrite((mot==0)?m_pin_m0_dir:m_pin_m1_dir,LOW); \
    digitalWrite((mot==0)?m_pin_m0_pwm:m_pin_m1_pwm,LOW); }
    
    
#elif(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L9110) //--// MOTOR_DRIVER_L9110


  #define MSPD(mot,dir) { \
    int speed = ((mot==0)?m_speed_L:m_speed_R); \
    digitalWrite((mot==0)?m_pin_m0_dir:m_pin_m1_dir,(dir==REV)?HIGH:LOW); \
    analogWrite ((mot==0)?m_pin_m0_pwm:m_pin_m1_pwm,(dir==REV)?(0xFF-speed):speed); }
    
  #define MSTOP(mot) { \
    digitalWrite((mot==0)?m_pin_m0_dir:m_pin_m1_dir,LOW); \
    digitalWrite((mot==0)?m_pin_m0_pwm:m_pin_m1_pwm,LOW);   }

    
#endif //-------------------------------------------// MOTOR_DRIVER



//#define GO_STOP {                     \
//  digitalWrite( m_pin_m0_pwm, LOW );  \
//  digitalWrite( m_pin_m0_dir1, LOW );  \
//  digitalWrite( m_pin_m1_pwm, LOW );  \
//  digitalWrite( m_pin_m1_dir1, LOW ); }

#define MOTOR_GO_FORWARD        { MSPD(0,(m_cfg&DIR_INV_ONE)?REV:FWD); MSPD(1,FWD); } // [^][^] 車體前進                                
#define MOTOR_GO_BACKWARD       { MSPD(0,(m_cfg&DIR_INV_ONE)?FWD:REV); MSPD(1,REV); } // [v][v] 車體後退
#define MOTOR_GO_LEFT           { MSPD(0,(m_cfg&DIR_INV_ONE)?FWD:REV); MSPD(1,FWD); } // [v][^] 車體左旋
#define MOTOR_GO_RIGHT          { MSPD(0,(m_cfg&DIR_INV_ONE)?REV:FWD); MSPD(1,REV); } // [^][v] 車體右旋
#define MOTOR_GO_LEFT_FORWARD   { MSPD(0,(m_cfg&DIR_INV_ONE)?FWD:REV); MSTOP(1);    } // [v][ ] 右後
#define MOTOR_GO_RIGHT_FORWARD  { MSPD(0,(m_cfg&DIR_INV_ONE)?REV:FWD); MSPD(1,REV); } // [ ][v] 左後 
#define MOTOR_GO_LEFT_BACKWARD  { MSPD(0,(m_cfg&DIR_INV_ONE)?REV:FWD); MSTOP(1);    } // [^][ ] 右前    
#define MOTOR_GO_RIGHT_BACKWARD { MSTOP(0);                            MSPD(1,FWD); } // [ ][^] 左前   
//#define GO_HARD_STOP              { DW(R0,HIGH); DW(M0,HIGH); DW(R1,HIGH); DW(M1,HIGH); } // [ ][ ] 車體



// macro for inverse all direction 
#define GO_STOP     { MSTOP(0);    MSTOP(1);    } 
#define GO_FORWARD  { if(m_cfg&DIR_INV_ALL) MOTOR_GO_BACKWARD else MOTOR_GO_FORWARD; }
#define GO_BACKWARD { if(m_cfg&DIR_INV_ALL) MOTOR_GO_FORWARD else MOTOR_GO_BACKWARD; }
#define GO_LEFT     { if(m_cfg&DIR_INV_ALL) MOTOR_GO_RIGHT else MOTOR_GO_LEFT; }
#define GO_RIGHT    { if(m_cfg&DIR_INV_ALL) MOTOR_GO_LEFT else MOTOR_GO_RIGHT; }



/*
// [^][^]  
#define GO_FORWARD  { \
  pinMode( m_pin_m0_pwm, OUTPUT ); \
  pinMode( m_pin_m0_dir, OUTPUT ); \
  pinMode( m_pin_m1_pwm, OUTPUT ); \
  pinMode( m_pin_m1_dir, OUTPUT ); \
  digitalWrite(m_pin_m0_dir,HIGH); \
  digitalWrite(m_pin_m1_dir,HIGH); \
  digitalWrite(m_pin_m0_pwm,HIGH); \
  digitalWrite(m_pin_m1_pwm,HIGH); \
  analogWrite(m_pin_m0_pwm,m_speed_L); \
  analogWrite(m_pin_m1_pwm,m_speed_R); \
  }  
  
// [v][v]   
#define GO_BACKWARD { \
  pinMode( m_pin_m0_pwm, OUTPUT ); \
  pinMode( m_pin_m0_dir, OUTPUT ); \
  pinMode( m_pin_m1_pwm, OUTPUT ); \
  pinMode( m_pin_m1_dir, OUTPUT ); \
  digitalWrite(m_pin_m0_dir,LOW); \
  digitalWrite(m_pin_m1_dir,LOW); \
  digitalWrite(m_pin_m0_pwm,HIGH); \
  digitalWrite(m_pin_m1_pwm,HIGH); \
  analogWrite(m_pin_m0_pwm,m_speed_L); \
  analogWrite(m_pin_m1_pwm,m_speed_R); \
  } 
  
// [v][^] 
#define GO_LEFT { \
  pinMode( m_pin_m0_pwm, OUTPUT ); \
  pinMode( m_pin_m0_dir, OUTPUT ); \
  pinMode( m_pin_m1_pwm, OUTPUT ); \
  pinMode( m_pin_m1_dir, OUTPUT ); \
  digitalWrite(m_pin_m0_dir,HIGH); \
  digitalWrite(m_pin_m1_dir,LOW); \
  digitalWrite(m_pin_m0_pwm,HIGH); \
  digitalWrite(m_pin_m1_pwm,HIGH); \
  analogWrite(m_pin_m0_pwm,m_speed_L); \
  analogWrite(m_pin_m1_pwm,m_speed_R); \
  } 
  
// [^][v] 
#define GO_RIGHT { \
  pinMode( m_pin_m0_pwm, OUTPUT ); \
  pinMode( m_pin_m0_dir, OUTPUT ); \
  pinMode( m_pin_m1_pwm, OUTPUT ); \
  pinMode( m_pin_m1_dir, OUTPUT ); \
  digitalWrite(m_pin_m0_dir,LOW); \
  digitalWrite(m_pin_m1_dir,HIGH); \
  digitalWrite(m_pin_m0_pwm,HIGH);  \
  digitalWrite(m_pin_m1_pwm,HIGH);  \
  analogWrite(m_pin_m0_pwm,m_speed_L); \
  analogWrite(m_pin_m1_pwm,m_speed_R); \
  } 
*/

/*
 **********************************************************
 *                                                        *
 *                 GLOBAL VARIABLES                       *
 *                                                        *
 **********************************************************
 */ 
#define SL              (m_speed_L)         // Speed Left
#define SR              (m_speed_R)         // speed Right
#define SZ              0         



/*
 **********************************************************
 *                                                        *
 *                       CONSTRUCTOR                      *
 *                                                        *
 **********************************************************
 */ 
MotorDriver::MotorDriver() {
  //#ifdef COM_DEBUG
  //Serial.println(F("MotorDriver::L9110()"));
  //#endif
  
  // i/o pin definition 

  
  #if(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L298)    // MOTOR_DRIVER_L298
  
    m_pin_m0_pwm     = 0;
    m_pin_m0_dir1    = 0;
    m_pin_m0_dir2    = 0;
    m_pin_m1_pwm     = 0;
    m_pin_m1_dir1    = 0;
    m_pin_m1_dir2    = 0;
    
  #else
    
    m_pin_m0_pwm     = 0;
    m_pin_m0_dir     = 0;
    m_pin_m1_pwm     = 0;
    m_pin_m1_dir     = 0;

  #endif //----------------------------------------// MOTOR_DRIVER
  
  // i/o pin definition 

  m_status         = 0;
  reset_vars();
}

//**************************************
//*   reset_vars()                     *
//**************************************
void MotorDriver::reset_vars() {
  m_speed_L        = SPEED;
  m_speed_R        = SPEED;
  m_speed_offset_L = 0; 
  m_speed_offset_R = 0; 
  m_cfg            = 0;
}



#if(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L298)      // MOTOR_DRIVER_L298
//**************************************
//*   L298()                           *
//**************************************
int MotorDriver::L298( int PIN_ENA, int PIN_IN1, int PIN_IN2, 
                       int PIN_IN3, int PIN_IN4, int PIN_ENB )
{
  //#ifdef COM_DEBUG
  //Serial.println(F("L298::L298(int PIN_M0_PWM, int PIN_M0_DIR,int PIN_M1_PWM,int PIN_M1_DIR)") );
  //#endif
  // check
  if( (PIN_ENA == 0) ||
      (PIN_IN1 == 0) ||
      (PIN_IN2 == 0) ||
      (PIN_IN3 == 0) ||
      (PIN_IN4 == 0) ||
      (PIN_ENB == 0) )
    //#ifdef COM_DEBUG
    //Serial.println(F("L298 init Error(1)"));
    //#endif
    return -1;
    
  // i/o pin definition   
  m_pin_m0_pwm  = PIN_ENA;
  m_pin_m0_dir1 = PIN_IN1;
  m_pin_m0_dir2 = PIN_IN2;
  m_pin_m1_pwm  = PIN_ENB;
  m_pin_m1_dir1 = PIN_IN4;
  m_pin_m1_dir2 = PIN_IN3;
  // i/o pin definition 
  
  reset_vars();;
  
  return 0;
}
#endif //------------------------------------------// MOTOR_DRIVER_L298


    
#if(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L298_DIR2) // MOTOR_DRIVER_L298_DIR2
int MotorDriver::L298(int PIN_ENA, int PIN_IN1, int PIN_IN3, int PIN_ENB)
{
  //#ifdef COM_DEBUG
  //Serial.println(F("L298::L298(int PIN_M0_PWM, int PIN_M0_DIR,int PIN_M1_PWM,int PIN_M1_DIR)") );
  //#endif
  // check
  if( (PIN_ENA == 0) ||
      (PIN_IN1 == 0) ||
      (PIN_IN3 == 0) ||
      (PIN_ENB == 0) )
    //#ifdef COM_DEBUG
    //Serial.println(F("L298 init Error(1)"));
    //#endif
    return -1;
    
  // i/o pin definition   
  m_pin_m0_pwm  = PIN_ENA;
  m_pin_m0_dir  = PIN_IN1;
  m_pin_m1_pwm  = PIN_ENB;
  m_pin_m1_dir  = PIN_IN3;
  // i/o pin definition 
  
  reset_vars();;
  
  return 0;
}
#endif //------------------------------------------// MOTOR_DRIVER_L298_DIR2



#if(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L9110)  // MOTOR_DRIVER_L9110
//**************************************
//*   L9110()                          *
//**************************************
int MotorDriver::L9110(int L9110_A_1A, int L9110_A_1B,int L9110_B_1A,int L9110_B_1B) {
  // check
  if( (L9110_A_1A == 0) ||
      (L9110_A_1B == 0) ||
      (L9110_B_1A == 0) ||
      (L9110_B_1B == 0) )
    #ifdef COM_DEBUG
    Serial.println(F("L9110 init Error(1)"));
    #endif
    return -1;
  
  // i/o pin definition 
  m_pin_m0_pwm  = L9110_A_1A;
  m_pin_m0_dir  = L9110_A_1B;
  m_pin_m1_pwm  = L9110_B_1A;
  m_pin_m1_dir  = L9110_B_1B;
  // i/o pin definition 
  
  reset_vars();
  
  return 0;
}
#endif //----------------------------------------// MOTOR_DRIVER_L9110




//**************************************
//*   init()                           *
//**************************************
int MotorDriver::init() 
{
  /*
  // check
  if( (m_pin_m0_pwm == 0) ||
      (m_pin_m0_dir1 == 0) ||
      (m_pin_m1_pwm == 0) ||
      (m_pin_m1_dir1 == 0) )
    #ifdef COM_DEBUG
    Serial.println(F("L9110 init Error(1)"));
    #endif
    return -1;
  */  
    
  //----  set i/o pin state enable ------------------------------------   
  #if(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L298) // MOTOR_DRIVER_L298
  
    pinMode( m_pin_m0_pwm, OUTPUT );
    pinMode( m_pin_m1_pwm, OUTPUT );
    pinMode( m_pin_m0_dir1, OUTPUT );
    pinMode( m_pin_m0_dir2, OUTPUT );
    pinMode( m_pin_m1_dir1, OUTPUT ); 
    pinMode( m_pin_m1_dir2, OUTPUT ); 
    digitalWrite( m_pin_m0_pwm, LOW );
    digitalWrite( m_pin_m1_pwm, LOW );
    digitalWrite( m_pin_m0_dir1, LOW );
    digitalWrite( m_pin_m0_dir2, LOW );
    digitalWrite( m_pin_m1_dir1, LOW );
    digitalWrite( m_pin_m1_dir2, LOW );
    
  #else
    
    pinMode( m_pin_m0_pwm, OUTPUT );
    pinMode( m_pin_m0_dir, OUTPUT );
    pinMode( m_pin_m1_pwm, OUTPUT );
    pinMode( m_pin_m1_dir, OUTPUT );  
    digitalWrite( m_pin_m0_pwm, LOW );
    digitalWrite( m_pin_m1_pwm, LOW );
    digitalWrite( m_pin_m0_dir, LOW );
    digitalWrite( m_pin_m1_dir, LOW ); 
         
  #endif //-------------------------------------// MOTOR_DRIVER
  //----  set i/o pin state enable ------------------------------------  
     
}
 
 
//**************************************
//*   reset()                          *
//**************************************
//  reset i/o pin state enable  
void MotorDriver::reset() {
  init();
}


//**************************************
//*   setDirInvAll()                   *
//**************************************
void MotorDriver::setDirInvAll(bool is_dir_inverse) {
  if(is_dir_inverse)
    m_cfg|= DIR_INV_ALL;
  else
    m_cfg&= (~DIR_INV_ALL);
}

//**************************************
//*   setDirInvOne()                   *
//**************************************
void MotorDriver::setDirInvOne(bool is_dir_inverse) {
  if(is_dir_inverse)
    m_cfg|= DIR_INV_ONE;
  else
    m_cfg&= (~DIR_INV_ONE);
}

//**************************************
//*   goStop()                         *
//************************************** 
// Soft stop (preferred)
void MotorDriver::goStop() {
  #ifdef COM_DEBUG
  Serial.println(F("stop...") );
  #endif
  
  GO_STOP;
  /*
  digitalWrite( PIN_M0_DIR, LOW );
  digitalWrite( PIN_M0_PWM, LOW );
  digitalWrite( PIN_M1_DIR, LOW );
  digitalWrite( PIN_M1_PWM, LOW );*/
}
/*
// Hard stop (use with caution)
void MotorDriver::goHardStop() {
  #ifdef COM_DEBUG
  Serial.println(F("Hard stop (brake)..."));
  #endif
  
  GO_HARD_STOP;
}
*/

//**************************************
//*   goForward()                      *
//************************************** 
void MotorDriver::goForward() {
  #ifdef COM_DEBUG
  Serial.println(F("forward..."));
  #endif
  
  GO_FORWARD;
  /*
  // always stop motors briefly before abrupt changes
  digitalWrite( m_pin_m0_dir1, LOW );
  digitalWrite( m_pin_m0_pwm, LOW );
  digitalWrite( m_pin_m1_dir1, LOW );
  digitalWrite( m_pin_m1_pwm, LOW );
  delay( DIR_DELAY );
  
  // set the motor speed and direction
  digitalWrite( m_pin_m0_dir1, HIGH ); // direction = forward
  analogWrite( m_pin_m0_pwm, 255-200 ); // PWM speed = fast
  digitalWrite( m_pin_m1_dir1, HIGH ); // direction = forward
  analogWrite( m_pin_m1_pwm, 255-200 ); // PWM speed = fast*/
}

void MotorDriver::goBackward() {
  #ifdef COM_DEBUG
  Serial.println(F("Backward..." ));
  #endif
  
  GO_BACKWARD;
  /*
  // always stop motors briefly before abrupt changes
  digitalWrite( m_pin_m0_dir1, LOW );
  digitalWrite( m_pin_m0_pwm, LOW );
  digitalWrite( m_pin_m1_dir1, LOW );
  digitalWrite( m_pin_m1_pwm, LOW );
  delay( DIR_DELAY );
  
  // set the motor speed and direction
  digitalWrite( m_pin_m0_dir1, LOW ); // direction = forward
  analogWrite( m_pin_m0_pwm, 200 ); // PWM speed = fast
  digitalWrite( m_pin_m1_dir1, LOW ); // direction = forward
  analogWrite( m_pin_m1_pwm, 200 ); // PWM speed = fast*/
}

void MotorDriver::turnLeft()        { GO_LEFT; }
void MotorDriver::turnRight()       { GO_RIGHT; }
void MotorDriver::goLeftForward()   { MOTOR_GO_LEFT_FORWARD; }   // [^][ ] 
void MotorDriver::goRightForward()  { MOTOR_GO_RIGHT_FORWARD; }  // [ ][^]
void MotorDriver::goLeftBackward()  { MOTOR_GO_LEFT_BACKWARD; }  // [v][ ]  
void MotorDriver::goRightBackward() { MOTOR_GO_RIGHT_BACKWARD; } //  [ ][v] 

void MotorDriver::setSpeed(int speed) {
  m_speed_L        = speed;
  m_speed_R        = speed;
}

void MotorDriver::setSpeed(int speed_L, int speed_R) {
  m_speed_L        = speed_L;
  m_speed_R        = speed_R;
}

void MotorDriver::setSpeed(int speed_L, int speed_R, int speed_offset_L, int speed_offset_R) {
  m_speed_L        = speed_L;
  m_speed_R        = speed_R;
  m_speed_offset_L = speed_offset_L; 
  m_speed_offset_R = speed_offset_R; 
}

void MotorDriver::setSpeedOffset( int speed_offset_L, int speed_offset_R) {
  m_speed_offset_L = speed_offset_L; 
  m_speed_offset_R = speed_offset_R; 
}







