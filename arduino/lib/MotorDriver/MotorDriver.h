
/*
 **********************************************************
 *
 *              _MOTOR_DRIVER_H
 *
 **********************************************************
 */ 

#ifndef _MOTOR_DRIVER_H
#define _MOTOR_DRIVER_H



#include <arduino.h>


//************************
//*      DEBUG           *
//************************
//#define DEBUG
#define COM_DEBUG


/*
 **************************************************
 *                                                *
 *                 USE / UNUSE                     *
 *                                                *
 **************************************************
 */
#define MOTOR_DRIVER_NONE       0
#define MOTOR_DRIVER_L298       1
#define MOTOR_DRIVER_L298_DIR2  2
#define MOTOR_DRIVER_L9110      3



//****************************
//*   SELECT MOROR DRIVER    *
//****************************

#define MOTOR_DRIVER_SELECT  MOTOR_DRIVER_L298
//#define MOTOR_DRIVER_SELECT  MOTOR_DRIVER_L298_DIR2
//#define MOTOR_DRIVER_SELECT  MOTOR_DRIVER_L9110


 
/*
 **********************************************************
 *                                                        *
 *                 GLOBAL VARIABLES                       *
 *                                                        *
 **********************************************************
 */ 
#define DIR_DELAY 1000 // brief delay for abrupt motor changes
#define SPEED     200


// m_cfg
#define DIR_INV_ONE   0x0001    // invert one side direction
#define DIR_INV_ALL   0x0002    // invert all side direction



class MotorDriver
{

  public:
  
    // constructor
    MotorDriver();
    
    //MotorDriver(int PIN_M0_PWM, int PIN_M0_DIR, int PIN_M1_PWM, int PIN_M1_DIR);
    //L9110();
    //L9110(int PIN_M0_PWM, int PIN_M0_DIR, int PIN_M1_PWM, int PIN_M1_DIR);
    
    #if(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L298)    // MOTOR_DRIVER_L298
    
      int L298( int PIN_ENA, int PIN_IN1, int PIN_IN2, 
                int PIN_IN3, int PIN_IN4, int PIN_ENB );
                
    #elif(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L298_DIR2) // MOTOR_DRIVER_L298_DIR2
    
      int L298(int PIN_ENA, int PIN_IN1, int PIN_IN4, int PIN_ENB);
    
    #elif(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L9110) // MOTOR_DRIVER_L9110
    
      int L9110(int L9110_A_1A, int L9110_A_1B, int L9110_B_1A, int L9110_B_1B);
    
    #endif //----------------------------------------// MOTOR_DRIVER
    
    void setDirInvAll(bool is_dir_inverse);
    void setDirInvOne(bool is_dir_inverse);
    
    int init();
    void reset();
    void goStop();
    //void goHardStop();
    void goForward();  
    void goBackward();
    void goLeftForward();   // [^][ ] Left Forward
    void goRightForward();  // [ ][^] Right Forward
    void goLeftBackward();  // [v][ ] Left Backward
    void goRightBackward(); // [ ][v] Right Backward
    void turnLeft();
    void turnRight();
    void setSpeed(int speed);
    void setSpeed(int speed_L, int speed_R);
    void setSpeed(int speed_L, int speed_R, int speed_offset_L, int speed_offset_R);
    void setSpeedOffset( int speed_offset_L, int speed_offset_R);
      
  private:   
    void reset_vars();
    
    int m_pin_m0_pwm;
    int m_pin_m1_pwm;
    
    #if(MOTOR_DRIVER_SELECT == MOTOR_DRIVER_L298)    // MOTOR_DRIVER_L298
    int m_pin_m0_dir1;
    int m_pin_m0_dir2;
    int m_pin_m1_dir1;
    int m_pin_m1_dir2;
    #else
    int m_pin_m0_dir;
    int m_pin_m1_dir;
    #endif //----------------------------------------// MOTOR_DRIVER
    
    int m_speed_L;
    int m_speed_R;
    int m_speed_offset_L; 
    int m_speed_offset_R; 
    int m_cfg; 
    int m_status;
};


 

#endif // _MOTOR_DRIVER_H

