#ifndef _api_H_
#define _api_H_
//
//**********************************************************
//                   Final Project
//==========================================================
//                  GLOBAL VARIABLES
//==========================================================
// general
extern volatile unsigned int state;
// UART
extern volatile unsigned int first_byte_MSG;  // UART RX
extern unsigned int state_stage;
//------------------------------------------------
// stepper motor
extern volatile int SM_Step;
extern volatile int SM_Half_Step;
extern volatile int StepperDelay;
//------------------------------------------------
// state 3 - calibration
extern volatile int SM_Counter;
//------------------------------------------------
// state 3 - Script mode
extern int acknowledge;
//================================================
//        SERVICE FUNCTIONS
//================================================
// state 1
extern void stepper_motor_manual_control(void);
// state 2
extern void Joystick_based_PC_painter(void);
// state 3
extern void stepper_motor_calibration(void);
// state 4
void script_mode(void);
// state 5
extern void moving_stepper_motor(void);

















////////////// OLD //////////////
//==========================================================
//                 GLOBAL VARIABLES
//==========================================================

extern volatile int sum_up_value;                           // state 2
extern volatile int sum_down_value;                         // state 3
extern volatile char POT[5];
//================================================
//        SERVICE FUNCTIONS
//================================================
//extern void RGBBlink(int delay);                           // state 1
//------------------------------------------------
extern void UpCounter(int delay);                          // state 2
//------------------------------------------------
extern void DownCounter(int delay);                        // state 3
//------------------------------------------------
extern void Potentiometer(void);                           // state 5
//------------------------------------------------
extern void clear_and_initialize(void);                    // state 6
//------------------------------------------------
extern void Transmit_menu(void);                           // state 8

#endif

