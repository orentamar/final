#include <msp430.h>
///
#include "../header/bsp.h"
#include "../header/hal.h"
#include "../header/api.h"

//  GLOBAL VARIABLES

volatile unsigned int state = 0;  //Sleeping mode, enable interrupts

//volatile int SM_Step = 0x8;       //0-0001-000
//volatile int SM_Half_Step = 0x18; //0-0011-000
//volatile int StepperDelay = 20;   // f = MHz
//
//volatile unsigned int Phi;
//volatile int steps = 7;
//volatile char Phi_str[4];

////////////////////////////////////////////////
//volatile char X[10];
//volatile unsigned int x = 500;

//volatile int sum_up_value = 0;
//volatile int sum_down_value = 65535;


void main(void){
	

    InitGPIO();
	InitUART();

    while(1){
        // FSM:  
        switch(state){
            case sleep_mode:
//                UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine
//                IE2 &= ~UCA0TXIE;                        // Disable TX interrupt
//                IE2 |= UCA0RXIE;                         // Enable RX interrupt
                __bis_SR_register(LPM0_bits + GIE);   // Enter LPM0
                break;

            case ManualMotorControl:
                stepper_motor_manual_control();
                break;

            case JoystickPainter:
                Joystick_based_PC_painter();
                break;

            case calibration:
                stepper_motor_calibration();
                break;

            case ScriptMode:
                stepper_motor_calibration();
                break;

            case move_motor_freely:
                moving_stepper_motor();
                break;



                ////////////////////////////////////////////////


//            case 1:
//                RGBBlink(x);
//                break;
//            case 2:
//                UpCounter(x);
//                break;
//            case 3:
//                DownCounter(x);
//                break;
//            case 4:
//                __bis_SR_register(LPM0_bits + GIE);   // Enter LPM0
//                break;
//            case 5:
//                Potentiometer();
//                break;
//            case 6:
//                clear_and_initialize();
//                break;
//            case 7:
//                __bis_SR_register(LPM0_bits + GIE);   // Enter LPM0
//                 break;
//			case 8:
//				Transmit_menu();
//				__bis_SR_register(LPM0_bits + GIE);   // Enter LPM0
//                break;
//            case 9:
//                __bis_SR_register(LPM0_bits + GIE);   // Enter LPM0
//                break;
//            case 10:
//                SM_Counter = 0;
//                while (state ==10){
//                    _BIS_SR(GIE);
//                    move_forward();
//                    SM_Counter +=1;
//                }
//
//                break;
            default:
                state = 0;
                break;
        }
    }
}
