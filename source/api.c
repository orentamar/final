#include <msp430.h>

#include  "../header/api.h"             
#include  "../header/hal.h"             
#include  "../header/bsp.h"
//**********************************************************
//                   Final Project
//==========================================================
//                    Variables
//==========================================================


//==========================================================
//                     STATE 3
//==========================================================
void stepper_motor_calibration(void){
        SM_Counter = 0;
        moving_stepper_motor();
        Phi_calculation();
//        while (state_stage==1){ // motor is moving until state_stage changes
//            _BIS_SR(GIE);
//            move_forward();
//        }
//        if (state_stage==2){ // end of calibration
//            Phi_calculation();
//            state_stage = 0;
//            state = 0; // sleeping mode
//        }
}
//==========================================================
//                     STATE 5
//==========================================================
void moving_stepper_motor(void){
    while (state_stage==start_move_forwards){ // moving forwards until state_stage changes
        _BIS_SR(GIE);
        move_forward();
    }
    while (state_stage==start_move_backwards){ // moving backwards until state_stage changes
        _BIS_SR(GIE);
        move_backward();
    }
    if (state_stage==stop_mottor){ // end of calibration
        state_stage = no_action;
        state = sleep_mode; // sleeping mode

    }
}
//**********************************************************

//--------------------------------------------------
volatile int Out_to_RGB = 0x01;
//==========================================================
//                     STATE 1
//==========================================================
void RGBBlink(int delay){

    clear_RGB();
    while(state == 1){

        blink_RGB(delay);
        _BIS_SR(GIE);
    }
}
//==========================================================
//                     STATE 2
//==========================================================
void UpCounter(int delay){

    unsigned int SumValTmp;
    char SumValTXT[20] ={'\0'};
    lcd_clear();
    while(state == 2){
        delay_ms(delay);
        sum_up_value++;

        SumValTmp = (unsigned int) sum_up_value;
        int2str(SumValTXT, SumValTmp);
        lcd_home();
        lcd_puts("c_up: ");
        lcd_puts(SumValTXT);    // print initial label 1 on LCD
        _BIS_SR(GIE);

    }
}
//==========================================================
//                     STATE 3
//==========================================================
void DownCounter(int delay){

    unsigned int SumValTmp;
    char SumValTXT[20] ={'\0'};

    while(state == 3){
        delay_ms(delay);
        sum_down_value--;

        SumValTmp = (unsigned int) sum_down_value;
        int2str(SumValTXT, sum_down_value);
        lcd_home();
        lcd_puts("c_down: ");
        lcd_puts(SumValTXT);    // print initial label 1 on LCD
        _BIS_SR(GIE);

    }
    }


//==========================================================
//                     STATE 5
//==========================================================
void Potentiometer(void){
    adc10_config();
    SC_from_POT();
    int2str(POT,ADC10MEM);   // get pot value from ADC10MEM
    enable_transmition();
}
//==========================================================
//                     STATE 6
//==========================================================
void clear_and_initialize(void){
    sum_up_value = 0;
    sum_down_value = 65535;

    clearing();
    state = 9;
}

//==========================================================
//                     STATE 8
//==========================================================
void Transmit_menu(void){
    enable_transmition();
}

//***********************************************************
//***********************************************************

