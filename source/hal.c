#include <msp430.h>
#include <stdlib.h>
#include <stdio.h>

#include "../header/hal.h"
#include "../header/bsp.h"


//
//================================================
//               Variables
//================================================
//------------------------------------------------
//                  General
//------------------------------------------------
unsigned int i,j;
volatile unsigned long Phi = 0; // Current_angle*100

//------------------------------------------------
//                    UART
//------------------------------------------------
volatile unsigned int first_byte_MSG;  // UART RX

//              --- Status ---

volatile char StatusArray[40]; // as Msg_Size
unsigned int status_flg;

unsigned int TXindex=0;
volatile unsigned int Buff_index;

volatile char str_1[1];
volatile char str_2[2];
volatile char str_3[3];
volatile char str_4[4];
volatile char str_5[5];
volatile char str_6[6];

//volatile int size;  // no use

//               --- State ---

unsigned int state_flg;
unsigned int Msg_location = 0;
unsigned int state_stage = 0;

//------------------------------------------------
//               STATE 3 - calibration
//------------------------------------------------
volatile int SM_Counter= 0;
volatile unsigned long Phi_step=Phi_deg; // The nominal angle * 100
volatile long Phi_tmp = 0;

//------------------------------------------------
//              STATE 4 - Script Mode
//------------------------------------------------

Scripts s = {{0}, {FLASH_INFO_SEG_B_START, FLASH_INFO_SEG_C_START, FLASH_INFO_SEG_D_START}, {0}, {0} , 1};
int ScriptModeDelay = 50;
int write_to_flash = 0;
int offset = 0;
int acknowledge = 0;

volatile char p_rx[10];
//volatile char p_tx[10];
int index;
int degree;
int start;



//================================================
//                   Joy Stick
//================================================
//------------------------------------------------
//                  Variables
//------------------------------------------------
volatile unsigned int Vx=103; // the middle - 103
volatile unsigned int Vy=103;
unsigned int res[2];

volatile unsigned int JOISTICK_MODE=0;


void adc10_config(){
//    ADC10CTL1 = INCH_3 + ADC10SSEL_0;             // Repeat single channel, A3, ADC10OSC
//    ADC10CTL0 = ADC10SHT_0 + ADC10IE;             //ADC10 Interrupt Enable
    ADC10CTL0 = ADC10IE;                      //ADC10 Interrupt Enable
}

//void SC_from_POT(void){
//    ADC10CTL0 |= ADC10ON;                   // ADC10 ON
//    ADC10CTL0 &= ~ENC;                      // disable conversion
//    while(ADC10CTL1 & ADC10BUSY);           // Wait if ADC10 core is active
//    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
//    __bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
//    __no_operation();                       // For debugger
//    ADC10CTL0 &= ~ADC10ON;                  // ADC10 OFF
//}

void sampleVxy(void){
     ADC10CTL0 |= ADC10ON;                    // ADC10 ON

     ADC10CTL0 &= ~ENC;
     while (ADC10CTL1 & ADC10BUSY);           // Wait if ADC10 core is active
     ADC10SA = (int)res; // Data buffer start
     ADC10CTL0 |= ENC + ADC10SC;              // Sampling and conversion start
     __bis_SR_register(CPUOFF + GIE);         // LPM0, ADC10_ISR will force exit

     ADC10CTL0 &= ~ADC10ON;                   // ADC10 OFF


 }
//void sampleVxy(void){
//    ADC10CTL0 |= ADC10ON;                    // ADC10 ON
//    ADC10CTL0 &= ~ENC;
//    while (ADC10CTL1 & ADC10BUSY);           // Wait if ADC10 core is active
//    ADC10SA = (int)res;                      // Data buffer start - here you save  the info
//    ADC10CTL0 |= ENC + ADC10SC;              // Sampling and conversion start
//    __bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
//
////    __no_operation();                       // For debugger
////    ADC10CTL0 &= ~ADC10ON;                  // ADC10 OFF
//
// }
//==========================================================
//         ADC10 Interrupt Service Routine
//==========================================================
#pragma vector=ADC10_VECTOR
 __interrupt void ADC10_ISR(void){
    ADC10CTL0 &= ~ADC10IFG;        // clear interrupt flag
    Vx = res[0];
    Vy = res[1];
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)

}

 //#pragma vector=ADC10_VECTOR
 //__interrupt void ADC10_ISR(void){
 //    __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
 //}


//================================================
//                    Stepper Motor
//================================================
//------------------------------------------------
//                  Variables
//------------------------------------------------
volatile int SM_Step_Right = 0x80;       //1000-0000- h-4
volatile int SM_Step_Left = 0x10;        //0001-0000- h-1
//volatile int SM_Half_Step_Right = 0x0C;  //1100-0000- h-C
//volatile int SM_Half_Step_Left = 0x03;   //0011-0000- h-3
volatile int StepperDelay = 2;         // f = MHz

void step_angle_update(void){
    if (state_stage==start_move_forwards){
        SM_Counter++;
        Phi_tmp += Phi_step;
    }else if (state_stage==start_move_backwards){
        SM_Counter--;
        Phi_tmp -= Phi_step;
    }
    if (Phi_tmp > 360000){ // passed 360 degrees to the right --> turn to 0 degrees
        Phi_tmp =0;
    }else if (Phi_tmp < 0){ // passed 0 degrees to the left --> turn to 360 degrees
        Phi_tmp = 360000;
    }
    Phi = Phi_tmp%360000;
}

//----------------------------------------------------------
// Full step
void move_forward(void){
    SM_Step_Right >>= 1;
        if (SM_Step_Right == 0x08){
            SM_Step_Right = 0x80;
        }
        SMPortOUT = SM_Step_Right;
        Timer0_A_delay_ms(StepperDelay);
}
//----------------------------------------------------------

void move_backward(void){
    SM_Step_Left <<= 1;
        if (SM_Step_Left == 0x100){
            SM_Step_Left = 0x10;
        }
        SMPortOUT = SM_Step_Left;
        Timer0_A_delay_ms(StepperDelay);
}
//
//// Half step
//void move_forward_half(void){
//    SM_Half_Step_Right <<= 1;
//        if (SM_Half_Step_Right == 0x0C){
//            SM_Half_Step_Right = 0x60;
//        }
//        SMPortOUT = SM_Half_Step_Right;
//        Timer0_A_delay_ms(StepperDelay);
//}
//void move_backward_half(void){
//    SM_Half_Step_Left >>= 1;
//        if (SM_Half_Step_Left == 0xC0){
//            SM_Half_Step_Left = 0x18;
//        }
//        SMPortOUT = SM_Half_Step_Left;
//        Timer0_A_delay_ms(StepperDelay);
//}

//----------------------------------------------------------
//--------------------- Degree Scan ------------------------
//----------------------------------------------------------
volatile unsigned long Left_ang;
volatile unsigned long Right_ang;
volatile long Dist_in_degree;
volatile int Got_to_left_flg = 0;  // for Tx
volatile int Got_to_right_flg = 0; // for Tx
volatile scan_mode = 0;
void stepper_deg(unsigned long deg){
    stepper_scan(deg, deg);
}
//----------------------------------------------------------
void stepper_scan(unsigned long l, unsigned long r){
    Left_ang = l*1000;
    Right_ang = r*1000;
    Got_to_left_flg = 0;
    Got_to_right_flg = 0;

    // moving to Left angle
    move_to_angle(Left_ang);
    // Tell PC that motor arrived to Left angle
    if ((state == 4) && (scan_mode == 1)){ // maybe
        Got_to_left_flg = 1;
        enable_transmition();
    }
    // if Left angle = Right angle, end of operation
    if (Left_ang == Right_ang) return;
                                                        //  TODO: delay???
    // after getting to Left angle, scan area to Right angle
    scan_to_right();

    if ((state == 4) && (scan_mode == 1)){
        Got_to_right_flg = 1;   // update PC that motor arrived to Right angle
        enable_transmition();
        Timer0_A_delay_ms(50);

    }

}
void move_to_angle(unsigned long angle){
    if (Phi >= angle) {
        Dist_in_degree = Phi - angle;
    }else{
        Dist_in_degree = angle - Phi;
    }
//    Dist_in_degree = abs(Dist_in_degree);
    if (Dist_in_degree > 180000){  // distance greater than 180 degrees
        Dist_in_degree = 360000 - Dist_in_degree; // to take the shortest way
        if (Phi > angle){
            /* example: angle=20, Phi = 300
             * faster to move forwards to get to angle
             * with (360-Phi-angle) = 80 degrees to the right
             */
            forward();
        }else{ // Phi < angle
            /* example: angle=300, Phi = 20
             * faster to move backwards to get to angle
             * with (360-angle-Phi) = 80 degrees to the left
             */
            backward();
        }

    } else { // Dist_in_degree < 180000
        if (Phi > angle){
           /* example: angle=20, Phi = 120
            * faster to move backwards to get to angle
            * with (Phi-angle) = 100 degrees to the left
            */
            backward();
       }else{ // Phi < angle
           /* example: angle=120, Phi = 20
            * faster to move backwards to get to angle
            * with (angle-Phi) = 100 degrees to the right
            */
           forward();
       }
    }
}
//----------------------------------------------------------

void forward(void){
    state_stage = start_move_forwards;
    while (Dist_in_degree >= 0){
        move_forward();
        step_angle_update();
        Dist_in_degree -= Phi_step;
    }
    state_stage = stop_motor;

}
//----------------------------------------------------------

void backward(void){
    state_stage =start_move_backwards;
    while (Dist_in_degree >= 0){
        move_backward();
        step_angle_update();
        Dist_in_degree -= Phi_step;
    }
    state_stage = stop_motor;
}
//----------------------------------------------------------

void scan_to_right(void){
    if (Right_ang > Left_ang){
        Dist_in_degree = Right_ang - Left_ang;
    } else {
        Dist_in_degree = 360000 + Right_ang - Left_ang;
    }
    forward();
}
//----------------------------------------------------------

//==========================================================
//              STATE 3 - calibration
//==========================================================


void Phi_calculation(void){
    Phi_tmp=0;
    Phi =0;
    Phi_step = 360000;
    Phi_step = Phi_step/SM_Counter;
}
//==========================================================
//              STATE 4 - Script Mode
//==========================================================
volatile int num_byte;

int get_x_value(void){
    int x;
    num_byte = 0;
    while(read_mem(2) != 0x00) num_byte += 1;   // count the total bytes of information
    offset -= num_byte * 2;                     // retuen offset to read all
    x = read_mem(num_byte * 2);
    return x;
}
//----------------------------------------------------------
int receive_int(void){
    index = 0;
    return *p_rx - '0';  // TODO: check if -'0' necessary
}
//----------------------------------------------------------
void receive_string(int *data){
    while(1){
        __bis_SR_register(LPM0_bits + GIE);
//        if(p_rx[index - 1] == '\0'){
        if(p_rx[index - 1] == '-'){
            *data = str2int(p_rx);
            index = 0;
            break;
        }
    }
}
//----------------------------------------------------------

void send_ack(void){
    acknowledge = 1;
    IE2 &= ~UCA0RXIE;                         // Disable USCI_A0 RX interrupt
    enable_transmition();
    __bis_SR_register(LPM0_bits + GIE);
}

////----------------------------------------------------------
//void send_ss_data(int deg, int distance){
//    int len_deg = int2str(p_tx,deg);
//    int len_dist =int2str(p_tx + len_deg + 1,distance);
//    p_tx[len_deg + 1 + len_dist] = '\r';
//    IE2 &= ~UCA0RXIE;                         // Disable USCI_A0 RX interrupt
//    IE2 |= UCA0TXIE;                          // enable USCI_A0 TX interrupt
//    __bis_SR_register(LPM0_bits + GIE);
//}

//----------------------------------------------------------

void write_seg (char* flash_ptr, int offset){
    FCTL3 = FWKEY;                            // Clear Lock bit
    FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation
    flash_ptr[offset] = *p_rx;                // Write value to flash
    while((FCTL3 & WAIT) != WAIT);            // Wait for write to complete
    FCTL1 = FWKEY;                            // Clear WRT bit
    FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}
//----------------------------------------------------------

char read_char(char addr){
    char *segment;
    segment = s.pscript[s.num_script - 1] + offset;
    return segment[addr];
}
//----------------------------------------------------------

int read_mem(int num_bytes){
    int data = 0, i;
    char ch;
    for(i = 0; i < num_bytes; i++){
        ch = read_char(i);
        if(ch == 0)
            return 0;
        else if(ch > '9')
            data += (ch - 0x37) << 4 * (num_bytes - 1 - i);
        else
            data += (ch - 0x30) << 4 * (num_bytes - 1 - i);
    }
    offset += num_bytes;
    return data;
}
//----------------------------------------------------------
//------------------------- RGB ----------------------------
//----------------------------------------------------------

//void blink_RGB(int delay, int times){
//    unsigned char RGB;
//    while(times){
//        if(RGB == 0x1C)
//            P2OUT &= ~0x18;
//        else{
//            P2OUT += 0x04;
//            RGB = P2OUT & 0x1C;
//        }
//        Timer1_A_delay_10ms(delay); // 10ms
//        times--;
//    }
//}
volatile int Out_to_RGB = 0x01;

void blink_RGB(int delay, int times){
    while(times){
        if (Out_to_RGB == 0x04){
            Out_to_RGB = 0x01;
        }else{
            Out_to_RGB <<= 1;
        }
        RGBPortOUT = Out_to_RGB;
        Timer1_A_delay_10ms(delay); // 10ms
        times--;
    }
}
//----------------------------------------------------------

void clear_RGB(void){

    RGB_CLR;
}
//----------------------------------------------------------
//------------------------ LEDs ----------------------------
//----------------------------------------------------------

// Initialize Variables
volatile int first_rotate = 1;
volatile int PortNum = 1;
volatile int LEDs_val_P1 = 0x00;
volatile int LEDs_val_P2 = 0x00;
volatile int last_rotate = 0;
//----------------------------------------------------------

void rlc_leds(int delay, int times){
int rotate;
//    if ((last_rotate == right_rotate) && (LEDs_val_P1 == 0x01)){
//        PortNum = 1;
//    }
    while(times){
        rotate = 4;
        times --;
        while(rotate){
            if(first_rotate){
            first_rotate = 0;
            PortNum = 1;
            }
            if (PortNum == 1){
                LEDs_val_P2 = 0x00;
                if (LEDs_val_P1 == 0x00 || LEDs_val_P1 == 0x80){
                    LEDs_val_P1 = 0x01;         //0000-0001 P1.0
                }else if (LEDs_val_P1 == 0x01){
                    LEDs_val_P1 = 0x40;         //0100-0000 P1.6
                }else if (LEDs_val_P1 == 0x40){
                    LEDs_val_P1 = 0x80;         //1000-0000 P1.7
                    PortNum = 2;            // move to port 2 to next rotates

                }

            } else { // PortNum == 2
                LEDs_val_P1 = 0x00;
                LEDs_val_P2 = 0x10;            //0001-0000 P2.4
                PortNum = 1;               // move to port 1 to next rotates

            }

            rotate --;
            P1OUT = LEDs_val_P1;
            P2OUT = LEDs_val_P2;
            Timer1_A_delay_10ms(delay); // 10ms
        }



    }
//    last_rotate = left_rotate;
    P1OUT = 0;
    P2OUT = 0;
}

//----------------------------------------------------------

void rrc_leds(int delay, int times){
    int rotate;
//    if ((last_rotate == left_rotate) && (LEDs_val_P1 == 0x80)){
//        PortNum = 1;
//    }
    while(times){
        rotate = 4;
        times --;
        while(rotate){
            if(first_rotate){
            first_rotate = 0;
            PortNum = 2;
            }
            if (PortNum == 1){
                LEDs_val_P2 = 0x00;
                if (LEDs_val_P1 == 0x00 || LEDs_val_P1 == 0x01){
                    LEDs_val_P1 = 0x80;         //1000-0000 P1.7
                }else if (LEDs_val_P1 == 0x80){
                    LEDs_val_P1 = 0x40;         //0100-0000 P1.6
                }else if (LEDs_val_P1 == 0x40){
                    LEDs_val_P1 = 0x01;         //0000-0001 P1.0
                    PortNum = 2;            // move to port 2 to next rotates

                }

            } else { // PortNum == 2
                LEDs_val_P1 = 0x00;
                LEDs_val_P2 = 0x10;            //0001-0000 P2.4
                PortNum = 1;               // move to port 1 to next rotates

            }

            rotate --;
            P1OUT = LEDs_val_P1;
            P2OUT = LEDs_val_P2;
            Timer1_A_delay_10ms(delay); // 10ms

        }

    }
//    last_rotate = right_rotate;

    P1OUT = 0;
    P2OUT = 0;
}


//==========================================================
//                      UART
//==========================================================
// SEND BUFFER:
// | # | Buff_index | Phi step | - | Phi current | - | SM Counter | - | Vx | - | Vy | - | JOISTICK_MODE | - |
void GatherStatusInfo(void){
    int i=0;
    Buff_index = 0;
    TXindex = 0;
    // First char
    if ((state == 4) && (Got_to_left_flg == 1)){
        StatusArray[Buff_index]='<'; // sending PC that motor arrived to left angle
    } else if ((state == 4) && (Got_to_right_flg == 1)){
        StatusArray[Buff_index]='>';// sending PC that motor arrived to right angle
    } else {
        StatusArray[Buff_index]='#';
    }
    Buff_index+=3;
    bufferBuilder(Phi_step);
    bufferBuilder(Phi);
    bufferBuilder(SM_Counter);
    bufferBuilder(Vx);
    bufferBuilder(Vy);
    bufferBuilder(JOISTICK_MODE);
    // Buff_index - how many bytes will send without the first char
    int2str_TX(str_2,Buff_index);
    for (i=1;i<3;i++){
        StatusArray[i]=str_2[i-1];
    }
}

//----------------------------------------------------------
void bufferBuilder(unsigned int value){
    int i;
    if(value<10){
        int2str_TX(str_1,value);
        for (i=Buff_index;i<Buff_index+1;i++){
            StatusArray[i]=str_1[i-Buff_index];
        }
        Buff_index+=1;
    }else if(value<100){
        int2str_TX(str_2,value);
        for (i=Buff_index;i<Buff_index+2;i++){
            StatusArray[i]=str_2[i-Buff_index];
        }
        Buff_index+=2;
    }else if(value<1000){
        int2str_TX(str_3,value);
        for (i=Buff_index;i<Buff_index+3;i++){
            StatusArray[i]=str_3[i-Buff_index];
        }
        Buff_index+=3;
    }else if(value<10000){
        int2str_TX(str_4,value);
        for (i=Buff_index;i<Buff_index+4;i++){
            StatusArray[i]=str_4[i-Buff_index];
        }
        Buff_index+=4;
    }else if(value<100000){
        int2str_TX(str_5,value);
        for (i=Buff_index;i<Buff_index+5;i++){
            StatusArray[i]=str_5[i-Buff_index];
        }
        Buff_index+=5;
    }else{
        int2str_TX(str_6,value);
        for (i=Buff_index;i<Buff_index+6;i++){
            StatusArray[i]=str_6[i-Buff_index];
        }
        Buff_index+=6;
    }
    ///* Separation - 1 *///
    StatusArray[Buff_index]='-';
    Buff_index+=1;
}
//----------------------------------------------------------
void enable_transmition(void){
    UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine
    IE2 |= UCA0TXIE;                          // Enable TX interrupt
}
//----------------------------------------------------------
//             Interrupt Service Routines (ISR)
//----------------------------------------------------------
//==========================================================
//        UART-  Receiver Interrupt Service Routine
//==========================================================
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    first_byte_MSG = RxBuffer;
    if (first_byte_MSG == '#'){ // '#'- Ask for status (#- in hex-35)
        status_flg = 1;
        GatherStatusInfo();
        enable_transmition();
    } else if (first_byte_MSG == '!'){ // '!'- Starting state Msg (!- in hex-33)
        state_flg = 1;
        Msg_location = 1;  // for state value
    } else if (state_flg==1){
        if (Msg_location==1){ // Get status value
            state = RxBuffer;
            Msg_location ++;
        }else if ((Msg_location==2) && ((state ==3) || (state ==5))){ // Get status_stage value for calibration or moving freely
            state_stage =  RxBuffer;
            Msg_location = 0;
            state_flg = 0; // Done getting all state information
            __bic_SR_register_on_exit(LPM0_bits + GIE);  // Exit LPM0 on return to main
        }else if (state ==4) {
            Msg_location = 0;
            while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
            if(write_to_flash)
                p_rx[0] = RxBuffer;
            else
                p_rx[index++] = RxBuffer;

            __bic_SR_register_on_exit(LPM0_bits + GIE);  // Exit LPM0 on return to main
        }

    }

}
//==========================================================
//        UART-  Transmitter Interrupt Service Routine
//==========================================================
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    if (status_flg==1){
         TxBuffer = StatusArray[TXindex++];
        if (TXindex == Buff_index){                         // check if done with transmit
            TXindex = 0;
            IE2 &= ~UCA0TXIE;                            // Disable TX interrupt
            IE2 |= UCA0RXIE;                             // Enable RX interrupt
            status_flg = 0;
        }
    }else if ((state == 4) && (acknowledge == 1)){
        TxBuffer =  '!';
        // drop flags
        index = 0;
        acknowledge = 0;

        IE2 &= ~UCA0TXIE;                            // Disable USCI_A0 TX interrupt
        IE2 |= UCA0RXIE;                             // Enable USCI_A0 RX interrupt
        __bic_SR_register_on_exit(LPM0_bits + GIE);  // Exit LPM0 on return to main

    }else if ((state == 4) && (scan_mode == 1) && (Got_to_left_flg == 1)){
        GatherStatusInfo();
        Got_to_left_flg = 0;
        status_flg = 1;       // send PC current status

    }else if ((state == 4) && (scan_mode == 1) && (Got_to_right_flg == 1)){
        GatherStatusInfo();
        Got_to_right_flg = 0;
        status_flg = 1;       // send PC current status

    } else{
      IE2 &= ~UCA0TXIE;                                  // Disable TX interrupt
    }
}
//===========================================================
//          Timer A0 Interrupt Service Routine
//===========================================================
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0(void){
    TACCTL0 &= ~CCIE;                              // CCR0 interrupt enabled
    TA0CTL = TACLR;
    TA0CTL = MC_0 + TACLR;
    __bic_SR_register_on_exit(LPM0_bits + GIE);  // Exit LPM0 on return to main
}
//===========================================================
//          Timer A1 Interrupt Service Routine
//===========================================================
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0(void){
    TA1CCTL0 &= ~CCIE;                               // CCR0 interrupt enabled
    TA1CTL = TACLR;
    TA1CTL = MC_0 + TACLR;
    __bic_SR_register_on_exit(LPM0_bits + GIE);  // Exit LPM0 on return to main
}

//===========================================================
//                     Delay [ms]
//===========================================================
//
//void Timer0_A_delay_ms(int ms){
////  int tmp = ms;
//  TACCTL0 = CCIE;                             // CCR0 interrupt enabled
//  TACCR0 = ms*131;
//  TACTL = TASSEL_2 + ID_3 + MC_1 + TACLR;   // SMCLK/8 = 131072[Hz], upmode
//  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
//}

void Timer0_A_delay_ms(int ms){                // 1ms
//  int tmp = ms;
  TA0CCTL0 = CCIE;                             // CCR0 interrupt enabled
  TA0CCR0 = ms*131;
  TA0CTL = TASSEL_2 + ID_3 + MC_1 + TACLR;     // SMCLK/8 = 131072[Hz], upmode
  __bis_SR_register(LPM0_bits + GIE);          // Enter LPM0 w/ interrupt
}

void Timer1_A_delay_10ms(int ms_in10){           // 10ms
  TA1CCTL0 = CCIE;                             // CCR0 interrupt enabled
  TA1CCR0 = ms_in10 * 1310;
  TA1CTL = TASSEL_2 + ID_3 + MC_1 + TACLR;     // SMCLK/8 = 131072[Hz], upmode
  __bis_SR_register(LPM0_bits + GIE);          // Enter LPM0 w/ interrupt
}
//void delay_ms(unsigned int ms)
//{
//    while (ms)
//    {
//        __delay_cycles(1000); //1000 for 1MHz and 16000 for 16MHz
//        ms--;
//    }
//}
void StopTimers(){
    TA0CTL &= ~ 0x18;
    TA1CTL &= ~ 0x18;
}
//===========================================================
//            integer to string converter
//===========================================================
int int2str(char *str, unsigned int num){
  int strSize = 0;
  long tmp = num, len = 0;
  int j;

  // Find the size of the intPart by repeatedly dividing by 10
  while(tmp || len == 0){
    len++;
    tmp /= 10;
  }

  // Print out the numbers in reverse
  for(j = len - 1; j >= 0; j--){
    str[j] = (num % 10) + '0';
    num /= 10;
  }
  strSize += len;
  str[strSize] = '\n';
  return strSize;
}

//----------------------------------------------------------
void int2str_TX(char *str, unsigned int num){
  int strSize = 0;
  long tmp = num, len = 0;
  int j;

  // Find the size of the intPart by repeatedly dividing by 10
  while(tmp){
    len++;
    tmp /= 10;
  }

  // Print out the numbers in reverse
  for(j = len - 1; j >= 0; j--){
    str[j] = (num % 10) + '0';
    num /= 10;
  }
  strSize += len;
//  str[strSize] = '\0';
}
//===========================================================
//            string to integer converter
//===========================================================
int str2int( char volatile *str)
{
 int i,res = 0;
 for (i = 0; str[i] != '\0'; ++i) {
     if (str[i]> '9' || str[i]<'0')
         return -1;
     res = res * 10 + str[i] - '0';
 }
 return res;
}

















//------------------------------------------------------------------
//                        OLD CODE
//------------------------------------------------------------------
//------------------------------------------------------------------

//volatile char POT[5];
//
//const char MENU[] = "\n"
//                    "                       Menu\n"
//                    "*******************************************************\n"
//                    "1. Blink RGB LED, color by color with delay of X[ms]\n"
//                    "2. Count up onto LCD screen with delay of X[ms]\n"
//                    "3. Count down onto LCD screen with delay of X[ms]\n"
//                    "4. Get delay time X[ms]\n"
//                    "5. Potentiometer 3-digit value [v]\n"
//                    "6. Clear LCD screen\n"
//                    "7. On each PB1 pressed, Send a Message\n"
//                    "8. Show menu\n"
//                    "9. Sleep\n"
//                    "*******************************************************\r";
//
//volatile char REAL_Str[16] = "I love my Negev ";
//------------------------------------------------------------------
//--------------- Port1 Interrupt Service Routine ------------------
//------------------------------------------------------------------
//#pragma vector=PORT1_VECTOR
//__interrupt void PORT1_ISR(void){
//    if(PB1_IntPending & 0x01){
//        _buttonDebounceDelay(0x01);
////        menu_tx = 1;
//        if (state == 7){
//            UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
//            IE2 |= UCA0TXIE;                          // Enable USCI_A0 TX interrupt
//        }else {
//            state = 0;
//        }
//    }
//}

//==========================================================
//                     Real PB1 ISR
//==========================================================

//------------------------------------------------------------------
//void _buttonDebounceDelay(int button){
//    volatile unsigned int i;
//    for(i = 1000; i > 0; i--);                     //delay, button debounce
//    while(!(PB1_PortIN & button));          // wait of release the button
//    for(i = 1000; i > 0; i--);                     //delay, button debounce
//    PB1_IntPending &= ~button;              // manual clear of p1.button
//}

//==========================================================
//                     STATE 1
//==========================================================
//void clear_RGB(void){
//
//    RGB_CLR;
//}

//void blink_RGB(int delay){
//    if (Out_to_RGB == 0x04){
//        Out_to_RGB = 0x01;
//    }else{
//        Out_to_RGB <<= 1;
//    }
//    RGBPortOUT = Out_to_RGB;
//    Timer0_A_delay_ms(delay);
//}

//==========================================================
//                     STATE 5
//==========================================================

//void adc10_config(){
//    ADC10CTL1 = INCH_3 + ADC10SSEL_0;             // Repeat single channel, A3, ADC10OSC
//    ADC10CTL0 = ADC10SHT_0 + ADC10IE;             //ADC10 Interrupt Enable
//}

void SC_from_POT(void){
    ADC10CTL0 |= ADC10ON;                   // ADC10 ON
    ADC10CTL0 &= ~ENC;                      // disable conversion
    while(ADC10CTL1 & ADC10BUSY);           // Wait if ADC10 core is active
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
    __bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
    __no_operation();                       // For debugger
    ADC10CTL0 &= ~ADC10ON;                  // ADC10 OFF
}


//==========================================================
//         ADC10 Interrupt Service Routine
//==========================================================
//#pragma vector=ADC10_VECTOR
//__interrupt void ADC10_ISR(void){
//    __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
//}
//==========================================================
//                     STATE 6
//==========================================================
//void clearing(void){
//    RGB_CLR;
//    lcd_clear();
//}
//==========================================================
//                     STATE 8
//==========================================================
//void enable_transmition(void){
//    UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine
//    IE2 |= UCA0TXIE;                          // Enable TX interrupt
//}

/* - - - - - - - LCD interface - - - - - - - - -
*   This code will interface to a standard LCD controller
*  It uses it in 4 or 8 bit mode.
*/

//******************************************************************
//        send a command to the LCD
//******************************************************************
//void lcd_cmd(unsigned char c){
//
//    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
//
//    if (LCD_MODE == FOURBIT_MODE)
//    {
//        LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
//                LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
//        lcd_strobe();
//                LCD_DATA_WRITE &= ~OUTPUT_DATA;
//            LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
//        lcd_strobe();
//    }
//    else
//    {
//        LCD_DATA_WRITE = c;
//        lcd_strobe();
//    }
//}
//******************************************************************
//        send data to the LCD
//******************************************************************
//void lcd_data(unsigned char c){
//
//    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
//
//    LCD_DATA_WRITE &= ~OUTPUT_DATA;
//    LCD_RS(1);
//    if (LCD_MODE == FOURBIT_MODE)
//    {
//            LCD_DATA_WRITE &= ~OUTPUT_DATA;
//                LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
//        lcd_strobe();
//                LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
//                LCD_DATA_WRITE &= ~OUTPUT_DATA;
//        LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET;
//        lcd_strobe();
//    }
//    else
//    {
//        LCD_DATA_WRITE = c;
//        lcd_strobe();
//    }
//
//    LCD_RS(0);
//}
//******************************************************************
//        write a string of chars to the LCD
//******************************************************************
//void lcd_puts(const char *s){
//
//  while(*s)
//    lcd_data(*s++);
//}
//******************************************************************
//         initialize the LCD
//******************************************************************
//void lcd_init(){
//
//  char init_value;
//
//  if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
//  else init_value = 0x3F;
//
//  LCD_RS_DIR(OUTPUT_PIN);
//  LCD_EN_DIR(OUTPUT_PIN);
//  LCD_RW_DIR(OUTPUT_PIN);
//  LCD_DATA_DIR |= OUTPUT_DATA;
//  LCD_RS(0);
//  LCD_EN(0);
//  LCD_RW(0);
//
//  DelayMs(15);
//  LCD_DATA_WRITE &= ~OUTPUT_DATA;
//  LCD_DATA_WRITE |= init_value;
//  lcd_strobe();
//  DelayMs(5);
//  LCD_DATA_WRITE &= ~OUTPUT_DATA;
//  LCD_DATA_WRITE |= init_value;
//  lcd_strobe();
//  DelayUs(200);
//  LCD_DATA_WRITE &= ~OUTPUT_DATA;
//  LCD_DATA_WRITE |= init_value;
//  lcd_strobe();
//
//  if (LCD_MODE == FOURBIT_MODE){
//    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
//    LCD_DATA_WRITE &= ~OUTPUT_DATA;
//    LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
//    lcd_strobe();
//    lcd_cmd(0x28); // Function Set
//  }
//  else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots
//
//  lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
//  lcd_cmd(0x1); //Display Clear
//  lcd_cmd(0x6); //Entry Mode
//  lcd_cmd(0x80); //Initialize DDRAM address to zero
//}
////******************************************************************
////         Delay usec functions
////******************************************************************
//void DelayUs(unsigned int cnt){
//
//  unsigned char i;
//  for(i=cnt ; i>0 ; i--) asm(" nop"); // tha command asm(" nop") takes raphly 1usec
//
//}
//******************************************************************
//         Delay msec functions
//******************************************************************
void DelayMs(unsigned int cnt){

  unsigned char i;
  for(i=cnt ; i>0 ; i--) DelayUs(1000); // tha command asm(" nop") takes raphly 1usec

}
//******************************************************************
//             lcd strobe functions
//******************************************************************
//void lcd_strobe(){
//
//  LCD_EN(1);
//  asm(" nop");
//  asm(" nop");
//  LCD_EN(0);
//
//}


