#include <msp430.h>

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
volatile unsigned int Current_angle;
volatile char Current_angle_str[4];
//------------------------------------------------
//                    UART
//------------------------------------------------
volatile unsigned int first_byte_MSG;  // UART RX

//              --- Status ---

volatile char StatusArray[5]; // as Msg_Size
unsigned int status_flg;

//               --- State ---

unsigned int state_flg;
unsigned int Msg_location = 0;
unsigned int state_stage = 0;



//------------------------------------------------
//               Stepper Motor
//------------------------------------------------
volatile int SM_Step_Right = 0x40;       //0-0001-000
volatile int SM_Step_Left = 0x08;       //0-1000-000
volatile int SM_Half_Step_Right = 0x60; //0-0011-000
volatile int SM_Half_Step_Left = 0x18; //0-1100-000
volatile int StepperDelay = 20;   // f = MHz
//------------------------------------------------
//                  State 3
//------------------------------------------------
volatile int SM_Counter= 0;
volatile unsigned int Phi;
volatile char Phi_str[4];


//------------------------------------------------------------------

volatile char POT[5];

const char MENU[] = "\n"
                    "                       Menu\n"
                    "*******************************************************\n"
                    "1. Blink RGB LED, color by color with delay of X[ms]\n"
                    "2. Count up onto LCD screen with delay of X[ms]\n"
                    "3. Count down onto LCD screen with delay of X[ms]\n"
                    "4. Get delay time X[ms]\n"
                    "5. Potentiometer 3-digit value [v]\n"
                    "6. Clear LCD screen\n"
                    "7. On each PB1 pressed, Send a Message\n"
                    "8. Show menu\n"
                    "9. Sleep\n"
                    "*******************************************************\r";

volatile char REAL_Str[16] = "I love my Negev ";
//==========================================================
//                    Stepper Motor
//==========================================================
void step_angle_update(void){
    if (state_stage==start_move_forwards){
        SM_Counter++;
        Current_angle += Phi;
    }else if (state_stage==start_move_backwards){
        SM_Counter--;
        Current_angle -= Phi;
    }
}


// Full step
void move_forward(void){
    SM_Step_Right <<= 1;
        if (SM_Step_Right == 0x04){
            SM_Step_Right = 0x40;
        }
        SMPortOUT = SM_Step_Right;
        delay_ms(StepperDelay);
}

void move_backward(void){
    SM_Step_Left >>= 1;
        if (SM_Step_Left == 0x80){
            SM_Step_Left = 0x08;
        }
        SMPortOUT = SM_Step_Left;
        delay_ms(StepperDelay);
}

// Half step
void move_forward_half(void){
    SM_Half_Step_Right <<= 1;
        if (SM_Half_Step_Right == 0x0C){
            SM_Half_Step_Right = 0x60;
        }
        SMPortOUT = SM_Half_Step_Right;
        delay_ms(StepperDelay);
}
void move_backward_half(void){
    SM_Half_Step_Left >>= 1;
        if (SM_Half_Step_Left == 0xC0){
            SM_Half_Step_Left = 0x18;
        }
        SMPortOUT = SM_Half_Step_Left;
        delay_ms(StepperDelay);
}
//==========================================================
//                     STATE 3
//==========================================================
void Phi_calculation(void){
    Phi = 360*100;
    Phi = Phi/SM_Counter;
    int2str(Phi_str,Phi);
}
//==========================================================
//                      UART
//==========================================================

void GatherStatusInfo(void){
    int idx;
    StatusArray[0]='#';
    for (idx=1;idx<5;idx++){
        StatusArray[idx]=Phi_str[idx];
    }
//StatusArray[4]=;

}

void enable_transmition(void){
    UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine
    IE2 |= UCA0TXIE;                          // Enable TX interrupt
}

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
        }else if (Msg_location==2){ // Get status_stage value
            state_stage =  RxBuffer;
            Msg_location = 0;
            state_flg = 0; // Done getting all state information
            __bic_SR_register_on_exit(LPM0_bits + GIE);  // Exit LPM0 on return to main
        }

    }

//    clear_RGB();
//    if(state != 4){
//        state = RxBuffer - 48;   // 0 in ASCII is 48
//
//        __bic_SR_register_on_exit(LPM0_bits + GIE);  // Exit LPM0 on return to main
//    }
//    else{   // Get new delay
//      X[j++] = RxBuffer;
//      if (X[j-1] == '\0'){
//          j = 0;
//          x = str2int(X);
//          state = 9;
//      }
//    }
}


//==========================================================
//        UART-  Transmitter Interrupt Service Routine
//==========================================================
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    if (status_flg==1){
        TxBuffer = StatusArray[i++];
        if (i == sizeof StatusArray -1){                         // check if done with transmition
            i = 0;
            IE2 &= ~UCA0TXIE;                            // Disable TX interrupt
            IE2 |= UCA0RXIE;                             // Enable RX interrupt
            status_flg = 0;
        }
    }else{
      IE2 &= ~UCA0TXIE;                                  // Disable TX interrupt
    }

//    if(state == 5){
//        TxBuffer = POT[i++];
//        if (i == sizeof POT -1){                         // check if done with transmition
//            i = 0;
//            IE2 &= ~UCA0TXIE;                            // Disable TX interrupt
//            IE2 |= UCA0RXIE;                             // Enable RX interrupt
//            state = 9;
//            }
//    }
//    else if(state == 8){
//        TxBuffer = MENU[i++];
//        if (i == sizeof MENU - 1){                       // check if done with transmition
//                i = 0;
//                IE2 &= ~UCA0TXIE;                        // Disable TX interrupt
//                IE2 |= UCA0RXIE;                         // Enable RX interrupt
//        }
//    }
//    else if(state == 7){
//           TxBuffer = REAL_Str[i++];
//           if (i == sizeof REAL_Str - 1){                       // check if done with transmition
//               i = 0;
//               IE2 &= ~UCA0TXIE;                        // Disable TX interrupt
//               IE2 |= UCA0RXIE;                         // Enable RX interrupt
//               state = 9;
//           }
//       }
//    else{
//      IE2 &= ~UCA0TXIE;                                  // Disable TX interrupt
//    }
}


//------------------------------------------------------------------
//**************** Interrupt Service Routines (ISR) ****************
//------------------------------------------------------------------
//--------------- Port1 Interrupt Service Routine -------------------
//------------------------------------------------------------------
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void){
    if(PB1_IntPending & 0x01){
        _buttonDebounceDelay(0x01);
//        menu_tx = 1;
        if (state == 7){
            UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
            IE2 |= UCA0TXIE;                          // Enable USCI_A0 TX interrupt
        }else {
            state = 0;
        }
    }
}
/////////////////////////// OLD ////////////////////////////
//==========================================================
//                     Real PB1 ISR
//==========================================================

//------------------------------------------------------------------
void _buttonDebounceDelay(int button){
    volatile unsigned int i;
    for(i = 1000; i > 0; i--);                     //delay, button debounce
    while(!(PB1_PortIN & button));          // wait of release the button
    for(i = 1000; i > 0; i--);                     //delay, button debounce
    PB1_IntPending &= ~button;              // manual clear of p1.button
}

//==========================================================
//                     STATE 1
//==========================================================
void clear_RGB(void){

    RGB_CLR;
}

void blink_RGB(int delay){
    if (Out_to_RGB == 0x04){
        Out_to_RGB = 0x01;
    }else{
        Out_to_RGB <<= 1;
    }
    RGBPortOUT = Out_to_RGB;
    delay_ms(delay);
}

//==========================================================
//                     STATE 5
//==========================================================

void adc10_config(){
    ADC10CTL1 = INCH_3 + ADC10SSEL_0;             // Repeat single channel, A3, ADC10OSC
    ADC10CTL0 = ADC10SHT_0 + ADC10IE;             //ADC10 Interrupt Enalbe
}

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
//         ADC10 Interrupt Service Rotine
//==========================================================
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void){
    __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}
//==========================================================
//                     STATE 6
//==========================================================
void clearing(void){
    RGB_CLR;
    lcd_clear();
}
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
void lcd_cmd(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    if (LCD_MODE == FOURBIT_MODE)
    {
        LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
                LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
                LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }
}
//******************************************************************
//        send data to the LCD
//******************************************************************
void lcd_data(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_RS(1);
    if (LCD_MODE == FOURBIT_MODE)
    {
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
                LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
                LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
                LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }

    LCD_RS(0);
}
//******************************************************************
//        write a string of chars to the LCD
//******************************************************************
void lcd_puts(const char *s){

  while(*s)
    lcd_data(*s++);
}
//******************************************************************
//         initialize the LCD
//******************************************************************
void lcd_init(){

  char init_value;

  if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
  else init_value = 0x3F;

  LCD_RS_DIR(OUTPUT_PIN);
  LCD_EN_DIR(OUTPUT_PIN);
  LCD_RW_DIR(OUTPUT_PIN);
  LCD_DATA_DIR |= OUTPUT_DATA;
  LCD_RS(0);
  LCD_EN(0);
  LCD_RW(0);

  DelayMs(15);
  LCD_DATA_WRITE &= ~OUTPUT_DATA;
  LCD_DATA_WRITE |= init_value;
  lcd_strobe();
  DelayMs(5);
  LCD_DATA_WRITE &= ~OUTPUT_DATA;
  LCD_DATA_WRITE |= init_value;
  lcd_strobe();
  DelayUs(200);
  LCD_DATA_WRITE &= ~OUTPUT_DATA;
  LCD_DATA_WRITE |= init_value;
  lcd_strobe();

  if (LCD_MODE == FOURBIT_MODE){
    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
    lcd_strobe();
    lcd_cmd(0x28); // Function Set
  }
  else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots

  lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
  lcd_cmd(0x1); //Display Clear
  lcd_cmd(0x6); //Entry Mode
  lcd_cmd(0x80); //Initialize DDRAM address to zero
}
//******************************************************************
//         Delay usec functions
//******************************************************************
void DelayUs(unsigned int cnt){

  unsigned char i;
  for(i=cnt ; i>0 ; i--) asm(" nop"); // tha command asm(" nop") takes raphly 1usec

}
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
void lcd_strobe(){

  LCD_EN(1);
  asm(" nop");
  asm(" nop");
  LCD_EN(0);

}

//===========================================================
//            integer to string converter
//===========================================================
void int2str(char *str, unsigned int num){
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
  str[strSize] = '\0';
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
////===========================================================
////                     Delay [ms]
////===========================================================
//
//void delay_ms(int ms){
//  int tmp = ms;
//  TACCTL0 = CCIE;                             // CCR0 interrupt enabled
//  TACCR0 = ms*131;
//  TACTL = TASSEL_2 + ID_3 + MC_1 + TACLR;   // SMCLK/8 = 131072[Hz], upmode
//  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
//}
void delay_ms(unsigned int ms)
{
    while (ms)
    {
        __delay_cycles(1000); //1000 for 1MHz and 16000 for 16MHz
        ms--;
    }
}

////===========================================================
////          Timer A0 Interrupt Service Routine
////===========================================================
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void){
    TACCTL0 &= ~CCIE;                              // CCR0 interrupt enabled
    __bic_SR_register_on_exit(LPM0_bits + GIE);  // Exit LPM0 on return to main
}

