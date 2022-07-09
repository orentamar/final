#include <msp430.h>
#include  "../header/bsp.h"

void InitGPIO(void){
    WDTCTL = WDTHOLD | WDTPW;        // Stop WDT
	
	if (CALBC1_1MHZ==0xFF)					// If calibration constant erased
	  {											
		while(1);                               // do not load, trap CPU!!	
	  }
	DCOCTL = 0;                               // Select lowest DCOx and MODx settings
	BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
	DCOCTL = CALDCO_1MHZ;
////***********************************************
////            REAL- PB1 configuration
////***********************************************

	PB1_IntPending  &= ~BIT0;
	PB1_IntEnable |= BIT0;
	PB1_IntEdgeSel |= BIT0;
	PB1_PortSel &= ~BIT0;
	PB1_PortDir &= ~BIT0;
//***********************************************
//            UART configuration
//***********************************************

	P1SEL |= BIT1 + BIT2 ;                   // P1.1 = RXD, P1.2=TXD
	P1SEL2 |= BIT1 + BIT2 ;                  // P1.1 = RXD, P1.2=TXD
	P1OUT &= ~(BIT1 + BIT2);                 // P1.1 and P1.2 reset


    UCA0CTL1 |= UCSSEL_2;                    // CLK = SMCLK
    UCA0BR0 = 104;                           //
    UCA0BR1 = 0x00;                          //
    UCA0MCTL = UCBRS0;                       //

    _BIS_SR(GIE);

////***********************************************
////            POT configuration
////***********************************************
//
//    //  ADC 10 configuration
//        ADC10AE0 |= BIT3;       // P1.3 (A3) ADC option select
//    // Port 1 - POT Port configuration---------------
//        POTPortSel &= ~BIT3;    //POT-1.3 as GPIO
//        POTPortDir |= BIT3;     //POT as GPIO-output

//***********************************************
//         Stepper Motor configuration
//***********************************************
/// CHANGE BITS AFTER DELETE LCD
	SMPortSel &= ~(BIT3 + BIT4 + BIT5 + BIT6);     //Stepper Motor:Phases:A-1.3,B-1.4,C-1.5, D-1.6 as GPIO
	SMPortDir |= (BIT3 + BIT4 + BIT5 + BIT6);      //Stepper Motor: as GPIO-output

//***********************************************
//            RGB configuration
//***********************************************
    RGBPortSel &= ~(BIT0 + BIT1 + BIT2);     //RGB: B-2.0,G-2.1,R-2.2 as GPIO
    RGBPortDir |= (BIT0 + BIT1 + BIT2);      //RGB: as GPIO-output
    RGB_CLR;                                 //clear RGB

//***********************************************
//            LCD control configuration
//***********************************************

  LCD_CTRL_OUT_PORT  &= ~BIT5;       // lcd enable
  LCD_CTRL_SEL_PORT  &= ~BIT5;
  LCD_CTRL_DIR_PORT  |= BIT5;


  LCD_CTRL_OUT_PORT  &= ~BIT6;       // lcd RS
  LCD_CTRL_SEL_PORT  &= ~BIT6;
  LCD_CTRL_DIR_PORT  |= BIT6;

  LCD_CTRL_OUT_PORT  &= ~BIT7;       // lcd RW
  LCD_CTRL_SEL_PORT  &= ~BIT7;
  LCD_CTRL_DIR_PORT  |= BIT7;

}


