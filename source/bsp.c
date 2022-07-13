#include <msp430.h>
#include  "../header/bsp.h"


void InitUART(void){
//***********************************************
//            UART configuration
//***********************************************
    UCA0CTL1 &= ~UCSWRST;                    // Initialize USCI state machine
    IE2 &= ~UCA0TXIE;                        // Disable TX interrupt
    IE2 |= UCA0RXIE;                         // Enable RX interrupt

    P1SEL  |= (BIT1 + BIT2) ;                   // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= (BIT1 + BIT2) ;                  // P1.1 = RXD, P1.2=TXD
    P1OUT  &= ~(BIT1 + BIT2);                 // P1.1 and P1.2 reset


    UCA0CTL1 |= UCSSEL_2;                    // CLK = SMCLK
    UCA0BR0 = 104;                           //
    UCA0BR1 = 0x00;                          //
    UCA0MCTL = UCBRS0;                       //

    _BIS_SR(GIE);

}

void InitGPIO(void){
    WDTCTL = WDTHOLD | WDTPW;        // Stop WDT
	
	if (CALBC1_1MHZ==0xFF)					// If calibration constant erased
	  {											
		while(1);                               // do not load, trap CPU!!	
	  }
	DCOCTL = 0;                               // Select lowest DCOx and MODx settings
	BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
	DCOCTL = CALDCO_1MHZ;

//***********************************************
//         Stepper Motor configuration
//***********************************************
	// P2.3-P2.6 are both for motor and 4 leds
	SMPortSel &= ~(BIT3 + BIT4 + BIT5 + BIT6);     //Stepper Motor:Phases:A-2.3,B-2.4,C-2.5, D-2.6 as GPIO
	SMPortDir |= (BIT3 + BIT4 + BIT5 + BIT6);      //Stepper Motor: as GPIO-output

//***********************************************
//         Joy Stick configuration
//***********************************************
//	SMPortSel &= ~(BIT3 + BIT4 + BIT5);      //Joy Stick: A-1.3,B-1.4,C-1.5, D-1.6 as GPIO
//	SMPortDir &= ~(BIT3 + BIT4 + BIT5);      //Joy Stick: as GPIO-output
//***********************************************
//             LEDs configuration
//***********************************************
// P1 LEDs: 1.7,1.6,1.0, P2 LEDs: P2.7
// P2.3-P2.6 are both for motor and 4 leds
    LEDPort1Sel  &= ~(BIT7 + BIT6 + BIT0);   //LEDs: 1.7,1.6,1.0 as GPIO

    LEDPort1Dir  |= (BIT7 + BIT6 + BIT0);    //LEDs: as GPIO-input

    LEDPort2Sel  &= ~(BIT7);                 //LEDs: 2.7 as GPIO
    LEDPort2Dir  |= ~(BIT7);                 //LEDs: as GPIO-input

    Leds_CLR;
//***********************************************
//            RGB configuration
//***********************************************
    RGBPortSel &= ~(BIT0 + BIT1 + BIT2);     //RGB: B-2.0,G-2.1,R-2.2 as GPIO
    RGBPortDir |= (BIT0 + BIT1 + BIT2);      //RGB: as GPIO-output
    RGB_CLR;                                 //clear RGB

//***********************************************
//            ADC configuration
//***********************************************
    ADC_Ctrl0 = ADC_SHT + MSC + SREF_0;
    ADC_Ctrl1 = ADC_Sel + INCH_7 + CONSEQ_1;
    ADC_Ch_En |= BIT3 + BIT4 + BIT5;
    ADC_DTC = 0x03;

    //***********************************************
    //            UART configuration
    //***********************************************
    //
    //  P1SEL |= BIT1 + BIT2 ;                   // P1.1 = RXD, P1.2=TXD
    //  P1SEL2 |= BIT1 + BIT2 ;                  // P1.1 = RXD, P1.2=TXD
    //  P1OUT &= ~(BIT1 + BIT2);                 // P1.1 and P1.2 reset
    //
    //
    //    UCA0CTL1 |= UCSSEL_2;                    // CLK = SMCLK
    //    UCA0BR0 = 104;                           //
    //    UCA0BR1 = 0x00;                          //
    //    UCA0MCTL = UCBRS0;                       //
    //
    //    _BIS_SR(GIE);

    //***********************************************
    //            POT configuration
    //***********************************************
    //
    //    //  ADC 10 configuration
    //        ADC10AE0 |= BIT3;       // ADC10 interrupt enable
    //    // Port 1 - POT Port configuration---------------
    //        POTPortSel &= ~BIT3;    //POT-1.3 as GPIO
    //        POTPortDir |= BIT3;     //POT as GPIO-output
//
////***********************************************
////            REAL- PB1 configuration
////***********************************************
//
//        PB1_IntPending  &= ~BIT0;
//        PB1_IntEnable |= BIT0;
//        PB1_IntEdgeSel |= BIT0;
//        PB1_PortSel &= ~BIT0;
//        PB1_PortDir &= ~BIT0;
////***********************************************
////            LCD control configuration
////***********************************************
//
//  LCD_CTRL_OUT_PORT  &= ~BIT5;       // lcd enable
//  LCD_CTRL_SEL_PORT  &= ~BIT5;
//  LCD_CTRL_DIR_PORT  |= BIT5;
//
//
//  LCD_CTRL_OUT_PORT  &= ~BIT6;       // lcd RS
//  LCD_CTRL_SEL_PORT  &= ~BIT6;
//  LCD_CTRL_DIR_PORT  |= BIT6;
//
//  LCD_CTRL_OUT_PORT  &= ~BIT7;       // lcd RW
//  LCD_CTRL_SEL_PORT  &= ~BIT7;
//  LCD_CTRL_DIR_PORT  |= BIT7;


}


