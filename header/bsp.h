#ifndef _bsp_H_
#define _bsp_H_

extern void InitGPIO(void);
extern void InitUART(void);
//***********************************************
//            UART configuration
//***********************************************

#define RxBuffer UCA0RXBUF
#define TxBuffer UCA0TXBUF

//***********************************************
//         Stepper Motor configuration
//***********************************************
#define  SMPortSel     P2SEL
#define  SMPortDir     P2DIR
#define  SMPortOUT     P2OUT

//***********************************************
//         Joy Stick configuration
//***********************************************
#define  JSPortSel     P1SEL
#define  JSPortDir     P1DIR

//***********************************************
//            ADC configuration
//***********************************************
#define ADC_Ctrl0   ADC10CTL0
#define ADC_Ctrl1   ADC10CTL1
#define ADC_Ch_En   ADC10AE0
#define ADC_SHT     ADC10SHT_3
#define ADC_Sel     ADC10SSEL_3
#define ADC_On      ADC10ON
#define ADC_IE      ADC10IE
#define ADC_SC      ADC10SC
#define ADC_Flg     ADC10IFG
#define ADC_Mem     ADC10MEM
#define ADC_Busy    ADC10BUSY
#define ADC_DTC     ADC10DTC1
#define ADC_DCT_SA  ADC10SA
//***********************************************
//            RGB configuration
//***********************************************
#define  RGBPortSel     P2SEL
#define  RGBPortDir     P2DIR
#define  RGBPortOUT     P2OUT

#define RGB_CLR RGBPortOUT &= ~(BIT0 + BIT1 + BIT2);  //clear RGB

//***********************************************
//            LEDs configuration
//***********************************************
// P1 LEDs: 1.7,1.6,1.0, P2 LEDs: P2.7
// Port 1
#define  LEDPort1Sel     P1SEL
#define  LEDPort1Dir     P1DIR
#define  LEDPort1OUT     P1OUT
// Port 2
#define  LEDPort2Sel     P2SEL
#define  LEDPort2Dir     P2DIR
#define  LEDPort2OUT     P2OUT

#define Leds_CLR  LEDPort1OUT &= ~(BIT7 + BIT6 + BIT0);\
                  LEDPort2OUT &= ~(BIT7 + BIT6 + BIT5 + BIT4 + BIT2);


////***********************************************
////            POT configuration
////***********************************************
//#define  POTPortSel     P1SEL
//#define  POTPortDir     P1DIR
////***********************************************
////            REAL- PB1 configuration
////***********************************************
//#define PB1_PortIN           P1IN
//#define PB1_PortOUT          P1OUT
//#define PB1_IntPending       P1IFG
//#define PB1_IntEnable        P1IE
//#define PB1_IntEdgeSel       P1IES
//#define PB1_PortSel          P1SEL
//#define PB1_PortDir          P1DIR

//***********************************************
//            LCD data configuration
//***********************************************
//
//#define LCD_DATA_WRITE       P1OUT
//#define LCD_DATA_DIR         P1DIR
//#define LCD_DATA_READ        P1IN
//
//#define LCD_DATA_OFFSET 0x04 //data pin selection offset for 4 bit mode, variable range is 0-4, default 0 - Px.0-3, no offset
//
//

//***********************************************
//            LCD control configuration
//***********************************************
//
//#define LCD_CTRL_OUT_PORT    P2OUT
//#define LCD_CTRL_SEL_PORT    P2SEL
//#define LCD_CTRL_DIR_PORT    P2DIR
//
//#define LCD_EN(a)   (!a ? (LCD_CTRL_OUT_PORT&=~BIT5) : (LCD_CTRL_OUT_PORT|=BIT5))       // P2.5 is lcd enable pin
//#define LCD_EN_DIR(a)   (!a ? (LCD_CTRL_DIR_PORT&=~BIT5) : (LCD_CTRL_DIR_PORT|=BIT5))   // P2.5 pin direction
//
//#define LCD_RS(a)   (!a ? (LCD_CTRL_OUT_PORT&=~BIT6) : (LCD_CTRL_OUT_PORT|=BIT6))       // P2.6 is lcd RS pin
//#define LCD_RS_DIR(a)   (!a ? (LCD_CTRL_DIR_PORT&=~BIT6) : (LCD_CTRL_DIR_PORT|=BIT6))   // P2.6 pin direction
//
//#define LCD_RW(a)   (!a ? (LCD_CTRL_OUT_PORT&=~BIT7) : (LCD_CTRL_OUT_PORT|=BIT7))       // P2.7 is lcd RW pin
//#define LCD_RW_DIR(a)   (!a ? (LCD_CTRL_DIR_PORT&=~BIT7) : (LCD_CTRL_DIR_PORT|=BIT7))   // P2.7 pin direction
#endif
