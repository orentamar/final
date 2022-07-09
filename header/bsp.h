#ifndef _bsp_H_
#define _bsp_H_

extern void InitGPIO(void);
//***********************************************
//            REAL- PB1 configuration
//***********************************************
#define PB1_PortIN           P1IN
#define PB1_PortOUT          P1OUT
#define PB1_IntPending       P1IFG
#define PB1_IntEnable        P1IE
#define PB1_IntEdgeSel       P1IES
#define PB1_PortSel          P1SEL
#define PB1_PortDir          P1DIR



//***********************************************
//            UART configuration
//***********************************************

#define RxBuffer UCA0RXBUF
#define TxBuffer UCA0TXBUF

////***********************************************
////            POT configuration
////***********************************************
//#define  POTPortSel     P1SEL
//#define  POTPortDir     P1DIR

//***********************************************
//         Stepper Motor configuration
//***********************************************
#define  SMPortSel     P1SEL
#define  SMPortDir     P1DIR
#define  SMPortOUT     P1OUT

//***********************************************
//            RGB configuration
//***********************************************
#define  RGBPortSel     P2SEL
#define  RGBPortDir     P2DIR
#define  RGBPortOUT     P2OUT

#define RGB_CLR RGBPortOUT &= ~(BIT0 + BIT1 + BIT2);  //clear RGB
//***********************************************
//            LCD data configuration
//***********************************************

#define LCD_DATA_WRITE       P1OUT
#define LCD_DATA_DIR         P1DIR
#define LCD_DATA_READ        P1IN

#define LCD_DATA_OFFSET 0x04 //data pin selection offset for 4 bit mode, variable range is 0-4, default 0 - Px.0-3, no offset

//***********************************************
//            LCD control configuration
//***********************************************

#define LCD_CTRL_OUT_PORT    P2OUT
#define LCD_CTRL_SEL_PORT    P2SEL
#define LCD_CTRL_DIR_PORT    P2DIR

#define LCD_EN(a)   (!a ? (LCD_CTRL_OUT_PORT&=~BIT5) : (LCD_CTRL_OUT_PORT|=BIT5))       // P2.5 is lcd enable pin
#define LCD_EN_DIR(a)   (!a ? (LCD_CTRL_DIR_PORT&=~BIT5) : (LCD_CTRL_DIR_PORT|=BIT5))   // P2.5 pin direction

#define LCD_RS(a)   (!a ? (LCD_CTRL_OUT_PORT&=~BIT6) : (LCD_CTRL_OUT_PORT|=BIT6))       // P2.6 is lcd RS pin
#define LCD_RS_DIR(a)   (!a ? (LCD_CTRL_DIR_PORT&=~BIT6) : (LCD_CTRL_DIR_PORT|=BIT6))   // P2.6 pin direction

#define LCD_RW(a)   (!a ? (LCD_CTRL_OUT_PORT&=~BIT7) : (LCD_CTRL_OUT_PORT|=BIT7))       // P2.7 is lcd RW pin
#define LCD_RW_DIR(a)   (!a ? (LCD_CTRL_DIR_PORT&=~BIT7) : (LCD_CTRL_DIR_PORT|=BIT7))   // P2.7 pin direction


#endif
