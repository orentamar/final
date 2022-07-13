// HAL - Hardware Abstraction Layer

#ifndef _hal_H_
#define _hal_H_


extern volatile unsigned int state;
#define sleep_mode         0
#define ManualMotorControl 1
#define JoystickPainter    2
#define calibration        3
#define ScriptMode         4
#define move_motor_freely  5

#define Phi_deg 17
//extern volatile unsigned int x;
//extern volatile char X[10];

//================================================
//                     UART
//================================================
//------------------------------------------------
//                  Variables
//------------------------------------------------
//  state defines
#define State_Msg_Size = 4; // 2- State, 2- State_Stage, without '#'

// stage state defines
#define no_action 0
#define start_move_forwards 1
#define start_move_backwards 2
#define stop_motor 3

//  status defines
#define Status_Msg_Size = 30;
//------------------------------------------------
//              SERVICE FUNCTIONS
//------------------------------------------------
extern void GatherStatusInfo(void);
extern void enable_transmition(void);
__interrupt void USCI0RX_ISR(void);
__interrupt void USCI0TX_ISR(void);

//================================================
//                Stepper Motor
//================================================
//------------------------------------------------
//                  Variables
//------------------------------------------------
extern volatile int SM_Step;
extern volatile int SM_Half_Step;
extern volatile int StepperDelay;
//------------------------------------------------
//              SERVICE FUNCTIONS
//------------------------------------------------
void step_angle_update(void);
extern void move_forward(void);
extern void move_backward(void);
extern void move_forward_half(void);
extern void move_backward_half(void);

//================================================
//            State 3 - calibration
//================================================
//------------------------------------------------
//                  Variables
//------------------------------------------------
extern volatile unsigned int Phi;
//------------------------------------------------
//              SERVICE FUNCTIONS
//------------------------------------------------
extern void Phi_calculation(void);

//================================================
//             State 4 - Script Mode
//================================================
//------------------------------------------------
//          Flash memory configuration
//------------------------------------------------
#define FLASH_INFO_SEG_B_START     (char*)0xE080
#define FLASH_INFO_SEG_B_END       (char*)0xE0BF

#define FLASH_INFO_SEG_C_START     (char*)0xE040
#define FLASH_INFO_SEG_C_END       (char*)0xE07F

#define FLASH_INFO_SEG_D_START     (char*)0xE000
#define FLASH_INFO_SEG_D_END       (char*)0xE03F
//------------------------------------------------
//                  Variables
//------------------------------------------------

typedef struct Scripts{
    int written[3];
    char *pscript[3];
    int size[3];
    int lines[3];
    int num_script;
}Scripts;

//Scripts s = {{0}, {FLASH_INFO_SEG_B_START, FLASH_INFO_SEG_C_START, FLASH_INFO_SEG_D_START}, {0}, {0} , 1};
//int ScriptModeDelay = 50;
//int write_to_flash = 0;
//int offset = 0;

//------------------------------------------------
//              SERVICE FUNCTIONS
//------------------------------------------------
extern void clear_RGB(void);
extern void blink_RGB(int delay, int times);
extern void clear_RGB(void);
extern volatile int Out_to_RGB; // state 4

extern Scripts s;
extern int ScriptModeDelay;
extern int write_to_flash;
extern int offset;

extern int receive_int(void);
extern void receive_string(int *data);
extern void send_ack(int data);
extern void send_ss_data(int deg, int distance);


void write_seg (char* flash_ptr, int offset);
char read_char(char address);
int read_mem(int offset);
void blink_rgb(int delay, int times);
void rlc_leds(int delay, int times);
void rrc_leds(int delay, int times);
void servo_deg(int deg);
void servo_scan(int left, int right);

//extern int state;
extern volatile char p_tx[10];
extern volatile char p_rx[10];
extern int index;

extern void StopTimers();
extern void Timer0_A_delay_ms(int mili_sec);
extern void Timer1_A_delay_ms(int mili_sec);
extern void PWM_Servo_config(int deg);
extern int SS_Trig_config();
extern void SS_Echo_config();
//================================================
//                  Delay [ms]
//================================================
void Timer0_A_delay_ms(int ms);
void Timer1_A_delay_ms(int ms);
//void delay_ms(unsigned int ms);
__interrupt void Timer_A(void);




















////////// OLD ///////////
//===============================================
//                REAL
//================================================
extern void _buttonDebounceDelay(int button);
__interrupt void PORT1_ISR(void);




//================================================
// State 5
extern void SC_from_POT(void);

extern void adc10_config();
extern void adc10_enable(short enable);
__interrupt void ADC10_ISR(void);
//================================================
// State 6
extern void clearing(void);
//================================================
//// State 7
//extern void enable_transmition(void);
//================================================
// CONFIG: LCD BASIC FUNCTIONS
//================================================
#ifdef CHECKBUSY
    #define LCD_WAIT lcd_check_busy()
#else
    #define LCD_WAIT DelayMs(5)
#endif

#define FOURBIT_MODE    0x0
#define EIGHTBIT_MODE   0x1
#define LCD_MODE        FOURBIT_MODE

#define OUTPUT_PIN      1
#define INPUT_PIN       0
#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))
#define INPUT_DATA      0x00

#define LCD_STROBE_READ(value)  LCD_EN(1), \
                asm(" nop"), asm(" nop"), \
                value=LCD_DATA_READ, \
                LCD_EN(0)


#define lcd_cursor(x)           lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()             lcd_cmd(0x01)
#define lcd_putchar(x)          lcd_data(x)
#define lcd_goto(x)             lcd_cmd(0x80+(x))
#define lcd_cursor_right()      lcd_cmd(0x14)
#define lcd_cursor_left()       lcd_cmd(0x10)
#define lcd_display_shift()     lcd_cmd(0x1C)
#define lcd_home()              lcd_cmd(0x02)
#define cursor_off              lcd_cmd(0x0C)
#define cursor_on               lcd_cmd(0x0F)
#define lcd_function_set        lcd_cmd(0x3C) // 8bit,two lines,5x10 dots
#define lcd_new_line            lcd_cmd(0xC0)


extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char c);
extern void lcd_puts(const char *s);
extern void move_cursor_left(int mat_amount);
extern void lcd_init();
extern void lcd_strobe();
extern void DelayMs(unsigned int);
extern void DelayUs(unsigned int);


#endif
