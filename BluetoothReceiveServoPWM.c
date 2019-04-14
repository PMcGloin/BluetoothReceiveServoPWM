//#define _XTAL_FREQ 16000000     //18f45k20
#define _XTAL_FREQ 8000000    //18f4520
//#include <pic18f45k20.h>
#include <pic18f4520.h>
#include "uart.h"
#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */

// PIC18F4520 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1H
#pragma config OSC = INTIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#define ServoShoulderRoationPWM PORTBbits.RB6
#define ServoShoulderAnglePWM PORTBbits.RB1
#define ServoElbowAnglePWM PORTBbits.RB2
#define ServoWristAnglePWM PORTBbits.RB3
#define ServoWristRotationPWM PORTBbits.RB4
#define ServoGripperPWM PORTBbits.RB5
/* 
    * The system clock is 8 MHz. If no prescaler is used, the timer will step
    * every 0.5 microseconds.
    * 8.3us or 16.6 cycles per degree
    * 2^16-40000 = 25536 steps = 20ms
    * 2^16-4000 = 61536 steps = 2ms, 180deg
    * 2^16-1000 = 64536 steps = 0.5ms, 0deg
    */
void pulseDelay(unsigned int delay){
        TMR0ON = 1;             //start timer0
        TMR0 = 65536 - delay;
        //TMR0 = 65536 - 2166;  //test code
        while (TMR0IF == 0) {}  // wait for the timer to overflow
        TMR0IF = 0;             // clear the overflow flag
        TMR0ON = 0;             //stop timer0
}
void main()
{
    /*clock speed setup*/
    IRCF2 = 1;
    IRCF1 = 1;
    IRCF0 = 1;              //8MHz 4520 / 16MHz 45k20
    TRISB = 0x00;           //PORTB as Output
    UART_Init(9600);        //uart.h
    /*Timer0 Setup*/
    T08BIT = 0;             // 16-bit mode
    T0CS = 0;               // clock source internal osc
    PSA = 1;                // no prescaler
    unsigned char Output[6], i;
    double receivedAngle = 0;
    unsigned int delayCycles = 0, fianlDelayCycles = 0;
    for(i=0;i<6;i++){
        Output[i] = 70;
    }
    while(1){
        if (RCIF == 1){ //receive interrupt flag
            UART_Read_Text(Output, 6);    //uart.h
            receivedAngle = Output[0];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoShoulderRoationPWM = 1;
            pulseDelay(delayCycles);
            ServoShoulderRoationPWM = 0;
            receivedAngle = Output[1];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoShoulderAnglePWM = 1;
            pulseDelay(delayCycles);
            ServoShoulderAnglePWM = 0;
            receivedAngle = Output[2];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoElbowAnglePWM = 1;
            pulseDelay(delayCycles);
            ServoElbowAnglePWM = 0;
            receivedAngle = Output[3];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoWristAnglePWM = 1;
            pulseDelay(delayCycles);
            ServoWristAnglePWM = 0;
            receivedAngle = Output[4];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoWristRotationPWM = 1;
            pulseDelay(delayCycles);
            ServoWristRotationPWM = 0;
            receivedAngle = Output[5];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoGripperPWM = 1;
            pulseDelay(delayCycles);
            ServoGripperPWM = 0;
            fianlDelayCycles = (40000 - 16200) - fianlDelayCycles;
            PORTBbits.RB0 = 1;
            pulseDelay(fianlDelayCycles);
            PORTBbits.RB0 = 0;
            fianlDelayCycles = 0;
        }
        if (OERR == 1){ //overrun error
            CREN = 0;
            CREN = 1;   //Reset CREN
        }
        else{
            while(!RCIF){
            receivedAngle = Output[0];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoShoulderRoationPWM = 1;
            pulseDelay(delayCycles);
            ServoShoulderRoationPWM = 0;
            receivedAngle = Output[1];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoShoulderAnglePWM = 1;
            pulseDelay(delayCycles);
            ServoShoulderAnglePWM = 0;
            receivedAngle = Output[2];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoElbowAnglePWM = 1;
            pulseDelay(delayCycles);
            ServoElbowAnglePWM = 0;
            receivedAngle = Output[3];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoWristAnglePWM = 1;
            pulseDelay(delayCycles);
            ServoWristAnglePWM = 0;
            receivedAngle = Output[4];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoWristRotationPWM = 1;
            pulseDelay(delayCycles);
            ServoWristRotationPWM = 0;
            receivedAngle = Output[5];
            delayCycles = (unsigned int)(((receivedAngle/180.0)*3000)+1000);
            fianlDelayCycles = fianlDelayCycles + delayCycles;
            ServoGripperPWM = 1;
            pulseDelay(delayCycles);
            ServoGripperPWM = 0;
            fianlDelayCycles = (40000 - 16200) - fianlDelayCycles;
            PORTBbits.RB0 = 1;
            pulseDelay(fianlDelayCycles);
            PORTBbits.RB0 = 0;
            fianlDelayCycles = 0;
            }
        }
    }
}