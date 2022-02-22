// PIC16F886 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include <stdio.h>
#include <stdlib.h>
#define _XTAL_FREQ 4000000 //Crystal Oscillator Frequency


#include <xc.h>

unsigned short result;


void main(void) {
    
    // OSCILLATOR control register
//    OSCCONbits.IRCF = 
    
    // PORTB Tri-State Register
    TRISB = 0x0; // Port B is output
    TRISA = 0xff;
    
    // PORTB Register
    PORTA = 0x0;
    PORTB = 0x0; // PORT B is < V_IL
    
    // Analog Select Register
    ANSEL = 0x0;
    ANSELbits.ANS0 = 1; // A0 Analog Input
    
    // Analog Select High Register
    ANSELH = 0x0;
        
    // ADC Control Register 0
    ADCON0bits.ADCS = 0b11; // ADC Conversion Clock Select, FRC (internal oscillator)
    ADCON0bits.CHS = 0b0000; // Analog Channel Select, AN0
    
    // ADC Control Register 1
    ADCON1bits.VCFG1 = 0; // Voltage Ref, VSS
    ADCON1bits.VCFG0 = 0; // Voltage Ref, VDD
    ADCON1bits.ADFM = 1; // ADC Result Format, Right Justified
    
    // Peripheral Interrupt request register
    PIR1bits.ADIF = 0; // ADC interrupt flag
    
    // Peripheral Interrupt Enables
    PIE1bits.ADIE = 1; // ADC Interrupt enable bit

    ADCON0bits.ADON = 1; // ADC Enable Bit
    
    // 4 wait the required acquisition time
    while (1){
        // Start Conversion
        ADCON0bits.GO = 1;

        while(ADCON0bits.GO == 1); // Wait till conversion is done
        
        // Read results, stored in two registers (10 bit resolution)
        result = 0x0000;
        result = ADRESH;
        result = result << 8;
        result = result | ADRESL;
        
        // Clear all LEDs
        RB0 = 0;
        RB1 = 0;
        RB2 = 0;
        RB3 = 0;
        
        // Enable correct LEDs
        if ((result / 64) & 1){
            RB0 = 1;
        }
        if ((result / 64) & 2) {
            RB1 = 1;
        }
        if ((result / 64) & 4) {
            RB2 = 1;
        }
        if ((result / 64) & 8) {
            RB3 = 1;
        }
        
        PIR1bits.ADIF = 0; // ADC interrupt flag
       
    }
    
    return;
}
