// ADC14.c
// Runs on MSP432
// ADC input, software trigger, 14-bit conversion,
// 2.5 V static (always on) reference
// Daniel Valvano
// June 11, 2015

// Modified
// LJBeato
// March 2021

#include <stdint.h>
#include "msp.h"
#include "Common.h"
#include "CortexM.h"
#include "ADC14.h"
//
//
// We are going to use Port 4 Pin 7 which is ADC A6
// P4.7 = A6
//
//
void ADC0_InitSWTriggerCh6(void)
{
	// wait for reference to be idle
    // 1) configure reference for static 2.5V
    REF_A->CTL0 = 0x0039;   
    
    // wait for reference voltage to be ready
    while((REF_A->CTL0&0x1000) == 0){};

    // 2) ADC14ENC = 0 to allow programming
    ADC14->CTL0 &= ~BIT1;
        
    // 3) wait for BUSY to be zero
    while (ADC14->CTL0 & 0x00010000){};
    
    // 4) single, SMCLK, on, disabled, /1, 32 clocks, SHM	pulse-mode
    ADC14->CTL0 = 0x04203310;
    
    // 5) ADC14MEM0, 14-bit, ref on, regular power
    ADC14->CTL1 = 0x00000030;
    
	// 6) 0 to 2.5V, channel 6
    ADC14->MCTL[0] = 0x00000186;
    
    // 7) no interrupts
    ADC14->IER0 = 0;
    ADC14->IER1 = 1;

    // 8) analog mode on A6, P4.7
    P4->SEL0 |= BIT7;
    P4->SEL1 |= BIT7;

    // 9) enable
    ADC14->CTL0 |= BIT1;
}


// ADC14->IFGR0 bit 0 is set when conversion done
// cleared on read ADC14MEM0
// ADC14->CLRIFGR0 bit 0, write 1 to clear flag
// ADC14->IVx is 0x0C when ADC14MEM0 interrupt flag; Interrupt Flag: ADC14IFG0
// ADC14->MEM[0] 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
unsigned int  ADC_In(void)
{
	// 1) wait for BUSY ot be zero
    while (ADC14->CTL0 & 0x00010000);
    
    // 2) start single conversion
    ADC14->CTL0 |= 3;

    // 3) wait for ADC14->IFGR0, ADC14->IFGR0 bit 0 is set when conversion done
    while ((ADC14->IFGR0&BIT0) == 0);

    return ADC14->MEM[0] & 0xFFFF; // Return the result
}
