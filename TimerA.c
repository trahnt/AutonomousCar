// TimerA.c

/*  LJBeato
	2021
	TimerA functionality to drive DC motor
	and Servo Motor
 */

#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include "TimerA.h"
#include "Uart.h"

// Make these arrays 5 deep, since we are using indexes 1-4 for the pins
static uint32_t DEFAULT_PERIOD_A0[5] = {0, 0, 0, 0, 0};
static uint32_t DEFAULT_PERIOD_A2[5] = {0, 0, 0, 0, 0};

void (*TimerA2Task)(void); // pwm interrupt task

//***************************PWM_Init*******************************
// PWM output on P2.4, P2.5, P2.6, P2.7
// Inputs:  period of P2.4...P2.7 is number of counts before output changes state
//          percentDutyCycle (0 -> 1.0)
//          pin number (1,2,3,4)
// Outputs: none
int TIMER_A0_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin)
{
	uint16_t dutyCycle;

	// Timer A0.1
	if (pin == 1)
	{
		P2->SEL0 |= BIT4;
		P2->SEL1 &= ~BIT4;
		P2->DIR |= BIT4;
		P2->OUT &= ~BIT4;
	}
	// Timer A0.2
	else if (pin == 2)
	{
		P2->SEL0 |= BIT5;
		P2->SEL1 &= ~BIT5;
		P2->DIR |= BIT5;
		P2->OUT &= ~BIT5;
	}
	// Timer A0.3
	else if (pin == 3)
	{
		P2->SEL0 |= BIT6;
		P2->SEL1 &= ~BIT6;
		P2->DIR |= BIT6;
		P2->OUT &= ~BIT6;
	}
	// Timer A0.4
	else if (pin == 4)
	{
		P2->SEL0 |= BIT7;
		P2->SEL1 &= ~BIT7;
		P2->DIR |= BIT7;
		P2->OUT &= ~BIT7;
	}
	else
		return -2;

	// save the period for this timer instance
	// DEFAULT_PERIOD_A0[pin] where pin is the pin number
	DEFAULT_PERIOD_A0[pin] = period;

	// TIMER_A0->CCR[0]
	TIMER_A0->CCR[0] = period;

	// TIMER_A0->CCTL[pin]
	TIMER_A0->CCTL[pin] = 0x00E0; // 0b11100000 set/reset mode

	// set the duty cycle
	dutyCycle = (uint16_t)(percentDutyCycle * (double)DEFAULT_PERIOD_A0[pin]);

	// CCR[n] contains the dutyCycle just calculated, where n is the pin number
	// TIMER_A0->CCR[pin]
	TIMER_A0->CCR[pin] = dutyCycle;

	// Timer CONTROL register
	// TIMER_A0->CTL
	TIMER_A0->CTL = 0x0214; // SMCLK, MC up mode, TimerA clear 0x0214 = 0b0000001000010100?

	return 0;
}

//***************************PWM_Duty1*******************************
// change duty cycle of PWM output on pin
// Inputs:  dutycycle, pin
// Outputs: none
// percentDutyCycle is a number between 0 and 1  (ie. 0.5 = 50%)
void TIMER_A0_PWM_DutyCycle(double percentDutyCycle, uint16_t pin)
{
	// set the duty cycle
	uint16_t dutyCycle = (uint16_t)(percentDutyCycle * (double)DEFAULT_PERIOD_A0[pin]);

	// CCR[n] contains the dutyCycle just calculated, where n is the pin number
	// TIMER_A0->CCR[pin]
	TIMER_A0->CCR[pin] = dutyCycle;
}

//***************************PWM_Init*******************************
// PWM output on P5.6
// Inputs:  period of P5.6 is number of counts before output changes state
//          percentDutyCycle (0 -> 1.0)//          duty cycle
//          pin number (1,2,3,4), but always 1
// Outputs: none
int TIMER_A2_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin)
{
	uint16_t dutyCycle;

	// NOTE: Timer A2 only exposes 1 PWM pin
	// TimerA2.1
	if (pin == 1)
	{
		P5->SEL0 |= BIT6;
		P5->SEL1 &= ~BIT6;
		P5->DIR |= BIT6;
		P5->OUT &= ~BIT6;
	}
	else
		return -2;

	// NOTE: Setup similar to TimerA0
	// You will have to use the prescaler (clock divider) to get down to 20ms

	// save the period for this timer instance
	// DEFAULT_PERIOD_A2[pin] where pin is the pin number
	DEFAULT_PERIOD_A2[pin] = period;

	// TIMER_A2->CCR[0]
	TIMER_A2->CCR[0] = period;

	// TIMER_A2->CCTL[pin]
	TIMER_A2->CCTL[pin] = 0x00E0; // 0b11100000 set/reset mode

	// set the duty cycle
	dutyCycle = (uint16_t)(percentDutyCycle * (double)DEFAULT_PERIOD_A2[pin]);

	// CCR[n] contains the dutyCycle just calculated, where n is the pin number
	// TIMER_A2->CCR[pin]
	TIMER_A2->CCR[pin] = dutyCycle;

	// Timer CONTROL register
	// TIMER_A2->CTL
	TIMER_A2->CTL = 0x02D4; // SMCLK, MC up mode, TimerA clear 0x0214 = 0b0000001000010100?

	return 0;
}

//***************************PWM_Duty1*******************************
// change duty cycle of PWM output on P5.6
// Inputs:  percentDutyCycle, pin  (should always be 1 for TimerA2.1)
//
// Outputs: none
//
void TIMER_A2_PWM_DutyCycle(double percentDutyCycle, uint16_t pin)
{
	// set the duty cycle
	uint16_t dutyCycle = (uint16_t)(percentDutyCycle * (double)DEFAULT_PERIOD_A2[pin]);

	// CCR[n] contains the dutyCycle just calculated, where n is the pin number
	// TIMER_A2->CCR[pin]
	TIMER_A2->CCR[pin] = dutyCycle;
}

//***************************PWM_Init*******************************
// PWM output through interrupts
// Inputs:  period is number of counts before interrupt is triggered
//          percentDutyCycle (0 -> 1.0)//          duty cycle
//          handler is the function to call on interrupt triggers
// Outputs: none
void TIMER_A2_PWM_Interrupts(uint16_t period, double percentDutyCycle, void (*handler)(void))
{
	uint16_t dutyCycle;
	TimerA2Task = handler;

	// save the period for this timer instance
	// DEFAULT_PERIOD_A2[pin] where pin is the pin number
	DEFAULT_PERIOD_A2[1] = period;

	// TIMER_A2->CCR[0]
	TIMER_A2->CCR[0] = period;

	// TIMER_A2->CCTL[1] (with interrupts)
	TIMER_A2->CCTL[1] = 0x00E0; // 0b11100000 set/reset mode

	// set the duty cycle
	dutyCycle = (uint16_t)(percentDutyCycle * (double)DEFAULT_PERIOD_A2[1]);

	// CCR[n] contains the dutyCycle just calculated, where n is the pin number
	// TIMER_A2->CCR[1]
	TIMER_A2->CCR[1] = dutyCycle;

	// Timer CONTROL register (with interrupts)
	// TIMER_A2->CTL
	TIMER_A2->CTL = 0x02D4; // SMCLK, MC up mode, TimerA clear, interrupt enabled 0x0216 = 0b0000001000010110?

	// priorty 2
	// NVIC->IP[3] = (NVIC->IP[3] & 0xFFFFFF00) | 0x00000040;
	// NVIC register
	// NVIC->ISER[0]
	NVIC->ISER[0] = 0x00002000; // 0x00008000 = 0b00000000000000001000000000000000
	TIMER_A2->CCTL[1] &= ~0x0001;
}

void TA2_N_IRQHandler(void)
{
	if (TIMER_A2->CCTL[1] & TIMER_A_CCTLN_CCIFG)
	{
		TIMER_A2->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;
		(*TimerA2Task)();
	}
}
