// car.c
// Autonomous car code
// Mohammed Fareed and Trent Wesley
// December 10, 2023
#include <stdio.h>
#include <stdlib.h>

#include "msp.h"
#include "uart.h"
#include "Timer32.h"
#include "CortexM.h"
#include "Common.h"
#include "ADC14.h"
#include "ControlPins.h"
#include "TimerA.h"

// motors pins
#define LEFT_MOTOR_FORWARD 2
#define LEFT_MOTOR_BACKWARD 1
#define RIGHT_MOTOR_FORWARD 4
#define RIGHT_MOTOR_BACKWARD 3

// straight path detection parameters
#define STRAIGHT_PATH_THRESHOLD (0.8)
#define STRAIGHT_PATH_SPEED_FACTOR (1.0)
// curved path detection parameters
#define CURVED_PATH_THRESHOLD (2)
#define CURVED_PATH_SPEED_FACTOR (1.0)

// PID parameters
#define IntegralErrorHistoryLength (3)
#define ERROR_HISTORY_LENGTH (10)

// camera variables
uint16_t line[128];
BOOLEAN g_sendData;

// PID variables
int integralErrorHistoryIndex = 0;
int integralErrorHistory[IntegralErrorHistoryLength];
double previousError = 0;
double previousError2 = 0;
double errorHistory[ERROR_HISTORY_LENGTH];
int errorHistoryIndex = 0;

// Waits for a delay (in milliseconds)
void delay_ms(int del)
{
    volatile int i;
    for (i = 0; i < del * 2224; i++)
    {
        ; // Do nothing
    }
}

// Initialize the line scan camera
void INIT_Camera(void)
{
    g_sendData = FALSE;      // initialize flag to no new data
    ControlPin_SI_Init();    // initialize SI signal
    ControlPin_CLK_Init();   // initialize CLK signal
    ADC0_InitSWTriggerCh6(); // initialize ADC0 for line scan camera
}

// Initialize the motors and servo
void INIT_Motors(void)
{
    // configure motor 1 enable pin
    P3->SEL0 &= ~BIT6;
    P3->SEL1 &= ~BIT6;
    P3->DIR |= BIT6;
    P3->OUT &= ~BIT6;
    // configure motor 2 enable pin
    P3->SEL0 &= ~BIT7;
    P3->SEL1 &= ~BIT7;
    P3->DIR |= BIT7;
    P3->OUT &= ~BIT7;

    // initialize PWM for motors and servo
    TIMER_A0_PWM_Init(2400, 0.0, 1);
    TIMER_A0_PWM_Init(2400, 0.0, 2);
    TIMER_A0_PWM_Init(2400, 0.0, 3);
    TIMER_A0_PWM_Init(2400, 0.0, 4);
    TIMER_A2_PWM_Init(60000, 0.075, 1);

    // configure motor 1 direction pins
    delay_ms(250);
    P3->OUT |= BIT6;
    P3->OUT |= BIT7;
}

// Initialize the board's switches
void Switch_Init(void)
{
    // configure switch 1 pins as GPIO
    P1->SEL0 &= ~BIT4;
    P1->SEL1 &= ~BIT4;
    // configure as input
    P1->DIR &= ~BIT4;
    P1->REN |= BIT4;
    P1->OUT |= BIT4;

    // configure switch 2 pins as GPIO
    P1->SEL0 &= ~BIT1;
    P1->SEL1 &= ~BIT1;
    // configure as input
    P1->DIR &= ~BIT1;
    P1->REN |= BIT1;
    P1->OUT |= BIT1;
}

// Check if a switch is pressed
int Switch_Pressed(void)
{
    if (P1->IN & BIT4 && P1->IN & BIT1)
    { // if nothing pressed
        return 0;
    }
    else if (P1->IN & BIT4)
    { // if switch 1.1 pressed
        return 2;
    }
    else
    { // if switch 1.4 pressed
        return 1;
    }
}

// Update the error history array
void updateErrorHistory(double error)
{
    errorHistory[errorHistoryIndex] = error;
    errorHistoryIndex = (errorHistoryIndex + 1) % ERROR_HISTORY_LENGTH;
}

int main(void)
{
    // initialize integral error history array
    for (int i = 0; i <= IntegralErrorHistoryLength; i++)
    {
        integralErrorHistory[i] = 0;
    }
    int i = 0;

    // wait for a switch to be pressed
    int switchVal = 0;
    while (1)
    {
        if (Switch_Pressed() == 1)
        {
            switchVal = 1;
            break;
        }
        else if (Switch_Pressed() == 2)
        {
            switchVal = 2;
            break;
        }
    }

    // initializations
    DisableInterrupts();
    uart0_init();
    Switch_Init();
    INIT_Motors();
    INIT_Camera();
    EnableInterrupts();

    // pid parameters
    double kp = -0.0048;
    double ki = 0.0017;
    double kd = 0.0011;

    while (1)
    {
        // process new camera data
        if (g_sendData == TRUE)
        {
            double sum = 0.0;          // sum of all pixels
            double totalLight = 0.0;   // total light in the image
            double weighted_sum = 0.0; // weighted sum of all pixels
            double integral = 0;       // integral term

            // calculate the sum, weighted sum, and total light
            for (i = 0; i < 128; i++)
            {
                totalLight += line[i];
                if (line[i] > 16380)
                {
                    sum += 1.0;
                    weighted_sum += i;
                }
            }

            // calculate the midpoint, error, and integral error
            double midpoint = weighted_sum / sum;
            double error = midpoint - 64.5;
            integralErrorHistory[integralErrorHistoryIndex] = error;
            integralErrorHistoryIndex = (integralErrorHistoryIndex + 1) % IntegralErrorHistoryLength;
            updateErrorHistory(error);

            // calculate the derivative term
            double derivative = (error - 2 * previousError + previousError2);
            previousError2 = previousError;
            previousError = error;

            // calculate the integral term
            for (i = 0; i <= IntegralErrorHistoryLength; i++)
            {
                integral += integralErrorHistory[i];
            }
            integral = integral / IntegralErrorHistoryLength;

            // calculate the servo value
            double servoVal;
            servoVal = kp * error + 0.075;               // add P term
            servoVal += ki * integral + kd * derivative; // add PID terms
            // set min and max servo values
            if (servoVal < 0.05)
                servoVal = 0.05;
            else if (servoVal > 0.1)
                servoVal = 0.1;
            // set servo value
            TIMER_A2_PWM_DutyCycle(servoVal, 1);

            // add light cutoff to stop the car on the finish line
            if (totalLight < 1750000)
            {
                TIMER_A0_PWM_DutyCycle(0.0, LEFT_MOTOR_FORWARD);
                TIMER_A0_PWM_DutyCycle(0.0, RIGHT_MOTOR_FORWARD);
            }

            // use PID to control the motors if switch is pressed
            else if (switchVal == 1)
            {
                // center servo value around 0
                if (servoVal >= 0.075)
                    servoVal -= 0.075;
                else
                    servoVal = 0.075 - servoVal;

                // calculate motor speed
                double motorSpeed = -18.0 * servoVal + 0.434;
                if (motorSpeed < 0.36)
                    motorSpeed = 0.36;

                // calculate error history sum
                double sum = 0.0;
                for (int i = 0; i < ERROR_HISTORY_LENGTH; i++)
                {
                    if (errorHistory[i] < 0)
                        errorHistory[i] *= -1;
                    sum += errorHistory[i];
                }

                // adjust motor speed based on error history
                if ((sum / ERROR_HISTORY_LENGTH) < STRAIGHT_PATH_THRESHOLD)
                {
                    motorSpeed *= STRAIGHT_PATH_SPEED_FACTOR; // speed up
                }
                else if ((sum / ERROR_HISTORY_LENGTH) > CURVED_PATH_THRESHOLD)
                {
                    motorSpeed *= CURVED_PATH_SPEED_FACTOR; // slow down
                }

                // set motor speed
                TIMER_A0_PWM_DutyCycle(motorSpeed, LEFT_MOTOR_FORWARD);
                TIMER_A0_PWM_DutyCycle(motorSpeed, RIGHT_MOTOR_FORWARD);
            }

            else
            {
                // backup mode with no PID
                TIMER_A0_PWM_DutyCycle(0.36, LEFT_MOTOR_FORWARD);
                TIMER_A0_PWM_DutyCycle(0.36, RIGHT_MOTOR_FORWARD);
            }
            g_sendData = FALSE; // reset flag
        }
    }
}
