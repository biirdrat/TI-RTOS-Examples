/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/PWM.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "uartstdio.h"

extern ti_sysbios_knl_Semaphore_Handle Task0Semaphore;
extern ti_sysbios_knl_Semaphore_Handle Task1Semaphore;
volatile uint64_t startCount;
volatile uint64_t endCount;
volatile uint32_t periodPWMCount;
volatile uint32_t ADC0Buffer[4];

void EnablePeripherals();
void InitializeUART0LocalTerminal();
void InitializeUART1Bluetooth();
void InitializeWTimer1();
void InitializeWTimer2();
void InitializeGPIOAInterrupt();
void InitializeADC0();
void AdjustPWMDutyCycle(uint8_t adjustPercentage);
void Forward();
void SlowForward();
void Brake();
void Reverse();

void UART1IntHandler()
{
    UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, true));
    char receivedChar;
    if (UARTCharsAvail(UART1_BASE))
    {
        receivedChar = UARTCharGet(UART1_BASE);
    }
    UARTCharPut(UART1_BASE, receivedChar);
    if (receivedChar == 'f')
    {
        Forward();
    }
    else if (receivedChar == 's')
    {
        SlowForward();
    }
    else if (receivedChar == 'b')
    {
        Brake();
    }
    else if (receivedChar == 'r')
    {
        Reverse();
    }
}

void WTimer1IntHandler()
{
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    Semaphore_post(Task0Semaphore);
}

void WTimer2IntHandler()
{
    TimerIntClear(WTIMER2_BASE, TIMER_TIMA_TIMEOUT);
    Semaphore_post(Task1Semaphore);
}

void GPIOAIntHandler()
{
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_4);
    endCount = TimerValueGet64(WTIMER0_BASE);

    uint64_t elapsedCount = endCount - startCount;
    if (elapsedCount > 40000)
    {
        UARTprintf("On a dark surface\n");
    }
    else
    {
        UARTprintf("On a white surface\n");
    }
}

void Task0Func()
{
    while(1)
    {
        Semaphore_pend(Task0Semaphore, BIOS_WAIT_FOREVER);
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        SysCtlDelay(SysCtlClockGet()/3 * 10 * 1.0/1000000);
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);
        startCount = TimerValueGet64(WTIMER0_BASE);
    }
}

void Task1Func()
{
    while(1)
    {
        Semaphore_pend(Task1Semaphore, BIOS_WAIT_FOREVER);
        ADCIntClear(ADC0_BASE, 1);
        ADCProcessorTrigger(ADC0_BASE, 1);
        while(!ADCIntStatus(ADC0_BASE, 1, false))
        {
        }
        ADCSequenceDataGet(ADC0_BASE, 1, (uint32_t*)ADC0Buffer);
        UARTprintf("ADC Val: %i Voltage mV: %i\n", ADC0Buffer[0], (int)(ADC0Buffer[0] * 3.3/4095 * 1000));
    }
}

void EnablePeripherals()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
}

void InitializeUART0LocalTerminal()
{
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    UARTEnable(UART0_BASE);

    UARTStdioConfig(0, 115200, SysCtlClockGet());
}

void InitializeUART1Bluetooth()
{
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    UARTEnable(UART1_BASE);
}

void InitializeWTimer0()
{
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_ONE_SHOT_UP);
    TimerLoadSet64(WTIMER0_BASE, 18446744073709551615);
    TimerEnable(WTIMER0_BASE, TIMER_A);
}

void InitializeWTimer1()
{
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);
    TimerIntEnable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_WTIMER1A);
    TimerLoadSet(WTIMER1_BASE, TIMER_A, SysCtlClockGet() * 0.5);
    TimerEnable(WTIMER1_BASE, TIMER_A);
}

void InitializeWTimer2()
{
    TimerConfigure(WTIMER2_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);
    TimerIntEnable(WTIMER2_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_WTIMER2A);
    TimerLoadSet(WTIMER2_BASE, TIMER_A, SysCtlClockGet() * 0.3);
    TimerEnable(WTIMER2_BASE, TIMER_A);
}

void InitializeGPIOAInterrupt()
{
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_4);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
}

void InitializePWM0MotorControl()
{
    // Sys Clock = 50Mhz PWMClock = 50/16 = 2.5MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    uint32_t PWMClock = SysCtlClockGet() / 16;
    // Desired frequency = 10Khz
    uint32_t desiredFrequency = 10000;

    // 2.5MHz / 10Khz = 250 clock ticks
    periodPWMCount = PWMClock/desiredFrequency;

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, periodPWMCount);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);

    // Setup PWM pin
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);

    // Setup phase pin
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);

    // Enable generator and output
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
}

void InitializeGPIOFLEDs()
{
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

void AdjustPWMDutyCycle(uint8_t adjustPercentage)
{
    float adjustDecimal = (float)adjustPercentage/100;
    uint32_t newPWMPeriod = (1-adjustDecimal)*periodPWMCount;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, newPWMPeriod);
}

void InitializeADC0()
{
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH10 | ADC_CTL_END | ADC_CTL_IE);
    GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);
    ADCSequenceEnable(ADC0_BASE, 1);
}

void Forward()
{
    AdjustPWMDutyCycle(0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
}

void SlowForward()
{
    AdjustPWMDutyCycle(50);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
}

void Brake()
{
    AdjustPWMDutyCycle(100);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
}

void Reverse()
{
    AdjustPWMDutyCycle(0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
}


/*
 *  ======== main ========
 */
int main(void)
{
    // Running at 40 Mhz for main clock
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    IntMasterEnable();

    EnablePeripherals();
    InitializeUART0LocalTerminal();
    InitializeUART1Bluetooth();
    InitializeWTimer0();
//    InitializeWTimer1();
    InitializeWTimer2();
    InitializeGPIOAInterrupt();
    InitializePWM0MotorControl();
    InitializeGPIOFLEDs();
    InitializeADC0();
    UARTprintf("Main Program is running!\n");

    /* Start BIOS */
    BIOS_start();

    return (0);
}
