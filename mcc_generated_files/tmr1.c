/**
  TMR1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr1.c

  @Summary
    This is the generated driver implementation file for the TMR1 driver using MPLAB? Code Configurator

  @Description
    This source file provides APIs for TMR1.
    Generation Information :
        Product Revision  :  MPLAB? Code Configurator - v2.25.2
        Device            :  PIC18F25K22
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

/**
  Section: Included Files
 */

#include <xc.h>
#include "tmr1.h"
#include "eusart1.h"

/**
  Section: Global Variable Definitions
 */
volatile uint16_t timer1ReloadVal;

/**
  Section: TMR1 APIs
 */

void TMR1_Initialize(void) {
    //Set the Timer to the options selected in the GUI

    //T1CKPS 1:1; TMR1CS FOSC/4; T1OSCEN disabled; T1SYNC synchronize; T1RD16 disabled; TMR1ON disabled; 
    T1CON = 0x00;

    //T1GTM disabled; T1GGO done; T1GPOL low; T1GVAL disabled; TMR1GE disabled; T1GSS T1G; T1GSPM disabled; 
    T1GCON = 0x00;

    //TMR1H 99; 
    TMR1H = 0x63;

    //TMR1L 192; 
    TMR1L = 0xC0;

    // Load the TMR value to reload variable
    timer1ReloadVal = TMR1;

    // Clearing IF flag before enabling the interrupt.
    PIR1bits.TMR1IF = 0;

    // Enabling TMR1 interrupt.
    PIE1bits.TMR1IE = 1;

    // Start TMR1
    TMR1_StartTimer();
}

void TMR1_StartTimer(void) {
    // Start the Timer by writing to TMRxON bit
    T1CONbits.TMR1ON = 1;
}

void TMR1_StopTimer(void) {
    // Stop the Timer by writing to TMRxON bit
    T1CONbits.TMR1ON = 0;
}

uint16_t TMR1_ReadTimer(void) {
    uint16_t readVal;
    uint8_t readValHigh;
    uint8_t readValLow;

    readValLow = TMR1L;
    readValHigh = TMR1H;

    readVal = ((uint16_t) readValHigh << 8) | readValLow;

    return readVal;
}

void TMR1_WriteTimer(uint16_t timerVal) {
    if (T1CONbits.nT1SYNC == 1) {
        // Stop the Timer by writing to TMRxON bit
        T1CONbits.TMR1ON = 0;

        // Write to the Timer1 register
        TMR1H = (timerVal >> 8);
        TMR1L = (uint8_t) timerVal;

        // Start the Timer after writing to the register
        T1CONbits.TMR1ON = 1;
    } else {
        // Write to the Timer1 register
        TMR1H = (timerVal >> 8);
        TMR1L = (uint8_t) timerVal;
    }
}

void TMR1_Reload(void) {
    // Write to the Timer1 register
    TMR1H = (timer1ReloadVal >> 8);
    TMR1L = (uint8_t) timer1ReloadVal;
}

void TMR1_StartSinglePulseAcquisition(void) {
    T1GCONbits.T1GGO = 1;
}

uint8_t TMR1_CheckGateValueStatus(void) {
    return T1GCONbits.T1GVAL;
}

void TMR1_ISR(void) {

    // Clear the TMR1 interrupt flag
    PIR1bits.TMR1IF = 0;

    // Write to the Timer1 register
    TMR1H = (timer1ReloadVal >> 8);
    TMR1L = (uint8_t) timer1ReloadVal;

    // Add your TMR1 interrupt custom code
    USART1_RX_STA|=1<<15;
    TMR1_StopTimer();
}


/**
 End of File
 */
