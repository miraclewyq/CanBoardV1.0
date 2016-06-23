/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB? Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
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

#include "mcc_generated_files/mcc.h"
#include "UserApp/ECAN.h"

uint16_t PM25 = 0;
/* ��ʱ���� */
void delay(uint16_t x)
{
	uint16_t a,b;
	for(a=x;a>0;a--)
		for(b=110;b>0;b--);
}
/**********************************************************************
* ������: unsigned char FucCheckSum(uchar *i,ucharln)
* ��������:���У�飨ȡ���͡�����Э���1\2\3\4\5\6\7�ĺ�ȡ��+1��
* ����˵��:�������Ԫ��1-�����ڶ���Ԫ����Ӻ�ȡ��+1��Ԫ�ظ����������2��
********************************************************************/
unsigned char FucCheckSum(unsigned char *i,unsigned char ln)
{
    unsigned char j,tempq=0;
    i+=1;
    for(j=0;j<(ln-2);j++)
    {
        tempq+=*i;
        i++;
    }
    tempq=(~tempq)+1;
    return(tempq);
}
/*
                         Main application
 */

void main(void) {
    // Initialize the device
    SYSTEM_Initialize();
    InitECAN();
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    while (1) {
        // Add your application code
        if(USART1_RX_STA&0X8000)
        {
            USART1_RX_STA = 0;
            //LED��˸��ʾ���ܵ�һ֡����
            IO_RC3_LED8_PORT = 1;
            IO_RC2_LED7_PORT = 1;
            delay(200);
            IO_RC3_LED8_PORT = 0;
            IO_RC2_LED7_PORT = 0;
            //���ݴ���
            if(USART1_RX_BUF[0] == 0x42)    //�����ϴ�PM2.5����
            {
                uint16_t CheckSum = 0x00;
                uint16_t temp;
                for(unsigned char i=0;i<22;i++)
                {
                    CheckSum += USART1_RX_BUF[i];
                }
                temp = (uint16_t)USART1_RX_BUF[22]<<8 + USART1_RX_BUF[23];
                if(CheckSum == temp)        //���ܵ������ݺ�У��ֵ�ȶ���ȷ
                {
                    PM25 = (uint16_t)USART1_RX_BUF[12]<<8 + USART1_RX_BUF[13];
                }
            }
        }
        //CAN code part
        if(ECAN_Receive())
        {
            ECAN_Transmit();
        }
    }
}
/**
 End of File
 */