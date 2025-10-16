/**
  ECCP3 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    eccp3.c

  @Summary
    This is the generated driver implementation file for the ECCP3 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This source file provides implementations for driver APIs for ECCP3.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC16F1936
        Driver Version    :  2.02
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.36 and above
         MPLAB 	          :  MPLAB X 6.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

/**
  Section: Included Files
*/

#include <xc.h>
#include "eccp3.h"

/**
  Section: Capture Module APIs:
*/
extern void ECCP3_CallBack(uint16_t capturedValue);

void ECCP3_Initialize(void)
{
    // Set the ECCP3 to the options selected in the User Interface
	
	// CCP3M Every rising edge; DC3B 0; P3M single; 
	CCP3CON = 0x05;    
	
	// CCPR3H 0; 
	CCPR3H = 0x00;    
	
	// CCPR3L 0; 
	CCPR3L = 0x00;    
    
    // Clear the ECCP3 interrupt flag
    PIR3bits.CCP3IF = 0;

    // Enable the ECCP3 interrupt
    PIE3bits.CCP3IE = 1;
	
}

void ECCP3_CaptureISR(void)
{
    CCP3_PERIOD_REG_T module;

    // Clear the ECCP3 interrupt flag
    PIR3bits.CCP3IF = 0;
    
    // Copy captured value.
    module.ccpr3l = CCPR3L;
    module.ccpr3h = CCPR3H;
    
    // Return 16bit captured value
    ECCP3_CallBack(module.ccpr3_16Bit);
}

// void ECCP3_CallBack(uint16_t capturedValue)
// {
//     // Add your code here
// }
/**
 End of File
*/