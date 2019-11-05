// BumpInt.c
// Runs on MSP432, interrupt version
// Provide low-level functions that interface bump switches the robot.
// Daniel Valvano and Jonathan Valvano
// July 2, 2017

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"

void (*Port4Task)(uint8_t);   // user function

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
void BumpInt_Init(void(*task)(uint8_t)){
    // write this as part of Lab 3 Challenge
    Port4Task = task;
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;                 // configure P1.7, 1.6, 1.5, 1.3, 1.2, 1.1 and P1.0 as GPIO
    P4->DIR &= ~0xED;                  // make P1.7, 1.6, 1.5, 1.3, 1.2, 1.1 and P1.0 in (built-in Button 1 and Button 2)
    P4->REN |= 0xED;                   // enable pull resistors on P1.7, 1.6, 1.5, 1.3, 1.2, 1.1 and P1.0
    P4->OUT |= 0xED;                   // P1.7, 1.6, 1.5, 1.3, 1.2, 1.1 and P1.0 are pull-up
    P4->IES |= 0xED;                   // P1.7, 1.6, 1.5, 1.3, 1.2, 1.1 and P1.0 are falling edge event
    P4->IFG &= ~0xED;                  // clear flag4 and flag1 (reduce possibility of extra interrupt)
    P4->IE |= 0xED;                    // arm interrupt on P1.4 and P1.1
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00400000; // priority 2
    NVIC->ISER[1] = 0x00000040;        // enable interrupt 38 in NVIC
}

void BumpInt_Stop()
{
    NVIC->ICER[1] = 0x00000040;     // disable interrupt 38 in NVIC
}

// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){
    // write this as part of Lab 3 Challenge
    uint8_t result;
    result = (P4->IN&0x01)| (P4->IN&0x04)>>1 | (P4->IN&0x08)>>1 | (P4->IN&0x20)>>2|(P4->IN&0x40)>>2 | (P4->IN&0x80)>>2;
    return (result);
}
// we do not care about critical section/race conditions
// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    // write this as part of Lab 3 Challenge
    uint8_t status;
//
    status = P4->IV;
    if (status != 0x0000)
    {
//        (*Port4Task)(Bump_Read());
        switch(Bump_Read())
        {
            //Right to Left Bumper
            case 0x3E:  Motor_Backward(3000,3000);
                        Clock_Delay1ms(1000);
                        Motor_Left(3000,3000);
                        Clock_Delay1ms(500);
                        Motor_Forward(3000,3000);
                        break;

            case 0x3D:  Motor_Backward(3000,3000);
                        Clock_Delay1ms(1000);
                        Motor_Left(3000,3000);
                        Clock_Delay1ms(450);
                        Motor_Forward(3000,3000);
                        break;

            case 0x3B:  Motor_Backward(3000,3000);
                        Clock_Delay1ms(1000);
                        Motor_Left(3000,3000);
                        Clock_Delay1ms(400);
                        Motor_Forward(3000,3000);
                        break;

            case 0x37:  Motor_Backward(3000,3000);
                        Clock_Delay1ms(1000);
                        Motor_Right(3000,3000);
                        Clock_Delay1ms(400);
                        Motor_Forward(3000,3000);
                        break;

            case 0x2F:  Motor_Backward(3000,3000);
                        Clock_Delay1ms(1000);
                        Motor_Right(3000,3000);
                        Clock_Delay1ms(450);
                        Motor_Forward(3000,3000);
                        break;

            case 0x1F:  Motor_Backward(3000,3000);
                        Clock_Delay1ms(1000);
                        Motor_Right(3000,3000);
                        Clock_Delay1ms(500);
                        Motor_Forward(3000,3000);
                        break;
        }

    }
    status = P4->IV;
}




