// RSLK Self Test via UART

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

#include "msp.h"
#include <stdint.h>
#include <string.h>
#include "..\inc\UART0.h"
#include "..\inc\EUSCIA0.h"
#include "..\inc\FIFO0.h"
#include "..\inc\Clock.h"
//#include "..\inc\SysTick.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\TimerA1.h"
//#include "..\inc\Bump.h"
#include "..\inc\BumpInt.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "../inc/IRDistance.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "..\inc\Reflectance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Tachometer.h"

#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))

// At 115200, the bandwidth = 11,520 characters/sec
// 86.8 us/character
// normally one would expect it to take 31*86.8us = 2.6ms to output 31 characters
// Random number generator
// from Numerical Recipes
// by Press et al.
// number from 0 to 31
uint32_t Random(void){
static uint32_t M=1;
  M = 1664525*M+1013904223;
  return(M>>27);
}
char WriteData,ReadData;
uint32_t NumSuccess,NumErrors;
void TestFifo(void){char data;
  while(TxFifo0_Get(&data)==FIFOSUCCESS){
    if(ReadData==data){
      ReadData = (ReadData+1)&0x7F; // 0 to 127 in sequence
      NumSuccess++;
    }else{
      ReadData = data; // restart
      NumErrors++;
    }
  }
}





/*************************************************************************Lab 2*******************************************************************************/
//uint8_t Reflectance_Read(uint32_t time){
//    uint8_t result;
//    // write this as part of Lab 2
//    P5->OUT = 0x08;
//    Port7_Output();
//    Clock_Delay1us(10);
//    Port7_Input();
//    Clock_Delay1us(time);
//    result = P7->IN;
//    P5->OUT = 0x00;
//
//  return result;
//          //((((~(P7->IN))&0x10)>>3)|(((~(P7->IN))&0x02)>>1));
//}
//
//// Perform sensor integration
//// Input: data is 8-bit result from line sensor
//// Output: position in 0.1mm relative to center of line
//int32_t Reflectance_Position(uint8_t data){
//    uint32_t position;
//    // write this as part of Lab 2
//    int32_t w[8] = {332, 237, 142, 47, -47, -142, -237, -332};
//    int32_t bi[8];
//    for (int x = 0; x<8; x++)
//    {
//        bi[x] = ((data >> x)&1);
//    }
//
//    int totalbiW = 0;
//    int totalBi = 0;
//    for (int x =0; x<8; x++)
//    {
//        totalBi = totalBi+bi[x];
//        totalbiW = totalbiW + w[x]*bi[x];
//    }
//
//    position = totalbiW/totalBi;
//    return position;
//}



/*************************************************************************Lab 3*******************************************************************************/
//Bump interrupt movement
void MotorMovt(void){
//    static uint32_t count=0;
//    static uint8_t motor_state=0;

    //Write this as part of lab3 Bonus Challenge
    Motor_Forward(5000, 5000);
}

// Driver test
void TimedPause(uint32_t time){
  Clock_Delay1ms(time);          // run for a while and stop
  Motor_Stop();
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

uint8_t CollisionData, CollisionFlag;  // mailbox
void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   CollisionFlag = 1;
   P4->IFG &= ~0xED;                  // clear interrupt flags (reduce possibility of extra interrupt)
}


//Motor_Forward(3000,3000);  // your function
//TimedPause(1000);
//Motor_Backward(3000,3000); // your function
//TimedPause(1000);
//Motor_Left(3000,3000);     // your function
//TimedPause(1000);
//Motor_Right(3000,3000);    // your function
//TimedPause(1000);

void Bumper_Init(){
    CollisionFlag = 0;
    BumpInt_Init(&HandleCollision);
    TimerA1_Init(&MotorMovt,50000);  // 10 Hz
}

/*************************************************************************Lab 4*******************************************************************************/
//**************Program 15.1*******************
volatile uint32_t ADCvalue;
volatile uint32_t ADCflag;

//**************Program 15.2*******************
volatile uint32_t nr,nc,nl;
void SensorRead_ISR(void){  // runs at 2000 Hz
  uint32_t raw17,raw12,raw16;
  P1OUT ^= 0x01;         // profile
  P1OUT ^= 0x01;         // profile
  ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
  nr = LPF_Calc(raw17);  // right is channel 17 P9.0
  nc = LPF_Calc2(raw12); // center is channel 12, P4.1
  nl = LPF_Calc3(raw16); // left is channel 16, P9.1
  ADCflag = 1;           // semaphore
  P1OUT ^= 0x01;         // profile
}

void IRSensor_Init(){
    uint32_t raw17,raw12,raw16;
    uint32_t s;
    ADCflag = 0;
    s = 256; // replace with your choice
    ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
    ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
    LPF_Init(raw17,s);     // P9.0/channel 17
    LPF_Init2(raw12,s);    // P4.1/channel 12
    LPF_Init3(raw16,s);    // P9.1/channel 16
    UART0_Init();          // initialize UART0 115,200 baud rate
    TimerA1_Init(&SensorRead_ISR,250);    // 2000 Hz sampling
}


/***********************************************************EXTRA*****************************************************/
void DecimalToBinary(int n)
{
    // array to store binary number
        int binaryNum[32];


        // counter for binary array
        int i = 0;
        while (n > 0) {

            // storing remainder in binary array
            binaryNum[i] = n % 2;
            n = n / 2;
            i++;
        }

        for (int k = 8 - i; k >0; k--)
        {
            UART0_OutUDec5(0);
        }
        // printing binary array in reverse order
        for (int j = i - 1; j >= 0; j--)
        {
            UART0_OutUDec5(binaryNum[j]);
        }
        UART0_OutString("\r\n");

}




uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0;               // Timer A3 first edge, P10.4
int Done0;                     // set each rising
// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure0(uint16_t time){
  P2_0 = P2_0^0x01;           // thread profile, P2.0
  Period0 = (time - First0)&0xFFFF; // 16 bits, 83.3 ns resolution
  First0 = time;                   // setup for next
  Done0 = 1;
}
uint16_t Period2;              // (1/SMCLK) units = 83.3 ns units
uint16_t First2;               // Timer A3 first edge, P8.2
int Done2;                     // set each rising
// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure2(uint16_t time){
  P2_2 = P2_2^0x01;           // thread profile, P2.4
  Period2 = (time - First2)&0xFFFF; // 16 bits, 83.3 ns resolution
  First2 = time;                   // setup for next
  Done2 = 1;
}

#define PERIOD 1000  // must be even

void toggle_GPIO(void){
    P2_4 ^= 0x01;     // create output
}


uint32_t main_count=0;

void Tachometer_Init(){
    //P2->SEL0 &= ~0x15;
    //P2->SEL1 &= ~0x15;   // configure P2.0 and P2.2 as GPIO
    //P2->DIR |= 0x15;     // P2.0 and P2.2 outputs
    P2->SEL0 &= ~0x11;
    P2->SEL1 &= ~0x11;  // configure P2.0 and P2.4 as GPIO
    P2->DIR |= 0x11;    // P2.0 and P2.4 outputs
    First0 = First2 = 0; // first will be wrong
    Done0 = Done2 = 0;   // set on subsequent
}



uint32_t Size;
int Program5_1(void){
//int main(void){
    // test of TxFifo0, NumErrors should be zero
  uint32_t i;
  Clock_Init48MHz();
  WriteData = ReadData = 0;
  NumSuccess = NumErrors = 0;
  TxFifo0_Init();
  TimerA1_Init(&TestFifo,43);  // 83us, = 12kHz
  EnableInterrupts();
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      TxFifo0_Put(WriteData);
      WriteData = (WriteData+1)&0x7F; // 0 to 127 in sequence
    }
    Clock_Delay1ms(10);
  }
}

char String[64];
uint32_t MaxTime,First,Elapsed;
int Program5_2(void){
//int main(void){
    // measurement of busy-wait version of OutString
  uint32_t i;
  DisableInterrupts();
  Clock_Init48MHz();
  UART0_Init();
  WriteData = 'a';
  SysTick_Init(0x1000000,2); //OHL - using systick INT api
  MaxTime = 0;
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      String[i] = WriteData;
      WriteData++;
      if(WriteData == 'z') WriteData = 'a';
    }
    String[i] = 0; // null termination
    First = SysTick->VAL;
    UART0_OutString(String);
    Elapsed = ((First - SysTick->VAL)&0xFFFFFF)/48; // usec

    if(Elapsed > MaxTime){
        MaxTime = Elapsed;
    }
    UART0_OutChar(CR);UART0_OutChar(LF);
    Clock_Delay1ms(100);
  }
}

int Program5_3(void){
//int main(void){
    // measurement of interrupt-driven version of OutString
  uint32_t i;
  DisableInterrupts();
  Clock_Init48MHz();
  EUSCIA0_Init();
  WriteData = 'a';
  SysTick_Init(0x1000000,2); //OHL - using systick INT api
  MaxTime = 0;
  EnableInterrupts();
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      String[i] = WriteData;
      WriteData++;
      if(WriteData == 'z') WriteData = 'a';
    }
    String[i] = 0; // null termination
    First = SysTick->VAL;
    EUSCIA0_OutString(String);
    Elapsed = ((First - SysTick->VAL)&0xFFFFFF)/48; // usec
    if(Elapsed > MaxTime){
        MaxTime = Elapsed;
    }
    EUSCIA0_OutChar(CR);EUSCIA0_OutChar(LF);
    Clock_Delay1ms(100);
  }
}
int Program5_4(void){
//int main(void){
    // demonstrates features of the EUSCIA0 driver
  char ch;
  char string[20];
  uint32_t n;
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();
  EUSCIA0_OutString("\nLab 5 Test program for EUSCIA0 driver\n\rEUSCIA0_OutChar examples\n");
  for(ch='A'; ch<='Z'; ch=ch+1){// print the uppercase alphabet
     EUSCIA0_OutChar(ch);
  }
  EUSCIA0_OutChar(LF);
  for(ch='a'; ch<='z'; ch=ch+1){// print the lowercase alphabet
    EUSCIA0_OutChar(ch);
  }
  while(1){
    EUSCIA0_OutString("\n\rInString: ");
    EUSCIA0_InString(string,19); // user enters a string
    EUSCIA0_OutString(" OutString="); EUSCIA0_OutString(string); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUDec: ");   n=EUSCIA0_InUDec();
    EUSCIA0_OutString(" OutUDec=");  EUSCIA0_OutUDec(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix1="); EUSCIA0_OutUFix1(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix2="); EUSCIA0_OutUFix2(n); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUHex: ");   n=EUSCIA0_InUHex();
    EUSCIA0_OutString(" OutUHex=");  EUSCIA0_OutUHex(n); EUSCIA0_OutChar(LF);
  }
}

void RSLK_Reset(void){
    DisableInterrupts();
    LaunchPad_Init();
    //Initialise modules used e.g. Reflectance Sensor, Bump Switch, Motor, Tachometer etc
    // ... ...
    Motor_Init();
    Motor_Stop();
    IRSensor_Init();
    Reflectance_Init();
    Tachometer_Init();

    EnableInterrupts();
}


// RSLK Self-Test
int main(void) {
  uint32_t cmd=0xDEAD, menu=0;
  uint8_t Data; // QTR-8RC
  int32_t Position; // 332 is right, and -332 is left of center
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  //SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
  LaunchPad_Init();
  Motor_Init();
  Motor_Stop();
  //Bump_Init();
  //Bumper_Init();
  IRSensor_Init();
  Tachometer_Init();
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();

  while(1){                     // Loop forever
      // write this as part of Lab 5
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("RSLK Testing"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[0] RSLK Reset"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[1] Motor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[2] IR Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[3] Bumper Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[4] Reflectance Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[5] Tachometer Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[6] Sensor Avoidance Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      EUSCIA0_OutString("CMD: ");
      cmd=EUSCIA0_InUDec();
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      switch(cmd){
          case 0:
              RSLK_Reset();
              UART0_OutString("Device Reset\r\n");
              menu =1;
              cmd=0xDEAD;
              break;

          case 1: //Lab3
                  Motor_Forward(3000,3000);  // your function
                  TimedPause(1000);
                  Motor_Backward(3000,3000); // your function
                  TimedPause(1000);
                  Motor_Left(3000,3000);     // your function
                  TimedPause(1000);
                  Motor_Right(3000,3000);    // your function
                  TimedPause(1000);
                  Motor_Stop();
                  break;

          case 2: //Lab4
                  while(LaunchPad_Input()==0){
                      for(int32_t n=0; n<2000; n++){
                          while(ADCflag == 0){};
                          ADCflag = 0; // show every 2000th point
                      }
                      UART0_OutUDec5(LeftConvert(nl));UART0_OutString(" mm,");
                      UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" mm,");
                      UART0_OutUDec5(RightConvert(nr));UART0_OutString(" mm\r\n");
                  }
                  break;

          case 3: //Lab3
                  Bumper_Init();
                  while(LaunchPad_Input()==0);  // wait for touch
                  while(LaunchPad_Input());     // wait for release
                  TimerA1_Stop();
                  BumpInt_Stop();
                  Motor_Stop();
                  break;

          case 4: //Lab2
                  while(!(LaunchPad_Input()!=0)){
                      Data = Reflectance_Read(1000);
                      Clock_Delay1ms(10);
                      DecimalToBinary(Data);
                  }
                  break;

          case 5: //Lab4
                  int value= 0;
                  int motorSpeed;


                  while(value<=0 || value > 4)
                  {
                        UART0_OutString("Duty Cycle %?\r\n");
                        UART0_OutString("1. 25%\r\n");
                        UART0_OutString("2. 50%?\r\n");
                        UART0_OutString("3. 75%?\r\n");
                        UART0_OutString("4. 100%?\r\n");
                        value = EUSCIA0_InUDec();

                        switch(value)
                        {
                            case 1: motorSpeed = 1500; break;
                            case 2: motorSpeed = 3000; break;
                            case 3: motorSpeed = 4500; break;
                            case 4: motorSpeed = 6000; break;
                            default: UART0_OutString("Invalid Option. Please choose in again...\r\n"); break;
                        }
                  }

                  TimerA1_Init(&toggle_GPIO,10);    // 50Khz sampling
                  TimerA3Capture_Init(&PeriodMeasure0,&PeriodMeasure2);

                  while(!(LaunchPad_Input()!=0)){
                    Motor_Forward(motorSpeed,motorSpeed);
                    WaitForInterrupt();
                    main_count++;
                    if(main_count%1000){
                        UART0_OutString("Period0 = ");UART0_OutUDec5(Period0);UART0_OutString(" Period2 = ");UART0_OutUDec5(Period2);UART0_OutString(" \r\n");
                    }
                  }
                  Motor_Stop();
                  break;

          case 6 : while(!(LaunchPad_Input()!=0))
                   {
                       if(LeftConvert(nl) >=70 && CenterConvert(nc) >=70 && RightConvert(nr) >=70)
                       {
                           Motor_Forward(3000,3000);
                           Clock_Delay1ms(10);
                       }
                       else
                       {
                           if(LeftConvert(nl) < 70)
                           {
                               Motor_Right(3000,3000);
                               Clock_Delay1ms(1000);
                           }
                           else if(RightConvert(nr) < 70)
                           {
                               Motor_Left(3000,3000);
                               Clock_Delay1ms(1000);
                           }
                           else
                           {
                               Motor_Backward(3000,3000);
                               Clock_Delay1ms(500);
                               Motor_Left(3000,3000);
                               Clock_Delay1ms(1000);
                           }
                       }
                   }
                   Motor_Stop();
                   break;

          default: menu=1;
                   break;
      }
      if(!menu)Clock_Delay1ms(1000);
      else{
          menu=0;
      }

      // ....
      // ....
  }
}
