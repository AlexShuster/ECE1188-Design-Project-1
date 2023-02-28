// LineFollowFSMmain.c
// Runs on MSP432
// Simple line follower with 2 inputs and 2 outputs.
// Rather than real sensors and motors, it uses LaunchPad I/O
// Daniel and Jonathan Valvano
// January 24, 2018

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

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
#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/BumpInt.h"
#include "../inc/TExaS.h"
#include "../inc/TimerA1.h"
#include "../inc/Reflectance.h"

/*(Left,Right) Motors, call LaunchPad_Output (positive logic)
3   1,1     both motors, yellow means go straight
2   1,0     left motor,  green  means turns right
1   0,1     right motor, red    means turn left
0   0,0     both off,    dark   means stop
(Left,Right) Sensors, call LaunchPad_Input (positive logic)
3   1,1     both buttons pushed means on line,
2   1,0     SW2 pushed          means off to right
1   0,1     SW1 pushed          means off to left
0   0,0     neither button      means lost
 */

// Wait for Button Press
void Pause(void)
{
    while(LaunchPad_Input() == 0);
    while(LaunchPad_Input());
}

// Linked data structure
struct State {
  uint8_t  out;
  uint32_t leftDuty;                // 2-bit output
  uint32_t rightDuty;
  char outString;
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[16]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center &fsm[0]
#define Left   &fsm[1]
#define Right  &fsm[2]
#define HeavyRight &fsm[3]
#define HeavyLeft &fsm[4]
#define PreviousLeftLost &fsm[5]
#define PreviousRightLost &fsm[6]
#define CenterLeft &fsm[7]
#define CenterRight &fsm[8]


// Color    LED(s) Port2
// dark     ---    0
// red      R--    0x01
// blue     --B    0x04
// green    -G-    0x02
// yellow   RG-    0x03
// sky blue -GB    0x06
// white    RGB    0x07
// pink     R-B    0x05

State_t fsm[9]={
//  {0x00, 2700, 2700, 'C', 2000, { Center, HeavyRight, Right, Center, Left, Center, Center, Center, HeavyLeft, Center, Center, Center, HeavyLeft, Center, Center, Center  }},  // Center
//  {0x01, 2400, 1300, 'l', 1000, { PreviousLeftLost, HeavyRight, Right, Right, Left, Left, CenterLeft, Left, HeavyLeft, Left, Left, Left, HeavyLeft, Left, Left, Left}},  // Left
//  {0x02, 2500, 1200, 'L', 700, { PreviousLeftLost, HeavyRight, Right, HeavyRight, Left, HeavyRight, Left, Left, HeavyLeft, HeavyLeft, Left, Left, HeavyLeft, HeavyLeft, Left, HeavyLeft }}, // Heavy Left
//  {0x03, 1300, 2400, 'r', 1000, { PreviousRightLost, HeavyRight, Right, HeavyRight, Left, Right, CenterRight, Right, HeavyLeft, Right, Right, Right, HeavyLeft, Right, Right, Right}},   // Right
//  {0x04, 1200, 2500, 'R', 700, { PreviousRightLost, HeavyRight, Right, HeavyRight, Left, HeavyRight, Left, HeavyRight, HeavyLeft, HeavyRight, HeavyRight, HeavyRight, HeavyLeft, HeavyRight, HeavyRight, HeavyRight}}, // Heavy Right
//  {0x05, 2700, 10,    'p', 700, { PreviousLeftLost, HeavyLeft, Left, HeavyRight, Left, PreviousLeftLost, Left, PreviousLeftLost, HeavyLeft, PreviousLeftLost, PreviousLeftLost, PreviousLeftLost, HeavyLeft, PreviousLeftLost, PreviousLeftLost, PreviousRightLost }}, // Previous Left Lost
//  {0x06, 10, 2700,    'P', 700, { PreviousRightLost, HeavyRight, Right, HeavyRight, Left, PreviousRightLost, Right, PreviousRightLost, HeavyLeft, PreviousRightLost, PreviousRightLost, PreviousRightLost, HeavyLeft, PreviousRightLost, PreviousRightLost, PreviousLeftLost }},
//  {0x07, 1675, 2700, 'd', 1000, { Center, HeavyRight, Right, Right, Left, Center, Center, Center, HeavyLeft, Center, Center, Center, Left, Center, Center, Center}},
//  {0x07, 2700, 1675, 'D', 1000, { Center, HeavyRight, Right, Right, Left, Center, Center, Center, HeavyLeft, Center, Center, Center, Left, Center, Center, Center}}
    {0x00   , 7000, 7000, 'C', 2000, { Center, HeavyRight, Right, HeavyRight, Left, Center, Center, HeavyRight, HeavyLeft, Center, Center, Center, HeavyLeft, Center, Center, Center  }},  // Center
    {0x01   , 4000, 3000, 'C', 2000, { Center, HeavyRight, Right, HeavyRight, Left, Center, Center, HeavyRight, HeavyLeft, Center, Center, Center, HeavyLeft, Center, Center, Center  }},  // Center
    {0x02   , 5000, 2000, 'C', 2000, { Center, HeavyRight, Right, HeavyRight, Left, Center, Center, HeavyRight, HeavyLeft, Center, Center, Center, HeavyLeft, Center, Center, Center  }},  // Center
    {0x03   , 3000, 4000, 'C', 2000, { Center, HeavyRight, Right, HeavyRight, Left, Center, Center, HeavyRight, HeavyLeft, Center, Center, Center, HeavyLeft, Center, Center, Center  }},  // Center
    {0x04   , 2000, 5000, 'C', 2000, { Center, HeavyRight, Right, HeavyRight, Left, Center, Center, HeavyRight, HeavyLeft, Center, Center, Center, HeavyLeft, Center, Center, Center  }},  // Center
    {0x05   , 7000, 0, 'C', 2000, { Center, HeavyRight, Right, HeavyRight, Left, Center, Center, HeavyRight, HeavyLeft, Center, Center, Center, HeavyLeft, Center, Center, Center  }},  // Center
    {0x06   , 0, 7000, 'C', 2000, { Center, HeavyRight, Right, HeavyRight, Left, Center, Center, HeavyRight, HeavyLeft, Center, Center, Center, HeavyLeft, Center, Center, Center  }},  // Center
    {0x07   , 6750, 7000, 'C', 2000, { Center, HeavyRight, Right, HeavyRight, Left, Center, Center, HeavyRight, HeavyLeft, Center, Center, Center, HeavyLeft, Center, Center, Center  }},  // Center
    {0x00   , 7000, 6750, 'C', 2000, { Center, HeavyRight, Right, HeavyRight, Left, Center, Center, HeavyRight, HeavyLeft, Center, Center, Center, HeavyLeft, Center, Center, Center  }},  // Center

};
State_t *Spt;  // pointer to the current state
uint8_t Input;
uint32_t Output;
char outputString;
/*Run FSM continuously
1) Output depends on State (LaunchPad LED)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */
int main(void)
{
    uint8_t reading;
    uint32_t heart=0;
    Reflectance_Init();
    Clock_Init48MHz();
    Motor_Init();        // activate Lab 13 software
    LaunchPad_Init();
//    Motor_InitSimple();
    TExaS_Init(LOGICANALYZER);  // optional
    Spt = Center;
    Pause();
    while(1){
        Output = Spt->out;            // set output from FSM
        outputString = Spt-> outString;
        LaunchPad_Output(Output);     // do output to two motors
        Motor_Forward(Spt->leftDuty,Spt->rightDuty);
//        if (outputString == 'C')
//        {
//            Motor_Forward(7000,7000);
//        }
//        else if (outputString == 'r')
//        {
//            Motor_RightSimple(1500, Spt->delay);
//        }
//        else if (outputString == 'l')
//        {
//            Motor_LeftSimple(1500, Spt->delay);
//        }
//        else if (outputString == 'R')
//        {
//            Motor_RightSimple(2000, Spt->delay);
//        }
//        else if (outputString == 'L')
//        {
//            Motor_LeftSimple(2000, Spt->delay);
//        }
//        else if (outputString == 'p')
//        {
//            Motor_LeftSimple(2500, Spt->delay);
//        }
//        else if (outputString == 'P')
//        {
//            Motor_RightSimple(2500, Spt->delay);
//        }
//        else
//        {
//            Motor_StopSimple();
//        }


//        TExaS_Set(Input<<2|Output);   // optional, send data to logic analyzer
        Clock_Delay1us(Spt->delay);   // wait
        // Input = LaunchPad_Input();    // read sensors
        reading = Reflectance_Read(1000);
        uint8_t InputMSB = (reading >> 3) & 0x08;
        uint8_t InputMid = (reading >> 2) & 0x06;
        uint8_t InputLSB = (reading >> 1) & 0x01;
//        uint8_t InputMSB =  (((reading >> 7) & 0x01) | ((reading >> 6) & 0x01)) << 0x03;
//        uint8_t InputMid1 = (((reading >> 5) & 0x01) | ((reading >> 4) & 0x01)) << 0x02;
//        uint8_t InputMid2 = (((reading >> 3) & 0x01) | ((reading >> 2) & 0x01)) << 0x01;
//        uint8_t InputLSB =  (((reading >> 1) & 0x01) | (reading & 0x01));
        Input = InputMSB + InputMid + InputLSB;

        Spt = Spt->next[Input];       // next depends on input and state
        heart = heart^1;
        LaunchPad_LED(heart);         // optional, debugging heartbeat
    }
}


