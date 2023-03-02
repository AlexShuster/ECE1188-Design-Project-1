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
//#include "../inc/Clock.h"
//#include "../inc/CortexM.h"
//#include "../inc/LaunchPad.h"
//#include "../inc/Motor.h"
//#include "../inc/BumpInt.h"
//#include "../inc/TExaS.h"
//#include "../inc/TimerA1.h"
//#include "../inc/Reflectance.h"
//#include "../inc/SysTickInts.h"
#include "/Applications/ti/tirslk_max_1_00_02/inc/CortexM.h"
#include "/Applications/ti/tirslk_max_1_00_02/inc/SysTickInts.h"
#include "/Applications/ti/tirslk_max_1_00_02/inc/LaunchPad.h"
#include "/Applications/ti/tirslk_max_1_00_02/inc/Clock.h"
#include "/Applications/ti/tirslk_max_1_00_02/inc/Reflectance.h"
#include "/Applications/ti/tirslk_max_1_00_02/inc/BumpInt.h"
#include "/Applications/ti/tirslk_max_1_00_02/inc/TExaS.h"
#include "/Applications/ti/tirslk_max_1_00_02/inc/Motor.h"
#include "/Applications/ti/tirslk_max_1_00_02/inc/TimerA1.h"

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
  int32_t leftDuty;                // 2-bit output
  int32_t rightDuty;
//  char outString;
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[16]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define C       &fsm[0]
#define L       &fsm[1]
#define R       &fsm[2]
#define HR      &fsm[3]
#define HL      &fsm[4]
#define ML      &fsm[5]
#define MR       &fsm[6]
#define CL      &fsm[7]
#define CR      &fsm[8]
#define STOP    &fsm[9]
#define CLOST   &fsm[10]


// Color    LED(s) Port2
// dark     ---    0
// red      R--    0x01
// blue     --B    0x04
// green    -G-    0x02
// yellow   RG-    0x03
// sky blue -GB    0x06
// white    RGB    0x07
// pink     R-B    0x05

//State_t fsm[11]={
//    {0x00   , 6000, 6000,    1, { CLOST,  HR, R, CR, C, CL, L, HL}},  // CENTER
//    {0x01   , 4000, 5000,    1, { HR, HR, R, CR, C, CL, L, HL    }},    // LEFT
//    {0x02   , 5000, 4000,    1, { HR, HR, R, CR, C, CL, L, HL}},     // RIGHT
//    {0x03   , 2000, 0,    1, { HR, HR, R, CR, C, CL, L, HL}},     // HEAVY RIGHT
//    {0x04   , 0, 2000,    1, { HR, HR, R, CR, C, CL, L, HL  }},   // HEAVY LEFT
//    {0x05   , 3500, 1000,    1, { HR, HR, R, CR, C, CL, L, HL }},  // PREVIOUS LEFT LOST
//    {0x06   , 7000, 7000,    1, { CLOST,  CR, CR, CR, PRL, CL, CL, CL }},  // PRL
//    {0x07   , 5000, 5500,    1, { HR, HR, R, CR, C, CL, L, HL }},  // CENTER LEFT
//    {0x00   , 5500, 5000,    1, { HL, HR, R, CR, C, CL, L, HL  }},  // CENTER RIGHT
//    {0x00   , 0,    0,       1, { STOP, STOP, STOP, STOP, STOP, STOP, STOP, STOP  }},  // STOP
//    {0x00   , 7000, 7000,    1, { C, C, C, C, C, C, C,C  }}  // CENTER LOST
//};
//State_t fsm[11]={
//    {0x00   , 6000, 6000,    1, { CLOST,  MR, R, CR, C, CL, L, ML}},            // CENTER
//    {0x01   , 4000, 5000,    1, { MR, MR, R, CR, C, CL, L, ML    }},            // LEFT
//    {0x02   , 5000, 4000,    1, { MR, MR, R, CR, C, CL, L, ML }},               // RIGHT
//    {0x03   , 2000, -2000,       1, { MR, MR, R, CR, C, CL, L, ML }},               // HEAVY RIGHT
//    {0x04   , -2000, 2000,       1, { MR, MR, R, CR, C, CL, L, ML   }},             // HEAVY LEFT
//    {0x05   , 0, 2000,    1, { HR, HR, R, CR, C, CL, L, HL }},               // MED LEFT
//    {0x06   , 2000, 0,    1, { HR, HR, R, CR, C, CL, L, HL }},               // MED RIGHT
//    {0x07   , 5000, 5500,    1, { MR, MR, R, CR, C, CL, L, ML }},               // CENTER LEFT
//    {0x00   , 5500, 5000,    1, { MR, MR, R, CR, C, CL, L, ML  }},              // CENTER RIGHT
//    {0x00   , 0,    0,       1, { STOP, STOP, STOP, STOP, STOP, STOP, STOP, STOP  }},  // STOP
//    {0x00   , 7000, 7000,    1, { C, C, C, C, C, C, C,C  }}  // CENTER LOST
//};
State_t fsm[11]={
    {0x00   , 4250, 4250, 1, { CLOST,  HR, R, CR, C, CL, L, HL}},  // CENTER
    {0x01   , 4250, 4150, 5, { HR, HR, R, CR, R, CL, L, HL    }},    // LEFT
    {0x02   , 4150, 4250, 5, { HL, HR, R, CR, L, CL, L, HL}},     // RIGHT
    {0x03   , 4250, -500, 5, { HR, HR, R, CR, L, CL, L, HL}},     // HEAVY RIGHT
    {0x04   , -500, 4250, 5, { HL, HR, R, CR, R, CL, L, HL  }},   // HEAVY LEFT
    {0x05   , 3250, 1750, 5, { HR, HR, R, CR, C, CL, L, HL }},  // PREVIOUS LEFT LOST
    {0x06   , 1750, 3250, 5, { HL, HR, R, CR, C, CL, L, HL }},  // PRL
    {0x07   , 3750, 4250, 5, { HR, HR, R, CR, C, CL, L, HL }},  // CENTER LEFT
    {0x00   , 4250, 3750, 5, { HL, HR, R, CR, C, CL, L, HL  }},  // CENTER RIGHT
    {0x00   , 0, 0, 200, { STOP, STOP, STOP, STOP, STOP, STOP, STOP, STOP  }},  // STOP
    {0x00   , 5000, 5000, 5, { C, C, C, C, C, C, C,C  }}  // CENTER LOST

};
State_t *Spt;  // pointer to the current state
uint8_t Input, SensorData;
uint8_t Output, systick_count;
uint8_t CollisionData, CollisionFlag;  // mailbox
char outputString;

void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   CollisionFlag = 1;
}

void SysTick_Handler(void) {
    if (systick_count % 10 == 0) {
        Reflectance_Start();
    } else if (systick_count % 10 == 1) {
        SensorData = Reflectance_Position(Reflectance_End());
    }
    systick_count = systick_count + 1;
}

void decodeMotor(int32_t left, int32_t right){
    if (left < 0 && right < 0) {
        Motor_Backward(-left, -right);
    } else if (left > 0 && right < 0) {
        Motor_Right(left, -right);
    } else if (left < 0 && right > 0) {
        Motor_Left(-left, right);
    } else {
        Motor_Forward(left, right);
    }
}


/*Run FSM continuously
1) Output depends on State (LaunchPad LED)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */
int main(void)
{
    EnableInterrupts();
    Clock_Init48MHz();
    SysTick_Init(48000, 2);
    Reflectance_Init();
    Motor_Init();               // activate Lab 13 software
    LaunchPad_Init();
    TExaS_Init(LOGICANALYZER);  // optional
    BumpInt_Init(&HandleCollision);

    Spt = C;
    Pause();

    while(1){
        Output = Spt->out;            // set output from FSM
//        outputString = Spt-> outString;
        LaunchPad_Output(Output);     // do output to two motors
        decodeMotor(Spt->leftDuty,Spt->rightDuty);
        Clock_DelayLess(Spt->delay);
        Spt = Spt->next[SensorData];       // next depends on input and state

//        if (CollisionFlag) Spt = STOP;
    }
}
