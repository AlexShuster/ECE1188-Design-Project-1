// BumpInt.c
// Runs on MSP432, interrupt version
// Provide low-level functions that interface bump switches on the robot.
// Daniel Valvano and Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

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

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"
void (*BumpTask)(uint8_t);
// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
void BumpInt_Init(void(*task)(uint8_t)){
    // write this as part of Lab 14

    ///////////// PIN SETUP //////////////
    // 1110 1101 ==> ED
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;    // configure pins as GPIO (clear bits)

    P4->DIR  &= ~0xED;     // make pins as inputs

    P4->REN  |= 0xED;      // set pins' internal resistors
    P4->OUT  |= 0xED;      // initialize the pins resistors as pull-ups


    ////////// INTERRUPT SETUP //////////
    P4->IES |= 0xED;      // set interrupts as falling-edge --> when pressed
    P4->IFG |= 0x00;      // clear initial flag
    P4->IE  |= 0xED;      // enable interrupts on the pins

    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00400000; // Port 4 IRQ Handler (bits 23-21 = priority 2)
    NVIC->ISER[1] = 0x00000040;                        // IRQ 32-63 set enable interrupt on bit 38 in NVIC

    BumpTask = task;

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
    // write this as part of Lab 14
    // 1110 1101 ==> ED
    // 1100 ==> C
    // 1110 ==> E

    uint8_t result, input;
    input = ~(P4->IN);              // change to positive logic
    result  = (input)&0x01;       // BUMP0 = bit 0;
    result |= ((input)&0x0C)>>1;  // BUMP2-BUMP1 = bits 3 and 2;
    result |= ((input)&0xE0)>>2;  // BUMP5-BUMP3 = bits 7, 6, and 5;

    return result;
}
// we do not care about critical section/race conditions
// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    // write this as part of Lab 14
    P4->IFG |= 0x00;     // clear existing flags
    (*BumpTask)(Bump_Read());

}

