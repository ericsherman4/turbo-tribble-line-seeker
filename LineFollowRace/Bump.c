// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"
#include "Bump.h"
#include "Motor.h"


extern uint8_t bump_detected;

void BumpInt_Init(void){
    P4->IE |= 0xED; //Enabling interrupts for bump pins on port 4
    P4->IES |= 0xED;   //Enabling rising edge interrupt bump pins (I think)
    P4->IFG &= ~0xED; //Clearing interrupt flag
    NVIC_EnableIRQ(PORT4_IRQn);
    //P4->IV |= 0x0F; //Setting interrupt to low priority (not sure if this is a mistake, or what "low" implies)
}

void PORT4_IRQHandler(void){
    Motor_Stop();
    bump_detected = 1;
    P4->IFG &= ~0xED;       //Clearing Flags
}

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
void Bump_Init(void){
    P4-> SEL0 &= ~0xED; //Initializing P4 bumper pins
    P4-> SEL1 &= ~0xED;
    P4-> DIR |= 0xED;
    P4-> REN |= 0xED; //Enabling pull-up resistors
    P4-> OUT |= 0xED;
}

