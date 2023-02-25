
#include <Reflectance.h>
#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "LaunchPad.h"
#include "MotorSimple.h"
#include "SysTick.h"


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

// Linked data structure
struct State {
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[16]; // Next if 4-bit input is 0-15
};

typedef const struct State State_t;

#define Center &fsm[0]
#define Left1  &fsm[1]
#define Left2  &fsm[2]
#define Left3  &fsm[3]
#define Right1 &fsm[4]
#define Right2 &fsm[5]
#define Right3 &fsm[6]
#define Stop   &fsm[7]
#define Error  &fsm[8]

State_t fsm[9]={
  {0x03, 50,  {Error,  Left3, Left2, Left2, Right2, Center, Center, Center, Right3, Center, Center, Center, Right2, Center, Center, Error}}, // Center
  {0x03, 25,  {Left3,  Left3, Left2, Left2, Right2, Center, Center, Center, Right3, Center, Center, Center, Right2, Center, Center, Error}}, // Left1
  {0x02, 25,  {Left3,  Left3, Left1, Left1, Right2, Center, Left1,  Left1,  Right3, Center, Center, Center, Right2, Center, Center, Error}}, // Left2
  {0x02, 50,  {Left3,  Left3, Left1, Left1, Right2, Center, Left1,  Left1,  Right3, Center, Center, Center, Right2, Center, Center, Error}}, // Left3

  {0x03, 25,  {Right3, Left3, Left2, Left2, Right2, Center, Center, Center, Right3, Center, Center, Center, Right2, Center, Center, Error}}, // Right1
  {0x01, 25,  {Right3, Left3, Left2, Left2, Right1, Center, Right1, Center, Right3, Center, Center, Center, Right1, Center, Center, Error}}, // Right2
  {0x01, 50,  {Right3, Left3, Left2, Left2, Right1, Center, Right1, Center, Right3, Center, Center, Center, Right1, Center, Center, Error}}, // Right3

  {0x03, 50,  {Stop,   Stop,  Stop,  Stop,  Stop,   Stop,   Stop,   Stop,   Stop,   Stop,   Stop,   Stop,   Stop,   Stop,  Stop,  Stop}}, // Stop
  {0x03, 50,  {Error,  Left3, Left2, Left2, Right2, Error,  Center, Error,  Right3, Error,  Error,  Error,  Right2, Error, Error, Error}}, // Error
};

State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;
uint32_t time;


/*Run FSM continuously
1) Output depends on State (LaunchPad LED)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */

// Drive motors based on state output
void drive(int32_t out, int32_t time) {
    uint16_t duty = 800; // duty cycle

    if (out == 0x01) { // If right of line, drive right motor
        Motor_RightSimple(duty, time);
    } else if (out == 0x02) { // If left of line, drive left motor
        Motor_LeftSimple(duty, time);
    } else if (out == 0x03) { // If center, drive both motors
        Motor_ForwardSimple(duty, time);
    }
}

// Flip the output from the reflectance center function
uint8_t read (void) {
    uint8_t data = Reflectance_Read(1000);
    uint8_t input = 0x00;

    input |= (data & 0x01) | ((data & 0x02) >> 1);     // Shift bits 0 and 1 to bit 0
    input |= ((data & 0x04) >> 1) | ((data & 0x08) >> 2); // Shift bits 2 and 3 to bit 1
    input |= ((data & 0x10) >> 2) | ((data & 0x20) >> 3); // Shift bits 4 and 5 to bit 2
    input |= ((data & 0x40) >> 3) | ((data & 0x80) >> 4); // Shift bits 6 and 7 to bit 3

    /*
    input |= (data & 0x01) | ((data & 0x02) >> 1) | ((data & 0x04) >> 2); // Shift all to bit 0
    input |= (data & 0x08) >> 2; // Shift from bit 3 to bit 1
    input |= (data & 0x10) >> 2; // Shift from bit 4 to bit 2
    input |= ((data & 0x20) >> 2) | ((data & 0x40) >> 3) | ((data & 0x80) >> 4); // Shift all to bit 3
    */

    return input;
}


int main(void){

  // Initialize everything
  Clock_Init48MHz();
  LaunchPad_Init();
  Reflectance_Init();
  Motor_InitSimple();
  SysTick_Init();

  Spt = Center;
  while(1){
    Output = Spt->out;            // set output from FSM
    time = Spt->delay;
    drive(Output, time);     // do output to two motors
    Input = read();    // read sensors
    Spt = Spt->next[Input];       // next depends on input and state
  }
}

