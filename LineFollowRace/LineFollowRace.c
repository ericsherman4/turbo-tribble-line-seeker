
#include <Reflectance.h>
#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "LaunchPad.h"
#include "Motor.h"
#include "SysTick.h"
#include "Bump.h"


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

#define MOTOR_FORWARD 1
#define MOTOR_LEFT 2
#define MOTOR_RIGHT 3
#define MOTOR_BACKWARD 4

// Linked data structure
struct State {

  uint16_t right_PWM;           // Right wheel PWM
  uint16_t left_PWM;            // Left wheel PWM
  uint8_t function;            // motor function to call
  const struct State *next[64]; // Next if 6-bit input is 0-63
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
  {4000, 4000, MOTOR_FORWARD , {Error, Left3, Left2, Left2, Left1, Left1, Left1, Left1, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right3, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Error}}, // Center
  {2000, 3000, MOTOR_FORWARD , {Error, Left3, Left2, Left2, Left1, Left1, Left1, Left1, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right3, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Error}}, // Left1
  {1500, 3000, MOTOR_FORWARD , {Error, Left3, Left2, Left2, Left1, Left1, Left1, Left1, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right3, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Error}}, // Left2
  {1500, 1500, MOTOR_LEFT , {Error, Left3, Left2, Left2, Left1, Left1, Left1, Left1, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right3, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Error}}, // Left3

  {3000, 2000, MOTOR_FORWARD , {Error, Left3, Left2, Left2, Left1, Left1, Left1, Left1, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right3, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Error}}, // Right1
  {3000, 1500, MOTOR_FORWARD , {Error, Left3, Left2, Left2, Left1, Left1, Left1, Left1, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right3, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Error}}, // Right2
  {1500, 1500, MOTOR_RIGHT , {Error, Left3, Left2, Left2, Left1, Left1, Left1, Left1, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right3, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Error}}, // Right3

  {   0,    0, MOTOR_FORWARD , {Stop,  Stop,  Stop,  Stop,  Stop,  Stop,  Stop,  Stop,  Stop,   Stop,  Stop,  Stop,  Stop,   Stop,   Stop,   Stop,   Stop,   Stop,  Stop,  Stop,  Stop,  Stop,  Stop,  Stop,  Stop,   Stop,  Stop,  Stop,  Stop,   Stop,   Stop,   Stop,   Stop,   Stop,  Stop,  Stop,  Stop,  Stop,  Stop,  Stop,  Stop,   Stop,  Stop,  Stop,  Stop,   Stop,   Stop,   Stop,   Stop,   Stop,  Stop,  Stop,  Stop,  Stop,  Stop,  Stop,  Stop,   Stop,  Stop,  Stop,  Stop,   Stop,   Stop,   Stop}},   // Stop
  { 2000,  2000, MOTOR_BACKWARD , {Error, Left3, Left2, Left2, Left1, Left1, Left1, Left1, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right3, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Center, Right2, Error, Error, Error, Error, Error, Error, Error, Right1, Error, Error, Error, Center, Center, Center, Error}}, // Error
};

State_t *Spt;  // pointer to the current state

State_t *prevstates[4] = {Center, Center, Center, Center};



/*Run FSM continuously
1) Output depends on State (LaunchPad LED)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */

// Convert output from reflectance read function to 6 bits
uint8_t read (void) {
    uint8_t data = Reflectance_Read(1000);
    uint8_t input = 0x00;

    input |= (data & 0x01) | ((data & 0x02) >> 1);        // Shift bits 0 and 1 to bit 0
    input |= ((data & 0x04) >> 1);                        // Shift bit 2 to bit 1
    input |= ((data & 0x08) >> 1);                        // Shift bit 3 to bit 2
    input |= ((data & 0x10) >> 1);                        // Shift bit 4 to bit 3
    input |= ((data & 0x20) >> 1);                        // Shift bit 5 to bit 4
    input |= ((data & 0x40) >> 1) | ((data & 0x80) >> 2); // Shift bits 6 and 7 to bit 5

    return input;
}

volatile uint8_t bump_detected = 0;

int main(void){

  // Initialize everything
  Clock_Init48MHz();
  LaunchPad_Init();
  Reflectance_Init();
  Motor_Init();
  SysTick_Init();
  Bump_Init();
  BumpInt_Init();


  Spt = Center;
  while(1){
      if (!bump_detected)
      {
          //Motor_Forward(Spt->left_PWM, Spt->right_PWM);     // do output to two motors
          switch(Spt->function)
          {
            case MOTOR_FORWARD:
                Motor_Forward(Spt->left_PWM, Spt->right_PWM);
                break;
            case MOTOR_RIGHT:
                Motor_Right(Spt->left_PWM, Spt->right_PWM);
                break;
            case MOTOR_LEFT:
                Motor_Left(Spt->left_PWM, Spt->right_PWM);
                break;
            case MOTOR_BACKWARD:
                Motor_Backward(Spt->left_PWM, Spt->right_PWM);
                break;
          }

          SysTick_Wait10ms(1);
          uint8_t Input = read();    // read sensors

          Spt = Spt->next[Input];       // next depends on input and state

          // shift all of the previous states
          prevstates[3] = prevstates[2];
          prevstates[2] = prevstates[1];
          prevstates[1] = prevstates[0];
          prevstates[0] = Spt;

          uint8_t count,i;
          for(i = 0; i < 4; i++)
          {
            if(prevstates[i] == Error) count++;
          }

          if(count >= 3)
          {
            Motor_Forward(6000,6000);
            SysTick_Wait10ms(20);
          }

      }
  }
}

