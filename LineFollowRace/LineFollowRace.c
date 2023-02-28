
#include <Reflectance.h>
#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "LaunchPad.h"
#include "Motor.h"
#include "SysTick.h"
#include "Bump.h"

#define MOTOR_FORWARD 1
#define MOTOR_LEFT 2
#define MOTOR_RIGHT 3
#define MOTOR_BACKWARD 4

#define MAXSPEED 6000

volatile uint8_t bump_detected = 0;

// Linked data structure
struct State {
  uint16_t right_PWM;           // Right wheel PWM
  uint16_t left_PWM;            // Left wheel PWM
  uint8_t  function;            // Number for motor function
  const struct State *next[32]; // Next if 5-bit input is 0-32
};

typedef const struct State State_t;

#define Center    &fsm[0]
#define Left1     &fsm[1]
#define Left2     &fsm[2]
#define Left3     &fsm[3]
#define LostLeft  &fsm[4]
#define Right1    &fsm[5]
#define Right2    &fsm[6]
#define Right3    &fsm[7]
#define LostRight &fsm[8]
#define Stop      &fsm[9]
#define Error     &fsm[10]

State_t fsm[11]={
  {MAXSPEED, MAXSPEED, MOTOR_FORWARD, {Error,      Left1,       Left1,   Left1,       Center,  Error,   Left1,   Left1,   Right1,  Error,   Error,   Error,   Right1,  Error,   Error,   Error,   Right1,      Error,   Error,   Error,   Error,   Error,   Error,   Error,   Right1,      Error,   Error,   Error,   Right1,  Error,   Error,   Error}}, // Center

  {MAXSPEED * 0.9, MAXSPEED, MOTOR_FORWARD, {LostLeft,   Left2,       Left2,   Left2,       Center,  Error,   Left1,   Left1,   Error,   Error,   Error,   Error,   Right1,  Error,   Error,   Error,   Error,       Error,   Error,   Error,   Error,   Error,   Error,   Error,   Error,       Error,   Error,   Error,   Right1,  Error,   Error,   Error}}, // Left1
  {MAXSPEED * 0.6, MAXSPEED, MOTOR_FORWARD, {LostLeft,   Left3,       Left2,   Left2,       Right1,  Error,   Left1,   Left1,   Error,   Error,   Error,   Error,   Right1,  Error,   Error,   Error,   Error,       Error,   Error,   Error,   Error,   Error,   Error,   Error,   Error,       Error,   Error,   Error,   Right1,  Error,   Error,   Error}}, // Left2
  {MAXSPEED * 0.5, MAXSPEED, MOTOR_LEFT,    {LostLeft,   Left3,       Left2,   Left2,       Right2,  Error,   Left2,   Left2,   Error,   Error,   Error,   Error,   Right2,  Error,   Error,   Error,   Error,       Error,   Error,   Error,   Error,   Error,   Error,   Error,   Error,       Error,   Error,   Error,   Right2,  Error,   Error,   Error}}, // Left3
  {MAXSPEED,       MAXSPEED, MOTOR_LEFT,    {LostLeft,   Left3,       Left3,   Left3,       Error,   Error,   Error,   Error,   Error,   Error,   Error,   Error,   Error,   Error,   Error,   Error,   LostRight,   Error,   Error,   Error,   Error,   Error,   Error,   Error,   LostRight,   Error,   Error,   Error,   Error,   Error,   Error,   Error}}, // Lost Left

  {MAXSPEED, MAXSPEED * 0.9, MOTOR_FORWARD, {LostRight,  Error,       Error,   Error,       Center,  Error,   Left1,   Left1,   Right2,  Error,   Error,   Error,   Right1,  Error,   Error,   Error,   Right2,      Error,   Error,   Error,   Error,   Error,   Error,   Error,   Right2,      Error,   Error,   Error,   Right1,  Error,   Error,   Error}}, // Right1
  {MAXSPEED, MAXSPEED * 0.6, MOTOR_FORWARD, {LostRight,  Error,       Error,   Error,       Left1,   Error,   Left1,   Left1,   Right2,  Error,   Error,   Error,   Right1,  Error,   Error,   Error,   Right3,      Error,   Error,   Error,   Error,   Error,   Error,   Error,   Right2,      Error,   Error,   Error,   Right1,  Error,   Error,   Error}}, // Right2
  {MAXSPEED, MAXSPEED * 0.5, MOTOR_RIGHT,   {LostRight,  Error,       Error,   Error,       Left2,   Error,   Left2,   Left2,   Right2,  Error,   Error,   Error,   Right2,  Error,   Error,   Error,   Right3,      Error,   Error,   Error,   Error,   Error,   Error,   Error,   Right2,      Error,   Error,   Error,   Right2,  Error,   Error,   Error}}, // Right3
  {MAXSPEED, MAXSPEED,       MOTOR_RIGHT,   {LostRight,  LostLeft,    Error,   LostLeft,    Error,   Error,   Error,   Error,   Right3,  Error,   Error,   Error,   Error,   Error,   Error,   Error,   Right3,      Error,   Error,   Error,   Error,   Error,   Error,   Error,   Right3,      Error,   Error,   Error,   Error,   Error,   Error,   Error}}, // Lost Right

  {       0,        0, MOTOR_FORWARD, {Stop,       Stop,        Stop,    Stop,        Stop,    Stop,    Stop,    Stop,    Stop,    Stop,    Stop,    Stop,    Stop,    Stop,    Stop,    Stop,    Stop,        Stop,    Stop,    Stop,    Stop,    Stop,    Stop,    Stop,    Stop,        Stop,    Stop,    Stop,    Stop,    Stop,    Stop,    Stop}},   // Stop
  {MAXSPEED, MAXSPEED, MOTOR_FORWARD, {Error,      Left3,       Left2,   Left2,       Center,  Error,   Left1,   Left1,   Right2,  Error,   Error,   Error,   Right2,  Error,   Error,   Error,   Right3,      Error,   Error,   Error,   Error,   Error,   Error,   Error,   Right2,      Error,   Error,   Error,   Right1,  Error,   Error,   Error}}, // Error
};

State_t *Spt;  // pointer to the current state


// Convert output from reflectance read function to 6 bits
uint8_t read (void) {
    uint8_t data = Reflectance_Read(1000);
    uint8_t input = 0x00;

    input |= (data & 0x01) | ((data & 0x02) >> 1);        // Shift bits 0 and 1 to bit 0
    input |= ((data & 0x04) >> 1);                        // Shift bit 2 to bit 1
    input |= ((data & 0x08) >> 1) | ((data & 0x10) >> 2); // Shift bit 3 and 4 to bit 2
    input |= ((data & 0x20) >> 2);                        // Shift bit 5 to bit 3
    input |= ((data & 0x40) >> 2) | ((data & 0x80) >> 3); // Shift bits 6 and 7 to bit 4

    return input;
}

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
      if (!bump_detected) {
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
      }
  }
}

