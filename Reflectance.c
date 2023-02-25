// Reflectance.c
// Provide functions to take measurements using the kit's built-in
// QTRX reflectance sensor array.  Pololu part number 3672. This works by outputting to the
// sensor, waiting, then reading the digital value of each of the
// eight phototransistors.  The more reflective the target surface is,
// the faster the voltage decays.
// Daniel and Jonathan Valvano
// July 11, 2019


// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp432.h"
#include "..\inc\Clock.h"

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
void Reflectance_Init(void){

    // Set up even sensor LED output (P5.3)
    P5->SEL0 &= ~0x08; // Configure P5.3 as GPIO
    P5->SEL1 &= ~0x08;
    P5->DIR |= 0x08; // Configure P5.3 as output
    P5->OUT &= ~0x08; // Turn off even sensor LED output

    // Set up odd sensor LED output (P9.2)
    P9->SEL0 &= ~0x04; // Configure P9.2 as GPIO
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04; // Configure P9.2 as output
    P9->OUT &= ~0x04; // Turn off odd sensor LED output

    // Set up sensor 1-8 as inputs (P7.0 - P7.7)
    P7->SEL0 &= ~0xFF; // Configure P7.0 - P7.7 as GPIO
    P7->SEL1 &= ~0xFF;
    P7->DIR &= ~0xFF; // Configure P7.0 - P7.7 as inputs
}

// ------------Reflectance_Read------------
// Read the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Read(uint32_t time){

    // Create the result variable
    uint8_t result;

    // Turn on the 8 IR LEDs
    P5->OUT |= 0x08; // Turn even sensor LEDs (P5.3) on
    P9->OUT |= 0x04; // Turn odd sensor LEDs (P9.2) on

    // Pulse 8 sensors high for 10 us
    P7->DIR |= 0xFF; // Switch 8 sensors to outputs
    P7->OUT |= 0xFF; // Send sensors high
    Clock_Delay1us(10); // Pulse for 10us

    // Switch the sensor pins to input
    //P7->DIR &= ~0xFF;
    P7->DIR = 0x00;

    // Wait time us
    Clock_Delay1us(time);

    // Read in the values of the sensors
    result = P7->IN;

    // Turn off the 8 IR LEDs
    P5->OUT &= ~0x08; // Turn even sensor LEDs (P5.3) off
    P9->OUT &= ~0x04; // Turn odd sensor LEDs (P9.2) off

    // Return the result
    return result;
}

// ------------Reflectance_Center------------
// Read the two center sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: 0 (off road), 1 off to left, 2 off to right, 3 on road
// (Left,Right) Sensors
// 1,1          both sensors   on line
// 0,1          just right     off to left
// 1,0          left left      off to right
// 0,0          neither        lost
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Center(uint32_t time){

    // Call the function to read the sensors
    uint8_t sensors = Reflectance_Read(time);

    // Store result as bit-shifted and masked sensor data
    uint8_t result = (sensors & 0x18) >> 3;

  return result;
}


// Perform sensor integration
// Input: data is 8-bit result from line sensor
// Output: position in um relative to center of line
int32_t Reflectance_Position(uint8_t data){

    // Set the weights of each sensor
    int32_t weights[8] = {-33400, -23800, -14300, -4800, 4800, 14300, 23800, 33400};

    // Calculate the weighted average of the binary sensor states
    int32_t num = 0; // Calculate the numerator
    int i;
    for (i=0;i<8;i++) {
        num += ((data >> i) & 0x01) * weights[i];
    }
    int32_t den = 0; // Calculate the denominator
    for (i=0;i<8;i++) {
        den += ((data >> i) & 0x01);
    }

    // Calculate the distance from the center line
    int32_t distance = num / den;

 return distance;
}


// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void){
    // write this as part of Lab 10
}


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void){
    // write this as part of Lab 10
 return 0; // replace this line
}


