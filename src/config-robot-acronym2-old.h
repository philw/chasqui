#pragma once

#include <Arduino.h>

#define NAME "ACRONYM2"

const uint32_t BAUDRATE = 9600;


// encoder polarity is either 1 or -1 and is used to account for reversal of the encoder phases
#define ENCODER_LEFT_POLARITY (1)
#define ENCODER_RIGHT_POLARITY (-1)

// similarly, the motors may be wired with different polarity and that is defined here so that
// setting a positive voltage always moves the robot forwards
const int MOTOR_LEFT_POLARITY = (1);
const int MOTOR_RIGHT_POLARITY = (-1);

//***************************************************************************//
// Some physical constants that are likely to be robot-specific
// with robot against back wall, how much travel is there to the cell center?
const int BACK_WALL_TO_CENTER = 48;

//***************************************************************************//
// We need to know about the drive mechanics.
// The encoder pulse counts should be obvious from the encoder itself.
// Work out the gear ratio by rotating the wheel a number of turns and counting
// the pulses.
// Finally, move the mouse in a straight line through 1000mm of travel to work
// out the wheel diameter.
const float ENCODER_PULSES = 6.00;
const float GEAR_RATIO = 50.00;
const float WHEEL_DIAMETER = 32.00;

// Mouse radius is the distance between the contact patches of the drive wheels.
// A good starting approximation is half the distance between the wheel centres.
// After testing, you may find the working value to be larger or smaller by some
// small amount. AFTER you have the wheel diameter and gear ratio calibrated,
// have the mouse turn in place and adjust the MOUSE_RADIUS until these turns are
// as accurate as you can get them
const float MOUSE_RADIUS = 38.70;  // 39.50; // Adjust on test

// The robot is likely to have wheels of different diameters or motors of slightly
// different characteristics and that must be compensated for if the robot is to
// reliably drive in a straight line.
// This number adjusts the encoder count and must be  added to the right
// and subtracted from the left motor.
const float ROTATION_BIAS = 0.0025;  // Negative makes robot curve to left

// Now we can pre-calculate the key constats for the motion control
const float MM_PER_COUNT = PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * MM_PER_COUNT;
const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * MM_PER_COUNT;
const float DEG_PER_MM_DIFFERENCE = (180.0 / (2 * MOUSE_RADIUS * PI));


//*** MOTION CONTROLLER CONSTANTS **********************************************//

//***************************************************************************//
// Control loop timing. Pre-calculate to save time in interrupts
const float LOOP_FREQUENCY = 500.0;
const float LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY);
