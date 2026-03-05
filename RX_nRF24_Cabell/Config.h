
#ifndef __Config_h__
#define __Config_h__

#include <Arduino.h>

/*
  *****************************************************************************************************************************************
  RC receiver configuration manual. See examples below "Custom configuration"
  *****************************************************************************************************************************************
  Servo and motor output selection
  SERVO_12CH            Separate servo outputs (2 to 12 servo channels)
  MOTOR1_2              Motor output 1 and 2
  MIX_TANK_MOTOR1_2     "Tank-arcade" mix of motor 1 and 2
  SERVO_10CH_MOTOR1     Motor 1 and servo output (1 to 10 servo channels)
  SERVO_8CH_MOTOR1_2PB  ATmega328PB only! Motor 1 and 2 and servo output (1 to 8 servo channels)
  
  Setting the number of servo channels
  SERVO_CHANNELS  1 to 12
  
  Setting the PWM prescaler according to the requirements and limitations of the timers/counters. Details in the "PWM" file
  30HZ to 62500HZ
  
  Setting the motor reaction point. Prevents initial rotor magnetic resistance
  REACTION_MOTOR1, REACTION_MOTOR2  0 to 255
  
  Setting the maximum motor power
  MAX_FORWARD_MOTOR1, MAX_REVERSE_MOTOR1, MAX_FORWARD_MOTOR2, MAX_REVERSE_MOTOR2  0 to 255
  
  Brake setting, no brake 0, maximum brake 255
  BRAKE_MOTOR1, BRAKE_MOTOR2  0 to 255
*/

//*********************************************************************************************************************
// Custom configuration for a specific RC model
//*********************************************************************************************************************
#define SERVO_12CH           // Glider Let L-13 Blanik 4ch
//#define MOTOR1_2             // Buggy 1:32 2ch
//#define MIX_TANK_MOTOR1_2    // Eachine Monster 2ch
//#define SERVO_10CH_MOTOR1    // Ferari F-40 2ch
//#define SERVO_8CH_MOTOR1_2PB // Tank T-34/85 3ch

//*******************************
// Glider Let L-13 Blanik 4ch
//*******************************
#if defined(SERVO_12CH)
  #define SERVO_CHANNELS  12
#endif

//*******************************
// Buggy 1:32 2ch
//*******************************
#if defined(MOTOR1_2)
  // Motor 1
  #define TIMER2_122HZ
  #define REACTION_MOTOR1  0
  #define MAX_FORWARD_MOTOR1  255
  #define MAX_REVERSE_MOTOR1  255
  #define BRAKE_MOTOR1  0
  // Motor 2
  #define TIMER1_122HZ
  #define REACTION_MOTOR2  0
  #define MAX_FORWARD_MOTOR2  255
  #define MAX_REVERSE_MOTOR2  255
  #define BRAKE_MOTOR2  0
#endif

//*******************************
// Eachine Monster 2ch
//*******************************
#if defined(MIX_TANK_MOTOR1_2)
  // Motor 1
  #define TIMER2_122HZ
  #define REACTION_MOTOR1  0
  #define MAX_FORWARD_MOTOR1  255
  #define MAX_REVERSE_MOTOR1  255
  #define BRAKE_MOTOR1  255
  // Motor 2
  #define TIMER1_122HZ
  #define REACTION_MOTOR2  0
  #define MAX_FORWARD_MOTOR2  255
  #define MAX_REVERSE_MOTOR2  255
  #define BRAKE_MOTOR2  255
#endif

//*******************************
// Ferari F-40 2ch
//*******************************
#if defined(SERVO_10CH_MOTOR1)
  #define SERVO_CHANNELS  10
  // Motor 1
  #define TIMER2_122HZ
  #define REACTION_MOTOR1  0
  #define MAX_FORWARD_MOTOR1  255
  #define MAX_REVERSE_MOTOR1  255
  #define BRAKE_MOTOR1  255
#endif

//*******************************
// Tank T-34/85 3ch
//*******************************
#if defined(SERVO_8CH_MOTOR1_2PB)
  #define SERVO_CHANNELS  8
  // Motor 1
  #define TIMER2_122HZ
  #define REACTION_MOTOR1  0
  #define MAX_FORWARD_MOTOR1  255
  #define MAX_REVERSE_MOTOR1  255
  #define BRAKE_MOTOR1  255
  // Motor 2 ATmega328PB
  #define TIMER4_122HZ
  #define REACTION_MOTOR2  0
  #define MAX_FORWARD_MOTOR2  255
  #define MAX_REVERSE_MOTOR2  255
  #define BRAKE_MOTOR2  255
#endif

//*********************************************************************************************************************
// Number of motor outputs
//*********************************************************************************************************************
#if defined(MOTOR1_2)
  #define MOTOR_CHANNELS  2
  #define MOTOR1
  #define MOTOR2
#endif

#if defined(SERVO_10CH_MOTOR1)
  #define MOTOR_CHANNELS  1
  #define MOTOR1
#endif

#if defined(SERVO_8CH_MOTOR1_2PB)
  #define MOTOR_CHANNELS  2
  #define MOTOR1
  #define MOTOR2PB
#endif

//*********************************************************************************************************************
// If there is no output to servos or motors, set RC channels to 0
//*********************************************************************************************************************
#ifndef SERVO_CHANNELS
  #define SERVO_CHANNELS  0
#endif

#ifndef MOTOR_CHANNELS
  #define MOTOR_CHANNELS  0
#endif

//*********************************************************************************************************************
// Setting the dead zone of poor quality joysticks TX for the motor controller
//*********************************************************************************************************************
#define DEAD_ZONE  15

#endif // End __Config_h__
 
