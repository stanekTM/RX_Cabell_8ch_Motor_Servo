/*
  ***************************************************************************************************************
  Test RC receiver with "Cabell" protocol (motor-servo driver, telemetry) using OpenAVRc and OpenTX Multiprotocol
  ***************************************************************************************************************
  Simple RC receiver from my repository https://github.com/stanekTM/RX_Cabell_8ch_Motor_Servo
  
  Works with RC transmitters:
  OpenAVRc                   https://github.com/Ingwie/OpenAVRc_Dev
  Multiprotocol from my fork https://github.com/stanekTM/TX_FW_Multi_Stane
  ***************************************************************************************************************
*/

/*
  Copyright 2017 by Dennis Cabell
  KE8FZX
  
  To use this software, you must adhere to the license terms described below, and assume all responsibility for the use
  of the software.  The user is responsible for all consequences or damage that may result from using this software.
  The user is responsible for ensuring that the hardware used to run this software complies with local regulations and that 
  any radio signal generated from use of this software is legal for that user to generate.  The author(s) of this software 
  assume no liability whatsoever.  The author(s) of this software is not responsible for legal or civil consequences of 
  using this software, including, but not limited to, any damages cause by lost control of a vehicle using this software.  
  If this software is copied or modified, this disclaimer must accompany all copies.
  
  This project is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  RC_RX_CABELL_V3_FHSS is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with RC_RX_CABELL_V3_FHSS.  If not, see http://www.gnu.org/licenses.
*/

/*
  Library dependencies:
  
  Aaduino Core, SPI, and EEPROM
  
  https://github.com/nRF24/RF24 library copied and modified to streamline
  opeerations specific to this application in order to improve loop timing.
  See http://tmrh20.github.io/RF24  for documentation on the standard version of the library.
  
  Arduino Servo was modified and is included with this source.  It was changed to not directly define the Timer 1 ISR
*/

#include "RX.h"
#include "Pins.h"
#include "PWM_Frequency.h"

//*********************************************************************************************************************
void setup(void)
{
  //Serial.begin(9600);
  
  pinMode(PIN_PWM_1_MOTOR_A, OUTPUT);
  pinMode(PIN_PWM_2_MOTOR_A, OUTPUT);
  pinMode(PIN_PWM_3_MOTOR_B, OUTPUT);
  pinMode(PIN_PWM_4_MOTOR_B, OUTPUT);
  
  pinMode(PIN_BUTTON_BIND, INPUT_PULLUP);
  
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  
  // Setting the motor frequency
  setPWMPrescaler(PIN_PWM_1_MOTOR_A, PWM_MOTOR_A);
  setPWMPrescaler(PIN_PWM_3_MOTOR_B, PWM_MOTOR_B);
  
  // Initial analog reads for A6/A7. Initial call returns bad value so call 3 times to get a good starting value from each pin
  ADC_Processing();
  // Wait for conversion
  while (!bit_is_clear(ADCSRA, ADSC));
  ADC_Processing();
  // Wait for conversion
  while (!bit_is_clear(ADCSRA, ADSC));
  ADC_Processing();
  
  setupReciever();
  attachServoPins();
}

//*********************************************************************************************************************
void loop()
{
  while (true)
  {
    // Loop forever without going back to Arduino core code
    if (getPacket())
    {
      outputChannels();
    }
    // Process ADC to asynchronously read A6 and A7 for telemetry analog values. Non-blocking read
    ADC_Processing();
  }
}
 
