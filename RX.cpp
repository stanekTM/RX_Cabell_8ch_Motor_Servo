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

#include "RX.h"
#include "Pins.h"
#include "My_RF24.h"
#include "My_nRF24L01.h"
#include "RSSI.h"
#include "Rx_Tx_Util.h"
#include <EEPROM.h>
#include "PWM_Frequency.h"
#include "ServoTimer2.h"
#include <DigitalIO.h> // v1.0.1


My_RF24 radio(PIN_CE, PIN_CSN);

My_RF24* Reciever = NULL;

uint8_t radioConfigRegisterForTX = 0;
uint8_t radioConfigRegisterForRX_IRQ_Masked = 0;
uint8_t radioConfigRegisterForRX_IRQ_On = 0;
  
uint16_t channelValues[CABELL_NUM_CHANNELS];

uint8_t currentModel = 0;
uint64_t radioPipeID;
uint64_t radioNormalRxPipeID;

const int currentModelEEPROMAddress = 0;
const int radioPipeEEPROMAddress = currentModelEEPROMAddress + sizeof(currentModel);
const int softRebindFlagEEPROMAddress = radioPipeEEPROMAddress + sizeof(radioNormalRxPipeID);
const int failSafeChannelValuesEEPROMAddress = softRebindFlagEEPROMAddress + sizeof(uint8_t); // uint8_t is the sifr of the rebind flag

uint16_t failSafeChannelValues[CABELL_NUM_CHANNELS];

bool bindMode = false; // when true send bind command to cause receiver to bind enter bind mode
bool failSafeMode = false;
bool failSafeNoPulses = false;
bool packetMissed = false;
uint32_t packetInterval = DEFAULT_PACKET_INTERVAL;

uint8_t radioChannel[CABELL_RADIO_CHANNELS];

volatile bool packetReady = false;

bool telemetryEnabled = false;
int16_t analogValue[2] = {0, 0};

uint16_t initialTelemetrySkipPackets = 0;

uint8_t currentChannel = CABELL_RADIO_MIN_CHANNEL_NUM; // Initializes the channel sequence

RSSI rssi;


// Create servo object -----------------------------------------------------------------------------------------------------
ServoTimer2 servo1, servo2, servo3, servo4, servo5, servo6;

void attachServoPins()
{
  servo1.attach(PIN_SERVO_1);
  servo2.attach(PIN_SERVO_2);
  servo3.attach(PIN_SERVO_3);
  servo4.attach(PIN_SERVO_4);
  servo5.attach(PIN_SERVO_5);
  servo6.attach(PIN_SERVO_6);
}

void outputServo()
{
  servo1.write(channelValues[CH_SERVO_1]);
  servo2.write(channelValues[CH_SERVO_2]);
  servo3.write(channelValues[CH_SERVO_3]);
  servo4.write(channelValues[CH_SERVO_4]);
  servo5.write(channelValues[CH_SERVO_5]);
  servo6.write(channelValues[CH_SERVO_6]);
}

//--------------------------------------------------------------------------------------------------------------------------
void outputPWM()
{
  setPWMPrescaler(PIN_PWM_1_MOTOR_A, PWM_MOTOR_A);
  setPWMPrescaler(PIN_PWM_3_MOTOR_B, PWM_MOTOR_B);
  
  int value_motorA = 0, value_motorB = 0;
  
  // Forward motorA
  if (channelValues[CH_MOTOR_A] > MID_CONTROL_VAL + DEAD_ZONE)
  {
    value_motorA = map(channelValues[CH_MOTOR_A], MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL, ACCELERATE_MOTOR_A, MAX_FORW_MOTOR_A);
    value_motorA = constrain(value_motorA, ACCELERATE_MOTOR_A, MAX_FORW_MOTOR_A);
    analogWrite(PIN_PWM_2_MOTOR_A, value_motorA); 
    digitalWrite(PIN_PWM_1_MOTOR_A, LOW);
  }
  // Back motorA
  else if (channelValues[CH_MOTOR_A] < MID_CONTROL_VAL - DEAD_ZONE)
  {
    value_motorA = map(channelValues[CH_MOTOR_A], MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL, ACCELERATE_MOTOR_A, MAX_BACK_MOTOR_A);
    value_motorA = constrain(value_motorA, ACCELERATE_MOTOR_A, MAX_BACK_MOTOR_A);
    analogWrite(PIN_PWM_1_MOTOR_A, value_motorA);
    digitalWrite(PIN_PWM_2_MOTOR_A, LOW);
  }
  else
  {
    analogWrite(PIN_PWM_1_MOTOR_A, BRAKE_MOTOR_A);
    analogWrite(PIN_PWM_2_MOTOR_A, BRAKE_MOTOR_A);
  }
  //Serial.println(value_motorA);
  
  
  // Forward motorB
  if (channelValues[CH_MOTOR_B] > MID_CONTROL_VAL + DEAD_ZONE)
  {
    value_motorB = map(channelValues[CH_MOTOR_B], MID_CONTROL_VAL + DEAD_ZONE, MAX_CONTROL_VAL, ACCELERATE_MOTOR_B, MAX_FORW_MOTOR_B);
    value_motorB = constrain(value_motorB, ACCELERATE_MOTOR_B, MAX_FORW_MOTOR_B);
    analogWrite(PIN_PWM_4_MOTOR_B, value_motorB);
    digitalWrite(PIN_PWM_3_MOTOR_B, LOW);
  }
  // Back motorB
  else if (channelValues[CH_MOTOR_B] < MID_CONTROL_VAL - DEAD_ZONE)
  {
    value_motorB = map(channelValues[CH_MOTOR_B], MID_CONTROL_VAL - DEAD_ZONE, MIN_CONTROL_VAL, ACCELERATE_MOTOR_B, MAX_BACK_MOTOR_B);
    value_motorB = constrain(value_motorB, ACCELERATE_MOTOR_B, MAX_BACK_MOTOR_B);
    analogWrite(PIN_PWM_3_MOTOR_B, value_motorB);
    digitalWrite(PIN_PWM_4_MOTOR_B, LOW);
  }
  else
  {
    analogWrite(PIN_PWM_3_MOTOR_B, BRAKE_MOTOR_B);
    analogWrite(PIN_PWM_4_MOTOR_B, BRAKE_MOTOR_B);
  }
  //Serial.println(value_motorB);
}

//--------------------------------------------------------------------------------------------------------------------------
void outputChannels()
{
  outputServo();
  outputPWM();
}

//--------------------------------------------------------------------------------------------------------------------------
// Reads ADC value then configures next conversion. Alternates between pins A6 and A7
// based on ADC Interrupt example from https://www.gammon.com.au/adc
void ADC_Processing()
{
  static byte adcPin = PIN_RX_BATT_A1;
  
  if (bit_is_clear(ADCSRA, ADSC))
  {
    analogValue[(adcPin == PIN_RX_BATT_A1) ? 0 : 1] = ADC;
    
    adcPin = (adcPin == PIN_RX_BATT_A2) ? PIN_RX_BATT_A1 : PIN_RX_BATT_A2; // Choose next pin to read
    
    ADCSRA  = bit (ADEN);                              // Turn ADC on
    ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2); // Pre-scaler of 128
    ADMUX   = bit (REFS0) | (adcPin & 0x07);           // AVcc and select input port
    ADCSRA |= bit (ADSC);                              // Start next conversion
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void setupReciever()
{
  uint8_t softRebindFlag;
  
  EEPROM.get(softRebindFlagEEPROMAddress, softRebindFlag);
  EEPROM.get(radioPipeEEPROMAddress, radioNormalRxPipeID);
  EEPROM.get(currentModelEEPROMAddress, currentModel);
  
  if (softRebindFlag == BOUND_WITH_FAILSAFE_NO_PULSES)
  {
    softRebindFlag = DO_NOT_SOFT_REBIND;
    failSafeNoPulses = true;
  }
  
  if ((digitalRead(PIN_BUTTON_BIND) == LOW) || (softRebindFlag != DO_NOT_SOFT_REBIND))
  {
    bindMode = true;
    radioPipeID = CABELL_BIND_RADIO_ADDR;
    digitalWrite(PIN_LED, HIGH);          // Turn on LED to indicate bind mode
    radioNormalRxPipeID = 0x01 << 43;     // This is a number bigger than the max possible pipe ID, which only uses 40 bits. This makes sure the bind routine writes to EEPROM
  }
  else
  {
    bindMode = false;
    radioPipeID = radioNormalRxPipeID;
  }
  
  getChannelSequence(radioChannel, CABELL_RADIO_CHANNELS, radioPipeID);
  
  radio.begin();
  Reciever = &radio;
  
  RADIO_IRQ_SET_INPUT;
  RADIO_IRQ_SET_PULLUP;
  
  initializeRadio(Reciever);
  
  setTelemetryPowerMode(CABELL_OPTION_MASK_MAX_POWER_OVERRIDE);
  
  Reciever->flush_rx();
  packetReady = false;
  
  outputFailSafeValues(false); // Initialize default values for output channels
  
  setNextRadioChannel(true);
  
  // Setup pin change interrupt
  cli();         // Switch interrupts off while messing with their settings
  PCICR  = 0x02; // Enable PCINT1 interrupt
  PCMSK1 = RADIO_IRQ_PIN_MASK;
  sei();
}

//--------------------------------------------------------------------------------------------------------------------------
  ISR(PCINT1_vect)
  {
    if (IS_RADIO_IRQ_on)
    {
      packetReady = true; // Pulled low when packet is received
    }
  }

//--------------------------------------------------------------------------------------------------------------------------
void setNextRadioChannel(bool missedPacket)
{
  Reciever->write_register(NRF_CONFIG, radioConfigRegisterForTX); // This is in place of stop listening to make the change to TX more quickly. Also sets all interrupts to mask
  Reciever->flush_rx();
  
  unsigned long expectedTransmitCompleteTime = 0;
  
  if (telemetryEnabled)
  {
    // Don't send the first 500 telemetry packets to avoid annoying warnings at startup
    if (initialTelemetrySkipPackets >= INITIAL_TELEMETRY_PACKETS_TO_SKIP)
    {
      expectedTransmitCompleteTime = sendTelemetryPacket();
    }
    else
    {
      initialTelemetrySkipPackets++;
    }
  }
  
  currentChannel = getNextChannel(radioChannel, CABELL_RADIO_CHANNELS, currentChannel);
  
  if (expectedTransmitCompleteTime != 0)
  {
    long waitTimeLeft = (long)(expectedTransmitCompleteTime - micros());
    
    // Wait here for the telemetry packet to finish transmitting
    if (waitTimeLeft > 0)
    {
      delayMicroseconds(waitTimeLeft);
    }
  }
  
  Reciever->write_register(NRF_CONFIG, radioConfigRegisterForTX); // This is in place of stop listening to make the change to TX more quickly. Also sets all interrupts to mask
  Reciever->flush_rx();
  Reciever->setChannel(currentChannel);
  Reciever->write_register(NRF_CONFIG, radioConfigRegisterForRX_IRQ_Masked);   // This is in place of stop listening to make the change to TX more quickly. Also sets all interrupts to mask
  Reciever->write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)); // This normally happens in StartListening
  
  packetReady = false;
  
  Reciever->write_register(NRF_CONFIG, radioConfigRegisterForRX_IRQ_On); // Turn on RX interrupt
}

//--------------------------------------------------------------------------------------------------------------------------
bool getPacket()
{
  static unsigned long lastPacketTime = 0;
  static bool inititalGoodPacketRecieved = false;
  static unsigned long nextAutomaticChannelSwitch = micros() + RESYNC_WAIT_MICROS;
  static unsigned long lastRadioPacketeRecievedTime = millis() - (long)RESYNC_TIME_OUT;
  static bool hoppingLockedIn = false;
  static uint16_t sequentialHitCount = 0;
  static uint16_t sequentialMissCount = 0;
  static bool powerOnLock = false;
  bool goodPacket_rx = false;
  bool strongSignal = false;
  
  // Wait for the radio to get a packet, or the timeout for the current radio channel occurs
  if (!packetReady)
  {
    // If timed out the packet was missed, go to the next channel
    if ((long)(micros() - nextAutomaticChannelSwitch) >= 0)
    {
      // Packet will be picked up on next loop through
      if (Reciever->available())
      {
        packetReady = true;
        rssi.secondaryHit();
      }
      else
      {
        packetMissed = true;
        sequentialHitCount = 0;
        sequentialMissCount++;
        rssi.miss();
        setNextRadioChannel(true); // True indicates that packet was missed
        
        // if a long time passed, increase timeout duration to re-sync with the TX
        if ((long)(nextAutomaticChannelSwitch - lastRadioPacketeRecievedTime) > ((long)RESYNC_TIME_OUT))
        {
          telemetryEnabled = false;
          hoppingLockedIn = false;
          sequentialHitCount = 0;
          sequentialMissCount = 0;
          packetInterval = DEFAULT_PACKET_INTERVAL;
          initialTelemetrySkipPackets = 0;
          nextAutomaticChannelSwitch += RESYNC_WAIT_MICROS;
        }
        else
        {
          nextAutomaticChannelSwitch += packetInterval;
        }
        
        checkFailsafeDisarmTimeout(lastPacketTime, inititalGoodPacketRecieved); // at each timeout, check for failsafe and disarm
      }
    }
  }
  else
  {
    lastRadioPacketeRecievedTime = micros(); // Use this time to calculate the next expected packet so when we miss packets we can change channels
    
    // Save this now while the value is latched. To save loop time only do this before initial lock as the initial lock process is the only thing that needs this
    if (!powerOnLock)
    {
      strongSignal = Reciever->testRPD();
    }
    
    goodPacket_rx = readAndProcessPacket();
    
    nextAutomaticChannelSwitch = lastRadioPacketeRecievedTime + packetInterval + INITIAL_PACKET_TIMEOUT_ADD; // Must ne set after readAndProcessPacket because packetInterval may get adjusted
    
    // During the initial power on lock process only consider the packet good if the signal was strong (better than -64 DBm)
    if (!powerOnLock && !strongSignal)
    {
      goodPacket_rx = false;
    }
    
    if (goodPacket_rx)
    {
      sequentialHitCount++;
      sequentialMissCount = 0;
      inititalGoodPacketRecieved = true;
      lastPacketTime = micros();
      failSafeMode = false;
      packetMissed = false;
      rssi.hit();
    }
    else
    {
      sequentialHitCount = 0;
      sequentialMissCount++;
      rssi.badPacket();
      packetMissed = true;
    }
  }
  
  // Below tries to detect when a false lock occurs and force a re-sync when detected in order to get a good lock.
  // This happens when while syncing the NRF24L01 successfully receives a packet on an adjacent channel to the current channel,
  // which locks the algorithm into the wrong place in the channel progression.  If the module continues to occasionally receive a 
  // packet like this, a re-sync does not happen, but the packet success rate is very poor.  This manifests as studdering control surfaces.
  // This seems to only happen when the TX is close to the RX as the strong signal swamps the RX module.
  // Checking for 5 good packets in a row to confirm lock, or 5 misses to force re-sync.
  
  // For the initial lock when the receiver is powered on, the rule is much more stringent to get a lock, and all packets are flagged bad until
  // the power on lock is obtained.  This is so that the model cannot be controlled until the initial lock is obtained.
  // This is only for the first lock.  A re-sync is less stringent so that if lock is lost for a model in flight then control is easier to re-establish.
  // Also, a re-sync that is not yet locked are considered good packets so that a weak re-sync can still control the model.
  
  if (!hoppingLockedIn)
  {
    if (!powerOnLock)
    {
      goodPacket_rx = false; // Always consider a bad packet until initial lock is obtained so no control signals are output
      
      // Ensure strong signal on all channels
      if (sequentialHitCount > (CABELL_RADIO_CHANNELS * 5))
      {
        powerOnLock = true;
        hoppingLockedIn = true;
      }
    }
    else if (sequentialHitCount > 5)
    {
      hoppingLockedIn = true;
    }
    
    // If more tnan 5 misses in a row assume it is a bad lock, or if after 100 packets there is still no lock
    if ((sequentialMissCount > 5) || (sequentialMissCount + sequentialHitCount > 100))
    {
      // If this happens then there is a bad lock and we should try to sync again.
      lastRadioPacketeRecievedTime = millis() - (long)RESYNC_TIME_OUT;
      nextAutomaticChannelSwitch = millis() + RESYNC_WAIT_MICROS;
      telemetryEnabled = false;
      setNextRadioChannel(true); // Getting the next channel ensures radios are flushed and properly waiting for a packet
    }
  }
  
  return goodPacket_rx;
}

//--------------------------------------------------------------------------------------------------------------------------
void checkFailsafeDisarmTimeout(unsigned long lastPacketTime, bool inititalGoodPacketRecieved)
{
  unsigned long holdMicros = micros();
  
  if ((long)(holdMicros - lastPacketTime) > ((long)RX_CONNECTION_TIMEOUT))
  {
    outputFailSafeValues(true);
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void outputFailSafeValues(bool callOutputChannels)
{
  loadFailSafeDefaultValues();
  
  for (uint8_t x = 0; x < CABELL_NUM_CHANNELS; x++)
  {
    channelValues[x] = failSafeChannelValues[x];
  }
  
  if (!failSafeMode)
  {
    failSafeMode = true;
  }
  
  if (callOutputChannels)
  {
    outputChannels();
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void unbindReciever()
{
  uint8_t value = 0xFF;
  
  // Reset all of flash memory to unbind receiver
  for (int x = 0; x < 1024; x++)
  {
    EEPROM.put(x, value);
  }
  
  outputFailSafeValues(true);

  bool ledState = false;
  
  // Flash LED forever indicating unbound
  while (true)
  {
    digitalWrite(PIN_LED, ledState);
    ledState = !ledState;
    delay(250); // Fast LED flash
  }  
}

//--------------------------------------------------------------------------------------------------------------------------
void bindReciever(uint8_t modelNum, uint16_t tempHoldValues[], CABELL_RxTxPacket_t :: RxMode_t RxMode)
{
  // new radio address is in channels 11 to 15
  uint64_t newRadioPipeID = (((uint64_t)(tempHoldValues[11] - 1000)) << 32) +
                            (((uint64_t)(tempHoldValues[12] - 1000)) << 24) +
                            (((uint64_t)(tempHoldValues[13] - 1000)) << 16) +
                            (((uint64_t)(tempHoldValues[14] - 1000)) << 8)  +
                            (((uint64_t)(tempHoldValues[15] - 1000))); // Address to use after binding
                            
  if ((modelNum != currentModel) || (radioNormalRxPipeID != newRadioPipeID))
  {
    EEPROM.put(currentModelEEPROMAddress, modelNum);
    radioNormalRxPipeID = newRadioPipeID;
    EEPROM.put(radioPipeEEPROMAddress, radioNormalRxPipeID);
    digitalWrite(PIN_LED, LOW); // Turn off LED to indicate successful bind
    
    if (RxMode == CABELL_RxTxPacket_t :: RxMode_t :: bindFalesafeNoPulse)
    {
      EEPROM.put(softRebindFlagEEPROMAddress, (uint8_t)BOUND_WITH_FAILSAFE_NO_PULSES);
    }
    else
    {
      EEPROM.put(softRebindFlagEEPROMAddress, (uint8_t)DO_NOT_SOFT_REBIND);
    }
    
    setFailSafeDefaultValues();
    outputFailSafeValues(true);

    bool ledState = false;
    
    // Flash LED forever indicating bound
    while (true)
    {
      digitalWrite(PIN_LED, ledState);
      ledState = !ledState;
      delay(2000); // Slow flash
    }
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void setFailSafeDefaultValues()
{
  uint16_t defaultFailSafeValues[CABELL_NUM_CHANNELS];
  
  for (int x = 0; x < CABELL_NUM_CHANNELS; x++)
  {
    defaultFailSafeValues[x] = MID_CONTROL_VAL;
  }
  
  setFailSafeValues(defaultFailSafeValues);
}

//--------------------------------------------------------------------------------------------------------------------------
void loadFailSafeDefaultValues()
{
  EEPROM.get(failSafeChannelValuesEEPROMAddress, failSafeChannelValues);
  
  for (int x = 0; x < CABELL_NUM_CHANNELS; x++)
  {
    // Make sure failsafe values are valid
    if (failSafeChannelValues[x] < MIN_CONTROL_VAL || failSafeChannelValues[x] > MAX_CONTROL_VAL)
    {
      failSafeChannelValues[x] = MID_CONTROL_VAL;
    }
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void setFailSafeValues(uint16_t newFailsafeValues[])
{
  for (int x = 0; x < CABELL_NUM_CHANNELS; x++)
  {
    failSafeChannelValues[x] = newFailsafeValues[x];
  }
  
  EEPROM.put(failSafeChannelValuesEEPROMAddress, failSafeChannelValues);
}

//--------------------------------------------------------------------------------------------------------------------------
bool validateChecksum(CABELL_RxTxPacket_t const& packet, uint8_t maxPayloadValueIndex)
{
  // Caculate checksum and validate
  uint16_t packetSum = packet.modelNum + packet.option + packet.RxMode + packet.reserved;
  
  for (int x = 0; x < maxPayloadValueIndex; x++)
  {
    packetSum = packetSum +  packet.payloadValue[x];
  }
  
  if (packetSum != ((((uint16_t)packet.checkSum_MSB) << 8) + (uint16_t)packet.checkSum_LSB))
  {
    return false; // Don't take packet if checksum bad
  }
  else
  {
    return true;
  }
}

//--------------------------------------------------------------------------------------------------------------------------
// Only call when a packet is available on the radio
bool readAndProcessPacket()
{
  CABELL_RxTxPacket_t RxPacket;
  
  Reciever->read(&RxPacket, sizeof(RxPacket));
  
  int tx_channel = RxPacket.reserved & CABELL_RESERVED_MASK_CHANNEL;
  
  if (tx_channel != 0)
  {
    currentChannel = tx_channel;
  }
  
  setNextRadioChannel(false); // Also sends telemetry if in telemetry mode. Doing this as soon as possible to keep timing as tight as possible
                              // False indicates that packet was not missed
                              
  // Remove 8th bit from RxMode because this is a toggle bit that is not included in the checksum
  // This toggle with each xmit so consecutive payloads are not identical. This is a work around for a reported bug in clone NRF24L01 chips that mis-took this case for a re-transmit of the same packet.
  uint8_t* p = reinterpret_cast<uint8_t*>(&RxPacket.RxMode);
  *p &= 0x7F;  // ensure 8th bit is not set. This bit is not included in checksum
  
  // putting this after setNextRadioChannel will lag by one telemetry packet, but by doing this the telemetry can be sent sooner, improving the timing
  telemetryEnabled = (RxPacket.RxMode == CABELL_RxTxPacket_t :: RxMode_t :: normalWithTelemetry) ? true : false;
  
  bool packet_rx = false;
  uint16_t tempHoldValues[CABELL_NUM_CHANNELS];
  uint8_t channelReduction = constrain((RxPacket.option & CABELL_OPTION_MASK_CHANNEL_REDUCTION), 0, CABELL_NUM_CHANNELS - CABELL_MIN_CHANNELS); // Must be at least 4 channels, so cap at 12
  uint8_t packetSize = sizeof(RxPacket) - ((((channelReduction - (channelReduction % 2)) / 2)) * 3); // reduce 3 bytes per 2 channels, but not last channel if it is odd
  uint8_t maxPayloadValueIndex = sizeof(RxPacket.payloadValue) - (sizeof(RxPacket) - packetSize);
  uint8_t channelsRecieved = CABELL_NUM_CHANNELS - channelReduction;
  
  // putting this after setNextRadioChannel will lag by one telemetry packet, but by doing this the telemetry can be sent sooner, improving the timing
  if (telemetryEnabled)
  {
    setTelemetryPowerMode(RxPacket.option);
    packetInterval = DEFAULT_PACKET_INTERVAL + (constrain(((int16_t)channelsRecieved - (int16_t)6), (int16_t)0, (int16_t)10) * (int16_t)100); // increase packet period by 100 us for each channel over 6
  }
  else
  {
    packetInterval = DEFAULT_PACKET_INTERVAL;
  }
  
  packet_rx = validateChecksum(RxPacket, maxPayloadValueIndex);
  
  if (packet_rx)
  {
    packet_rx = decodeChannelValues(RxPacket, channelsRecieved, tempHoldValues);
    
    // If bind or unbind happens, this will never return
    packet_rx = processRxMode(RxPacket.RxMode, RxPacket.modelNum, tempHoldValues);
  }
  
  // if packet is good, copy the channel values
  if (packet_rx)
  {
    for (int b = 0 ; b < CABELL_NUM_CHANNELS; b++)
    {
      channelValues[b] = (b < channelsRecieved) ? tempHoldValues[b] : MID_CONTROL_VAL; // use the mid value for channels not received
    }
  }
  
  return packet_rx;
}

//--------------------------------------------------------------------------------------------------------------------------
bool processRxMode(uint8_t RxMode, uint8_t modelNum, uint16_t tempHoldValues[])
{
  static bool failSafeValuesHaveBeenSet = false;
  bool packet_rx = true;
  
  // fail safe settings can come in on a failsafe packet, but also use a normal packed if bind mode button is pressed after start up
  if (failSafeButtonHeld())
  {
    if (RxMode == CABELL_RxTxPacket_t :: RxMode_t :: normal || RxMode == CABELL_RxTxPacket_t :: RxMode_t :: normalWithTelemetry)
    {
      RxMode = CABELL_RxTxPacket_t :: RxMode_t :: setFailSafe;
    }
  }
  
  switch (RxMode)
  {
    case CABELL_RxTxPacket_t :: RxMode_t :: bindFalesafeNoPulse :
    case CABELL_RxTxPacket_t :: RxMode_t :: bind :
    
    if (bindMode)
    {
      bindReciever(modelNum, tempHoldValues, RxMode);
    }
    else
    {
      packet_rx = false;
    }
    break;
    
    case CABELL_RxTxPacket_t :: RxMode_t :: setFailSafe :
    
    if (modelNum == currentModel)
    {
      digitalWrite(PIN_LED, HIGH);
      
      // only set the values first time through
      if (!failSafeValuesHaveBeenSet)
      {
        failSafeValuesHaveBeenSet = true;
        setFailSafeValues(tempHoldValues);
      }
    }
    else
    {
      packet_rx = false;
    }
    break;
    
    case CABELL_RxTxPacket_t :: RxMode_t :: normalWithTelemetry :
    case CABELL_RxTxPacket_t :: RxMode_t :: normal :
    
    if (modelNum == currentModel)
    {
      digitalWrite(PIN_LED, LOW);
      failSafeValuesHaveBeenSet = false; // Reset when not in setFailSafe mode so next time failsafe is to be set it will take
    }
    else
    {
      packet_rx = false;
    }
    break;
    
    case CABELL_RxTxPacket_t :: RxMode_t :: unBind :
    
    if (modelNum == currentModel)
    {
      unbindReciever();
    }
    else
    {
      packet_rx = false;
    }
    break;
  }
  
  return packet_rx;
}

//--------------------------------------------------------------------------------------------------------------------------
bool decodeChannelValues(CABELL_RxTxPacket_t const& RxPacket, uint8_t channelsRecieved, uint16_t tempHoldValues[])
{
  bool packet_rx = true;
  int payloadIndex = 0;
  
  // decode the 12 bit numbers to temp array
  for (int b = 0; (b < channelsRecieved); b++)
  {
    tempHoldValues[b]  = RxPacket.payloadValue[payloadIndex];
    payloadIndex++;
    tempHoldValues[b] |= ((uint16_t)RxPacket.payloadValue[payloadIndex]) << 8;
    
    //channel number is ODD
    if (b % 2)
    {
      tempHoldValues[b] = tempHoldValues[b] >> 4;
      payloadIndex++;
    }
    //channel number is EVEN
    else
    {
      tempHoldValues[b] &= 0x0FFF;
    }
    
    if ((tempHoldValues[b] > MAX_CONTROL_VAL) || (tempHoldValues[b] < MIN_CONTROL_VAL))
    {
      packet_rx = false; // throw out entire packet if any value out of range
    }
  }
  
  return packet_rx;
}

//--------------------------------------------------------------------------------------------------------------------------
unsigned long sendTelemetryPacket()
{
  static int8_t packetCounter = 0; // this is only used for toggling bit
  uint8_t sendPacket[4] = {CABELL_RxTxPacket_t :: RxMode_t :: telemetryResponse};
 
  packetCounter++;
  sendPacket[0] &= 0x7F;               // clear 8th bit
  sendPacket[0] |= packetCounter << 7; // This causes the 8th bit of the first byte to toggle with each xmit so consecutive payloads are not identical. This is a work around for a reported bug in clone NRF24L01 chips that mis-took this case for a re-transmit of the same packet.
  sendPacket[1]  = rssi.getRSSI();
  sendPacket[2]  = analogValue[0] / 4; // Send a 8 bit value (0 to 255) of the analog input. Can be used for LiPo voltage or other analog input for telemetry
  sendPacket[3]  = analogValue[1] / 4; // Send a 8 bit value (0 to 255) of the analog input. Can be used for LiPo voltage or other analog input for telemetry
  
  uint8_t packetSize = sizeof(sendPacket);
  
  Reciever->startFastWrite(&sendPacket[0], packetSize, 0);
  
  // calculate transmit time based on packet size and data rate of 1MB per sec
  // This is done because polling the status register during xmit to see when xmit is done causes issues sometimes.
  // bits = packet-size * 8  +  73 bits overhead
  // at 250 kbps per sec, one bit is 4 uS
  // then add 140 uS which is 130 uS to begin the xmit and 10 uS fudge factor
  // Add this to micros() to return when the transmit is expected to be complete
  return micros() + (((((unsigned long)packetSize * 8ul)  +  73ul) * 4ul) + 140ul);
}

//--------------------------------------------------------------------------------------------------------------------------
// use the bind button because bind mode is only checked at startup. Once RX is started and not in bind mode it is the set failsafe button
bool failSafeButtonHeld()
{
  static unsigned long heldTriggerTime = 0;
  
  // invert because pin is pulled up so low means pressed
  if (!bindMode && !digitalRead(PIN_BUTTON_BIND))
  {
    if (heldTriggerTime == 0)
    {
      heldTriggerTime = micros() + 1000000ul; // Held state achieved after button is pressed for 1 second
    }
    
    if ((long)(micros() - heldTriggerTime) >= 0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  
  heldTriggerTime = 0;
  return false;
}

//--------------------------------------------------------------------------------------------------------------------------
void setTelemetryPowerMode(uint8_t option)
{
  static uint8_t prevPower = RF24_PA_MIN;
  uint8_t newPower;
  
  // Set transmit power to max or high based on the option byte in the incoming packet.
  // This should set the power the same as the transmitter module
  if ((option & CABELL_OPTION_MASK_MAX_POWER_OVERRIDE) == 0)
  {
    newPower = RF24_PA_HIGH;
  }
  else
  {
    newPower = RF24_PA_MAX;
  }
  
  if (newPower != prevPower)
  {
    Reciever->setPALevel(newPower);
    prevPower = newPower;
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void initializeRadio(My_RF24* radioPointer)
{
  radioPointer->maskIRQ(true, true, true);     // Mask all interrupts. RX interrupt (the only one we use) gets turned on after channel change
  radioPointer->enableDynamicPayloads();
  radioPointer->setDataRate(RF24_250KBPS);
  radioPointer->setChannel(0);                 // start out on a channel we don't use so we don't start receiving packets yet. It will get changed when the looping starts
  radioPointer->setAutoAck(0);
  radioPointer->openWritingPipe(~radioPipeID); // Invert bits for writing pipe so that telemetry packets transmit with a different pipe ID.
  radioPointer->openReadingPipe(1, radioPipeID);
  radioPointer->startListening();
  radioPointer->csDelay = 0; // Can be reduced to 0 because we use interrupts and timing instead of polling through SPI
  radioPointer->txDelay = 0; // Timing works out so a delay is not needed
  
  // Stop listening to set up module for writing then take a copy of the config register so we can change to write mode more quickly when sending telemetry packets
  radioPointer->stopListening();
  radioConfigRegisterForTX = radioPointer->read_register(NRF_CONFIG); // This saves the config register state with all interrupts masked and in TX mode. Used to switch quickly to TX mode for telemetry
  radioPointer->startListening();
  radioConfigRegisterForRX_IRQ_Masked = radioPointer->read_register(NRF_CONFIG); // This saves the config register state with all interrupts masked and in RX mode. Used to switch radio quickly to RX after channel change
  radioPointer->maskIRQ(true, true, false);
  radioConfigRegisterForRX_IRQ_On = radioPointer->read_register(NRF_CONFIG);     // This saves the config register state with Read Interrupt ON and in RX mode. Used to switch radio quickly to RX after channel change
  radioPointer->maskIRQ(true, true, true);
}
 
