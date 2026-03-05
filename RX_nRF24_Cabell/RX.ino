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

//*********************************************************************************************************************
// Output RC channels
//*********************************************************************************************************************
void output_rc_channels()
{
  servo_control();
  motor_control();
}

//*********************************************************************************************************************
// Battery voltage monitoring
//*********************************************************************************************************************
void batt_monitoring()
{
  if (millis() - adc_time > 1000) // Battery reading delay
  {
    adc_time = millis();
    
    analogValue[0] = map(analogRead(PIN_RX_BATT_A1), 0, 1023, 0, 255);
    analogValue[1] = map(analogRead(PIN_RX_BATT_A2), 0, 1023, 0, 255);
  }
}

//*********************************************************************************************************************
// Radio setup
//*********************************************************************************************************************
void radio_setup()
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
    radioPipeID = BIND_RF_ADDR;
    digitalWrite(PIN_LED, HIGH);      // Turn on LED to indicate bind mode
    radioNormalRxPipeID = 0x01 << 43; // This is a number bigger than the max possible pipe ID, which only uses 40 bits. This makes sure the bind routine writes to EEPROM
  }
  else
  {
    bindMode = false;
    radioPipeID = radioNormalRxPipeID;
  }
  
  getChannelSequence(radioChannel, RF_CHANNELS, radioPipeID);
  
  radio.begin();
  
  RADIO_IRQ_SET_INPUT;
  RADIO_IRQ_SET_PULLUP;
  
  radio_init();
  
  setTelemetryPowerMode(OPTION_MASK_MAX_RF_POWER_OVERRIDE);
  
  radio.flush_rx();
  packetReady = false;
  
  outputFailSafeValues(false); // Initialize default values for output channels
  
  setNextRadioChannel(true);
  
  // Setup pin change interrupt
  cli();         // Switch interrupts off while messing with their settings
  PCICR  = 0x02; // Enable PCINT1 interrupt
  PCMSK1 = RADIO_IRQ_PIN_MASK;
  sei();
}

//*********************************************************************************************************************
// Set next radio channel
//*********************************************************************************************************************
void setNextRadioChannel(bool)
{
  radio.write_register(NRF_CONFIG, radioConfigRegisterForTX); // This is in place of stop listening to make the change to TX more quickly. Also sets all interrupts to mask
  radio.flush_rx();
  
  uint32_t expectedTransmitCompleteTime = 0;
  
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
  
  currentChannel = getNextChannel(radioChannel, RF_CHANNELS, currentChannel);
  
  if (expectedTransmitCompleteTime != 0)
  {
    uint32_t waitTimeLeft = (uint32_t)(expectedTransmitCompleteTime - micros());
    
    // Wait here for the telemetry packet to finish transmitting
    if (waitTimeLeft > 0)
    {
      delayMicroseconds(waitTimeLeft);
    }
  }
  
  radio.write_register(NRF_CONFIG, radioConfigRegisterForTX); // This is in place of stop listening to make the change to TX more quickly. Also sets all interrupts to mask
  radio.flush_rx();
  radio.setChannel(currentChannel);
  radio.write_register(NRF_CONFIG, radioConfigRegisterForRX_IRQ_Masked);   // This is in place of stop listening to make the change to TX more quickly. Also sets all interrupts to mask
  radio.write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)); // This normally happens in StartListening
  
  packetReady = false;
  
  radio.write_register(NRF_CONFIG, radioConfigRegisterForRX_IRQ_On); // Turn on RX interrupt
}

//*********************************************************************************************************************
// Get packet
//*********************************************************************************************************************
bool getPacket()
{
  static uint32_t lastPacketTime = 0;
  static bool inititalGoodPacketRecieved = false;
  static uint32_t nextAutomaticChannelSwitch = micros() + RESYNC_WAIT_MICROS;
  static uint32_t lastRadioPacketeRecievedTime = millis() - (uint32_t)RESYNC_TIME_OUT;
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
    if ((int32_t)(micros() - nextAutomaticChannelSwitch) >= 0)
    {
      // Packet will be picked up on next loop through
      if (radio.available())
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
        
        // If a long time passed, increase timeout duration to re-sync with the TX
        if ((uint32_t)(nextAutomaticChannelSwitch - lastRadioPacketeRecievedTime) > ((uint32_t)RESYNC_TIME_OUT))
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
        
        checkFailsafeDisarmTimeout(lastPacketTime, inititalGoodPacketRecieved); // At each timeout, check for failsafe and disarm
      }
    }
  }
  else
  {
    lastRadioPacketeRecievedTime = micros(); // Use this time to calculate the next expected packet so when we miss packets we can change channels
    
    // Save this now while the value is latched. To save loop time only do this before initial lock as the initial lock process is the only thing that needs this
    if (!powerOnLock)
    {
      strongSignal = radio.testRPD();
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
      if (sequentialHitCount > (RF_CHANNELS * 5))
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
      lastRadioPacketeRecievedTime = millis() - (uint32_t)RESYNC_TIME_OUT;
      nextAutomaticChannelSwitch = millis() + RESYNC_WAIT_MICROS;
      telemetryEnabled = false;
      setNextRadioChannel(true); // Getting the next channel ensures radios are flushed and properly waiting for a packet
    }
  }
  
  return goodPacket_rx;
}

//*********************************************************************************************************************
// Check fail safe disarm timeout
//*********************************************************************************************************************
void checkFailsafeDisarmTimeout(uint32_t lastPacketTime, bool)
{
  uint32_t holdMicros = micros();
  
  if ((uint32_t)(holdMicros - lastPacketTime) > ((uint32_t)RX_CONNECTION_TIMEOUT))
  {
    outputFailSafeValues(true);
  }
}

//*********************************************************************************************************************
// Output fail safe values
//*********************************************************************************************************************
void outputFailSafeValues(bool callOutputChannels)
{
  loadFailSafeDefaultValues();
  
  for (uint8_t i = 0; i < RC_CHANNELS; i++)
  {
    rc_channel[i] = failSafeChannelValues[i];
  }
  
  if (!failSafeMode)
  {
    failSafeMode = true;
  }
  
  if (callOutputChannels)
  {
    output_rc_channels();
  }
}

//*********************************************************************************************************************
// Unbind reciever
//*********************************************************************************************************************
void unbindReciever()
{
  uint8_t value = 0xFF;
  
  // Reset all of flash memory to unbind receiver
  for (uint16_t i = 0; i < 1024; i++)
  {
    EEPROM.put(i, value);
  }
  
  outputFailSafeValues(true);
  
  // Flash LED forever indicating unbound
  bool ledState = false;
  while (true)
  {
    digitalWrite(PIN_LED, ledState);
    ledState = !ledState;
    delay(200);
  }  
}

//*********************************************************************************************************************
// Bind reciever
//*********************************************************************************************************************
void bindReciever(uint8_t modelNum, uint16_t tempHoldValues[], uint8_t RxMode)
{
  // New radio address is in channels 11 to 15
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
    
    if (RxMode == RxTxPacket_t :: RxMode_t :: bindFalesafeNoPulse)
    {
      EEPROM.put(softRebindFlagEEPROMAddress, (uint8_t)BOUND_WITH_FAILSAFE_NO_PULSES);
    }
    else
    {
      EEPROM.put(softRebindFlagEEPROMAddress, (uint8_t)DO_NOT_SOFT_REBIND);
    }
    
    setFailSafeDefaultValues();
    outputFailSafeValues(true);
    
    // Flash LED forever indicating bound
    bool ledState = false;
    while (true)
    {
      digitalWrite(PIN_LED, ledState);
      ledState = !ledState;
      delay(900);
    }
  }
}

//*********************************************************************************************************************
// Set fail safe default values
//*********************************************************************************************************************
void setFailSafeDefaultValues()
{
  uint16_t defaultFailSafeValues[RC_CHANNELS];
  
  for (uint8_t i = 0; i < RC_CHANNELS; i++)
  {
    defaultFailSafeValues[i] = MID_CONTROL_VAL;
  }
  
  setFailSafeValues(defaultFailSafeValues);
}

//*********************************************************************************************************************
// Load fail safe default values
//*********************************************************************************************************************
void loadFailSafeDefaultValues()
{
  EEPROM.get(failSafeChannelValuesEEPROMAddress, failSafeChannelValues);
  
  for (uint8_t i = 0; i < RC_CHANNELS; i++)
  {
    // Make sure failsafe values are valid
    if (failSafeChannelValues[i] < MIN_CONTROL_VAL || failSafeChannelValues[i] > MAX_CONTROL_VAL)
    {
      failSafeChannelValues[i] = MID_CONTROL_VAL;
    }
  }
}

//*********************************************************************************************************************
// Set fail safe values
//*********************************************************************************************************************
void setFailSafeValues(uint16_t newFailsafeValues[])
{
  for (uint8_t i = 0; i < RC_CHANNELS; i++)
  {
    failSafeChannelValues[i] = newFailsafeValues[i];
  }
  
  EEPROM.put(failSafeChannelValuesEEPROMAddress, failSafeChannelValues);
}

//*********************************************************************************************************************
// Validate checksum
//*********************************************************************************************************************
bool validateChecksum(const RxTxPacket_t &rc_packet, uint8_t max_rc_packet_size)
{
  // Caculate checksum and validate
  uint16_t packetSum = rc_packet.modelNum + rc_packet.option + rc_packet.RxMode + rc_packet.reserved;
  
  for (uint8_t i = 0; i < max_rc_packet_size; i++)
  {
    packetSum = packetSum + rc_packet.payloadValue[i];
  }
  
  if (packetSum != ((((uint16_t)rc_packet.checkSum_MSB) << 8) + (uint16_t)rc_packet.checkSum_LSB))
  {
    return false; // Don't take packet if checksum bad
  }
  else
  {
    return true;
  }
}

//*********************************************************************************************************************
// Read and process packet. Only call when a packet is available on the radio
//*********************************************************************************************************************
bool readAndProcessPacket()
{
  RxTxPacket_t rc_packet;
  
  radio.read(&rc_packet, sizeof(rc_packet));
  
  uint8_t tx_channel = rc_packet.reserved & RESERVED_MASK_RF_CHANNEL;
  
  if (tx_channel != 0)
  {
    currentChannel = tx_channel;
  }

  // Also sends telemetry if in telemetry mode. Doing this as soon as possible to keep timing as tight as possible
  // False indicates that packet was not missed
  setNextRadioChannel(false);
  
  // Remove 8th bit from RxMode because this is a toggle bit that is not included in the checksum
  // This toggle with each xmit so consecutive payloads are not identical. This is a work around for a reported bug in clone NRF24L01 chips that mis-took this case for a re-transmit of the same packet.
  uint8_t* p = reinterpret_cast<uint8_t*>(&rc_packet.RxMode);
  *p &= 0x7F; // Ensure 8th bit is not set. This bit is not included in checksum
  
  // Putting this after setNextRadioChannel will lag by one telemetry packet, but by doing this the telemetry can be sent sooner, improving the timing
  telemetryEnabled = (rc_packet.RxMode == RxTxPacket_t :: RxMode_t :: normalWithTelemetry) ? true : false;
  
  uint8_t channelReduction = constrain((rc_packet.option & OPTION_MASK_RF_CHANNEL_REDUCTION), 0, RC_CHANNELS - MIN_RC_CHANNELS); // Must be at least 4 channels, so cap at 12
  uint8_t rc_packet_size = sizeof(rc_packet) - ((((channelReduction - (channelReduction % 2)) / 2)) * 3); // Reduce 3 bytes per 2 channels, but not last channel if it is odd
  uint8_t max_rc_packet_size = sizeof(rc_packet.payloadValue) - (sizeof(rc_packet) - rc_packet_size);
  uint8_t channelsRecieved = RC_CHANNELS - channelReduction;
  
  // Putting this after setNextRadioChannel will lag by one telemetry packet, but by doing this the telemetry can be sent sooner, improving the timing
  if (telemetryEnabled)
  {
    setTelemetryPowerMode(rc_packet.option);
    packetInterval = DEFAULT_PACKET_INTERVAL + (constrain(((uint8_t)channelsRecieved - (uint8_t)6), (uint8_t)0, (uint8_t)10) * (uint8_t)100); // Increase packet period by 100 us for each channel over 6
  }
  else
  {
    packetInterval = DEFAULT_PACKET_INTERVAL;
  }
  
  bool packet_rx = false;
  
  packet_rx = validateChecksum(rc_packet, max_rc_packet_size);
  
  uint16_t tempHoldValues[RC_CHANNELS];
  
  if (packet_rx)
  {
    packet_rx = decodeChannelValues(rc_packet, channelsRecieved, tempHoldValues);
    
    // If bind or unbind happens, this will never return
    packet_rx = processRxMode(rc_packet.RxMode, rc_packet.modelNum, tempHoldValues);
  }
  
  // If packet is good, copy the channel values
  if (packet_rx)
  {
    for (uint8_t i = 0; i < RC_CHANNELS; i++)
    {
      rc_channel[i] = (i < channelsRecieved) ? tempHoldValues[i] : MID_CONTROL_VAL; // Use the mid value for channels not received
    }
  }
  
  return packet_rx;
}

//*********************************************************************************************************************
// Process RX mode
//*********************************************************************************************************************
bool processRxMode(uint8_t RxMode, uint8_t modelNum, uint16_t tempHoldValues[])
{
  static bool failSafeValuesHaveBeenSet = false;
  bool packet_rx = true;
  
  // Fail safe settings can come in on a failsafe packet, but also use a normal packed if bind mode button is pressed after start up
  if (failSafeButtonHeld())
  {
    if (RxMode == RxTxPacket_t :: RxMode_t :: normal || RxMode == RxTxPacket_t :: RxMode_t :: normalWithTelemetry)
    {
      RxMode = RxTxPacket_t :: RxMode_t :: setFailSafe;
    }
  }
  
  switch (RxMode)
  {
    case RxTxPacket_t :: RxMode_t :: bindFalesafeNoPulse :
    case RxTxPacket_t :: RxMode_t :: bind :
      if (bindMode)
      {
        bindReciever(modelNum, tempHoldValues, RxMode);
      }
      else
      {
        packet_rx = false;
      }
    break;
    
    case RxTxPacket_t :: RxMode_t :: setFailSafe :
      if (modelNum == currentModel)
      {
        digitalWrite(PIN_LED, HIGH);
        
        // Only set the values first time through
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
    
    case RxTxPacket_t :: RxMode_t :: normalWithTelemetry :
    case RxTxPacket_t :: RxMode_t :: normal :
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
    
    case RxTxPacket_t :: RxMode_t :: unBind :
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

//*********************************************************************************************************************
// Decode channel values
//*********************************************************************************************************************
bool decodeChannelValues(const RxTxPacket_t &rc_packet, uint8_t channelsRecieved, uint16_t tempHoldValues[])
{
  bool packet_rx = true;
  uint8_t payloadIndex = 0;
  
  // Decode the 12 bit numbers to temp array
  for (uint8_t i = 0; i < channelsRecieved; i++)
  {
    tempHoldValues[i]  = rc_packet.payloadValue[payloadIndex];
    payloadIndex++;
    tempHoldValues[i] |= ((uint16_t)rc_packet.payloadValue[payloadIndex]) << 8;
    
    // Channel number is ODD
    if (i % 2)
    {
      tempHoldValues[i] = tempHoldValues[i] >> 4;
      payloadIndex++;
    }
    // Channel number is EVEN
    else
    {
      tempHoldValues[i] &= 0x0FFF;
    }
    
    if ((tempHoldValues[i] > MAX_CONTROL_VAL) || (tempHoldValues[i] < MIN_CONTROL_VAL))
    {
      packet_rx = false; // Throw out entire packet if any value out of range
    }
  }
  
  return packet_rx;
}

//*********************************************************************************************************************
// Send telemetry packet
//*********************************************************************************************************************
uint32_t sendTelemetryPacket()
{
  static int8_t packet_counter = 0; // This is only used for toggling bit
  uint8_t telemetry_packet[4] = {RxTxPacket_t :: RxMode_t :: telemetryResponse};
  
  packet_counter++;
  
  telemetry_packet[0] &= 0x7F;               // Clear 8th bit
  telemetry_packet[0] |= packet_counter << 7; // This causes the 8th bit of the first byte to toggle with each xmit so consecutive payloads are not identical. This is a work around for a reported bug in clone NRF24L01 chips that mis-took this case for a re-transmit of the same packet.
  telemetry_packet[1]  = rssi.getRSSI();
  telemetry_packet[2]  = analogValue[0]; // Send a 8 bit value (0 to 255) of the analog input. Can be used for LiPo voltage or other analog input for telemetry
  telemetry_packet[3]  = analogValue[1]; // Send a 8 bit value (0 to 255) of the analog input. Can be used for LiPo voltage or other analog input for telemetry
  
  uint8_t telemetry_packet_size = sizeof(telemetry_packet);
  
  radio.startFastWrite(&telemetry_packet[0], telemetry_packet_size, 0);
  
  // Calculate transmit time based on packet size and data rate of 1MB per sec
  // This is done because polling the status register during xmit to see when xmit is done causes issues sometimes.
  // bits = packet-size * 8  +  73 bits overhead
  // at 250 kbps per sec, one bit is 4 uS
  // then add 140 uS which is 130 uS to begin the xmit and 10 uS fudge factor
  // Add this to micros() to return when the transmit is expected to be complete
  return micros() + (((((uint32_t)telemetry_packet_size * 8ul)  +  73ul) * 4ul) + 140ul);
}

//*********************************************************************************************************************
// Fail safe button held. Use the bind button because bind mode is only checked at startup. Once RX is started and not
// in bind mode it is the set failsafe button
//*********************************************************************************************************************
bool failSafeButtonHeld()
{
  static uint32_t heldTriggerTime = 0;
  
  // Invert because pin is pulled up so low means pressed
  if (!bindMode && !digitalRead(PIN_BUTTON_BIND))
  {
    if (heldTriggerTime == 0)
    {
      heldTriggerTime = micros() + 1000000ul; // Held state achieved after button is pressed for 1 second
    }
    
    if ((int32_t)(micros() - heldTriggerTime) >= 0)
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

//*********************************************************************************************************************
// Set telemetry power mode
//*********************************************************************************************************************
void setTelemetryPowerMode(uint8_t option)
{
  static uint8_t prevPower = RF24_PA_MIN;
  uint8_t newPower;
  
  // Set transmit power to max or high based on the option byte in the incoming packet.
  // This should set the power the same as the transmitter module
  if ((option & OPTION_MASK_MAX_RF_POWER_OVERRIDE) == 0)
  {
    newPower = RF24_PA_HIGH;
  }
  else
  {
    newPower = RF24_PA_MAX;
  }
  
  if (newPower != prevPower)
  {
    radio.setPALevel(newPower);
    prevPower = newPower;
  }
}

//*********************************************************************************************************************
// Radio init
//*********************************************************************************************************************
void radio_init()
{
  radio.maskIRQ(true, true, true);     // Mask all interrupts. RX interrupt (the only one we use) gets turned on after channel change
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(0);                 // Start out on a channel we don't use so we don't start receiving packets yet. It will get changed when the looping starts
  radio.setAutoAck(0);
  radio.openWritingPipe(~radioPipeID); // Invert bits for writing pipe so that telemetry packets transmit with a different pipe ID.
  radio.openReadingPipe(1, radioPipeID);
  radio.startListening();
  radio.csDelay = 0; // Can be reduced to 0 because we use interrupts and timing instead of polling through SPI
  radio.txDelay = 0; // Timing works out so a delay is not needed
  
  // Stop listening to set up module for writing then take a copy of the config register so we can change to write mode more quickly when sending telemetry packets
  radio.stopListening();
  radioConfigRegisterForTX = radio.read_register(NRF_CONFIG); // This saves the config register state with all interrupts masked and in TX mode. Used to switch quickly to TX mode for telemetry
  radio.startListening();
  radioConfigRegisterForRX_IRQ_Masked = radio.read_register(NRF_CONFIG); // This saves the config register state with all interrupts masked and in RX mode. Used to switch radio quickly to RX after channel change
  radio.maskIRQ(true, true, false);
  radioConfigRegisterForRX_IRQ_On = radio.read_register(NRF_CONFIG);     // This saves the config register state with Read Interrupt ON and in RX mode. Used to switch radio quickly to RX after channel change
  radio.maskIRQ(true, true, true);
}

//*********************************************************************************************************************
// ISR
//*********************************************************************************************************************
ISR(PCINT1_vect)
{
  if (IS_RADIO_IRQ_on)
  {
    packetReady = true; // Pulled low when packet is received
  }
}
 
