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

#ifndef __RX_h__
#define __RX_h__

#include <Arduino.h>
#include <stdint.h>
#include "My_RF24.h"
#include "Pins.h"
#include "RSSI.h"

My_RF24 radio(PIN_CE, PIN_CSN);

RSSI rssi;

#define RC_CHANNELS      16 // The maximum number of RC channels that can be sent in one packet
#define MIN_RC_CHANNELS  4  // The minimum number of RC channels that must be included in a packet, the number of channels cannot be reduced any further than this
#define PAYLOAD_BYTES    24 // 12 bits per value * 16 channels

uint16_t rc_channel[RC_CHANNELS];

#define MIN_CONTROL_VAL  1000
#define MID_CONTROL_VAL  1500
#define MAX_CONTROL_VAL  2000

#define BIND_RF_ADDR      0xA4B7C123F7LL

#define RF_CHANNELS       9 // This is 1/5 of the total number of radio channels used for FHSS
#define MIN_RF_CHANNEL    3 // Channel 0 is right on the boarder of allowed frequency range, so move up to avoid bleeding over

#define RESERVED_MASK_RF_CHANNEL  0x3F

#define OPTION_MASK_MAX_RF_POWER_OVERRIDE  0x40
#define OPTION_MASK_RF_CHANNEL_REDUCTION   0x0F

#define RX_CONNECTION_TIMEOUT  1000000 // If no packet received in this time frame apply failsafe settings. In microseconds

// FHSS parameters
#define DEFAULT_PACKET_INTERVAL     ((uint32_t) 3000) 
#define MAX_PACKET_INTERVAL         ((uint32_t) 4000) // Max packet interval - used with telemetry and 16 channels
#define INITIAL_PACKET_TIMEOUT_ADD  200ul
#define RESYNC_TIME_OUT             ((uint32_t) 2000000) // Go to re-sync if no packet received in 3 seconds
#define RESYNC_WAIT_MICROS          (((((uint32_t) RF_CHANNELS) * 5ul) + 8ul) * MAX_PACKET_INTERVAL) // when syncing listen on each channel for slightly longer than the time it takes to cycle through all channels

#define INITIAL_TELEMETRY_PACKETS_TO_SKIP  1000 // Dont send initial telemetry packets to avoid anoying warnings at startup

#define DO_NOT_SOFT_REBIND             0xAA
#define BOUND_WITH_FAILSAFE_NO_PULSES  0x99

// Radio setup, Bind reciever
uint8_t currentModel = 0;
uint64_t radioPipeID;
uint64_t radioNormalRxPipeID;
const uint16_t currentModelEEPROMAddress = 0;
const uint16_t radioPipeEEPROMAddress = currentModelEEPROMAddress + sizeof(currentModel);
const uint16_t softRebindFlagEEPROMAddress = radioPipeEEPROMAddress + sizeof(radioNormalRxPipeID);

// Load fail safe default values
const uint16_t failSafeChannelValuesEEPROMAddress = softRebindFlagEEPROMAddress + sizeof(uint8_t); // uint8_t is the sifr of the rebind flag

// Output fail safe values
uint16_t failSafeChannelValues[RC_CHANNELS];

// Get packet
uint32_t packetInterval = DEFAULT_PACKET_INTERVAL;
bool packetMissed = false;
bool failSafeMode = false;

// Radio setup
uint8_t radioChannel[RF_CHANNELS];
bool bindMode = false; // When true send bind command to cause receiver to bind enter bind mode
volatile bool packetReady = false;
bool failSafeNoPulses = false;

// Battery voltage monitoring
uint32_t adc_time = 0;
uint8_t analogValue[2] = {0, 0};

// Set next radio channel
uint8_t currentChannel = MIN_RF_CHANNEL; // Initializes the channel sequence

uint16_t initialTelemetrySkipPackets = 0;
bool telemetryEnabled = false;

uint8_t radioConfigRegisterForTX = 0;
uint8_t radioConfigRegisterForRX_IRQ_Masked = 0;
uint8_t radioConfigRegisterForRX_IRQ_On = 0;

typedef struct
{
  enum RxMode_t : uint8_t
  {
    normal              = 0,
    bind                = 1,
    setFailSafe         = 2,
    normalWithTelemetry = 3,
    telemetryResponse   = 4,
    bindFalesafeNoPulse = 5,
    unBind              = 127
  }
  RxMode;
  
  uint8_t reserved = 0; // Contains the channel number that the packet was sent on in bits 0-5
  
  uint8_t option; // mask 0x0F    : Channel reduction. The number of channels to not send (subtracted from the 16 max channels) at least 4 are always sent
                  // mask 0x30>>4 : Receiver output mode
                  //                0 (00) = Single PPM on individual pins for each channel
                  //                1 (01) = SUM PPM on channel 1 pin
                  //                2 (10) = SBUS output
                  //                3 (11) = Unused
                  // mask 0x40>>6 : Contains max power override flag for Multi-protocol TX module. Also sent to RX
                  // mask 0x80>>7 : Unused
  
  uint8_t modelNum;
  uint8_t checkSum_LSB; // Checksum least significant byte
  uint8_t checkSum_MSB; // Checksum most significant byte
  uint8_t payloadValue [PAYLOAD_BYTES] = {0}; // 12 bits per channel value, unsigned
}
RxTxPacket_t;

void output_rc_channels();
void batt_monitoring();
void radio_setup();
void setNextRadioChannel(bool);
bool getPacket();
void checkFailsafeDisarmTimeout(uint32_t lastPacketTime, bool);
void outputFailSafeValues(bool callOutputChannels);
void unbindReciever();
void bindReciever(uint8_t modelNum, uint16_t tempHoldValues[], uint8_t RxMode);
void setFailSafeDefaultValues();
void loadFailSafeDefaultValues();
void setFailSafeValues(uint16_t newFailsafeValues[]);
bool validateChecksum(const RxTxPacket_t &rc_packet, uint8_t max_rc_packet_size);
bool readAndProcessPacket();
bool processRxMode(uint8_t RxMode, uint8_t modelNum, uint16_t tempHoldValues[]);
bool decodeChannelValues(const RxTxPacket_t &rc_packet, uint8_t channelsRecieved, uint16_t tempHoldValues[]);
uint32_t sendTelemetryPacket(); // Returns micros of when the transmit is expected to be complete
bool failSafeButtonHeld();
void setTelemetryPowerMode(uint8_t option);
void radio_init();

#endif // End __RX_h__
 
