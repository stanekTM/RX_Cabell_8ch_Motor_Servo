# RX_Cabell_8ch_Motor_Servo
It is a modified firmware of the receiver with the **"Cabell"** protocol, which is supported by the RC transmitter [**OpenAVRc**](https://github.com/Ingwie/OpenAVRc_Dev) in RF SPI mode or in serial [**Multiprotocol TX Module**](https://github.com/pascallanger/DIY-Multiprotocol-TX-Module).

This RX software accepts 8 channels, which are conveniently located on the Arduino Nano or Pro Mini. RX includes 2x motor controller with brake and has 6x servo output. I recommend reducing the number of channels as much as possible based on what your model requires. Fewer channels will use a smaller packet size, which improves transmission reliability (fewer bytes sent means less opportunity for errors).

The motor driver IC is based on MX1208, MX1508, MX1515, MX1616L, TC1508S, SA8302 and others similar, which uses 4x PWN input control signals.
The option to adjust the brake is included in the code.

[Video](https://www.youtube.com/watch?v=5skLsVCN05g) from testing.

## Arduino pins
```
D2  - servo 1
D3  - servo 2
D4  - servo 3
D7  - servo 4
D8  - servo 5
D11 - servo 6

D5  - pwm1/Motor A
D6  - pwm2/Motor A
D9  - pwm3/Motor B
D10 - pwm4/Motor B

D12 - bind button
D13 - LED

A6 - telemetry analog input A1
A7 - telemetry analog input A2

nRF24L01:
A0 - CE
A1 - CSN
A2 - SCK
A3 - MOSI
A4 - MISO
A5 - IRQ
```

## The Protocol
The protocol used is named CABELL_V3 (the third version, but the first version publicly released). It is a FHSS protocol using the NRF24L01+ 2.4 GHz transceiver. 45 channels are used from 2.403 through 2.447 GHz. The reason for using 45 channels is to keep operation within the overlap area between the 2.4 GHz ISM band (governed in the USA by FCC part 15) and the HAM portion of the band (governed in the USA by FCC part 97). This allows part 15 compliant use of the protocol, while allowing licensed amateur radio operators to operate under the less restrictive part 97 rules if desired.

Each transmitter is assigned a random ID (this is handled by the Multiprotocol TX Module) based on this ID one of 362880 possible channel sequences is used for the frequency hopping. The hopping pattern algorithm ensures that each hop moves at least 9 channels from the previous channel. One packet is sent every 3 - 4 milliseconds (depending on options chosen), changing channels with each packet. All 45 channels are used equally.

The protocol also assigns each model a different number so one model setting does not control the wrong model. The protocol can distinguish up to 255 different models, but be aware that the Multiprotocol TX Module transmitter software only does 16.

## Binding receiver
There are several ways for the receiver to enter bind mode:
* A new Arduino will start in bind mode automatically. Only an Arduino that was flashed for the first time (not previously bound) does this. Re-flashing the software will retain the old binding unless the EEPROM has been erased.
* Erasing the EEPROM on the Arduino will make it start up in bind mode just like a new Arduino. The Arduino sketch [here](https://github.com/soligen2010/Reset_EEPROM) will erase the EEPROM.
* Connect the bind jumper, or press the bind button while the receiver powers on.
* The protocol has a un-bind command (it erases the EEPROM), after which a re-start will cause the receiver to enter bind mode just like a new Arduino.

### procedure:
* Turn on the receiver in bind mode.
* Navigate to the model "SETUP" page.
* Highlight bind "Bnd" and press enter.
* When the pairing is successful, the LED of the receiver will flash slowly.
* Switch off the transmitter from bind mode and restart the receiver without bind mode.

## Unbinding Receiver
In order to un-bind a receiver using the transmitter, a model bound to the receiver must be configured in the transmitter. With a model selected that is bound to the receiver:
* Navigate to the model "SETUP" page.
* Change the sub-protocol to "Unbind". The model-bound receiver is disconnected. This happens immediately when the subprotocol is set to "Unbind".
* The receiver LED will blink when the un-bind is successful.

When the receiver is restarted, it will start in Bind mode.

## Fail-safe
* At fail-safe the channels are set to the fail-safe value.
* When a receiver is bound the failsafe values are reset to the default values and all channels at mid-point. The receiver also dis-arms if an RC signal is lost for 3 seconds.

Powering the transmitter off before the model will cause the receiver to dis-arm after 3 seconds.

**Do not set fail-safe values while in flight!** Due to the length of time it takes to write the new fail-safe values to EEPROM, the receiver may go into fail-safe mode while saving the values, causing loss of control of the model. Before flying a model, always test the fail-safe values after they have been set.

Fail-safe set mode will set the fail-safe values.
This can be done one of two ways:
* A set fail-safe packet can be sent from the transmitter. The values from the first packet in a series for set fail-safe packets are saved as the new fail-safe values. The LED is turned on when a set fail-safe packet is received, and stays on as long as set fail-safe packets continue to be received. The LED is turned off when set fail-safe values stop being received.
* After the receiver has initialized, the bind button (or bind jumper) can be held for one to 2 seconds until the LED is turned on. The values from the first packet received after the LED is turned on will be saved as the new fail-safe values. The LED will turn off when the button is released (or jumper removed).
 
When fail-safe set mode is entered, the LED is turned on and stays on until the fail-safe set mode is exited. Only the values from the first packet received in fail-safe set mode are saved (this is to avoid accidentally using up all of the EEPROMs limited number of write operations).

### procedure:
* Navigate to the model "SETUP" page.
* Place all switches in the desired fail-safe state.
* Move the sticks to the desired fail-safe state. Hold them in this position until the fail-safe settings are recorded by the receiver.
* While holding the sticks, change the sub-protocol to "F-Safe". DO not go past "F-Safe". If you even briefly go to "Unbind" the receiver will un-bind.
* When the LED is turned on, the fail-safe settings are recorded.
* Change the sub-protocol back to the original setting and the LED will turn off.

Always test the Fail-safe settings before flying. Turning off the transmitter should initiate a fail-safe after one second.

## Telemetry
When the sub-protocol is set to "V3 Telm", the receiver sends telemetry packets back to the transmitter. Three values are returned, a simulated RSSI, and the voltages on the Arduino pins A6 and A7 as telemetry values A1 and A2.

### RSSI
Because the NRF24L01 does not have an RSSI feature, the RSSI value is simulated based on the packet rate. The base of the RSSI calculation is the packet success rate from 0 to 100. This value is re-calculated approximately every 1/2 second (every 152 expected packets). This success percentage is then modified in real time based on packets being missed, so that if multiple packets in a row are missed the RSSI value drops without having to wait for the next re-calculation of the packet rate.

In practice, the packet rate seems to stay high for a long range, then drop off quickly as the receiver moves out of range. Typically, the telemetry lost warning happens before the RSSI low warning.

The RSSI class encapsulates the RSSI calculations. If you are so inclined, feel free play with the calculation. If anyone finds an algorithm that works better, please contribute it.

### A1, A2 analog values
Analog values are read on Arduino pins A6 and A7. Running on a, Arduino with VCC of 5V, only values up to 5V can be read. **A value on A6 or A7 that exceeds the Arduino VCC will cause damage**, so care must be taken to ensure the voltage is in a safe range.

The values from pins A6 and A7 come to the transmitter with OpenTX or OpenAVRc as telemetry values A1 and A2. You can use either of these to read battery voltage or the output of current sensor. The following article explains how to input battery voltage to A2 on an Frsky receiver using a voltage divider. The same method can be used to read battery voltage on this receiver. [http://olex.biz/tips/lipo-voltage-monitoring-with-frsky-d-receivers-without-sensors](http://olex.biz/tips/lipo-voltage-monitoring-with-frsky-d-receivers-without-sensors).

The values sent are 0 - 255 corresponding to 0V - 5V. This will need to be re-scaled to the actual voltage (or current, etc.) in the transmitter on the telemetry  configuration screen.

## Packet Format
```
typedef struct
{
  enum RxMode_t : uint8_t
  {
    normal              = 0, 
    bind                = 1,
    setFailSafe         = 2,
    normalWithTelemetry = 3,
    telemetryResponse   = 4,
    bindFalesafeNoPulse = 5, (experimental)
    unBind              = 127
  }
  RxMode;
  
  uint8_t reserved = 0; // Contains the channel number that the packet was sent on in bits 0-5
  
                  /*
  uint8_t  option; * mask 0x0F    : Channel reduction. The number of channels to not send (subtracted from the 16 max channels) at least 4 channels are always sent.
                   * mask 0x30>>4 : Receiver output mode:
                   *                0 (00) = Single PPM on individual pins for each channel
                   *                1 (01) = SUM PPM on channel 1 pin
                   *                2 (10) = SBUS output
                   *                3 (11) = Unused
                   *
                   * mask 0x40>>6   Contains max power override flag for Multiprotocol TX module. Also sent to RX.
                   *                The RX uses MAX power when 1, HIGH power when 0.
                   * mask 0x80>>7   Unused
                   */
                   
  uint8_t  modelNum;
  uint8_t  checkSum_LSB; // Checksum least significant byte
  uint8_t  checkSum_MSB; // Checksum most significant byte
  uint8_t  payloadValue [24] = {0}; // 12 bits per channel value, unsigned
}
CABELL_RxTxPacket_t;
```
Each 12 bits in payloadValue is the value of one channel. Valid values are in the range 1000 to 2000. The values are stored big endian. Using channel reduction reduces the number of bytes sent, thereby trimming off the end of the payloadValue array.

## License Info
Copyright 2017 - 2019 by Dennis Cabell (KE8FZX)
To use this software, you must adhere to the license terms described below, and assume all responsibility for the use of the software. The user is responsible for all consequences or damage that may result from using this software. The user is responsible for ensuring that the hardware used to run this software complies with local regulations and that any radio signal generated from use of this software is legal for that user to generate. The author(s) of this software assume no liability whatsoever. The author(s) of this software is not responsible for legal or civil consequences of using this software, including, but not limited to, any damages cause by lost control of a vehicle using this software. If this software is copied or modified, this disclaimer must accompany all copies.

This project is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

RC_RX_CABELL_V3_FHSS is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with RC_RX_CABELL_V3_FHSS.  If not, see <http://www.gnu.org/licenses>
