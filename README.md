# Mossbauer Driver
This is the firmware for the Mossbauer experiment voice coil driver. It runs on a STM32F4 Discovery board. The microcontroller waits for a rising slope trigger, then outputs a 200ms parabolic waveform, causing the Co57 source to experience constant acceleration. Proportional feedback is used to keep the motion of the source consistent and accurate.

### Connections

The board needs connections to the voice coil and to the displacement sensor for feedback, as well as a connection to the trigger line coming from the Spectech spectrum analyzer:

##### Board

| External Device     | Connection     |
|:--------------------|:---------------|
| Displacement sensor | Port C, Pin 2  |
| Trigger (3 V Max)   | Port B, Pin 0  |
| Voice Coil +        | Port A, Pin 5  |
| Voice Coil -        | GND            |
| Power               | +5 V, USB Mini |

##### Displacement Sensor

A model LD 701-2/5 inductive displacement sensor, obtained from Omega, is used for position feedback. It takes +16 VDC and outputs 1-9 V. It is run through a voltage divider to bring its output to a maximum of 3 V, suitable for the STM32F4 analog input.

|Color              |Connection       |
|:------------------|:----------------|
|Brown              |+16 VDC          |
|Blue               |GND              |
|Black              | Voltage Divider |
