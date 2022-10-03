# Attitude Determination and Control System

---
:information_source: A conceptual overview of the Attitude Determination and Control System (ADCS) can be found in our hardware repository, [quetzal1-hardware](https://github.com/Quetzal-1-CubeSat-Team/quetzal1-hardware/tree/master/ADCS). We recommend you read it first, if this is your first time here. Some terms mentioned throughout this document may be defined there.

---

## Directory Description

This directory contains the flight software for the ADCS. It was developed using [Arduino](https://www.arduino.cc/) and it containes the source files, as well as the necessary libraries.

The directory is organized as follows:

1. `src/`: contains the firmware programmed into the ADCS microcontroller.
3. `media/`: contains miscellaneous images that may be of use and serve as reference to the user.

## Design of the ADCS I<sup>2</sup>C sensor network

The ADCS carried the following I<sup>2</sup>C sensors to monitor the attitude (orientation) of the satellite:

1. 1X [Adafruit Breakout Board](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor) for the [Bosch, Cat. No. BNO055](https://cdn-learn.adafruit.com/assets/assets/000/036/832/original/BST_BNO055_DS000_14.pdf) - Inertial Measurement Unit (IMU)
2. 12X sun sensors ([Vishay, Cat. No. TEMD6010FX01](https://www.vishay.com/en/product/81308/)) - Photodiodes
3. 2X Analog-to-Digital Converters (ADC) ([Texas Instruments, Cat. No. ADC128D818](https://www.ti.com/product/ADC128D818)) - 8-channel, 12-bit (to read each photodiode voltage)
4. 1X [Texas Instruments, Cat. No. TMP100](https://www.ti.com/product/TMP100) - Temperature Sensor with I<sup>2</sup>C Interface

---
:information_source: The purpose of each sensor is better described in [[2]](#user-content-references), and a complete overview of the purpose of each sensor is given in [[1]](#user-content-references).

---

The following figure shows the layout of the internal ADCS I<sup>2</sup>C network:

![adcs-network](./media/adcs_network.png?raw=true "ADCS Sensor Network")

The ADCS &mu;C itself was connected as a slave with the address `0xAA` to the satellite´s main I<sup>2</sup>C bus, which had the On-Board Computer (OBC) as master. The ADCS internal I<sup>2</sup>C bus (shown in the previous figure) was implemented using the SofwareWire library from the Free Software Foundation, Inc. (available online at: [SoftwareWire Library](https://github.com/Testato/SoftwareWire)).

## Overview of ADCS &mu;C Pinout

The following picture shows the circuit schematic of the complete ADCS &mu;C pinout. Note that pin `PD3` was used to reset the ADM subsytem, when commanded by the OBC.

![adcs-pinout](./media/adcs_pinout.png?raw=true "ADCS Pinout")


---
:information_source: Further details regarding the electronic design of Quetzal-1's ADCS can be found in [[1]](#user-content-references) and [[2]](#user-content-references).

---

## Software and sensor start-up

The ADCS firmware was designed with the same structure of a typical Arduino script. The main script `ADCS_Flight_Software.ino` can be found [here](./src/ADCS_Flight_Software.ino). The setup section of the code executed certain power saving configurations and pin configuration of the microcontroller. Subsequently, the &mu;C initialized the internal ADCS I<sup>2</sup>C bus and all the sensors connected to it. To implement fault tolerance during sensor start-up, the µC was programmed to attempt the initial configuration of each I<sup>2</sup>C sensor a maximum of three times.

Any error during the start-up sequence of a sensor was recorded in a Communication Flags (CF) byte. The structure of the CF byte was the following:

__Table 1:__ ADCS Communication Flags (CF) byte (0 - Error; 1 - Success)
| Bit         | 7 | 6 | 5 | 4                            | 3                           | 2                 | 1                 | 0                 |
|-------------|---|---|---|------------------------------|-----------------------------|-------------------|-------------------|-------------------|
| Description | 0 | 0 | 0 | 0 | TMP100 Comm. Flag | ADC2 Comm. Flag | ADC1 Comm. Flag | IMU Comm. Flag |

If a sensor failed to communicate with the &mu;C three times during start-up, then the &mu;C would set that sensor’s CF flag to 0. Otherwise, it would set the sensor’s CF flag to 1. After finishing the configuration of all the sensors, the &mu;C would set `adcs_ready_flag` to a value of 1. This flag could be retrieved by the OBC to determine if the ADCS system had been initialized correctly.

In the main loop, the &mu;C would check if any flag was set, perform an action if so, and then go into the deepest sleep mode (to conserve power). The &mu;C would be woken up via an I2C command from the OBC, perform the requested action, and sleep once more.

## OBC-ADCS Communication

Interruptions were enabled in the ADCS &mu;C so it could receive commands sent by the OBC via the satellite’s main I<sup>2</sup>C bus. A typical I<sup>2</sup>C transaction between ADCS and OBC was executed in two steps: 

1. __Write:__  a write action would be performed by the OBC to set the value of the `wireCommand` variable stored in the ADCS &mu;C (see `receiveEvent`).
2. __Read:__ the OBC would then perform a read action, triggering a switch-case portion of the ADCS code (see `requestEvent`). This switch-case would then execute a specific action depending on the value of the `wireCommand` variable previously set by the OBC.

Two types of commands were implemented on the ADCS software: 

1. __Data retrieval commands:__ used to immediately send back the requested data to the OBC (e.g. temperature data, gyroscope data or all data the collected by ADCS). 
2. __Direct action commands__ used to activate (set to 1) one of the following internal ADCS flags: `collect_flag`, `checkcomm_flag`, `reset_flag`, `adm_reset_flag`. Subsequently, the microcontroller’s execution loop would perform the action corresponding to the activated flag, and it would deactivate (set to 0) the flag upon completion of the action.

The complete list of OBC-EPS commands can be found [here](./src/lib/ADCS/ADCS.h).

## References

[1] Alvarez, D. et al. (TBD): Design and On-Orbit Performance of the Attitude Determination and Passive Control System for the Quetzal-1 CubeSat, Journal of Small Satellites (JOSS), vol. TBD(TBD), p. TBD.

[2] Quetzal-1 CubeSat Team. (TBD): quetzal1-hardware, Available at: https://github.com/Quetzal-1-CubeSat-Team/quetzal1-hardware, (accessed TBD).