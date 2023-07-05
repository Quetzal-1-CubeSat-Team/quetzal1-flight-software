# Electrical Power System

---
:information_source: A conceptual overview of the Electrical Power System (EPS) can be found in our hardware repository, [quetzal1-hardware](https://github.com/Quetzal-1-CubeSat-Team/quetzal1-hardware/tree/master/EPS). We recommend you read it first, if this is your first time here. Some terms mentioned throughout this document may be defined there.

---

## Directory Description

This directory contains the flight software for the EPS. It was developed using [Arduino](https://www.arduino.cc/) and it contains the source files, as well as the necessary libraries.

The directory is organized as follows:

1. [src/EPS/](./src/EPS/): contains the firmware programmed into the EPS microcontroller.
3. [media/](./media/): contains miscellaneous images that may be of use and serve as reference to the user.

---
:warning: Note that local library paths (*e.g.*, `src/TMP100/TMP100.h`) are used for the libraries included in the main sketch, to avoid compiling libraries that may be named the same within your own system. This has been tested to compile correctly with the Arduino IDE `1.8.13` and `2.0.0`.

---

## Design of the EPS I<sup>2</sup>C sensor network

The EPS carried the following I<sup>2</sup>C sensors to monitor the power grid and the health of the on-board battery:

1. 3X [Texas Instruments, Cat. No. INA260](https://www.ti.com/product/INA260) - Precision Digital Current and Power Monitors, each functioning as:
    * Solar Channel Monitor (SCM)
    * Main Bus Monitor (MBM)
    * Secondary Bus Monitor (SBM)
2. 1X [Texas Instruments, Cat. No. BQ27741-G1](https://www.ti.com/product/BQ27741-G1) - Single-Cell Li-Ion Battery Fuel Gauge
3. 1X [Texas Instruments, Cat. No. TMP100](https://www.ti.com/product/TMP100) - Temperature Sensor with I<sup>2</sup>C Interface

The following figure shows the layout of the internal EPS I<sup>2</sup>C network:

![eps-network](./media/eps_network.png?raw=true "EPS Sensor Network")

The EPS &mu;C itself was connected as a slave with the address `0x99` to the satellite´s main I<sup>2</sup>C bus, which had the On-Board Computer (OBC) as master. The EPS internal I<sup>2</sup>C bus (shown in the previous figure) was implemented using the SoftwareWire library from the Free Software Foundation, Inc. (available online at: [SoftwareWire Library](https://github.com/Testato/SoftwareWire)).

## Design of the EPS Fault Protection Boards (FPB) network

The EPS incorporated four protection circuits denominated Fault Protection Boards (FPB). Each FPB was made up of two components connected in series: a power-distribution switch ([Texas Instruments, Cat. No. TPS2551](https://www.ti.com/product/TPS2551)) and a high-side unipolar current shunt monitor ([Texas Instruments, Cat. No. INA169](https://www.ti.com/product/INA169)). The FPBs were used to power up or down individual satellite systems (COMMS, ADCS, Payload, and battery heater) and monitor their power consumption [[1]](#user-content-references).

The following figure shows the connections between the EPS &mu;C and the FPBs:

![fpb-diagram](./media/eps_fpb_connection.png?raw=true "FPB Diagram")

The __FPB switches__ had active-high enable pins that could be used to control the power supply of individual satellite systems. Thus, the enable pin of each FPB was connected to a digital output pin of the EPS &mu;C. In this way, it could command the activation or deactivation of a system following the commands sent by the OBC via the main I<sup>2</sup>C bus.

The __FPB monitors__ measured the current consumption of each system using a shunt resistor, and they translated these measured currents into analog output voltages. The monitors were adjusted via hardware so that sensed currents between 0 mA and 2 A would translate into analog voltages between 0 V and 2.5 V. The output pin of each monitor was connected to an analog input pin of the EPS &mu;C, and the required conversion from analog voltages to currents was implemented in software.

The following picture shows the circuit schematic of the complete EPS &mu;C pinout including the pins connected to the FPBs:

![eps-pinout](./media/eps_pinout.png?raw=true "EPS Pinout")

Note that pin PD2 of the EPS &mu;C was used as a digital output pin that was connected to the active-high enable pin of a 5V voltage regulator integrated on the EPS circuit board.

---
:information_source: Further details regarding the electronic design of the Quetzal-1's EPS can be found in [[1]](#user-content-references) and [[2]](#user-content-references).

---

## Software and sensor start-up

The EPS firmware was designed with the same structure of a typical Arduino script. The main script `EPS.ino` can be found [here](./src/EPS/EPS.ino). The setup section of the code executed pin configuration of the microcontroller, activating the analog and digital pins connected to each of the EPS FPBs (ADCS, COMMS, Payload, Battery Heater). Subsequently, the &mu;C initialized the internal EPS I<sup>2</sup>C bus and all the sensors connected to it. To implement fault tolerance during sensor start-up, the µC was programmed to attempt the initial configuration of each I<sup>2</sup>C sensor a maximum of three times, as shown in the following code snippet:

```c++
  // Initial configuration of the Solar Channel Monitor (INA260 No.1)
  int INA_TRY_COUNTER1 = 0;
  while (INA_TRY_COUNTER1 < 3){
    if (ina260_1.begin()){
        comm_flags[0] = 1;
        break;
    }
    INA_TRY_COUNTER1 += 1;
    if (INA_TRY_COUNTER1 == 3){
      comm_flags[0] = 0;
    }
  }
```

Any error during the start-up sequence of a sensor was recorded in a Communication Flags (CF) byte. The structure of the CF byte was the following:

__Table 1:__ EPS Communication Flags (CF) byte (0 - Error; 1 - Success)
| Bit         | 7 | 6 | 5 | 4                            | 3                           | 2                 | 1                 | 0                 |
|-------------|---|---|---|------------------------------|-----------------------------|-------------------|-------------------|-------------------|
| Description | 0 | 0 | 0 | Thermal Sensor Comm. Flag | Battery Gauge Comm. Flag | SBM Comm. Flag | MBM Comm. Flag | SCM Comm. Flag |

If a sensor failed to communicate with the &mu;C three times during start-up, then the &mu;C would set that sensor’s CF flag to 0. Otherwise, it would set the sensor’s CF flag to 1. After finishing the configuration of all the sensors, the &mu;C would evaluate the resulting CF byte. If the CF byte had a value different from 0x00, then the &mu;C would set an `eps_ready` flag to a value of 1. This flag was retrieved by the OBC to determine if the EPS system had been initialized correctly. If the OBC received the `eps_ready` flag set to 0, it would command the EPS &mu;C to execute a software reset and retry the configuration of all the sensors. After the setup, the &mu;C would then start executing a loop every 10 ms without performing any specific action, unless the OBC commanded otherwise.

## OBC-EPS Communication

Interruptions were enabled in the EPS &mu;C so it could receive commands sent by the OBC via the satellite’s main I<sup>2</sup>C bus. A typical I<sup>2</sup>C transaction between EPS and OBC was executed in two steps: 

1. __Write:__  a write action would be performed by the OBC to set the value of the `WireCommand` variable stored in the EPS &mu;C. 
2. __Read:__ the OBC would then perform a read action, triggering a switch-case portion of the EPS code. This switch-case would then execute a specific action depending on the value of the `WireCommand` variable previously set by the OBC.

Three different types of commands were implemented on the EPS software: 

1. __Enable/disable commands:__ used to set or reset the enable pins connected to the FPBs; thus, activating or deactivating a specific system inside the satellite. 
2. __Data retrieval commands:__ used to immediately send back the requested data to the OBC (e.g. temperature data, voltage data or all data the collected by EPS). 
3. __Direct action commands__ used to activate (set to 1) one of the following internal EPS flags: `collect_data`, `collect_SOC`, `collect_FPB`, `eps_reset`. Subsequently, the microcontroller’s execution loop would perform the action corresponding to the activated flag, and it would deactivate (set to 0) the flag upon completion of the action.

The complete list of OBC-EPS commands can be found [here](./src/EPS/EPS.h).

## Power grid monitoring

When the EPS &mu;C was commanded by the OBC to collect data from all sensors, the `collect_data` flag would be activated, and the execution loop would then perform the collection of all EPS measurements. Firstly, the `collect_ready` flag would be deactivated to signal that the EPS &mu;C was in process of monitoring the satellite’s power grid. Then, the &mu;C would command each sensor to measure and return data for processing and storage. To implement fault tolerance during data collection, the &mu;C was programmed to try to obtain data from each EPS sensor a maximum of three times and to record any error in a Transmission Flags (TF) byte. The structure of the TF byte was the following:

__Table 2:__ EPS Transmission Flags (TF) byte (0 - Error; 1 - Success)
| Bit         | 7 | 6 | 5 | 4                          | 3                         | 2               | 1               | 0               |
|-------------|---|---|---|----------------------------|---------------------------|-----------------|-----------------|-----------------|
| Description | 0 | 0 | 0 | Thermal Sensor Trans. Flag | Battery Gauge Trans. Flag | SBM Trans. Flag | MBM Trans. Flag | SCM Trans. Flag |

If a sensor failed to transmit correct data back to the &mu;C three times during collection, then the &mu;C would set that sensor’s TF flag to 0. Otherwise, it would set the sensor’s TF flag to 1. Furthermore, error values were implemented to indicate the reason for a transmission failure. If a sensor failed to transmit data back to the &mu;C due to I<sup>2</sup>C miscommunication, all variables used to store data from the specific sensor would be loaded with the `0xFD` or `0x0FFD` values (Transmission Error). If a sensor sent data back to the &mu;C, but a measurement corresponding to a strictly positive quantity returned as a negative number, the incorrect variable would be reloaded with the `0xFE` or `0x0FFE` values (Negative Error). Lastly, if a sensor sent data back to the &mu;C, but a measurement was out of a nominal operating range, the incorrect variable would be reloaded with the `0xFF` or `0x0FFF` values (Range Error). 

It is important to mention that the Communication Flags (CF) byte also had an effect on the data collection process. If the CF flag of a specific sensor was set to 0, that meant that the sensor had failed its initial setup. Therefore, the EPS &mu;C would bypass data collection for that faulty sensor to avoid I<sup>2</sup>C bus hang-up caused by attempting to retrieve data from it.

During data collection, the EPS &mu;C also polled the analog voltages returned by each FPB current monitor. These analog voltages, mapped to the respective currents, were then compared to the overcurrent and short-circuit current limits hard-coded for each system. The ADCS and Payload systems were limited to 290 mA. The COMMS system was limited to 1 A. Lastly, the Battery Heater was limited to 450 mA. It is worth noting that these software limits added a layer of protection on top of the current limits that were set via hardware configuration of the FPB power switches. 

A Fault Flags (FPBF) byte was used to record any overcurrent or short-circuit failure detected by the EPS &mu;C when polling the FPBs. If the &mu;C detected that a specific system was consuming a current above its overcurrent limit, the overcurrent flag for that system would be set to a value of 1 to indicate an error. Furthermore, a short-circuit could be automatically detected in hardware by the FPB power switches. In these instances, the switches would automatically stop current flow. Therefore, in the event that a switch was enabled by the EPS &mu;C, but no current flow was detected, the short-circuit flag for that system would be set to a value of 1 (to indicate an error).

Additionally, the OBC was programmed to retrieve the FPBF byte from the EPS &mu;C every time it commanded the EPS to activate a system. This was done to allow the OBC to issue a quick deactivation command to shut down a failing system. The FPBF byte structure is shown in the following table:

__Table 3:__ EPS FPBF byte (SC - short-circuit; OC - overcurrent; 0 - All OK; 1 - Failure)
| Bit         | 7              | 6               | 5             | 4            | 3              | 2               | 1             | 0            |
|-------------|----------------|-----------------|---------------|--------------|----------------|-----------------|---------------|--------------|
| Description | Heater SC Flag | Payload SC Flag | Comms SC Flag | ADCS SC Flag | Heater OC Flag | Payload OC Flag | COMMS OC Flag | ADCS OC Flag |

---
:information_source: It is worth noting that the ADCS system was programmed in such a way that its sensors exited low power mode and executed their measurements after the EPS had finished its own measurements. For this reason, the EPS &mu;C always detected that the ADCS was enabled but consuming almost 0 mA during ground tests (reduced quiescent current in low power mode). This caused the ADCS short-circuit flag to always be set to 1 even though there was no real short-circuit condition. Since the ADCS would always be in failure according to the FPBF byte, the OBC was programmed to ignore the ADCS short-circuit flag to avoid an accidental shutdown of the system when the satellite launched into space.

---

After the EPS &mu;C finished collecting all the data, the `collect_ready` flag would be activated to indicate to the OBC that there was new data to be retrieved. The OBC would then command the EPS &mu;C to send back all the collected data, and the EPS &mu;C would subsequently deactivate the `collect_ready` flag again to indicate that there was no new data to be retrieved at the moment. The OBC could also command the EPS &mu;C to directly measure the battery state of charge and the FPB currents without having to execute the complete set of measurements by activating the `collect_SOC` and `collect_FPB` flags, respectively. It could also command the EPS &mu;C to execute a software reset by activating the `eps_reset` flag, which basically forced the program’s pointer to return to the zero position in the program memory. This allowed the EPS &mu;C to execute the software setup again.

## References

[1] Aguilar-Nadalini, A. et al. (2023): Design and On-Orbit Performance of the Electrical Power System for the Quetzal-1 CubeSat, Journal of Small Satellites (JOSS), vol. 12(2), p. 1201–1229.

[2] Quetzal-1 CubeSat Team. (TBD): quetzal1-hardware, Available at: https://github.com/Quetzal-1-CubeSat-Team/quetzal1-hardware, (accessed TBD).