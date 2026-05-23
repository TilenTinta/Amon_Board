# Amon Board — Flight Controller & Firmware

**Project Status:** In Progress

# Description:
Flight controller hardware and firmware used in the Amon Lander project. Two hardware lines are maintained:
- **Board v1 (ESP32 WROOM-32)** — never released (critical error).
- **Board v2 (STM32F405)** — the current, working design.
- **Board v3 (STM32H743VIT6)** — in development.
- **Board v4 (STM32xxx (Zephyr) + Raspberry Pi CM4)** — the next-gen design with a new MCU running Zephyr RTOS and a companion **Raspberry Pi Compute Module 4 (CM4)**.

> Part of the Amon Lander project:
>
> - **Amon Lander** — overall vehicle, frame, test/control SW, and data  
>   https://github.com/TilenTinta/Amon_Lander  
> - **Amon Link** — PC - drone communications (PCB + firmware)  
>   https://github.com/TilenTinta/Amon_Link  
> - **Amon Ground Control** — desktop GCS for telemetry & control  
>   https://github.com/TilenTinta/Amon_GroundControl_Software

---

## Table of Contents

- [Overview](#overview)
- [Hardware Versions](#hardware-versions)
  - [Board v2 — STM32F405](#board-v1--stm32f405)
  - [Board v3 — STM32H743](#board-v1--stm32h743)
  - [Board v4 — Zephyr + CM4](#board-v2--zephyr--cm4)
- [Version Matrix](#version-matrix)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
  - [Build & Flash: Board v2 & v3](#build--flash-board-v1)
  - [Build & Flash: Board v4 (Zephyr)](#build--flash-board-v2-zephyr)
  - [First Boot: CM4 Companion](#first-boot-cm4-companion)
- [Calibration & Testing](#calibration--testing)
- [Roadmap](#roadmap)
- [Status & Disclaimer](#status--disclaimer)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## Overview

This board is a custom flight controller for **Amon Lander**. The **v2** generation uses an **STM32F405RGT6 (LQFP-64)** and a hand-written firmware stack (drivers from scratch, for learning and control). Because of the flight algorithm planned to be used in this project, the decision was made to modify the current **v2** PCB with a new MCU. This is the **STM32H743VIT6 (LQFP-100)** and to fix some issues and add features that were missing or incorrect in the previous version. A **v3** generation is being designed around a **Zephyr-supported MCU** for real-time control, that will be paired with a **Raspberry Pi CM4** for high-level tasks (GCS link, logging, navigation stack, future CV/ML, etc.) in **v4**.

Everything was built from the ground up — **electronics, firmware, and software**. Most tools used are **free** and widely available to makers. Expect a learning-first approach over simplicity: some solutions could be easier, but the point is to **learn and explore**.

---

## Hardware Versions

### Board v1 — ESP32 WROOM-32

**Highlights**

- **MCU:** ESP32 WROOM-32 module
- **Power & Protection**
  - Dual vertical **XT60** inputs (PCB PWR and EDF voltage)
  - **Reverse-polarity diode**
  - Max input tested: **12 V**
  - Two **LDO regulators**: 3.3V (MCU/sensors) & 5V/5A (servos)
- **Sensors**
  - **MPU6050** (IMU)
  - **BME280** (temp/pressure/humidity)
  - TOF connector for **VL53L1X**
- **Navigation & Comms**
  - **GPS** connector (GY-NEO6MV2)
  - **single NRF24L01+ PA/LNA** (RX–TX) module
- **I/O**
  - **PWM:** 4× servos + 1× EDF
  - Expansion: **SPI** & **I²C** headers
  - **USB** with surge protection (over CH340C)
  - **4-pin JTAG** header
  - **RGB status LED**
  - **Buzzer**
- **Storage:** **microSD** slot
- **PCB:** 4-layer

**Firmware (v1)**
- **Status:** Never really worked (ESP32 was eliminated from the project)

> ![PCBv1_1](https://github.com/TilenTinta/Amon_Board/blob/main/Images/PCB_v1_1.png)

### Board v2 — STM32F405

**Highlights**

- **MCU:** STM32F405RGT6 (LQFP-64) - Cortex M4, 168MHz, 196kB RAM, 1MB Flash
- **Power & Protection**
  - Dual vertical **XT60** inputs (PCB PWR and EDF voltage)
  - **Reverse-polarity diode**
  - Max input tested: **12 V**
  - Two **buck regulators**: 3.3 V (MCU/sensors) & 5 V (servos), ~5 A each
- **Switching/Startup:** Power-on by battery insertion or external switch on a terminal
- **Grounding:** Grounded mounting holes
- **Sensors**
  - **MPU6050** (IMU)
  - **BME280** (temp/pressure/humidity)
  - **HMC5883L** (compass) *added later
  - TOF connector for **VL53L1X**
- **Navigation & Comms**
  - **GPS** connector (GY-NEO6MV2; also reused for calibration)
  - **2× NRF24L01+ PA/LNA** (RX–TX)
- **I/O**
  - **PWM:** 4× servos + 1× EDF (configurable)
  - Expansion: **SPI** & **I²C** headers
  - **USB OTG** with surge protection
  - **10-pin JTAG** header
  - **RGB status LED**
- **Storage:** External flash (telemetry) + **microSD** slot
- **PCB:** 4-layer, working but slated for a future revision/refresh

**Firmware (v2)**

- **Approach:** Minimal dependencies; **drivers from scratch** (for learning)  
- **Status:** Runs and evolves; some drivers may still have rough edges  
- **Focus:** Bring-up, sensor fusion, basic control loops, telemetry logging and transmition...

> ![PCBv2_1](https://github.com/TilenTinta/Amon_Board/blob/main/Images/PCB_v2_1.PNG)

### Board v3 — STM32H743

**Highlights**

- **MCU:** STM32H743VIT66 (LQFP-100) - Cortex M7, 480MHz, 1MB RAM, 2MB Flash
- **Power & Protection**
  - Dual horizontal **XT60** inputs (PCB PWR and EDF voltage)
  - **Reverse-polarity diode**
  - Max input tested: **12 V**
  - Three **buck regulators**: 3.3 V (MCU/sensors) & 5 V (ESC and placeholder) & 7.2 V (servos), ~5 A each + more stable voltage
- **Switching/Startup:** Power-on sequence 3.3V -> 5V -> 7.2V (monitoring of regulators output voltages and enable/disable function)
- **Grounding:** Grounded mounting holes
- **Sensors**
  - **MPU6050** (IMU)
  - **BME280** (temp/pressure/humidity)
  - **HMC5883L** (compass)
  - Connector for TOF - **VL53L1X**
  - Connector for optical flow - **PMW3901**
- **Navigation & Comms**
  - Connector for GPS module -**GY-NEO6MV2**
  - **2× NRF24L01+ PA/LNA** (RX–TX) now on PCB - eliminated use of modules
  - **USB** over serial chip (CH340G) - eliminated use of USB OTG
- **I/O**
  - **PWM:** 4× servos + 1× EDF
  - Expansion: **SPI** & **I²C** headers
  - **10-pin JTAG** header (Olimex HAB-VCR-010-LF)
  - **RGB status LED**
  - **Single RED status LED**
  - Three connector low **LED modules** and additional AUX connector for future use
- **Storage:** External flash (telemetry) + **microSD** slot
- **PCB:** 4-layer

**Firmware (v3)**

- **Base:** Firmware is fully based on **Firmware v2**
- **Approach:** Minimal dependencies; **drivers from scratch** (for learning)  
- **Status:** Runs and evolves; some drivers may still have rough edges  
- **Focus:** sensor fusion, advance control algorithms, telemetry logging and transmition...

> ![PCBv3_1](https://github.com/TilenTinta/Amon_Board/blob/main/Images/PCB_v3_1.PNG)

---

### Board v4 — Zephyr + CM4

**Architecture (planned / in development)**

- **Real-time MCU (Zephyr)**  
  - New MCU (TBD; Zephyr-supported) handles **Real-time**:
    - Sensor acquisition & fusion
    - Control loops (attitude/rate, TVC/actuators, EDF control)
    - Safety interlocks & failsafes
    - Deterministic logging (to QSPI flash / SD)
  - **Zephyr RTOS** app with:
    - Devicetree-based pin/peripheral config
    - Workqueues/threads for IMU, baro, magnetometer, TOF, PWM
    - Shell/CLI over UART/USB for diagnostics
    - Config storage (NVS) and firmware update hooks

- **Companion Computer — Raspberry Pi CM4**  
  - Runs services for:
    - Ground link, mission control, high-rate logging
    - Navigation stack expansion (mapping, state estimation helpers)
    - Future CV/ML applications (optional)
  - **Link to MCU:** UART (primary) + SPI/CAN (optional)  
  - **Services:** Telemetry bridge (to Amon Link), data recorder, configuration UI server
  - **OS:** Raspberry Pi OS Lite (or other lightweight Linux)

**Electrical**
- Robust power tree with multiple separated rails for servos/EDF vs logic
- MCU - CM4 isolation where needed (level shifting/ESD)
- On-board mass storage for logs (eMMC on CM4 + SD/QSPI on MCU)
- External debug: SWD/JTAG for MCU; USB-C or Ethernet for CM4

**Firmware (v4)**

- **Zephyr** application (modules: sensors, control, comms, storage)
- **IPC format:** CBOR/Protobuf frame over UART (CRC-guarded)
- **Bootloaders & Updates:** MCU bootloader (serial/DFU), CM4 apt-based or containerized deployment
- **Testing:** HIL hooks; replay logs for control tuning

---

## Version Matrix

## Version Matrix

| Capability                  | Board v1 — ESP32 | Board v2 — STM32F405 | Board v3 — STM32H743 | Board v4 — Zephyr + CM4 |
|----------------------------|------------------|----------------------|----------------------|--------------------------|
| MCU                         | ESP32 WROOM-32   | STM32F405RGT6        | STM32H743VIT6        | **TBD (Zephyr-supported)** |
| RTOS                        | None             | None (bare-metal)    | None (bare-metal)    | **Zephyr RTOS**          |
| Companion Computer          | —                | —                    | —                    | **Raspberry Pi CM4**     |
| Sensors                     | MPU6050, BME280, VL53L1X | MPU6050, BME280, HMC5883L, VL53L1X | MPU6050, BME280, HMC5883L, VL53L1X | Same class; final selection TBD |
| Radio                       | 1× NRF24L01+ PA/LNA | 2× NRF24L01+ PA/LNA | 2× NRF24L01+ PA/LNA | TBD (NRF / other via MCU or CM4) |
| Navigation                  | GPS (GY-NEO6MV2) | GPS (GY-NEO6MV2)     | GPS (GY-NEO6MV2)     | GPS via MCU or CM4 module |
| Storage                     | microSD          | Flash + microSD      | Flash + microSD      | CM4 eMMC + MCU flash/SD  |
| USB                         | USB (CH340C)     | USB OTG              | USB OTG + CH340G     | USB (MCU + CM4)          |
| Programming / Debug         | 4-pin JTAG       | 10-pin JTAG          | 10-pin JTAG          | SWD/JTAG (MCU) + SSH/USB/Ethernet (CM4) |
| PWM Outputs                 | 4× servos + 1× EDF | 4× servos + 1× EDF | 4× servos + 1× EDF | Same (handled by MCU)    |
| Expansion                   | SPI, I²C         | SPI, I²C             | SPI, I²C             | SPI, I²C, UART, CAN (planned) |
| Power                       | XT60, LDO (3.3V + 5V/5A) | XT60, buck (3.3V + 5V/5A + 7.2V) | XT60, improved buck (3.3V + 5V/5A) | Segregated rails, higher headroom (TBD) |
| PCB                         | 4-layer          | 4-layer              | 4-layer              | TBD                      |
| Firmware Status             | Not working    | Working, evolving | In development | In development        |
| Firmware Approach           | —                | Bare-metal, custom drivers | Based on v2, improved | Zephyr modules + Linux services |
| Firmware Updates            | USB              | JTAG/SWD/DFU         | JTAG/SWD/DFU         | MCU bootloader + CM4 (apt/container) |

---

## Getting Started

### Build & Flash: Board v1

1. **Clone**
   ```bash
   git clone https://github.com/<your-org-or-user>/Amon_Board.git
   cd Amon_Board/firmware/Amon_board_firmware-MCU
   ```
2. **Toolchain**
   - Install `arm-none-eabi-gcc` and your preferred STM32 flashing tool.
3. **Configure - can be automatic**
   - Target: **STM32F405RG**; enable FPU; set clock tree; verify linker script.
4. **Build**
   ```bash
   make
   ```
   or use your IDE’s Build action.
5. **Flash**
   - Via **JTAG/SWD** (10-pin header) using your probe (ST-Link, J-Link, etc.).
6. **Verify**
   - Rails at 5 V/3.3 V, sensor presence on I²C/SPI, status RGB LED patterns.

---

### Build & Flash: Board v2 (Zephyr)
Instructions bellow are only "place holder". Detailed instructions will be provided once the development will be started. For flashing and debugging will be used Segger Ozone software.

1. **Prerequisites**
   - Install **Zephyr SDK**, Python, CMake, Ninja.
   - Install **west**: `pip install west`
2. **Get Sources**
   ```bash
   cd Amon_Board/firmware/v2-zephyr
   west init -l .
   west update
   ```
3. **Board Definition**
   - Use the custom board under `boards/amon_v2/` (edit SoC, pins, clocks, and flash layout as needed).
4. **Build**
   ```bash
   west build -b amon_v2 app
   ```
5. **Flash**
   ```bash
   west flash
   ```
6. **Console**
   ```bash
   west espressif monitor   # or 'west zephyr monitor' / minicom, depending on target
   ```
   *(Choose the appropriate monitor command for your MCU/OS.)*

---

### First Boot: CM4 Companion

1. **Flash OS**
   - Install **Raspberry Pi OS Lite** to CM4’s eMMC or carrier-board storage.
2. **Enable Interfaces**
   - `sudo raspi-config` → enable UART, I2C, SPI as needed.
3. **Install Services**
   ```bash
   sudo apt update
   sudo apt install python3-pip git
   git clone https://github.com/<your-org-or-user>/amon-cm4-services.git
   cd amon-cm4-services && ./install.sh
   ```
   - Starts a telemetry bridge (serial↔TCP/gRPC), data logger, and config UI.
4. **Connect to MCU**
   - Wire UART (and optionally SPI/CAN) between CM4 and MCU.
   - Confirm link with a ping/handshake command.
5. **Network Access**
   - SSH into CM4; forward telemetry to **Amon Ground Control** over Wi-Fi/Ethernet.

---

## Calibration & Testing

- **GPS/Calibration Port:** The v2 GPS connector doubles as a calibration interface during bring-up. PCB v3 do this over USB.  
- **Telemetry:** Log to external flash/microSD (v2 & v3) or CM4 eMMC + MCU storage (v4).  
- **LED Status:** On-board RGB LED signals power, init, errors, and link status.  
- **HIL/Replay (v2):** Reproduce flights by streaming recorded sensor frames back into the MCU (for controller tuning).

---

## Roadmap

- **v1**
   - Failed design
- **v2**
  - Incremental driver fixes and documentation cleanup
  - Manufacturing files refresh (Gerbers/BOM/PNP)
- **v3**
   - Modified v2 design with new MCU, connectors, sensors
   - Modified v2 firmware for new MCU (much higher clock: 168 MHz -> 480MHz)
   - Development of flight algorithm and controller + first flights 
- **v4**
  - Finalize MCU choice and Zephyr board port
  - Define IPC schema (CBOR/Proto) & reliability (CRC/seq/ACK)
  - CM4 services (logger, bridge, config UI)
  - Development of new flight algorithm and controllers + flights 

---

## Status & Disclaimer

- **Works, but in progress:** v2 firmware runs; development is active and some drivers may still have rough edges.  
- **New hardware incoming:** v2/v3/v4 will introduce a Zephyr-based MCU and CM4 companion; designs are evolving.
