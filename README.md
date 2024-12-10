# STM32 Command Parser and Control System

## Project Overview
This project implements a modular STM32-based command parser and control system using FreeRTOS. It enables communication via UART and processes JSON commands to control peripherals such as sensors (LM35, LDR) and relays. Designed for real-time task scheduling, the system emphasizes scalability and maintainability for embedded applications.


---

## Key Features
- **Asynchronous UART communication** for command reception and JSON data transmission.
- **Task-based design** using FreeRTOS for real-time command parsing and periodic operations.
- Modular peripheral drivers for LM35, LDR, and Relay.
- JSON-based structured communication using the lightweight cJSON library.
- Extendable architecture for adding more peripherals or command sets.

---

## System Requirements

### Hardware
- **STM32F4xx microcontroller** (e.g., STM32F401 or STM32F411 Blackpill board).
- **LM35 temperature sensor.**
- **LDR (Light Dependent Resistor)** for light measurement.
- **Relay module** for external device control.
- **Power supply:** 5V for the board and peripherals.

### Software
- **Development Tools:**
  - PlatformIO .
  - FreeRTOS.
  - Compiler: arm-none-eabi-gcc.
- **Libraries:**
  - cJSON (lightweight JSON parser).
- **OS:** Windows, Linux, or macOS.

---

## Architecture and Design

### System Architecture
The project is divided into multiple layers to ensure modularity:
1. **Application Layer**:
   - Handles command parsing and functional execution.
   - Implements tasks for JSON parsing and periodic operations.
2. **Middleware Layer**:
   - cJSON for JSON parsing and serialization.
   - FreeRTOS for real-time scheduling and task management.
3. **Hardware Abstraction Layer (HAL)**:
   - Manages low-level interaction with STM32 peripherals (UART, GPIO, ADC).

### Design Patterns
- **Task-Oriented Design:** FreeRTOS tasks manage parsing, command handling, and periodic operations.
- **Callback Mechanism:** UART reception is interrupt-driven via a callback.
- **Modular Peripheral Drivers:** Each peripheral is encapsulated for reuse.

---

## Functional Specifications

### Supported Commands
| Command | Node ID  | Data             | Functionality                           |
|---------|----------|------------------|-----------------------------------------|
| `ENA`   | 0x80/0x81 | null             | Enable LM35 or LDR sensor.             |
| `DIS`   | 0x80/0x81 | null             | Disable LM35 or LDR sensor.            |
| `DUR`   | 0x80/0x81 | Integer duration | Set measurement period for sensors.    |
| `ACT`   | 0x50      | "0" or "1"       | Activate/deactivate relay.             |
| `STA`   | 0x50      | null             | Fetch relay status as JSON response.   |

---

## Setup and Installation

### Hardware Setup
1. Connect the LM35 sensor to an ADC pin .
2. Connect the LDR sensor to another ADC pin .
3. Connect the relay module to a GPIO pin .
4. Connect UART pins for communication:
   - PA9 (TX) and PA10 (RX) for USART1.
5. Ensure proper power supply connections to all peripherals.

### Software Setup
1. Clone the Repository:
   ```bash
   git clone https://github.com/MahmoudAbou-Hawis/Task
   cd stm32-command-parser

2. compile the project using platformIO
