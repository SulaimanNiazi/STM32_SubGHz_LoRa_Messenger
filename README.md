# STM32 Sub-GHz LoRa Messenger

A lightweight STM32 CubeIDE project for NUCLEO-WL55JC1 boards enabling Sub-GHz LoRa messaging between master and slave nodes using a ping-pong synchronization mechanism to avoid collisions. Messages can be sent via UART while the nodes stay synchronized using the low-level radio driver.

---

## Features

* **Sub-GHz LoRa Communication**: Uses STM32 low-level radio driver for reliable messaging.
* **Master-Slave Ping-Pong**: Keeps nodes synchronized to prevent RF collisions.
* **UART Messaging**: Enter messages through UART; transmitted in between ping-pong cycles.
* **Configurable LoRa Parameters**: Frequency, power, bandwidth, spreading factor, coding rate, and preamble length are defined in the user code section.
* **High-Speed UART**: 115200 baud rate for fast and reliable serial communication.

---

## Hardware

* **MCU**: 2 NUCLEO-WL55JC1 boards

---

## Configuration

Modify the LoRa and buffer parameters in the `USER CODE BEGIN PM` section:

```c
/* USER CODE BEGIN PM */

#define MAX_BUFFER_SIZE         255
#define RF_FREQ                 868000000   // Hz = 868 MHz
#define TX_POWER                14          // dBm
#define LORA_BANDWIDTH          LORA_BW_125 // 125 kHz
#define LORA_SPREADING_FACTOR   LORA_SF7    // SF7
#define LORA_CODING_RATE        LORA_CR_4_5 // 4/5
#define LORA_PREAMBLE_LENGTH    8           // Same for Tx and Rx

/* USER CODE END PM */
```

> ⚠️ Ensure these parameters match on both master and slave nodes for proper communication.

---

## Usage

1. Build the project in **STM32CubeIDE**.
2. Flash the firmware to both NUCLEO-WL55JC1 boards.
3. Open a UART terminal at **115200 baud**.
4. Type your message and press Enter to send it.
5. The boards maintain synchronization using ping-pong messaging while transmitting user messages in between.

---

## Notes

* Designed to demonstrate Sub-GHz LoRa communication with collision avoidance using ping-pong timing.
* Can be adapted for other STM32 boards with Sub-GHz LoRa support by updating the pin and radio configurations.
* UART serves as the primary user input interface.

---

## License

This project is licensed under the **MIT License** – see the [LICENSE](LICENSE) file for details.
