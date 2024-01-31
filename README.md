# TinyHouseSmartSystem

This repository contains firmware for the TinyHouseSmartSystem, an STM32-based smart system designed for monitoring temperature, humidity, and air quality. The firmware interfaces with various sensors and an OLED display to provide real-time data visualization.

## Hardware Requirements

- STM32 microcontroller board
- SSD1306 OLED display
- DHT11 temperature and humidity sensor
- MQ135 air quality sensor

## Dependencies

- STM32CubeIDE for development and compiling
- STM32CubeMX for configuring peripherals
- SSD1306 library for OLED display
- Standard Peripheral Library provided by STMicroelectronics

## Setup Instructions

1. Clone the repository to your local machine.
2. Open the project in STM32CubeIDE.
3. Configure the project settings as per your hardware setup (e.g., GPIO pins, clock configuration).
4. Include the necessary libraries in the project.
5. Build and flash the firmware onto your STM32 board.

## Functionality

The firmware operates in the following states:

1. **Idle State**: Initial state where the microcontroller waits for a predefined duration before proceeding.
2. **Temperature and Humidity Read State**: Reads data from the DHT11 sensor, calculates temperature and humidity, and prepares it for display.
3. **Air Quality Read State**: Reads data from the MQ135 sensor, determines air quality, and prepares it for display.
4. **Display Results State**: Displays the collected temperature, humidity, and air quality data on the OLED display.

## Customization

- Modify GPIO configurations in `MX_GPIO_Init()` function according to your hardware connections.
- Adjust sensor read functions (`DHT11_Start()`, `DHT11_Read()`, `getAirQuality()`) if using different sensors or protocols.
- Customize OLED display content and layout in the respective state handler functions.

## Contributions

Contributions to this project were made by the Melon Electronics members.

