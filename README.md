# Automated Toll Collection System

## Project Overview
An embedded systems project implementing an automated toll collection mechanism using STM32F4 microcontroller with integrated RFID, GSM, and database.

## Hardware Components
- STM32F4 Microcontroller Board
- RFID Reader
- GSM Module
- Stepper Motor
- OLED Display

## Key Features
- Automated vehicle identification via RFID
- Real-time transaction processing
- Barrier control using stepper motor
- Transaction logging and tracking
- Cellular network integration for SMS transmission

## System Architecture
The system operates through the following workflow:
1. Vehicle approaches toll booth
2. RFID tag is scanned
3. Vehicle details verified against database
4. Transaction processed
5. Barrier raised/lowered
6. SMS sent to number attached with RFID tag (queried from database)
7. Transaction logged

## Technical Specifications
- Microcontroller: STM32F4 Series
- Communication Protocols: RFID RC522, GSM SIM868
- Display: OLED SSD1306
- Actuator: Stepper Motor SG90
- Data Storage: Structs stored locally on STM32F4 

## Setup and Installation
### Prerequisites
- Keil uVision 5
- STM32F4xx library for uVision 5
- ST-Link Utility (software)

### Compilation Steps
1. Clone repository
2. Create new project in Keil uVision 5
3. Configure project settings - enable I2C, SPI, GPIO; use 'classic' in STM32Cube framework in run time environment
4. Copy files from the repo to 'source group'
5. Build and flash to microcontroller

## Pin Configurations
PDF provided in the repo as well

| Component | STM32F4 Pin | Peripheral Pin Function |
|-----------|-------------|------------------------|
| OLED (SSD1306) | 5V | VCC Power Supply |
| OLED (SSD1306) | GND | Ground |
| OLED (SSD1306) | PB7 | SDA Data Line (I2C) |
| OLED (SSD1306) | PB6 | SCL Clock Line (I2C) |
| GSM SIM868 | GND | Ground |
| GSM SIM868 | PC10 | TX Transmit (UART) |
| GSM SIM868 | PC11 | RX Receive (UART) |
| RC522 RFID | 3.3V | VCC Power Supply |
| RC522 RFID | PA8 | RST Reset (GPIO Output) |
| RC522 RFID | PA6 | MISO Master In Slave Out (SPI) |
| RC522 RFID | PA7 | MOSI Master Out Slave In (SPI) |
| RC522 RFID | PA5 | SCK Serial Clock (SPI) |
| RC522 RFID | PB0 | NSS/CSP Chip Select (SPI) |
| RC522 RFID | GND | Ground |
| SG90 Servo | PA0 | Signal PWM Signal (Servo Control) |
| SG90 Servo | 5V | VCC Power Supply (5V) |
| SG90 Servo | GND | Ground |
