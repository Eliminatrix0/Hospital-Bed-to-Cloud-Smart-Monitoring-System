# Hospital Bed-to-Cloud Smart Monitoring System

An IoT-based patient monitoring system that acquires, processes, and transmits patient vitals to a cloud platform in real-time using **STM32**, **XBee**, **NodeMCU**, and **ThingsBoard**.

## System Architecture

```
[Sensors] → [STM32F407] → [XBee (Zigbee)] → [XBee Coordinator] → [NodeMCU (ESP8266)] → [ThingsBoard Cloud]
```

**Bedside Unit (STM32F407)** acquires sensor data via I2C and GPIO, processes it, and transmits wirelessly through **XBee (Zigbee)** to the **NodeMCU (ESP8266)** gateway, which publishes telemetry to **ThingsBoard** via MQTT.

## Features

- **Real-time vital monitoring**: Body temperature, heart rate, SpO2, room temperature, humidity, atmospheric pressure, and patient weight
- **Adaptive transmission**: 10s updates under normal conditions, 5s updates when critical thresholds are breached (e.g., fever > 100°F)
- **Low-power standby mode**: STM32 enters standby via PWR/SCB register control, waking on hardware reset with secure PIN re-entry
- **Local alerts**: Active-low buzzer triggers on critical vitals or incorrect PIN entry
- **Cloud dashboard**: ThingsBoard IoT platform with live gauges, graphs, and alert widgets accessible via web/mobile

## Hardware Components

| Component | Function | Protocol |
|-----------|----------|----------|
| STM32F407 Discovery | Main controller & sensor hub | - |
| DS18B20 | Body temperature | GPIO (1-Wire) |
| DHT11 | Room temperature & humidity | GPIO |
| MAX30102 | Heart rate & SpO2 | I2C |
| BMP180 | Atmospheric pressure | I2C |
| HX711 + Load Cell | Patient weight / bed occupancy | GPIO |
| SSD1306 OLED | Bedside display | I2C |
| XBee (x2) | Zigbee wireless link | UART |
| NodeMCU ESP8266 | Wi-Fi gateway to cloud | UART / MQTT |
| Buzzer | Local audible alerts | GPIO (Active Low) |
| 4x4 Keypad | PIN entry & system control | GPIO Matrix |

## Pin Configuration

### STM32F407

| Pin | Peripheral | Module | Function |
|-----|-----------|--------|----------|
| PA0 | GPIO | DS18B20 | Data (4.7kΩ pull-up) |
| PA1 | GPIO | DHT11 | Room Temp & Humidity |
| PA2 | USART2 | XBee | TX |
| PA3 | USART2 | XBee | RX |
| PB0 | GPIO | HX711 | SCK (Clock) |
| PB1 | GPIO | HX711 | DT (Data) |
| PB6 | I2C1 | OLED/BMP180/MAX30102 | SCL |
| PB7 | I2C1 | OLED/BMP180/MAX30102 | SDA |
| PD12 | GPIO | Buzzer | Signal (Active Low) |

### NodeMCU (ESP8266)

| Pin | GPIO | Function | Connects To |
|-----|------|----------|-------------|
| D5 | GPIO 14 | SoftSerial RX | XBee TX |
| D6 | GPIO 12 | SoftSerial TX | XBee RX |

## Project Structure

```
├── stm32_firmware/
│   └── main.c                  # STM32 firmware (sensor acquisition, processing, XBee TX)
├── nodemcu_gateway/
│   └── nodemcu_gateway.ino     # NodeMCU code (XBee RX, MQTT publish to ThingsBoard)
└── docs/
    └── pin_configuration.md    # Detailed pin mapping
```

## Data Flow

1. **STM32** reads sensors → processes data → formats string:
   ```
   DHT:24.0,Hum:30.0,BMP:984.7,DS:74.9,Stat:No Fever,HR:0,SpO2:0,Wt:0.00
   ```
2. Transmits via **UART → XBee EndPoint → Zigbee → XBee Coordinator → NodeMCU**
3. **NodeMCU** parses string, builds JSON, publishes to ThingsBoard via **MQTT**
4. **ThingsBoard** dashboard displays live data with adaptive refresh rates

## Adaptive Transmission Logic

| Condition | Update Interval | Action |
|-----------|----------------|--------|
| Normal (Temp < 100°F) | 10 seconds | Silent operation |
| Critical (Temp ≥ 100°F) | 5 seconds | Buzzer ON + frequent updates |

## Tools & Technologies

- **Firmware**: Bare-metal C (STM32), Arduino C++ (NodeMCU)
- **IDE**: Keil µVision (STM32), Arduino IDE (NodeMCU)
- **Communication**: UART, I2C, GPIO, Zigbee, Wi-Fi, MQTT
- **Cloud**: ThingsBoard IoT Platform
- **Configuration**: Digi XCTU (XBee)

