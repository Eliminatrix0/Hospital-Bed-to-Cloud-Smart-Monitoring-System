# Pin Configuration

## STM32F407IGHx

| STM32 Pin | Peripheral | Module / Sensor | Function | Connection Note |
|-----------|-----------|-----------------|----------|-----------------|
| PA0 | GPIO Input | DS18B20 | Data | Requires 4.7kΩ pull-up resistor |
| PA1 | GPIO In/Out | DHT11 | Data | Room Temp & Humidity |
| PA2 | USART2 | XBee / ESP | TX (Transmit) | Connects to ESP/XBee RX |
| PA3 | USART2 | XBee / ESP | RX (Receive) | Connects to ESP/XBee TX |
| PB0 | GPIO Output | HX711 | SCK (Clock) | Weight Scale Clock |
| PB1 | GPIO Input | HX711 | DT (Data) | Weight Scale Data |
| PB6 | I2C1 | OLED / Sensors | SCL (Clock) | Shared by OLED, BMP180, MAX30102 |
| PB7 | I2C1 | OLED / Sensors | SDA (Data) | Shared by OLED, BMP180, MAX30102 |
| PD12 | GPIO Output | Buzzer | Signal | Active Low (GND=On, VCC=Off) |

## NodeMCU (ESP8266)

| NodeMCU Pin | ESP GPIO | Function | Connects To |
|-------------|----------|----------|-------------|
| D5 | GPIO 14 | SoftSerial RX | STM32 PA2 (or XBee TX) |
| D6 | GPIO 12 | SoftSerial TX | STM32 PA3 (or XBee RX) |
| VIN / 5V | - | Power Input | 5V Source |
| GND | - | Ground | Common Ground |
