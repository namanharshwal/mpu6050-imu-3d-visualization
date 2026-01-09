# MPU6050 3D Visualization for ESP8266 NodeMCU

**Version**: 1.0.0  
**Author**: Naman Harshwal  
**License**: MIT  
**Platform**: ESP8266 NodeMCU (32-bit microcontroller)  
**Date**: January 2026

## Overview

Compact 3D visualization system for MPU6050 IMU sensor on ESP8266 NodeMCU with WiFi connectivity, ideal for IoT applications and wireless robotics.

### Key Features

- **WiFi Enabled**: Native WiFi for wireless data transmission
- **Low Power Consumption**: 80mA typical operation
- **Web Dashboard**: Real-time sensor data visualization
- **OTA Updates**: Over-the-air firmware updates
- **JSON Data Format**: Standard communication protocol
- **I2C Communication**: Standard I2C interface

## Hardware Requirements

- ESP8266 NodeMCU v3
- MPU6050 GY-521 breakout
- 4.7kΩ pull-up resistors (x2)
- USB power supply

## Wiring

```
ESP8266       MPU6050
D1 (GPIO5) -> SCL
D2 (GPIO4) -> SDA
3.3V       -> VCC
GND        -> GND
```

## Arduino IDE Setup

1. Install ESP8266 board support:
   - File → Preferences
   - Add: `http://arduino.esp8266.com/stable/package_esp8266com_index.json`
   - Boards → Board Manager → Install ESP8266

2. Install Libraries:
   ```
   - MPU6050 by InvenSense
   - ArduinoJson
   ```

3. Configure:
   - Board: NodeMCU 1.0
   - Upload Speed: 921600

## Firmware Installation

See: `MPU6050_ESP8266_NodeMCU.ino`

```bash
# Compile and upload via Arduino IDE
Sketch → Upload
```

## Web Access

- IP: `http://192.168.x.x:8080` (default)
- Real-time sensor dashboard with 3D visualization

## Power Specifications

| Parameter | Value |
|-----------|-------|
| Current (Idle) | 15-20mA |
| Current (Active) | 80-100mA |
| Sleep Power | 10µA |

## Troubleshooting

- **Won't Upload**: Check baud rate (921600)
- **No WiFi**: Verify SSID/password in code
- **I2C Error**: Check pull-up resistors

## Applications

- IoT sensor monitoring
- Wireless robot orientation tracking
- WiFi-enabled motion capture
- Smart device integration

## References

- [ESP8266 Documentation](https://docs.espressif.com/projects/esp8266-rtos-sdk/)
- [MPU6050 Datasheet](https://invensense.tdk.com/)
- [ArduinoJson Library](https://arduinojson.org/)

## License

MIT License
