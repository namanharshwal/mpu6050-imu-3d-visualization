# ESP32 MPU6050 3D Visualization

## Overview

This directory contains the ESP32 firmware and Python visualization script for reading MPU6050 sensor data in real-time and transmitting it to a Python OpenGL 3D visualization for orientation tracking.

## Hardware Setup

### Components Required
- **ESP32 Development Board** (NodeMCU-32S or similar)
- **MPU6050 IMU Sensor** (GY-521 breakout board recommended)
- **Jumper Wires**
- **USB Cable** (Micro-USB for programming)
- **Optional:** Power bank for portability

### Wiring Diagram

```
MPU6050  →  ESP32 (NodeMCU-32S)
---------     ------------------
VCC      →   3V3 (or 5V with voltage divider)
GND      →   GND
SDA      →   GPIO21 (D21)
SCL      →   GPIO22 (D22)
INT      →   GPIO14 (D14) - Optional, not used in basic firmware
```

**Note:** ESP32 has built-in 3.3V regulator. Connect MPU6050 VCC to ESP32 3V3 for stable operation.

## Installation & Setup

### Step 1: Install Arduino IDE for ESP32
1. Download [Arduino IDE](https://www.arduino.cc/en/software) (1.8.19 or later)
2. Go to File → Preferences
3. Add this URL to "Additional Boards Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to Tools → Board → Boards Manager
5. Search for "esp32" and install "ESP32 by Espressif Systems"
6. Select Board: Tools → Board → ESP32 → Node32s or your ESP32 variant

### Step 2: Install Required Libraries
1. Sketch → Include Library → Manage Libraries
2. Install the following:
   - **MPU6050** by Electronic Cats (recommended)
   - **ArduinoJson** by Benoit Blanchon (for JSON serialization)
   - **Wire** (built-in, for I2C communication)

### Step 3: Upload Firmware
1. Connect ESP32 via USB
2. Select correct COM port: Tools → Port → COM# (Windows) or /dev/ttyUSB# (Linux)
3. Set Upload Speed to **921600** (faster for ESP32)
4. Open `MPU6050_ESP32.ino`
5. Click Upload (→ button)
6. Wait for upload to complete

### Step 4: Verify Connection
1. Open Tools → Serial Monitor
2. Set Baud Rate to **115200**
3. You should see:
   ```
   MPU6050 ESP32 Initialization Starting...
   ✓ MPU6050 connected successfully
   ✓ MPU6050 configured
   === Starting Calibration ===
   ✓ Calibration complete!
   ✓ Initialization complete. Streaming data...
   {"status": "ready"}
   {"roll": 2.34, "pitch": -1.56, "yaw": 0.12, "temp": 28.5}
   ```

## Python Visualization Setup

### Step 1: Install Python Dependencies
```bash
pip install pygame pyopengl numpy-stl pyserial
```

### Step 2: Configure Serial Port
Edit `ESP32_Visualizer.py` and update:
```python
SERIAL_PORT = 'COM3'          # Change to your ESP32 port
BAUD_RATE = 115200            # Keep at 115200
STL_FILE = 'model.STL'        # Path to your 3D model
```

### Step 3: Run Visualization
```bash
python ESP32_Visualizer.py
```

You should see:
```
[ESP32] Connecting to COM3 at 115200 baud...
[ESP32] Connected!
✓ Loaded STL: model.STL
[ESP32 Visualizer] Starting 3D visualization...
[ESP32 Visualizer] Press 'Z' to toggle yaw rotation
[ESP32 Visualizer] Press 'ESC' to exit
[ESP32] FPS: 59.8 | Roll:  -0.23° | Pitch:   1.45° | Yaw:   0.00° | Temp:  28.5°C
```

## Features

✅ **High-Speed Communication**
- Baud rate: 115200 (faster than Arduino/ESP8266)
- JSON formatted output for easy parsing
- Real-time 100 Hz sampling rate

✅ **Advanced Sensor Fusion**
- Complementary filter algorithm
- Gyroscope drift correction
- Automatic calibration on startup
- Temperature monitoring

✅ **Robust I2C Implementation**
- Configurable I2C clock (400kHz)
- Automatic connection retry
- Error handling and logging

✅ **Enhanced Firmware Features**
- ArduinoJson library for clean JSON output
- 500-sample calibration routine
- Built-in temperature sensor reading
- Serial debugging information

## Firmware Configuration

Edit `MPU6050_ESP32.ino` to modify:

```cpp
#define SAMPLE_RATE 100        // Sampling rate in Hz (default: 100)
#define BAUD_RATE 115200       // Serial baud rate
const float ALPHA = 0.98;      // Complementary filter weight (0.95-0.99)

// I2C Configuration
Wire.begin(21, 22);            // SDA=GPIO21, SCL=GPIO22
Wire.setClock(400000);         // 400kHz I2C clock
```

## Data Format

Serial output is JSON-formatted:

```json
{"roll": -2.45, "pitch": 1.23, "yaw": 45.67, "temp": 28.5}
```

| Field | Range | Unit | Description |
|-------|-------|------|-------------|
| roll | -180 to 180 | degrees | Rotation around Z-axis |
| pitch | -90 to 90 | degrees | Rotation around X-axis |
| yaw | -180 to 180 | degrees | Rotation around Y-axis |
| temp | -40 to 85 | °C | Internal temperature sensor |

## Troubleshooting

### Issue: "Could not open port COM3"
**Solution:**
- Verify correct COM port in Python script
- Check USB cable connection
- Install CH340 drivers if using clone boards
- Try different USB ports

### Issue: "MPU6050 connection failed!"
**Solution:**
- Check I2C wiring (SDA to GPIO21, SCL to GPIO22)
- Verify power supply to MPU6050 (3.3V stable)
- Add 10k pull-up resistors on SDA/SCL lines
- Check I2C address with I2C scanner sketch

### Issue: Slow serial communication
**Solution:**
- Reduce SAMPLE_RATE in firmware
- Check baud rate matches (115200)
- Verify USB cable quality
- Close other serial monitors

### Issue: Drifting angles
**Solution:**
- Run calibration again (restart ESP32)
- Increase ALPHA value (0.98 → 0.99) for more gyro trust
- Check sensor mounting for vibrations
- Verify complementary filter coefficients

## Performance Specifications

| Parameter | Specification |
|-----------|---------------|
| Processor | Xtensa Dual-Core 32-bit LX6 @ 240 MHz |
| RAM | 520 KB SRAM |
| Serial Speed | 115200 baud (10x faster than Arduino) |
| I2C Speed | 400 kHz |
| Sampling Rate | 100 Hz |
| Update Latency | ~10 ms |
| Gyroscope Range | ±250 °/s |
| Accelerometer Range | ±2g |
| Power Consumption | ~80 mA (active), ~10 μA (deep sleep) |

## Advantages Over Arduino/ESP8266

| Feature | Arduino Uno | ESP8266 | ESP32 |
|---------|------------|---------|-------|
| Processor Speed | 16 MHz | 80 MHz | 240 MHz |
| RAM | 2 KB | 160 KB | 520 KB |
| Baud Rate | 38400 | 74880 | 115200 |
| WiFi Support | ❌ | ✓ | ✓ |
| Bluetooth | ❌ | ❌ | ✓ |
| Dual Core | ❌ | ❌ | ✓ |
| Cost | $$ | $ | $ |

## Advanced Usage

### WiFi Integration
ESP32 can transmit MPU6050 data over WiFi:
```cpp
#include <WiFi.h>
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
// Configure WiFi in setup()
```

### Bluetooth Visualization
Connect to Android/iOS app for wireless visualization:
```cpp
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
SerialBT.begin("MPU6050_ESP32");
```

### MQTT Data Streaming
Send sensor data to MQTT broker for IoT applications:
```cpp
#include <PubSubClient.h>
// Configure MQTT in setup() and loop()
```

## Energy Efficiency

ESP32 Deep Sleep Mode (5 μA consumption):
```cpp
esp_sleep_enable_timer_wakeup(5 * 1000000);  // Wake after 5 seconds
esp_deep_sleep_start();
```

## References

- [ESP32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
- [Arduino-ESP32 GitHub](https://github.com/espressif/arduino-esp32)
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [ArduinoJson Documentation](https://arduinojson.org/)

## License

MIT License - See LICENSE file in root directory

## See Also

- [Arduino Uno Implementation](../Arduino/Uno/README.md)
- [ESP8266 Implementation](../ESP8266/README.md)
- [LPC2148 Implementation](../LPC2148/README.md)
