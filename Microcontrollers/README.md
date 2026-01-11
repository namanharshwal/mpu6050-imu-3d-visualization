# Microcontroller Implementations

## Overview

This directory contains complete firmware implementations for various microcontrollers to interface with the MPU6050 IMU sensor and transmit real-time orientation data (roll, pitch, yaw) via serial communication to Python visualization scripts.

## Supported Microcontrollers

### 1. **Arduino Uno** 📦
Location: `./Arduino/Uno/`

**Features:**
- ✅ Complete corrected firmware with accurate Euler angle calculations
- ✅ Complementary filter for gyro + accelerometer fusion
- ✅ 100 Hz sampling rate (10ms update)
- ✅ Automatic sensor calibration
- ✅ JSON serial output at 115200 baud
- ✅ NaN/Inf safety checks
- ✅ Yaw mode toggle via serial command

**Hardware Requirements:**
- Arduino Uno (or compatible board)
- MPU6050 GY-521 module
- USB cable for programming and power
- I2C connections: SDA→A4, SCL→A5

**Libraries:**
- MPU6050 (by Electronic Cats or Jeff Rowberg)
- I2Cdev (by Jeff Rowberg)
- Wire (built-in)

**Setup:**
1. Install required libraries in Arduino IDE
2. Open `MPU6050_Arduino_Uno.ino`
3. Select Board: Arduino Uno
4. Select appropriate COM port
5. Upload firmware
6. Keep device still for 6 seconds during auto-calibration
7. Open Serial Monitor (115200 baud) to verify output

**Expected Output:**
```json
{"roll":0.00,"pitch":0.00,"yaw":0.00}
{"roll":1.23,"pitch":-2.45,"yaw":5.67}
```

**Python Compatibility:**
- Use: `Arduino_MPU6050_Visualizer.py`
- Baud Rate: 115200
- Data Format: JSON with keys: roll, pitch, yaw

---

### 2. **ESP8266 (NodeMCU v3)** 📡
Location: `./ESP8266/`

**Features:**
- ✅ WiFi-enabled microcontroller
- ✅ Same corrected firmware implementation as Arduino
- ✅ Serial communication or potential WiFi streaming
- ✅ 100 Hz sampling rate
- ✅ Low cost, widely available

**Hardware Requirements:**
- NodeMCU v3 or ESP8266 Development Board
- MPU6050 GY-521 module
- Micro-USB cable for programming
- I2C connections: SDA→D6/GPIO12, SCL→D5/GPIO14

**Setup:**
1. Install ESP8266 board support in Arduino IDE
2. Open `ESP8266_MPU6050_Firmware.ino`
3. Select Board: NodeMCU 1.0 (ESP-12E Module)
4. Select COM port
5. Upload firmware
6. Monitor via USB serial at 115200 baud

**Python Compatibility:**
- Use: `ESP8266_Visualizer.py`
- Baud Rate: 115200
- Same JSON output format as Arduino

---

## Wiring Diagrams

### Arduino Uno + MPU6050
```
MPU6050  →  Arduino Uno
VCC      →  5V
GND      →  GND
SCL      →  A5
SDA      →  A4
INT      →  Pin 2 (optional)
```

### ESP8266 + MPU6050
```
MPU6050  →  NodeMCU v3
VCC      →  3.3V
GND      →  GND
SCL      →  D5 (GPIO14)
SDA      →  D6 (GPIO12)
INT      →  D4 (GPIO2, optional)
```

---

## Firmware Specifications

### Sensor Configuration
- **Accelerometer Range:** ±2g (default)
- **Gyroscope Range:** ±250 deg/s (default)
- **Sample Rate:** 100 Hz (10ms interval)
- **Complementary Filter Alpha:** 0.98 (98% gyro, 2% accel)

### Calibration
- **Duration:** 6 seconds on startup
- **Samples:** 600 readings
- **Method:** Average offset subtraction
- **Status Messages:** JSON over serial

### Output Format
```json
{"roll":-45.32,"pitch":12.15,"yaw":-2.87}
```
where:
- **roll:** Rotation around X-axis (-180 to +180°)
- **pitch:** Rotation around Y-axis (-90 to +90°)
- **yaw:** Rotation around Z-axis (-180 to +180°)

---

## Troubleshooting

### Arduino Uno Issues

**Problem:** "Board not recognized"
- **Solution:** Install CH340 USB driver (for clone boards)
- **Check:** Device Manager for COM port

**Problem:** "Compilation errors"
- **Solution:** Verify all libraries are installed
- **Check:** Sketch → Include Library → Manage Libraries

**Problem:** "Serial data showing garbage values"
- **Solution:** Verify baud rate is 115200
- **Check:** Tools → Serial Monitor baud rate

**Problem:** "MPU6050 connection failed"
- **Solution:** Check I2C connections (A4, A5)
- **Verify:** I2C scanner sketch to detect address (0x68)
- **Try:** Different USB cable or power supply

### ESP8266 Issues

**Problem:** "Cannot upload to ESP8266"
- **Solution:** Hold GPIO0 to GND during upload
- **Verify:** Board selection is NodeMCU 1.0

**Problem:** "WiFi connection issues"
- **Note:** Firmware uses USB serial, not WiFi
- **Alternative:** Configure WiFi support in custom firmware

---

## Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| Sample Rate | 100 Hz | 10ms per update |
| Latency | ~15ms | Processing + serial transmission |
| Accuracy (Roll/Pitch) | ±2° | After calibration |
| Accuracy (Yaw) | ±5° | Gyro-only, subject to drift |
| Power Consumption (Arduino) | ~150mA | @ 5V |
| Power Consumption (ESP8266) | ~80mA | @ 3.3V |
| Serial Baud Rate | 115200 | Fixed |
| JSON Message Size | ~40 bytes | Per message |

---

## Advanced Features

### Yaw Mode Toggle
Send `'z'` or `'Z'` via serial to enable/disable yaw rotation:
```
Python → Arduino: b'z'
Arduino → Python: {"info":"Yaw mode toggled"}
```

### Calibration Reset
Restart the microcontroller to trigger new calibration cycle.

### Custom Sensitivity
Modify these constants in firmware:
```cpp
const float ACCEL_SENSITIVITY = 16384.0;  // LSB/g
const float GYRO_SENSITIVITY = 131.0;     // LSB/deg/s
const float ALPHA = 0.98;                 // Filter weight
```

---

## References

- **MPU6050 Datasheet:** [InvenSense MPU6050](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- **I2Cdev Library:** [Jeff Rowberg's I2Cdev](https://github.com/jrowberg/i2cdev)
- **Arduino Official:** [Arduino.cc](https://www.arduino.cc/)
- **ESP8266 Arduino:** [ESP8266 Community](https://github.com/esp8266/Arduino)

---

## License
MIT License - See LICENSE file in root directory

---

## Support
For issues or questions:
1. Check the troubleshooting section above
2. Verify hardware connections
3. Review Python visualization script error messages
4. Check GitHub Issues: [Project Issues](../../../issues)
