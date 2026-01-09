# MPU6050 3D Visualization for Arduino Uno

## Overview

Complete MPU6050 IMU implementation for Arduino Uno with I2C communication, complementary filter sensor fusion, and real-time 3D visualization via Python simulator.

**Author**: Naman Harshwal  
**License**: MIT  
**Arduino IDE Support**: 1.8.x and above

## Hardware Specifications

**Arduino Uno Rev3**
- **Microcontroller**: ATmega328P
- **Operating Voltage**: 5V
- **Clock Speed**: 16 MHz
- **Flash Memory**: 32 KB (0.5 KB bootloader)
- **SRAM**: 2 KB
- **EEPROM**: 1 KB
- **I2C Pins**: A4 (SDA), A5 (SCL)
- **Serial UART**: TX (D1), RX (D0)
- **Power**: USB or 7-12V DC connector

## Wiring Diagram

```
Arduino Uno Pinout:

MPU6050 Pin    Arduino Pin    Description
─────────────────────────────────────────────────
VCC     →      5V            Power Supply
GND     →      GND           Ground
SDA     →      A4            I2C Data (Analog 4)
SCL     →      A5            I2C Clock (Analog 5)
INT     →      D2            Interrupt (optional)
AD0     →      GND           Address select (0x68)
```

### Pull-up Resistors

**IMPORTANT**: Arduino Uno has internal 20kΩ pull-up resistors on I2C pins (A4/A5), but they may not be sufficient for long cables. Recommended:
- Add **4.7kΩ external pull-up resistors** on SDA and SCL lines
- Connect pull-ups to 5V power supply

## Circuit Diagram

```
        Arduino Uno
        ┌─────────────────┐
    5V  │ 5V        GND   │  GND
    ├───┤                 ├───┤
    │   │   A4 (SDA)──┬───┤   │
    │   │   A5 (SCL)──┼───┤   │
    │   │   D2 (INT)──┼───┤   │
    │   │             │   │   │
    │   └─────────────┼───┘   │
    │                 │       │
    │                 │       │
    │         MPU6050│       │
    │      ┌──────────┼───────┤
    ├──────┤VCC       │       │
    │      │GND───────┴───────┤
    │      │SDA───────┬───────┤
    │      │SCL───────┼───────┤
    │      │INT───────┤       │
    │      │AD0───────┤       │
    │      └──────────────────┘
    │                 │
    └─────────────────┘

4.7kΩ resistors: Pull SDA and SCL to 5V
```

## Installation Steps

### 1. Install Arduino IDE

- Download from https://www.arduino.cc/en/software
- Install ATmega328P board support (included by default)

### 2. Install Required Libraries

In Arduino IDE:
- **Sketch** → **Include Library** → **Manage Libraries...**
- Search and install:
  - `MPU6050` by InvenSense (or `MPU6050 by Joop Brooking`)
  - `I2Cdev` by Jeff Rowberg

**Alternative (Manual Installation)**:
```bash
# Download from GitHub:
# https://github.com/jrowberg/i2cdevlib
# Place in: Documents/Arduino/libraries/
```

### 3. Wiring

**Critical**: Double-check all connections before uploading code:
1. Arduino GND → MPU6050 GND
2. Arduino 5V → MPU6050 VCC
3. Arduino A4 → MPU6050 SDA (via 4.7kΩ resistor to 5V)
4. Arduino A5 → MPU6050 SCL (via 4.7kΩ resistor to 5V)
5. Arduino D2 → MPU6050 INT (optional)
6. MPU6050 AD0 → GND (for I2C address 0x68)

### 4. Upload Arduino Code

- Open `MPU6050_Arduino_Uno.ino` in Arduino IDE
- Select **Board**: Arduino Uno
- Select **Port**: COM port (USB)
- Click **Upload**
- Verify "Done uploading" message

### 5. Test Serial Communication

```bash
# Open Serial Monitor (Ctrl+Shift+M)
# Set baud rate to 115200
# You should see initialization messages
```

## Arduino Code Features

**MPU6050_Arduino_Uno.ino** includes:

- ✅ I2C communication setup
- ✅ MPU6050 initialization and self-test
- ✅ Accelerometer and gyroscope reading
- ✅ Temperature monitoring
- ✅ Serial data transmission (115200 baud)
- ✅ Complementary filter implementation
- ✅ Roll, pitch, yaw calculation
- ✅ Real-time data streaming (JSON format)

### Key Functions

```cpp
void setup()            // I2C, Serial, MPU6050 init
void loop()             // Main read/filter/transmit loop
void readMPU6050()      // Read sensor raw data
void complementaryFilter() // Sensor fusion
void transmitData()     // Send JSON to Python
```

## Python Visualizer

**Arduino_Uno_Visualizer.py** provides:

- ✅ Real-time serial data reception
- ✅ 3D OpenGL visualization
- ✅ Roll/Pitch/Yaw display
- ✅ Accelerometer/Gyroscope plots
- ✅ Temperature monitoring
- ✅ Data logging to CSV

### Running Python Visualizer

```bash
# Requirements
pip3 install pyserial numpy pygame PyOpenGL

# Run visualizer
python3 Arduino_Uno_Visualizer.py

# Select COM port when prompted
# Default: COM3 (Windows), /dev/ttyUSB0 (Linux), /dev/ttyUSB0 (macOS)
```

## Communication Protocol

Arduino sends JSON data every 10ms (100 Hz):

```json
{
  "timestamp": 1234567890,
  "accel_x": 0.05,
  "accel_y": -0.02,
  "accel_z": 9.81,
  "gyro_x": 0.001,
  "gyro_y": 0.002,
  "gyro_z": -0.0005,
  "roll": 2.5,
  "pitch": -1.8,
  "yaw": 45.2,
  "temp": 35.2
}
```

## Sensor Calibration

Arduino automatically calibrates on startup:
1. Keep device still for 2 seconds
2. Place on flat, level surface
3. Serial prints calibration status

Manual recalibration:
```cpp
// In Arduino code, uncomment and recompile:
// accel_calib_x = 0;  // Reset offsets
// accel_calib_y = 0;
// accel_calib_z = 0;
```

## Troubleshooting

### I2C Not Detected

**Error**: "Could not find MPU6050"

**Solutions**:
1. Check wiring (especially A4/A5)
2. Verify 4.7kΩ pull-up resistors installed
3. Test with I2C scanner sketch:
```cpp
#include <Wire.h>
void setup() {
  Wire.begin();
  Serial.begin(115200);
  for(byte i = 0x08; i < 0x78; i++) {
    Wire.beginTransmission(i);
    if(Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
    }
  }
}
void loop() {}
```

### Serial Communication Issues

**Error**: Python can't connect to COM port

**Solutions**:
1. Check device manager for correct COM port
2. Verify baud rate is 115200
3. Ensure Arduino code compiled successfully
4. Try different USB cable
5. Update Arduino drivers

### Noisy Sensor Data

**Issue**: Oscillating values in visualizer

**Solutions**:
1. Add **0.1µF capacitors** across MPU6050 power pins
2. Use shielded I2C cable
3. Keep cable away from power lines
4. Add ferrite choke to USB cable
5. Adjust complementary filter weights in code

## Performance Characteristics

| Metric | Value |
|--------|-------|
| Sampling Rate | 100 Hz |
| Data Transmission | 115200 baud |
| Latency | ~100ms |
| Memory Usage | ~1.8 KB SRAM |
| Power Consumption | 50mA |
| Update Rate | 10ms |

## Applications

- **Robotics**: Mobile robot orientation tracking
- **Gesture Recognition**: Hand motion capture
- **Drone Control**: Flight stabilization logging
- **Gaming**: Motion-controlled games
- **Education**: Physics demonstrations
- **DIY Projects**: Home automation sensors

## Libraries Used

- **Wire.h**: Arduino I2C library (built-in)
- **MPU6050.h**: Sensor driver library
- **I2Cdev.h**: I2C communication library

## Code Size

- **Flash**: ~8-12 KB (with libraries)
- **SRAM**: ~800 bytes (runtime)
- **EEPROM**: ~200 bytes (calibration storage)

## Limitations

- Limited computational power (8-bit, 16 MHz)
- 100 Hz max sampling (I2C limited)
- Simple complementary filter only
- No multi-threading capability
- Limited SRAM for data buffering

## Next Steps

1. Upload Arduino code from `MPU6050_Arduino_Uno.ino`
2. Run Python visualizer: `Arduino_Uno_Visualizer.py`
3. See 3D rotation in real-time
4. Experiment with different motions
5. Modify code for specific applications

## References

- **MPU6050 Datasheet**: InvenSense 6-Axis MEMS MotionTracking Device
- **Arduino Uno Datasheet**: Microchip ATmega328P
- **I2C Protocol**: NXP I2C-bus Specification
- **Arduino Documentation**: https://www.arduino.cc/en/Reference
- **GitHub I2CdevLib**: https://github.com/jrowberg/i2cdevlib

## Author

**Naman Harshwal**
- GitHub: @namanharshwal
- Repository: mpu6050-imu-3d-visualization
- License: MIT

## Support

For issues:
1. Check wiring connections
2. Verify library installation
3. Review Serial Monitor output
4. Test with I2C scanner sketch
5. Check Arduino IDE console for errors

## License

MIT License - Open source and free for personal/commercial use
