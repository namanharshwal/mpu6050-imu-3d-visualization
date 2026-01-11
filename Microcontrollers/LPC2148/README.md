# LPC2148 MPU6050 3D Visualization

## Overview

This directory contains the LPC2148 ARM Cortex-M3 firmware and Python visualization script for MPU6050 real-time 3D orientation tracking.

## Hardware Setup

### Components Required
- **LPC2148 Microcontroller Board** (ARM Cortex-M3 @ 60MHz)
- **MPU6050 IMU Sensor** (GY-521 breakout recommended)
- **Jumper Wires**
- **USB-to-Serial Adapter** (e.g., CH340, PL2303)
- **Development Tools**: Keil µVision or open-source ARM compiler

### Wiring Diagram

```
MPU6050  →  LPC2148 (ARM9)
---------     ----------
VCC      →   3.3V
GND      →   GND
SDA      →   P0.18 (I2C0_SDA)
SCL      →   P0.19 (I2C0_SCL)
```

## Installation & Setup

### Step 1: Development Environment
Install one of the following:
- **Keil µVision** (professional, requires license)
- **LPCXpresso** (free, NXP official tool)
- **GCC ARM Toolchain** (open-source alternative)

### Step 2: Compile Firmware
1. Open `MPU6050_LPC2148.c` in your IDE
2. Configure project for LPC2148
3. Set clock frequency: 60 MHz
4. Build project
5. Connect LPC2148 via USB-Serial adapter
6. Flash using your IDE's download tool

### Step 3: Serial Verification
Connect USB-Serial adapter and check serial monitor:
```
Baud Rate: 115200
Data Bits: 8
Stop Bits: 1
Parity: None

Expected Output:
MPU6050 LPC2148 Initialization...
MPU6050 initialized
Calibrating sensor...
Calibration complete
MPU6050 LPC2148 3D Visualizer Ready
{"roll": 0.00, "pitch": 0.00, "yaw": 0.00}
```

## Python Visualization

### Installation
```bash
pip install pygame pyopengl numpy-stl pyserial
```

### Configuration
Edit `LPC2148_Visualizer.py`:
```python
SERIAL_PORT = 'COM3'          # Your USB-Serial port
BAUD_RATE = 115200            # Must match firmware
STL_FILE = 'model.STL'        # Your 3D model
```

### Run
```bash
python LPC2148_Visualizer.py
```

## Features

✅ **ARM Cortex-M3 Architecture**
- 32-bit processor @ 60 MHz
- Fast I2C communication (400 kHz)
- Low-cost development platform

✅ **I2C Implementation**
- Master mode, 400 kHz clock
- Interrupt-driven or polling-based
- Error handling and retry logic

✅ **Serial Communication**
- UART0 @ 115200 baud
- JSON formatted output
- Real-time IMU data streaming

✅ **Sensor Processing**
- Complementary filter algorithm
- Auto-calibration routine
- Euler angle calculation

## Firmware Configuration

Edit constants in `MPU6050_LPC2148.c`:

```c
#define I2C_CLOCK_RATE 400000  // I2C frequency
#define UART_BAUD 115200       // Serial baud rate
#define SAMPLE_RATE 100        // Hz
#define ALPHA 0.98             // Filter coefficient
```

## Data Format

JSON serial output:
```json
{"roll": -2.45, "pitch": 1.23, "yaw": 45.67}
```

## Performance

| Parameter | Specification |
|-----------|---------------|
| Processor | ARM Cortex-M3 @ 60 MHz |
| RAM | 32 KB SRAM |
| Flash | 512 KB |
| I2C Speed | 400 kHz |
| UART Speed | 115200 baud |
| Sampling Rate | 100 Hz |
| Cost | $ (very affordable) |

## Advantages

- **Low Cost**: One of the cheapest ARM processors
- **Industrial Grade**: Used in many commercial products
- **Robust**: Reliable embedded platform
- **Extensive Support**: Well-documented, community resources
- **Real-time**: Suitable for real-time control systems

## Troubleshooting

### No serial output
- Check USB-Serial adapter is recognized
- Verify baud rate is 115200
- Check I2C connections to MPU6050
- Try different USB ports

### MPU6050 not detected
- Verify I2C address (0x68)
- Check SDA/SCL wiring (P0.18/P0.19)
- Add 4.7k pull-up resistors if needed
- Verify power supply to MPU6050

### Incorrect orientation data
- Recalibrate sensor (restart device)
- Check sensor orientation matches firmware
- Verify complementary filter coefficients
- Review Euler angle calculations

## References

- [LPC2148 Datasheet](https://www.nxp.com/products/microcontrollers/arm/lpc-cortex-m3/)
- [ARM Cortex-M3 Programming Guide](https://developer.arm.com/)
- [I2C Protocol](https://en.wikipedia.org/wiki/I%C2%B2C)
- [MPU6050 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)

## License

MIT License - See LICENSE file in root directory

## See Also

- [Arduino Uno Implementation](../Arduino/Uno/README.md)
- [ESP8266 Implementation](../ESP8266/README.md)
- [ESP32 Implementation](../ESP32/README.md)
