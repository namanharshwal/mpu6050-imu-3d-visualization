# Complete MPU6050 Setup Guide for All Microcontrollers

**Author**: Naman Harshwal  
**Repository**: mpu6050-imu-3d-visualization  
**License**: MIT  
**Version**: 2.0 (Reorganized Microcontroller Structure)

## Microcontroller Support Matrix

| Microcontroller | Processor | RAM | Flash | I2C | Baud | Status | Folder |
|-----------------|-----------|-----|-------|-----|------|--------|--------|
| **Arduino Uno** | ATmega328P | 2KB | 32KB | 100/400kHz | 115200 | ✅ Complete | `Microcontrollers/Arduino/Uno` |
| **Arduino Nano** | ATmega328P | 2KB | 30KB | 100/400kHz | 115200 | 📝 Planned | `Microcontrollers/Arduino/Nano` |
| **ESP8266 NodeMCU** | Xtensa L106 | 160KB | 4MB | 100/400kHz | 115200 | ✅ Complete | `Microcontrollers/ESP8266/NodeMCU` |
| **ESP32** | Dual-core Xtensa | 520KB | 4-16MB | 100/400kHz | 115200 | 📝 Planned | `Microcontrollers/ESP32` |

## Folder Structure

```
mpu6050-imu-3d-visualization/
├── Microcontrollers/
│   ├── Arduino/
│   │   ├── Uno/
│   │   │   ├── README.md
│   │   │   ├── MPU6050_Arduino_Uno.ino
│   │   │   └── Arduino_Uno_Visualizer.py
│   │   └── Nano/
│   │       ├── README.md
│   │       ├── MPU6050_Arduino_Nano.ino
│   │       └── Arduino_Nano_Visualizer.py
│   ├── ESP8266/
│   │   ├── NodeMCU/
│   │   │   ├── README.md
│   │   │   ├── MPU6050_ESP8266_NodeMCU.ino
│   │   │   └── ESP8266_Visualizer.py
│   │   └── SETUP_GUIDE.md
│   └── ESP32/
│       ├── README.md
│       ├── MPU6050_ESP32.ino
│       └── ESP32_Visualizer.py
├── MICROCONTROLLERS_COMPLETE_SETUP.md (this file)
├── RaspberryPi/
├── Jetson/
└── ...
```

## Arduino Uno

**✅ FULLY IMPLEMENTED**

### Quick Start

```bash
cd Microcontrollers/Arduino/Uno
# Read README.md for detailed wiring and setup
```

### Key Features
- ATmega328P 8-bit microcontroller
- I2C on A4 (SDA) and A5 (SCL)
- 100 Hz sampling rate
- 115200 baud serial transmission
- Built-in I2C pull-ups (20kΩ - add 4.7kΩ externally)

### Wiring
```
MPU6050   →  Arduino Uno
VCC       →  5V
GND       →  GND
SDA       →  A4
SCL       →  A5
INT       →  D2 (optional)
AD0       →  GND
```

### Files
- `README.md` - Comprehensive setup guide with circuit diagrams
- `MPU6050_Arduino_Uno.ino` - Arduino IDE firmware (Arduino C++)
- `Arduino_Uno_Visualizer.py` - Python 3D visualization script

### Installation
```bash
# 1. Install Arduino IDE from arduino.cc
# 2. Install libraries: MPU6050, I2Cdev
# 3. Upload MPU6050_Arduino_Uno.ino to board
# 4. Run: python3 Arduino_Uno_Visualizer.py
```

---

## Arduino Nano

**📝 IMPLEMENTATION STRUCTURE READY**

The Arduino Nano folder structure is ready for implementation. Nano uses identical I2C pins (A4/A5) as Uno.

### Specifications
- ATmega328P (same processor as Uno)
- Compact form factor (smaller board)
- Same I2C pins: A4 (SDA), A5 (SCL)
- 100 Hz sampling capability
- 115200 baud serial

### Implementation Plan
```bash
Microcontrollers/Arduino/Nano/
├── README.md                    # Setup guide with pinout
├── MPU6050_Arduino_Nano.ino     # Arduino IDE code
└── Arduino_Nano_Visualizer.py   # Python visualizer
```

### Note
Code will be nearly identical to Uno (same ATmega328P), with minor pin labeling differences in documentation.

---

## ESP8266 NodeMCU

**✅ FULLY IMPLEMENTED**

### Quick Start

```bash
cd Microcontrollers/ESP8266/NodeMCU
# Read README.md for WiFi and setup options
```

### Key Features
- Xtensa L106 32-bit processor
- WiFi connectivity (802.11 b/g/n)
- I2C on GPIO4 (SDA) and GPIO5 (SCL) - labeled D2/D1
- 50 Hz sampling rate (WiFi overhead)
- 115200 baud UART
- Low cost, popular hobbyist board

### Wiring
```
MPU6050   →  NodeMCU ESP8266
VCC       →  3.3V
GND       →  GND
SDA       →  D4 (GPIO2)
SCL       →  D3 (GPIO0)
INT       →  D8 (GPIO15) (optional)
AD0       →  GND
```

### Files
- `README.md` - WiFi setup, WebSocket visualization
- `MPU6050_ESP8266_NodeMCU.ino` - Arduino IDE code
- `ESP8266_Visualizer.py` - Python visualization
- `SETUP_GUIDE.md` - NodeMCU specific guide

### Installation
```bash
# 1. Install Arduino IDE
# 2. Add ESP8266 board support (Boards Manager)
# 3. Install libraries: MPU6050, I2Cdev, ArduinoJson
# 4. Upload MPU6050_ESP8266_NodeMCU.ino
# 5. Run: python3 ESP8266_Visualizer.py
```

---

## ESP32

**📝 IMPLEMENTATION STRUCTURE READY**

ESP32 is next-generation successor to ESP8266 with more features.

### Specifications
- Dual-core Xtensa LX6 processor (240 MHz each)
- 520 KB RAM (vs ESP8266's 160 KB)
- 4-16 MB Flash
- Dual I2C: pins 21 (SDA), 22 (SCL)
- 3.3V logic
- Bluetooth 4.2 + WiFi
- I2S audio support

### Implementation Plan
```bash
Microcontrollers/ESP32/
├── README.md                    # Specs and setup
├── MPU6050_ESP32.ino            # Arduino IDE code
└── ESP32_Visualizer.py          # Python visualizer
```

### Key Advantages
- Higher sampling rates (up to 150 Hz)
- Better WiFi performance
- More GPIO pins
- Dual I2C interfaces
- Lower latency

---

## Universal Wiring Notes

### I2C Address Selection
```
AD0 Connected to:  I2C Address
─────────────────────────────────
GND                0x68 (default)
3.3V or 5V         0x69
```

### Pull-up Resistors
- **Arduino**: 20kΩ internal pull-ups (add 4.7kΩ external)
- **ESP8266/ESP32**: No internal pull-ups (must add 4.7kΩ)

### Power Supply
- **Arduino**: 5V from USB or DC jack
- **ESP8266/ESP32**: 3.3V (sensitive to voltage)
- **MPU6050**: 3.3V or 5V tolerant

---

## Python Visualizer Setup (All Boards)

### Requirements
```bash
pip3 install pyserial numpy pygame PyOpenGL matplotlib
```

### Running Visualizer
```bash
# For Arduino boards
python3 Arduino_Uno_Visualizer.py

# For ESP8266 boards  
python3 ESP8266_Visualizer.py

# For ESP32 boards
python3 ESP32_Visualizer.py
```

### Port Selection
- **Windows**: COM3, COM4, etc. (check Device Manager)
- **Linux**: /dev/ttyUSB0, /dev/ttyUSB1
- **macOS**: /dev/tty.usbserial-*

---

## Microcontroller Comparison

| Feature | Arduino Uno | Arduino Nano | ESP8266 | ESP32 |
|---------|------------|--------------|---------|-------|
| **Processor** | ATmega328P | ATmega328P | Xtensa L106 | Xtensa LX6 |
| **Clock** | 16 MHz | 16 MHz | 80 MHz | 240 MHz (x2) |
| **RAM** | 2 KB | 2 KB | 160 KB | 520 KB |
| **Flash** | 32 KB | 30 KB | 4 MB | 4-16 MB |
| **I2C Pins** | A4/A5 | A4/A5 | D4/D3 | 21/22 |
| **Voltage** | 5V | 5V | 3.3V | 3.3V |
| **WiFi** | ❌ | ❌ | ✅ | ✅ |
| **Sample Rate** | 100 Hz | 100 Hz | 50 Hz | 150 Hz |
| **Cost** | $$ | $ | $ | $$ |
| **Complexity** | ⭐ | ⭐ | ⭐⭐ | ⭐⭐⭐ |

---

## Selecting Your Microcontroller

### Choose Arduino Uno if:
- You want simplicity and reliability
- You're a beginner learning microcontrollers
- You need USB power for prototyping
- Budget is not a constraint

### Choose Arduino Nano if:
- You need a compact form factor
- Integrating into small projects
- Same capabilities as Uno, smaller footprint

### Choose ESP8266 if:
- You need WiFi connectivity
- Building IoT projects
- Want to save cost
- Don't need maximum performance

### Choose ESP32 if:
- You need high performance
- Dual-core processing required
- Bluetooth + WiFi needed
- Maximum RAM for buffering

---

## Common Issues & Solutions

### I2C Communication Fails
```bash
# Test with I2C scanner sketch to find device address
# Check pull-up resistors (4.7kΩ)
# Verify power supply voltage and current
```

### Serial Port Won't Open
```bash
# Windows: Update CH340/CH341 drivers
# Linux: sudo usermod -aG dialout $USER
# macOS: Install VCP drivers
```

### Noisy Sensor Data
```bash
# Add 0.1µF capacitor across MPU6050 power pins
# Use shielded I2C cables
# Keep away from power lines
```

---

## Next Steps

1. **Select Your Microcontroller** - Choose based on your needs
2. **Navigate to Folder** - Go to appropriate folder in Microcontrollers/
3. **Read README** - Each folder has detailed setup guide
4. **Install Libraries** - Follow IDE-specific instructions
5. **Upload Firmware** - Use Arduino IDE to compile and upload
6. **Run Visualizer** - Start Python script to see 3D visualization
7. **Experiment** - Modify code for your specific application

---

## File References

- Arduino Uno: `Microcontrollers/Arduino/Uno/README.md`
- Arduino Nano: `Microcontrollers/Arduino/Nano/README.md`
- ESP8266: `Microcontrollers/ESP8266/NodeMCU/README.md`
- ESP32: `Microcontrollers/ESP32/README.md`

---

## Author

**Naman Harshwal**
- GitHub: @namanharshwal
- Repository: mpu6050-imu-3d-visualization
- License: MIT

## Support

Each microcontroller folder contains a dedicated README.md with:
- Hardware specifications
- Detailed wiring diagrams
- Installation instructions
- Troubleshooting guides
- Code examples

Start with the README in your chosen microcontroller's folder!
