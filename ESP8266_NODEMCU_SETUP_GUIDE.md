# NodeMCU v3 (ESP8266MOD) - MPU6050 3D Visualization Guide

## 🚀 Complete Setup & Installation Guide

This guide provides everything needed to use the MPU6050 IMU with NodeMCU v3 (ESP8266) and visualize the orientation on your desktop.

---

## 📋 Table of Contents
1. [Hardware Setup](#hardware-setup)
2. [Arduino IDE Configuration](#arduino-ide-configuration)
3. [Library Installation](#library-installation)
4. [Code Upload](#code-upload)
5. [Python Visualization](#python-visualization)
6. [Troubleshooting](#troubleshooting)

---

## 🔌 Hardware Setup

### Components Required:
- **NodeMCU v3 (ESP8266MOD)** board
- **MPU6050 GY-521** breakout board
- **Micro USB cable** for programming
- **4.7kΩ pull-up resistors** (x2) - optional but recommended
- **Breadboard and jumper wires**

### Wiring Diagram (NodeMCU v3 → MPU6050):
```
NodeMCU Pin  │  MPU6050 Pin
─────────────┼──────────────
3.3V         →  VCC
GND          →  GND
D4 (GPIO2)   →  SDA (Serial Data)
D3 (GPIO4)   →  SCL (Serial Clock)
D8 (GPIO15)  →  INT (Interrupt) [Optional]
```

### Pin Map Reference:
```
NodeMCU v3 Pin Mapping:
D0 → GPIO16  |  D4 → GPIO2   |  D8 → GPIO15
D1 → GPIO5   |  D5 → GPIO14  |  RSV → GPIO17
D2 → GPIO4   |  D6 → GPIO12  |  A0 → ADC
D3 → GPIO0   |  D7 → GPIO13  |  GND → GND
              |  D10 → GPIO1  |  3.3V → 3.3V
```

### Connection Steps:
1. Place NodeMCU v3 on breadboard
2. Place MPU6050 on breadboard (separate area)
3. Connect power (3.3V to VCC, GND to GND)
4. Connect I2C lines:
   - SDA: D4 (GPIO2) to MPU6050 SDA
   - SCL: D3 (GPIO4) to MPU6050 SCL
5. Optional: Add 4.7kΩ pull-up resistors on SDA and SCL lines

**Note:** Do NOT use 5V! ESP8266 is 3.3V only and will be damaged by 5V.

---

## 🖥️ Arduino IDE Configuration

### Step 1: Install ESP8266 Board Support

1. Open **Arduino IDE**
2. Go to **File → Preferences**
3. Find "Additional Boards Manager URLs" field
4. Paste this URL:
   ```
   http://arduino.esp8266.com/stable/package_esp8266com_index.json
   ```
5. Click **OK**
6. Go to **Tools → Board → Boards Manager**
7. Search for "esp8266"
8. Click on "esp8266 by ESP8266 Community"
9. Click **Install** (version 3.0.0 or later recommended)
10. Wait for installation (~3-5 minutes)

### Step 2: Select Board Settings

1. Go to **Tools → Board**
2. Select: **NodeMCU 1.0 (ESP-12E Module)**
3. Set these additional options:
   - **Tools → Flash Size**: 4MB (FS: 2MB OTA: ~1019KB)
   - **Tools → Upload Speed**: 115200
   - **Tools → CPU Frequency**: 80 MHz
   - **Tools → Erase Flash**: Only Sketch

### Step 3: Select Serial Port

1. **Tools → Port**
2. Select your COM port (e.g., COM3, COM4)
   - **Windows**: Look for "Silicon Labs CP210x" or "CH340"
   - **Linux**: /dev/ttyUSB0 or /dev/ttyACM0
   - **macOS**: /dev/tty.usbserial-*

---

## 📚 Library Installation

### Method 1: Library Manager (Recommended)

1. Open **Sketch → Include Library → Manage Libraries**
2. Search for and install these libraries:
   - **I2Cdev** by Jeff Rowberg (v3.20190703.0 or later)
   - **MPU6050** by Electronic Cats (or Jeff Rowberg's version)

3. Installation steps for each:
   - Type library name in search box
   - Select the latest version
   - Click **Install**
   - Wait for installation to complete

### Method 2: Manual Installation (If Method 1 fails)

1. Download libraries from GitHub:
   - I2Cdev: https://github.com/jrowberg/i2cdevlib
   - MPU6050: https://github.com/Electronic-Cats/mpu6050

2. Extract to Arduino libraries folder:
   - **Windows**: Documents\Arduino\libraries\
   - **Linux**: ~/Arduino/libraries/
   - **macOS**: ~/Documents/Arduino/libraries/

3. Restart Arduino IDE

### Verify Installation:

1. **Sketch → Include Library**
2. You should see "I2Cdev" and "MPU6050" in the list

---

## 💾 Code Upload

### Step 1: Upload NodeMCU Firmware

1. Copy the code from `ESP8266_MPU6050_Firmware.ino` (provided in this repo)
2. Open Arduino IDE and create new sketch
3. Paste the entire code
4. **Verify** (Sketch → Verify/Compile) to check for errors
5. Connect NodeMCU via USB to your computer
6. Click **Upload** (Sketch → Upload)
7. Wait for upload to complete (should say "Hard resetting...")
8. Open **Tools → Serial Monitor**
9. Set baud rate to **115200**
10. Reset NodeMCU (press Reset button)
11. You should see initialization messages and JSON data output

### Step 2: Verify Data Output

You should see output like:
```
ESP8266 MPU6050 Initialization
Initializing I2C devices...
Testing MPU6050 connection...
MPU6050 connection successful
Initializing DMP...
MPU6050 connection failed

OR (simplified mode without DMP):
{"roll": 2.45, "pitch": -1.23}
{"roll": 2.43, "pitch": -1.21}
```

---

## 🐍 Python Visualization

### Step 1: Install Python Requirements

1. Ensure Python 3.8+ is installed
2. Open Command Prompt/Terminal
3. Run:
   ```bash
   pip install numpy pygame PyOpenGL pyserial
   ```

### Step 2: Prepare Visualization Script

1. Create a file named `esp8266_visualizer.py`
2. Copy code from `ESP8266_Visualizer.py` in this repo
3. Update the serial port and STL file path

### Step 3: Run Visualization

1. Find NodeMCU serial port:
   - **Windows**: Device Manager (COM3, COM4, etc.)
   - **Linux**: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
   - **macOS**: `ls /dev/tty.usbserial*`

2. Update port in `esp8266_visualizer.py`:
   ```python
   ser = serial.Serial('COM3', 115200, timeout=1)  # Windows
   ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Linux
   ```

3. Run the script:
   ```bash
   python esp8266_visualizer.py
   ```

4. You should see:
   - OpenGL window with 3D model
   - Real-time rotation based on MPU6050 data
   - Console showing FPS and incoming data

---

## 🔧 Troubleshooting

### Issue 1: "ESP8266 board not found in Boards Manager"
**Solution:**
- Ensure you added the correct URL in Preferences
- Try re-adding the URL
- Restart Arduino IDE
- Check internet connection

### Issue 2: "Board selection shows unknown board"
**Solution:**
- Go to **Tools → Board Manager**
- Search "esp8266"
- Install "esp8266 by ESP8266 Community"
- Restart IDE

### Issue 3: "Upload fails / Cannot find serial port"
**Solution:**
- Install CH340 driver (for NodeMCU clones):
  - Windows: https://sparks.gogo.co.nz/ch340.html
  - macOS: https://github.com/johnny-five-io/johnny-five/wiki/Getting-started-with-Arduino-using-duino
  - Linux: Usually built-in, try `sudo apt install ch340-dkms`
- Try different USB cable (some are power-only)
- Reset NodeMCU before upload

### Issue 4: "MPU6050 connection failed" in Serial Monitor
**Solution:**
- Verify wiring (especially SDA/SCL)
- Check voltage (should be 3.3V)
- Ensure pull-up resistors (if not built-in)
- Try different I2C address (default 0x68, alt 0x69)
- Check I2C Scanner code to verify connection

### Issue 5: "Data shows but doesn't rotate smoothly"
**Solution:**
- Reduce update rate (change 100ms to 200ms delay)
- Ensure steady power supply
- Check serial connection stability
- Verify Python script serial port matches

### Issue 6: "No data received in Python script"
**Solution:**
- Verify serial port in script matches Arduino IDE
- Check baud rate is 115200
- Ensure NodeMCU is sending data (check Serial Monitor first)
- Close Serial Monitor before running Python script

### Issue 7: "ImportError: No module named 'pygame'"
**Solution:**
```bash
pip install --upgrade pygame
# or
pip3 install pygame
```

### Issue 8: "I2Cdev or MPU6050 library not found"
**Solution:**
- Go to **Sketch → Include Library → Manage Libraries**
- Search "I2Cdev" and "MPU6050"
- Install latest versions
- Restart Arduino IDE
- Recompile sketch

---

## 📊 Comparison: DMP vs Simplified Mode

### DMP Mode (Full Accuracy):
✅ Pros:
- High accuracy
- Quaternion-based calculations
- Gravity compensation
- Factory calibrated

❌ Cons:
- Memory intensive (may not work on ESP8266)
- Requires DMP initialization
- Longer startup time

### Simplified Mode (Accelerometer-based):
✅ Pros:
- Low memory usage
- Works reliably on ESP8266
- Instant data
- Simpler code

❌ Cons:
- Less accurate (±3-5 degrees)
- No gyro integration
- Only pitch/roll (no yaw)
- Drift over time

This guide uses **Simplified Mode** for ESP8266 stability.

---

## 📞 Support & Additional Resources

### Documentation Links:
- ESP8266 Arduino Core: https://github.com/esp8266/Arduino
- I2Cdev Library: https://github.com/jrowberg/i2cdevlib
- MPU6050 Datasheet: https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/

### Useful Commands:

**Check Serial Port (Linux/macOS):**
```bash
ls /dev/tty*
```

**Check Serial Port (Windows PowerShell):**
```powershell
Get-WmiObject Win32_SerialPort | Select-Object Name, DeviceID
```

**Test I2C Connection:**
Use I2C Scanner sketch to verify MPU6050 is at address 0x68

---

## ✅ Quick Checklist

- [ ] NodeMCU v3 physically connected to MPU6050
- [ ] USB cable connected and recognized by OS
- [ ] ESP8266 board installed in Arduino IDE
- [ ] Board set to "NodeMCU 1.0 (ESP-12E Module)"
- [ ] I2Cdev and MPU6050 libraries installed
- [ ] Serial Monitor shows sensor data
- [ ] Python packages installed (numpy, pygame, PyOpenGL, pyserial)
- [ ] Serial port updated in Python script
- [ ] STL file path set correctly
- [ ] 3D visualization working

---

**Last Updated**: January 2026
**Version**: 1.0.0
**Status**: Tested & Working
