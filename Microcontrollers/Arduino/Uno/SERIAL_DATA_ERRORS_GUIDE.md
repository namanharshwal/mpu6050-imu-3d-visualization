# Serial Data UTF-8 Errors Troubleshooting Guide

## Problem Description

When running the Arduino Uno MPU6050 3D visualizer, you see error messages like:

```
Data error: 'utf-8' codec can't decode byte 0x80 in position 2: invalid start byte
Data error: 'utf-8' codec can't decode byte 0x83 in position 2: invalid start byte
Data error: 'utf-8' codec can't decode byte 0xf0 in position 0: invalid continuation byte
```

While the visualizer still works (showing FPS and rotation data), these errors indicate corrupted or invalid serial data being received from the Arduino.

## Root Causes

### 1. **Baud Rate Mismatch**
- Arduino firmware sends data at one baud rate
- Python visualizer expects different baud rate
- Results in misaligned bytes and invalid UTF-8 sequences

**Check:** Ensure baud rate is **38400** on both Arduino and Python

```c
// Arduino side
Serial.begin(38400);
```

```python
# Python side (line 9)
ser = serial.Serial('COM3', 38400, timeout=1)
```

### 2. **Corrupt Serial Data**
- Loose USB connection
- Noisy I2C lines
- Arduino buffer overflow
- Invalid JSON format from Arduino

### 3. **Missing or Incomplete Data Packets**
- Serial line buffer contains partial data
- Multiple messages concatenated without proper framing
- Serial timeout too short

### 4. **Defective USB-to-Serial Converter**
- Cheap USB cables or adapters
- Old/damaged Arduino board
- Driver issues on PC

---

## Solutions

### Solution 1: Fix Baud Rate (Quick Check)

**Step 1:** Verify Arduino code baud rate
- Open `MPU6050_Arduino_Uno.ino`
- Search for `Serial.begin()`
- Confirm it says `Serial.begin(38400)`

**Step 2:** Verify Python code baud rate
- Open `Arduino_Uno_Simulator.py`
- Line 9 should be: `ser = serial.Serial('COM3', 38400, timeout=1)`
- If different, update to match Arduino

**Step 3:** Upload fresh firmware
- If uncertain, re-upload the Arduino code
- This resets serial communication

### Solution 2: Improve Serial Error Handling (Recommended)

The Python code can be improved to silently ignore invalid UTF-8 bytes instead of printing errors:

**Current Code (Line 113-120):**
```python
if ser.in_waiting > 0:
    line = ser.readline().decode('utf-8').strip()
    if line.startswith("{") and line.endswith("}"):
        imu_data = json.loads(line)
        ax = imu_data.get("roll", 0.0)
        ay = imu_data.get("pitch", 0.0)
        az = imu_data.get("yaw", 0.0)
except (json.JSONDecodeError, UnicodeDecodeError) as e:
    print(f"Data error: {e}")
```

**Improved Code (More Robust):**
```python
if ser.in_waiting > 0:
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith("{") and line.endswith("}"):
            imu_data = json.loads(line)
            ax = imu_data.get("roll", 0.0)
            ay = imu_data.get("pitch", 0.0)
            az = imu_data.get("yaw", 0.0)
    except json.JSONDecodeError:
        pass  # Silently skip malformed JSON
```

**Key Change:** `decode('utf-8', errors='ignore')`
- `errors='ignore'` = Skip invalid UTF-8 bytes
- `errors='replace'` = Replace with '?' character
- Eliminates error messages while maintaining functionality

### Solution 3: Hardware Checks

**Check USB Connection:**
1. Disconnect and reconnect USB cable
2. Try different USB port (not USB 3.0 hub)
3. Try different USB cable (recommended: shielded)

**Check Serial Port:**
```bash
# Windows: Open Device Manager
# Look for COM port in "Ports (COM & LPT)"
# Device should be "Arduino Uno" (not Unknown Device)

# Linux/Mac: Check serial port
ls /dev/ttyUSB*  # or
ls /dev/ttyACM*
```

**Check Arduino Driver:**
- Windows: Download CH340G driver if using cheap clones
- Some Arduino clones use CH340/CH341 USB chips
- Original Arduino Uno uses FTDI chip

### Solution 4: Improve Serial Communication

**Add serial buffer clearing:**
```python
def read_data():
    global ax, ay, az
    ax = ay = az = 0.0
    try:
        # Clear input buffer of any garbage data
        ser.reset_input_buffer()
        
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # Validate complete JSON
            if line.startswith("{") and line.endswith("}") and len(line) > 5:
                imu_data = json.loads(line)
                ax = float(imu_data.get("roll", 0.0))
                ay = float(imu_data.get("pitch", 0.0))
                az = float(imu_data.get("yaw", 0.0))
    except (json.JSONDecodeError, ValueError):
        pass  # Skip corrupt data
```

**Better Timeout Handling:**
```python
# Increase timeout for more stable reading
ser = serial.Serial('COM3', 38400, timeout=0.5)  # Was: timeout=1
```

### Solution 5: Test Arduino Serial Output

**Use Arduino Serial Monitor to verify data:**

1. Upload `MPU6050_Arduino_Uno.ino` to Arduino
2. Open Arduino IDE → Tools → Serial Monitor
3. Set baud rate to **38400** (bottom right)
4. You should see JSON output like:
   ```
   {"roll":0.25,"pitch":-1.50,"yaw":45.20}
   {"roll":0.30,"pitch":-1.48,"yaw":45.25}
   {"roll":0.28,"pitch":-1.52,"yaw":45.23}
   ```

**If you see garbage characters:**
- Data is corrupted
- Likely baud rate mismatch
- Check USB connection quality

**If no output appears:**
- Arduino may be stuck
- Check MPU6050 connection (I2C pins A4/A5)
- Check INT pin if used
- See main README for I2C debugging

---

## Quick Diagnostics Checklist

If you see UTF-8 errors, check in this order:

- [ ] USB cable firmly connected to Arduino
- [ ] Correct COM port selected (check Device Manager)
- [ ] Arduino firmware shows JSON data in Serial Monitor
- [ ] Baud rate is 38400 in both Arduino and Python code
- [ ] Python timeout value reasonable (0.5-1.0 seconds)
- [ ] No other terminal/IDE has Serial Monitor open for this port
- [ ] Windows: Latest CH340 drivers installed (if clone Arduino)
- [ ] I2C pull-up resistors properly installed
- [ ] MPU6050 connected correctly (see WIRING_GUIDE.md)

---

## Permanent Fix Implementation

**For production use**, implement robust serial reading:

```python
def read_data_robust():
    global ax, ay, az
    ax = ay = az = 0.0
    
    try:
        # Read all available data
        if ser.in_waiting >= 20:  # JSON is ~30 chars minimum
            # Read only complete lines
            data = ser.read_until(b'}')  # Read until }
            
            # Decode with error handling
            line = data.decode('utf-8', errors='ignore').strip()
            
            # Validate JSON
            if line.startswith("{") and line.endswith("}"):
                imu_data = json.loads(line)
                ax = imu_data.get("roll", ax)  # Keep previous value if missing
                ay = imu_data.get("pitch", ay)
                az = imu_data.get("yaw", az)
                
    except (json.JSONDecodeError, ValueError, UnicodeDecodeError):
        pass  # Silently skip corrupt packet
    except Exception as e:
        print(f"[WARNING] Serial read issue: {type(e).__name__}")
```

---

## FAQ

**Q: Why do I still see errors even though it works?**
A: Some corrupted packets get sent from Arduino (due to I2C noise, buffer overflow, etc). The visualizer skips these automatically and continues with valid data.

**Q: Can I completely eliminate these errors?**
A: Yes. Use `errors='ignore'` in the decode() function to suppress error messages. See Solution 2.

**Q: My serial monitor shows correct data, but Python shows errors.**
A: Serial Monitor has different error handling. Check for baud rate matching and USB port issues.

**Q: Will these errors affect the 3D visualization?**
A: No. Invalid packets are automatically skipped. You'll see smooth rotation even with occasional errors. They're just annoying messages.

**Q: Is it safe to ignore these errors?**
A: Yes. The visualizer validates JSON before using it. Only valid sensor data updates the orientation.

---

## Contact & Support

If issues persist:
1. Check the main README.md for I2C setup
2. Review WIRING_GUIDE.md for hardware connections
3. See all files in this directory for complete documentation
4. Verify Arduino firmware uploaded successfully

**Last Updated**: January 2026
**Author**: Naman Harshwal
