# MPU6050 IMU 3D Visualization System
![output](https://github.com/user-attachments/assets/c5ec19b3-cab9-47bc-9bb3-207997053c4a)

> A comprehensive Python-based real-time 3D visualization system for STL models synchronized with MPU6050 inertial measurement unit (IMU) sensor data. Designed for robotics engineers working with autonomous systems, SLAM, and hardware debugging.

## 📋 Overview

This project provides an integrated solution for visualizing 3D models with live motion sensor feedback. It combines Arduino-based IMU data acquisition with Python-based OpenGL rendering to create an intuitive interface for understanding and debugging robot orientation, SLAM visualization, and motion tracking systems.

**Ideal for:**
- Autonomous mobile robot orientation debugging
- SLAM system visualization and verification
- IMU sensor calibration and validation
- Robot arm and manipulator orientation tracking
- Real-time motion capture visualization
- Hardware integration testing

## ✨ Key Features

### Real-Time Sensor Integration
- **Live MPU6050 Data Streaming**: Continuous serial communication with Arduino-based MPU6050 at 38400 baud
- **High-Frequency Updates**: 10 Hz sampling rate for smooth real-time visualization
- **Accurate Orientation Tracking**: DMP (Digital Motion Processor) enables precise attitude estimation with quaternion-based calculations

### Advanced 3D Rendering
- **OpenGL-Based Visualization**: Hardware-accelerated 3D graphics using PyOpenGL
- **STL Model Support**: Load and visualize any STL file from your robotics projects
- **Dynamic Rotation**: Real-time model orientation updates based on roll, pitch, and yaw angles
- **Spatial Lighting**: Professional lighting setup with diffuse and specular components

### Flexible Control Interface
- **Yaw Mode Toggle**: Press 'Z' to enable/disable yaw rotation for debugging specific axes
- **Responsive Graphics**: Pygame-based window management with automatic viewport scaling
- **Performance Monitoring**: Real-time FPS display for performance analysis

## 🛠️ Hardware Requirements

### Microcontroller Setup
- **Arduino or Compatible Board** (Arduino Uno, Mega, Nano, etc.)
- **MPU6050 IMU Sensor Module** (GY-521 breakout board recommended)
- **I2C Connection**: SCL to Arduino pin A5, SDA to Arduino pin A4
- **Interrupt Pin**: Connect INT to Arduino Digital Pin 2
- **USB Connection**: For serial communication to PC

### PC Requirements
- **Python 3.8+** (3.9 or 3.10 recommended for better compatibility)
- **GPU with OpenGL support** (Intel, NVIDIA, or AMD)
- **Minimum 4GB RAM** for smooth rendering
- **Windows, Linux, or macOS** compatible

## 📦 Dependencies

```bash
pip install numpy pyopengl pygame numpy-stl pyserial
```

### Library Details
- **NumPy**: Fast numerical computations
- **PyOpenGL**: 3D graphics rendering
- **Pygame**: Window management and event handling
- **numpy-stl**: STL file parsing and mesh handling
- **PySerial**: Arduino serial communication

## 🚀 Quick Start

### Step 1: Arduino Setup

1. **Install Required Libraries**:
   - Open Arduino IDE
   - Go to Sketch → Include Library → Manage Libraries
   - Search and install:
     - "I2Cdev" by Jeff Rowberg
     - "MPU6050" by Electronic Cats (or Jeff Rowberg's complete library)

2. **Upload Firmware**:
   ```cpp
   // See MPU6050.ino for complete code
   // Key features:
   // - DMP initialization for accurate orientation
   // - Yaw/Pitch/Roll calculation from quaternions
   // - JSON-formatted serial output
   // - 10 Hz sampling frequency
   ```

3. **Find Serial Port**:
   - Windows: Check Device Manager (COM3, COM4, etc.)
   - Linux: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
   - macOS: `ls /dev/tty.usbserial*`

### Step 2: Python Configuration

1. **Update Serial Port** in `mpu6050_3d_visualizer.py`:
   ```python
   # Line 11: Update your serial port
   ser = serial.Serial('COM3', 38400, timeout=1)  # Windows
   ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)  # Linux
   ser = serial.Serial('/dev/tty.usbserial-0001', 38400, timeout=1)  # macOS
   ```

2. **Set STL File Path** (Line 17):
   ```python
   stl_path = "path/to/your/model.STL"
   ```
   - Use absolute paths for reliability
   - Supports any ASCII or binary STL format
   - Recommended: Keep STL files in the same directory as the Python script

### Step 3: Run the Visualization

```bash
# Navigate to project directory
cd /path/to/mpu6050-imu-3d-visualization

# Run the visualizer
python mpu6050_3d_visualizer.py
```

**Expected Output**:
- OpenGL window with 3D model
- Real-time rotation synchronized with IMU
- Console output showing FPS (typically 30-60 FPS)

## 📝 File Structure

```
mpu6050-imu-3d-visualization/
├── README.md                          # This file
├── MPU6050.ino                        # Arduino firmware for IMU data
├── mpu6050_3d_visualizer.py          # Main Python visualization script
└── your_model.STL                     # Example 3D model file
```

## 🎮 Controls & Interaction

| Key | Function |
|-----|----------|
| **Z** | Toggle Yaw rotation (useful for debugging specific axes) |
| **ESC** | Exit the application |
| **Mouse Scroll** | Zoom (if implemented) |
| **Window Close** | Graceful shutdown |

## 🔧 Configuration Guide

### Adjusting Rotation Sensitivity

Edit the rotation multipliers in `mpu6050_3d_visualizer.py`:
```python
# Line ~68: Modify these for different sensitivity
glRotatef(ay, 1.0, 0.0, 0.0)    # Pitch sensitivity
glRotatef(-1 * ax, 0.0, 0.0, 1.0)  # Roll sensitivity
glRotatef(az, 0.0, 1.0, 0.0)    # Yaw sensitivity
```

### Scaling the 3D Model

```python
# Line ~66: Adjust the scaling factor
scaling_factor = 0.4  # Increase for larger models
glScalef(scaling_factor, scaling_factor, scaling_factor)
```

### Changing Camera Distance

```python
# Line ~65: Adjust Z position for camera depth
glTranslatef(0, 0.0, -50.0)  # Negative values move camera away
```

### Modifying Colors

```python
# Line ~75: Change model color (RGB values 0.0-1.0)
glColor3f(1.0, 0.0, 0.0)  # Red color
# Example: glColor3f(0.0, 1.0, 0.0)  # Green
#         glColor3f(0.0, 0.0, 1.0)  # Blue
```

## 📊 Understanding IMU Data

The system receives data in JSON format from Arduino:
```json
{"yaw": -45.32, "pitch": 12.15, "roll": -2.87}
```

### Euler Angles Explained
- **Roll (Φ)**: Rotation around X-axis (side-to-side tilt)
- **Pitch (Θ)**: Rotation around Y-axis (forward-backward tilt)
- **Yaw (Ψ)**: Rotation around Z-axis (left-right spin)

### Calibration Notes
- Initial setup performs 6-second automatic calibration
- Keep MPU6050 stationary during Arduino startup
- Consider environmental magnetic interference if yaw drifts
- Use compass calibration for long-duration experiments

## 🐛 Troubleshooting

### Issue: "Serial port COM3 not found"
**Solution**:
- Check Device Manager (Windows) for connected devices
- Verify USB cable connection
- Install CH340 drivers if using GY-521 with cheap USB adapter
- Update the serial port in Python code

### Issue: Model doesn't rotate smoothly
**Solution**:
- Reduce FPS cap in code: `pygame.time.Clock().tick(30)`
- Check if JSON parsing is failing (check console output)
- Verify Arduino is sending data: Use Arduino Serial Monitor at 38400 baud

### Issue: "No module named 'OpenGL'"
**Solution**:
```bash
pip install --upgrade pyopengl
```

### Issue: Model appears distorted or upside-down
**Solution**:
- Adjust rotation multipliers (change signs in glRotatef)
- Modify scaling factor
- Check STL file orientation in external viewer first

### Issue: High CPU usage or low FPS
**Solution**:
- Simplify STL model (reduce polygon count)
- Lower refresh rate
- Close other applications
- Update graphics drivers

## 🚁 Real-World Applications

### 1. Autonomous Robot Debugging
Monitor your robot's actual orientation during navigation:
```
→ Verify roll/pitch during turning
→ Detect IMU drift over time
→ Validate sensor fusion algorithms
```

### 2. SLAM System Visualization
Visualize robot pose during mapping:
```
→ Real-time 6-DOF pose estimation
→ Validate visual odometry against IMU
→ Detect loop closure artifacts
```

### 3. Robotic Arm Control
Monitor end-effector orientation:
```
→ Verify MoveIt2 trajectory execution
→ Detect servo lag or jitter
→ Validate gripper orientation
```

### 4. Drone/Quadcopter Testing
Test flight controller response:
```
→ Monitor attitude stabilization
→ Verify PID tuning
→ Detect oscillations or vibrations
```

## 📚 Advanced Usage

### Loading Different STL Models
```python
# Change at runtime (modify the code)
stl_path = "C:/Models/manipulator_arm.STL"
stl_path = "/home/user/drone_frame.STL"  # Linux
```

### Data Logging
Add to record orientation data:
```python
import csv
from datetime import datetime

# Log IMU data for analysis
with open('imu_log.csv', 'a') as f:
    writer = csv.writer(f)
    writer.writerow([datetime.now(), ax, ay, az])
```

### Multi-Model Comparison
Load multiple STL files sequentially for comparison

## 🔗 References & Attribution

This project is inspired by and built upon the excellent work of [Omar-Dahy/MPU6050-3D-STL-Simulator](https://github.com/Omar-Dahy/MPU6050-3D-STL-Simulator), which provides the foundational concept and Arduino firmware for MPU6050 integration.

**Original Reference**:
- Repository: https://github.com/Omar-Dahy/MPU6050-3D-STL-Simulator
- Technology: Python 3.8+, Arduino, OpenGL, pygame
- License: Check original repository for details

## 📖 Learning Resources

- **MPU6050 Documentation**: https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/
- **I2Cdev Library**: https://github.com/jrowberg/i2cdev
- **PyOpenGL Guide**: https://pyopengl.sourceforge.net/
- **ROS Integration**: Connect with ROS2 topics for SLAM systems

## 💡 Tips for Best Results

1. **Mount MPU6050 Properly**: Ensure IMU is rigidly mounted to avoid vibration noise
2. **Cable Shielding**: Use shielded USB cables to reduce interference
3. **Thermal Management**: Avoid temperature extremes (0-40°C optimal)
4. **Model Optimization**: Pre-process STL files to reduce polygon count
5. **Distance Measurement**: Keep MPU6050 and viewing setup consistent

## 🤝 Contributing

Contributions are welcome! Areas for improvement:
- Multi-model support
- Recording video output
- ROS2 integration
- Real-time mesh deformation
- Extended Kalman Filter for sensor fusion

## 📄 License

This project is provided as-is for educational and research purposes.

## ✉️ Support & Questions

For issues, questions, or suggestions:
1. Check the Troubleshooting section above
2. Review the original reference repository
3. Test with the Arduino Serial Monitor first
4. Verify all dependencies are installed correctly

---

**Last Updated**: January 2026
**Version**: 1.0.0
**Status**: Production Ready for Development & Testing
