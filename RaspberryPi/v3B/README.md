# MPU6050 3D Visualization for Raspberry Pi 3B

**Version**: 1.0.0  
**Author**: Naman Harshwal  
**License**: MIT  
**Platform**: Raspberry Pi 3B (ARMv7)  
**Date**: January 2026

## Overview

Comprehensive 3D visualization system for MPU6050 IMU sensor integrated with Raspberry Pi 3B, providing real-time orientation tracking and 3D model rendering using OpenGL.

### Key Features

- **Real-time I2C Communication**: Direct GPIO-based I2C interface
- **10Hz Sampling Rate**: Synchronized sensor data streaming
- **Dual Implementation**: Both Python and C++ versions
- **3D OpenGL Visualization**: Hardware-accelerated rendering
- **Complementary Filter Fusion**: Gyroscope + Accelerometer fusion
- **SLAM Integration**: Compatible with ROS2 ecosystems

## Hardware Setup

### Components Required

- Raspberry Pi 3B (1GB RAM, ARMv7)
- MPU6050 GY-521 breakout board
- Micro USB power adapter (2.5A+)
- 4.7kΩ pull-up resistors (x2)
- Breadboard and jumper wires

### Wiring Diagram

```
Raspberry Pi 3B  │  MPU6050
─────────────────┼──────────────
Pin 1 (3.3V)     → VCC
Pin 6 (GND)      → GND
Pin 3 (GPIO2)    → SDA (I2C Data)
Pin 5 (GPIO3)    → SCL (I2C Clock)
Pin 7 (GPIO4)    → INT (Optional)
```

### GPIO Pin Reference

```
Raspberry Pi 3B GPIO Pinout:
- GPIO2 (Pin 3): I2C SDA - 1.8kΩ pull-up to 3.3V
- GPIO3 (Pin 5): I2C SCL - 1.8kΩ pull-up to 3.3V
- GPIO4 (Pin 7): Interrupt pin
- Ground: Pins 6, 9, 14, 20, 25, 30, 34, 39
- 3.3V: Pins 1, 17
```

## Software Setup

### Step 1: Enable I2C Interface

```bash
# Enable I2C via raspi-config
sudo raspi-config
# Navigate to: Interfacing Options → I2C → Enable
# Reboot
sudo reboot
```

### Step 2: Install Dependencies

```bash
# Update system packages
sudo apt update
sudo apt upgrade -y

# Python dependencies
sudo apt install -y python3-pip python3-dev
pip3 install smbus-cffi numpy pygame PyOpenGL pyserial

# C++ dependencies
sudo apt install -y gcc g++ cmake libi2c-dev i2c-tools

# Optional: ROS2 Humble for Ubuntu 22.04 or compatibility layer
curl https://repo.ros2.org/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'
```

### Step 3: Verify I2C Detection

```bash
# Install i2c-tools
sudo apt install i2c-tools

# Detect MPU6050 on I2C bus (should show 0x68)
i2cdetect -y 1
```

## Python Implementation

### mpu6050_rpi3b.py

See `mpu6050_rpi3b.py` for complete Python implementation with:
- Complementary filter IMU fusion
- I2C direct register access
- JSON serial output
- OpenGL 3D visualization

### Running Python Version

```bash
# Make executable
chmod +x mpu6050_rpi3b.py

# Run with visualization
python3 mpu6050_rpi3b.py

# Output appears at /dev/ttyAMA0 or USB serial device
```

## C++ Implementation

### mpu6050_rpi3b.cpp

See `mpu6050_rpi3b.cpp` for optimized C++ implementation featuring:
- Native I2C via libi2c-dev
- Faster execution (ARM NEON SIMD support)
- Standard input/output for piping
- ROS2 node compatibility

### Compiling C++ Version

```bash
# Compile
g++ -O3 -march=armv7-a -mfpu=neon mpu6050_rpi3b.cpp -o mpu6050_rpi3b -li2c -lm

# Run
./mpu6050_rpi3b
```

## Visualization Script

### visualizer_rpi3b.py

Real-time 3D visualization using OpenGL:

```bash
# Update serial port (default: /dev/ttyAMA0)
sed -i 's|/dev/ttyAMA0|/your/serial/port|' visualizer_rpi3b.py

# Run visualization
python3 visualizer_rpi3b.py
```

## Performance Specifications

| Parameter | Value |
|-----------|-------|
| **CPU Usage** | 15-25% (single core) |
| **RAM Usage** | ~40MB (Python), ~15MB (C++) |
| **I2C Clock** | 400 kHz |
| **Sampling Rate** | 10 Hz (100ms intervals) |
| **Visualization FPS** | 30-60 FPS |
| **Orientation Accuracy** | ±2-3 degrees |

## Troubleshooting

### I2C Connection Issues

```bash
# Check I2C device
lsmod | grep i2c

# Check device detection
i2cdetect -y 1

# Check pull-up resistors (should see "68" or "UU" at address 0x68)
```

### Permission Errors

```bash
# Add user to i2c group
sudo usermod -aG i2c $USER

# Logout and login for changes to take effect
logout
```

### Python Module Not Found

```bash
pip3 install --upgrade numpy pygame PyOpenGL
```

## Applications

- **Autonomous Robot Orientation Tracking**: Real-time 6DOF pose estimation
- **SLAM System Debugging**: IMU validation during mapping
- **Robotic Arm Control**: End-effector orientation monitoring
- **Gesture Recognition**: Motion-based input systems
- **Drone Flight Testing**: Attitude stabilization verification

## References

- [MPU6050 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [Raspberry Pi GPIO Guide](https://www.raspberrypi.org/documentation/usage/gpio/)
- [I2Cdev Library](https://github.com/jrowberg/i2cdevlib)
- [PyOpenGL Documentation](https://pyopengl.sourceforge.net/)

## Version History

- **v1.0.0** (Jan 2026): Initial release with Python + C++ support

## Support

For issues, refer to:
1. Main [README.md](../../README.md) for general guidance
2. [ESP8266_NODEMCU_SETUP_GUIDE.md](../../ESP8266_NODEMCU_SETUP_GUIDE.md) for reference

## License

MIT License - See [LICENSE](../../LICENSE) file
