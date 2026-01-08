# MPU6050 3D Visualization for Raspberry Pi 4

## Overview

Complete I2C-based MPU6050 IMU implementation for Raspberry Pi 4 (BCM2711) with 64-bit ARM processor. Features real-time complementary filter sensor fusion and OpenGL 3D visualization with enhanced performance.

## Hardware Specifications

**Raspberry Pi 4 Model B**
- CPU: ARMv8 Quad-core 1.5 GHz (Broadcom BCM2711)
- RAM: 1GB / 2GB / 4GB / 8GB variants
- I2C: I2C1 (GPIO2/SDA, GPIO3/SCL) - 400 kHz
- USB: Dual USB 3.0, Dual USB 2.0
- Ethernet: Gigabit (RJ45)

### Wiring Diagram

```
Raspberry Pi 4 GPIO (40-pin Header)

MPU6050 Connection:
- VCC → 5V (Pin 2 or 4)
- GND → GND (Pin 6, 9, 14, 20, 25, 30, 34, 39)
- SDA → GPIO 2 (Pin 3)
- SCL → GPIO 3 (Pin 5)
- INT → GPIO 4 (Pin 7) [Optional]
- AD0 → GND (for I2C address 0x68)
```

## Installation

### 1. Enable I2C

```bash
sudo raspi-config
# Navigate to: Interfacing Options > I2C > Enable
# Reboot
sudo reboot
```

### 2. Install Dependencies

```bash
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y python3-pip i2c-tools python3-smbus
sudo pip3 install numpy scipy pygame PyOpenGL
```

### 3. Verify I2C Connection

```bash
i2cdetect -y 1
# Should show device at 0x68 if connected
```

## Python Implementation

See `mpu6050_rpi4.py` - complete sensor driver with:
- I2C communication via SMBus
- Complementary filter sensor fusion
- Real-time JSON data output
- 100 Hz sampling rate

```bash
python3 mpu6050_rpi4.py
```

## C++ Implementation

High-performance implementation using:
- libi2c-dev for direct I2C access
- Eigen3 for sensor fusion calculations
- OpenGL 3.0+ for visualization

```bash
g++ -o mpu6050_viz mpu6050_rpi4.cpp -std=c++17 -li2c -lm -lGL -lGLU -lglfw3 -lX11
./mpu6050_viz
```

## Sensor Fusion Algorithm

**Complementary Filter** (95% gyro, 5% accelerometer)
- Roll = 0.95 * (Roll + Gyro_X * dt) + 0.05 * Atan2(Accel_Y, Accel_Z)
- Pitch = 0.95 * (Pitch + Gyro_Y * dt) + 0.05 * Atan2(-Accel_X, sqrt(Accel_Y² + Accel_Z²))
- Yaw += Gyro_Z * dt

## Performance

| Metric | Value |
|--------|-------|
| Sample Rate | 100 Hz |
| Computation Time | ~1.5 ms/sample |
| Visualization FPS | 60 FPS |
| End-to-End Latency | ~30 ms |
| Memory Usage | ~45 MB |

## Calibration

The implementation auto-calibrates on startup:

```bash
# During startup, keep device still and level
# Calibration takes ~1 second
```

## Applications

- Robot odometry and SLAM
- Gesture recognition
- Motion tracking systems
- Sensor fusion research
- Educational demonstrations

## Author

**Naman Harshwal**
- GitHub: @namanharshwal
- Repository: mpu6050-imu-3d-visualization
- License: MIT

## License

MIT License - See LICENSE file for details
