#!/bin/bash

# MPU6050 IMU 3D Visualization Setup Script for Raspberry Pi 5
# This script automates the complete setup process with latest performance features
# Author: Naman Harshwal
# License: MIT
# Note: Raspberry Pi 5 requires Raspberry Pi OS (Bookworm) 64-bit

set -e

echo "================================================"
echo "MPU6050 IMU 3D Visualization Setup - Raspberry Pi 5"
echo "================================================"
echo ""

# Check if running on Raspberry Pi 5
if ! grep -q "Raspberry Pi 5" /proc/device-tree/model 2>/dev/null; then
    echo "[WARNING] This script is optimized for Raspberry Pi 5"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "[1/8] Verifying OS compatibility..."
if ! uname -m | grep -q "aarch64"; then
    echo "[WARNING] This script is optimized for 64-bit Raspberry Pi OS"
fi

echo "[2/8] Updating system packages..."
sudo apt update
sudo apt upgrade -y

echo "[3/8] Installing Python dependencies..."
sudo apt install -y python3-pip python3-dev python3-smbus python3-venv
pip3 install --upgrade pip setuptools wheel
pip3 install numpy pygame PyOpenGL pyserial smbus-cffi

echo "[4/8] Installing performance optimization tools..."
sudo apt install -y python3-numpy-dev libblas-dev liblapack-dev libc6
sudo apt install -y libatlas-base-dev libjasper-dev libtiff5 libjasper1 libharfbuzz0b libwebp6 libtk8.6

echo "[5/8] Enabling I2C interface..."
sudo raspi-config nonint do_i2c 0

echo "[6/8] Installing I2C tools..."
sudo apt install -y i2c-tools

echo "[7/8] Adding user to i2c group..."
sudo usermod -aG i2c $USER

echo "[8/8] Verifying I2C setup..."
echo "Testing I2C bus..."
i2cdetect -y 1

echo ""
echo "================================================"
echo "Setup Complete!"
echo "================================================"
echo ""
echo "Note: Raspberry Pi 5 has improved performance and lower latency"
echo ""
echo "Next steps:"
echo "1. Log out and log back in for group changes to take effect"
echo "2. Connect MPU6050 sensor (I2C address 0x68)"
echo "3. Run: python3 mpu6050_rpi5.py"
echo ""
echo "For more information, see RaspberryPi/v5/README.md"
echo "================================================"
