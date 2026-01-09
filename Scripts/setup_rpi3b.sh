#!/bin/bash

# MPU6050 IMU 3D Visualization Setup Script for Raspberry Pi 3B
# This script automates the complete setup process
# Author: Naman Harshwal
# License: MIT

set -e

echo "================================================"
echo "MPU6050 IMU 3D Visualization Setup - Raspberry Pi 3B"
echo "================================================"
echo ""

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi 3 Model B" /proc/device-tree/model 2>/dev/null; then
    echo "[WARNING] This script is optimized for Raspberry Pi 3B"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "[1/6] Updating system packages..."
sudo apt update
sudo apt upgrade -y

echo "[2/6] Installing Python dependencies..."
sudo apt install -y python3-pip python3-dev python3-smbus
pip3 install --upgrade pip setuptools wheel
pip3 install numpy pygame PyOpenGL pyserial smbus-cffi

echo "[3/6] Enabling I2C interface..."
sudo raspi-config nonint do_i2c 0

echo "[4/6] Installing I2C tools..."
sudo apt install -y i2c-tools

echo "[5/6] Adding user to i2c group..."
sudo usermod -aG i2c $USER

echo "[6/6] Verifying I2C setup..."
echo "Testing I2C bus..."
i2cdetect -y 1

echo ""
echo "================================================"
echo "Setup Complete!"
echo "================================================"
echo ""
echo "Next steps:"
echo "1. Log out and log back in for group changes to take effect"
echo "2. Connect MPU6050 sensor (I2C address 0x68)"
echo "3. Run: python3 mpu6050_rpi3b.py"
echo ""
echo "For more information, see RaspberryPi/v3B/README.md"
echo "================================================"
