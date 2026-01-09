#!/bin/bash

# MPU6050 IMU 3D Visualization Setup Script for Raspberry Pi 4
# This script automates the complete setup process with enhanced performance features
# Author: Naman Harshwal
# License: MIT

set -e

echo "================================================"
echo "MPU6050 IMU 3D Visualization Setup - Raspberry Pi 4"
echo "================================================"
echo ""

# Check if running on Raspberry Pi 4
if ! grep -q "Raspberry Pi 4" /proc/device-tree/model 2>/dev/null; then
    echo "[WARNING] This script is optimized for Raspberry Pi 4"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "[1/7] Updating system packages..."
sudo apt update
sudo apt upgrade -y

echo "[2/7] Installing Python dependencies..."
sudo apt install -y python3-pip python3-dev python3-smbus
pip3 install --upgrade pip setuptools wheel
pip3 install numpy pygame PyOpenGL pyserial smbus-cffi

echo "[3/7] Installing performance optimization tools..."
sudo apt install -y python3-numpy-dev libblas-dev liblapack-dev libc6

echo "[4/7] Enabling I2C interface..."
sudo raspi-config nonint do_i2c 0

echo "[5/7] Installing I2C tools..."
sudo apt install -y i2c-tools

echo "[6/7] Adding user to i2c group..."
sudo usermod -aG i2c $USER

echo "[7/7] Verifying I2C setup..."
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
echo "3. Run: python3 mpu6050_rpi4.py"
echo ""
echo "For more information, see RaspberryPi/v4/README.md"
echo "================================================"
