#!/usr/bin/env python3
"""
Raspberry Pi 4 MPU6050 3D Visualizer
Real-time 3D STL model visualization with MPU6050 IMU sensor data

Author: Naman Harshwal
Date: January 2026
Version: 1.0.0

Requirements:
  pip install pygame pyopengl numpy-stl smbus

Hardware:
  - Raspberry Pi 4 (BCM2711 ARM Cortex-A72 @ 1.5GHz)
  - MPU6050 IMU sensor (I2C address 0x68)
  - 3D STL model file
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import json
import smbus
import time
from stl import mesh
import math
import threading
import sys

# Configuration
MPU6050_ADDR = 0x68
I2C_BUS = 1  # Use I2C bus 1 on Raspberry Pi
STL_FILE = 'model.STL'
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 960
FPS = 60

# MPU6050 Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

class MPU6050Reader(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)
        self.bus = smbus.SMBus(I2C_BUS)
        self.roll = self.pitch = self.yaw = 0.0
        self.lock = threading.Lock()
        self.running = True
        self.last_time = time.time()
        
        # Sensor calibration offsets
        self.gyro_x_offset = self.gyro_y_offset = self.gyro_z_offset = 0.0
        self.calibrate()
    
    def calibrate(self):
        """Calibrate gyroscope offsets"""
        print("[RPi4] Calibrating MPU6050...")
        samples = 200
        gx, gy, gz = 0, 0, 0
        
        for i in range(samples):
            try:
                # Read gyroscope data
                gx_raw = self.read_i2c_word(GYRO_XOUT_H)
                gy_raw = self.read_i2c_word(GYRO_XOUT_H + 2)
                gz_raw = self.read_i2c_word(GYRO_XOUT_H + 4)
                
                gx += gx_raw / 131.0  # LSB sensitivity
                gy += gy_raw / 131.0
                gz += gz_raw / 131.0
            except:
                pass
            time.sleep(0.01)
        
        self.gyro_x_offset = gx / samples
        self.gyro_y_offset = gy / samples
        self.gyro_z_offset = gz / samples
        print("[RPi4] Calibration complete!")
    
    def read_i2c_word(self, register):
        """Read 16-bit word from I2C register"""
        try:
            high = self.bus.read_byte_data(MPU6050_ADDR, register)
            low = self.bus.read_byte_data(MPU6050_ADDR, register + 1)
            value = (high << 8) | low
            if value >= 0x8000:
                value -= 0x10000
            return value
        except:
            return 0
    
    def run(self):
        """Read sensor data continuously"""
        print("[RPi4] Starting sensor reading...")
        
        # Wake up MPU6050
        try:
            self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
        except:
            print("[RPi4] Failed to wake up MPU6050")
            return
        
        time.sleep(0.1)
        
        while self.running:
            try:
                # Read accelerometer
                ax = self.read_i2c_word(ACCEL_XOUT_H) / 16384.0
                ay = self.read_i2c_word(ACCEL_XOUT_H + 2) / 16384.0
                az = self.read_i2c_word(ACCEL_XOUT_H + 4) / 16384.0
                
                # Read gyroscope
                gx = self.read_i2c_word(GYRO_XOUT_H) / 131.0 - self.gyro_x_offset
                gy = self.read_i2c_word(GYRO_XOUT_H + 2) / 131.0 - self.gyro_y_offset
                gz = self.read_i2c_word(GYRO_XOUT_H + 4) / 131.0 - self.gyro_z_offset
                
                # Time delta
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time
                
                # Complementary filter
                accel_roll = math.atan2(ay, az) * 180.0 / math.pi
                accel_pitch = math.asin(-ax / 9.81) * 180.0 / math.pi
                
                alpha = 0.98
                with self.lock:
                    self.roll = alpha * (self.roll + gx * dt) + (1 - alpha) * accel_roll
                    self.pitch = alpha * (self.pitch + gy * dt) + (1 - alpha) * accel_pitch
                    self.yaw = self.yaw + gz * dt
                
                time.sleep(0.01)  # 100 Hz
            except Exception as e:
                print(f"[RPi4] Sensor error: {str(e)}")
                time.sleep(0.1)
    
    def get_angles(self):
        """Get current orientation angles"""
        with self.lock:
            return self.roll, self.pitch, self.yaw
    
    def stop(self):
        """Stop sensor reading"""
        self.running = False

def main():
    pygame.init()
    pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), DOUBLEBUF | OPENGL)
    pygame.display.set_caption('Raspberry Pi 4 MPU6050 3D Visualizer')
    
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    gluPerspective(45, (WINDOW_WIDTH / WINDOW_HEIGHT), 0.1, 500.0)
    glTranslatef(0, 0, -50)
    
    # Load STL model
    try:
        stl_mesh = mesh.Mesh.from_file(STL_FILE)
        print(f"[RPi4] Loaded STL: {STL_FILE}")
    except:
        stl_mesh = None
        print("[RPi4] Using placeholder cube")
    
    # Start sensor reading
    sensor = MPU6050Reader()
    sensor.start()
    
    clock = pygame.time.Clock()
    yaw_enabled = True
    print("[RPi4] Press Z to toggle yaw, ESC to exit\n")
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
                elif event.key == K_z:
                    yaw_enabled = not yaw_enabled
                    print(f"[RPi4] Yaw: {'ON' if yaw_enabled else 'OFF'}")
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0, -50)
        
        roll, pitch, yaw = sensor.get_angles()
        glRotatef(pitch, 1.0, 0.0, 0.0)
        glRotatef(roll, 0.0, 0.0, 1.0)
        if yaw_enabled:
            glRotatef(yaw, 0.0, 1.0, 0.0)
        
        if stl_mesh:
            glBegin(GL_TRIANGLES)
            glColor3f(0.2, 0.7, 0.9)
            for triangle in stl_mesh.vectors:
                for vertex in triangle:
                    glVertex3f(vertex[0], vertex[1], vertex[2])
            glEnd()
        
        pygame.display.flip()
        clock.tick(FPS)
    
    sensor.stop()
    pygame.quit()
    print("[RPi4] Closing...")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"[RPi4 ERROR] {str(e)}")
        sys.exit(1)
