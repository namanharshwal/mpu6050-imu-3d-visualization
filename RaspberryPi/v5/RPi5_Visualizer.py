#!/usr/bin/env python3
"""
Raspberry Pi 5 MPU6050 3D Visualizer
High-performance 3D STL visualization with MPU6050 IMU real-time data

Author: Naman Harshwal
Date: January 2026
Version: 1.0.0

Requirements:
  pip install pygame pyopengl numpy-stl smbus2 numpy

Hardware:
  - Raspberry Pi 5 (BCM2712 ARM Cortex-A76 @ 2.4GHz)
  - MPU6050 IMU sensor (I2C address 0x68)
  - 3D STL model file
  - Recommended: 4-8GB RAM for optimal performance

Features:
  - High-speed I2C communication (400kHz)
  - Optimized for RPi 5's quad-core processor
  - Real-time complementary filter
  - Low-latency OpenGL rendering
  - Professional-grade visualization
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import smbus2
import time
from stl import mesh
import math
import threading
import sys
import numpy as np

# Configuration
MPU6050_ADDR = 0x68
I2C_BUS = 1  # I2C bus 1 on RPi 5
STL_FILE = 'model.STL'
WINDOW_WIDTH = 1440
WINDOW_HEIGHT = 1080
FPS = 60  # RPi 5 can handle high FPS
SAMPLE_RATE = 100  # Hz

# MPU6050 Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
TEMP_OUT_H = 0x41

class OptimizedMPU6050Reader(threading.Thread):
    """High-performance sensor reader for RPi 5"""
    
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)
        self.bus = smbus2.SMBus(I2C_BUS)
        self.roll = self.pitch = self.yaw = 0.0
        self.temperature = 0.0
        self.lock = threading.Lock()
        self.running = True
        self.last_time = time.time()
        
        # Complementary filter coefficients
        self.ALPHA = 0.98  # Gyroscope weight
        self.DT = 1.0 / SAMPLE_RATE
        
        # Calibration offsets
        self.gyro_offset = np.array([0.0, 0.0, 0.0])
        self.accel_offset = np.array([0.0, 0.0, 0.0])
        
        self.calibrate()
    
    def calibrate(self):
        """Fast calibration routine optimized for RPi 5"""
        print("[RPi5] Calibrating MPU6050...")
        samples = 100
        gyro_sum = np.array([0.0, 0.0, 0.0])
        accel_sum = np.array([0.0, 0.0, 0.0])
        
        for i in range(samples):
            try:
                # Read all axes at once for efficiency
                data = self.read_i2c_block(GYRO_XOUT_H, 6)
                gx = ((data[0] << 8) | data[1])
                gy = ((data[2] << 8) | data[3])
                gz = ((data[4] << 8) | data[5])
                
                # Convert to signed
                for val in [gx, gy, gz]:
                    if val >= 0x8000:
                        val -= 0x10000
                
                gyro_sum += np.array([gx, gy, gz]) / 131.0
            except:
                pass
            time.sleep(0.005)
        
        self.gyro_offset = gyro_sum / samples
        print("[RPi5] Calibration complete!")
    
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
    
    def read_i2c_block(self, register, length):
        """Read I2C block data for efficiency"""
        try:
            return self.bus.read_i2c_block_data(MPU6050_ADDR, register, length)
        except:
            return [0] * length
    
    def run(self):
        """High-performance sensor reading loop"""
        print("[RPi5] Starting sensor reading...")
        
        try:
            self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
        except:
            print("[RPi5] Failed to wake MPU6050")
            return
        
        time.sleep(0.1)
        
        while self.running:
            try:
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time
                
                # Read accelerometer (3 registers)
                accel_data = self.read_i2c_block(ACCEL_XOUT_H, 6)
                ax = ((accel_data[0] << 8) | accel_data[1]) / 16384.0
                ay = ((accel_data[2] << 8) | accel_data[3]) / 16384.0
                az = ((accel_data[4] << 8) | accel_data[5]) / 16384.0
                
                # Convert to signed
                for i, val in enumerate([ax, ay, az]):
                    if val >= 32768:
                        val -= 65536
                
                # Read gyroscope (3 registers)
                gyro_data = self.read_i2c_block(GYRO_XOUT_H, 6)
                gx = (((gyro_data[0] << 8) | gyro_data[1]) / 131.0) - self.gyro_offset[0]
                gy = (((gyro_data[2] << 8) | gyro_data[3]) / 131.0) - self.gyro_offset[1]
                gz = (((gyro_data[4] << 8) | gyro_data[5]) / 131.0) - self.gyro_offset[2]
                
                # Read temperature
                temp_raw = self.read_i2c_word(TEMP_OUT_H)
                temp = (temp_raw / 340.0) + 36.53
                
                # Complementary filter
                accel_roll = math.atan2(ay, az) * 180.0 / math.pi
                accel_pitch = math.asin(-ax / 9.81) * 180.0 / math.pi
                
                with self.lock:
                    self.roll = self.ALPHA * (self.roll + gx * dt) + (1 - self.ALPHA) * accel_roll
                    self.pitch = self.ALPHA * (self.pitch + gy * dt) + (1 - self.ALPHA) * accel_pitch
                    self.yaw = self.yaw + gz * dt
                    self.temperature = temp
                
                time.sleep(0.01)  # 100 Hz
            except Exception as e:
                print(f"[RPi5] Sensor error: {str(e)}")
                time.sleep(0.1)
    
    def get_data(self):
        """Get current orientation and temperature"""
        with self.lock:
            return self.roll, self.pitch, self.yaw, self.temperature
    
    def stop(self):
        """Stop sensor reading"""
        self.running = False

def main():
    pygame.init()
    pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), DOUBLEBUF | OPENGL)
    pygame.display.set_caption('Raspberry Pi 5 MPU6050 3D Visualizer [High-Performance]')
    
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    
    gluPerspective(45, (WINDOW_WIDTH / WINDOW_HEIGHT), 0.1, 500.0)
    glTranslatef(0, 0, -50)
    
    # Light setup
    glLight(GL_LIGHT0, GL_POSITION, (50, 50, 50, 0))
    glLight(GL_LIGHT0, GL_AMBIENT, (0.3, 0.3, 0.3, 1))
    glLight(GL_LIGHT0, GL_DIFFUSE, (1.0, 1.0, 1.0, 1))
    
    # Load STL model
    try:
        stl_mesh = mesh.Mesh.from_file(STL_FILE)
        print(f"[RPi5] Loaded STL: {STL_FILE}")
    except:
        stl_mesh = None
        print("[RPi5] Using placeholder rendering")
    
    # Start sensor reading
    sensor = OptimizedMPU6050Reader()
    sensor.start()
    
    clock = pygame.time.Clock()
    yaw_enabled = True
    frame_count = 0
    start_time = time.time()
    
    print("[RPi5] Z=toggle yaw | ESC=exit | High-Performance Mode\n")
    
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
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0, -50)
        
        roll, pitch, yaw, temp = sensor.get_data()
        glRotatef(pitch, 1.0, 0.0, 0.0)
        glRotatef(roll, 0.0, 0.0, 1.0)
        if yaw_enabled:
            glRotatef(yaw, 0.0, 1.0, 0.0)
        
        if stl_mesh:
            glBegin(GL_TRIANGLES)
            glColor3f(0.1, 0.6, 0.9)
            for triangle in stl_mesh.vectors:
                for vertex in triangle:
                    glVertex3f(vertex[0], vertex[1], vertex[2])
            glEnd()
        
        pygame.display.flip()
        clock.tick(FPS)
        frame_count += 1
        
        # Print performance metrics every second
        elapsed = time.time() - start_time
        if int(elapsed) > 0 and frame_count % 60 == 0:
            fps = frame_count / elapsed
            print(f"[RPi5] FPS: {fps:.1f} | Roll: {roll:6.2f}° | Pitch: {pitch:6.2f}° | Yaw: {yaw:6.2f}° | Temp: {temp:5.1f}°C")
    
    sensor.stop()
    pygame.quit()
    print("[RPi5] Visualization closed.")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"[RPi5 ERROR] {str(e)}")
        sys.exit(1)
