#!/usr/bin/env python3
"""
LPC2148 MPU6050 3D Visualizer
Real-time 3D visualization with serial IMU data from LPC2148 ARM processor.

Author: Naman Harshwal
Date: January 2026
Version: 1.0.0

Requirements:
  pip install pygame pyopengl numpy-stl pyserial
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import json
import serial
import threading
import time
from stl import mesh
import sys

SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
STL_FILE = 'model.STL'
WINDOW_WIDTH = 1024
WINDOW_HEIGHT = 768

class IMUData:
    def __init__(self):
        self.roll = self.pitch = self.yaw = 0.0
        self.lock = threading.Lock()
    
    def update(self, roll, pitch, yaw):
        with self.lock:
            self.roll, self.pitch, self.yaw = roll, pitch, yaw
    
    def get(self):
        with self.lock:
            return self.roll, self.pitch, self.yaw

class SerialReader(threading.Thread):
    def __init__(self, port, baudrate, imu_data):
        threading.Thread.__init__(self, daemon=True)
        self.port, self.baudrate, self.imu_data = port, baudrate, imu_data
        self.running = True
        self.ser = None
    
    def run(self):
        while self.running:
            try:
                if not self.ser:
                    print(f"[LPC2148] Connecting to {self.port} at {self.baudrate} baud...")
                    self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                    print("[LPC2148] Connected!")
                
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            data = json.loads(line)
                            self.imu_data.update(data['roll'], data['pitch'], data['yaw'])
                            print(f"[LPC2148] Roll: {data['roll']:.2f}° | Pitch: {data['pitch']:.2f}° | Yaw: {data['yaw']:.2f}°")
                        except: pass
            except serial.SerialException:
                if self.ser:
                    self.ser.close()
                    self.ser = None
                time.sleep(1)
    
    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()

def main():
    pygame.init()
    pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), DOUBLEBUF | OPENGL)
    pygame.display.set_caption('LPC2148 MPU6050 3D Visualizer')
    
    glEnable(GL_DEPTH_TEST)
    gluPerspective(45, (WINDOW_WIDTH / WINDOW_HEIGHT), 0.1, 500.0)
    glTranslatef(0, 0, -50)
    
    try:
        stl_mesh = mesh.Mesh.from_file(STL_FILE)
        print(f"✓ Loaded STL: {STL_FILE}")
    except:
        stl_mesh = None
        print("Using placeholder cube")
    
    imu_data = IMUData()
    reader = SerialReader(SERIAL_PORT, BAUD_RATE, imu_data)
    reader.start()
    
    clock = pygame.time.Clock()
    print("[LPC2148] Press ESC to exit\n")
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                running = False
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0, -50)
        
        roll, pitch, yaw = imu_data.get()
        glRotatef(pitch, 1.0, 0.0, 0.0)
        glRotatef(roll, 0.0, 0.0, 1.0)
        glRotatef(yaw, 0.0, 1.0, 0.0)
        
        if stl_mesh:
            glBegin(GL_TRIANGLES)
            glColor3f(0.2, 0.5, 0.9)
            for triangle in stl_mesh.vectors:
                for vertex in triangle:
                    glVertex3f(vertex[0], vertex[1], vertex[2])
            glEnd()
        
        pygame.display.flip()
        clock.tick(60)
    
    reader.stop()
    pygame.quit()
    print("\n[LPC2148] Closing...")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"[LPC2148 ERROR] {str(e)}")
        sys.exit(1)
