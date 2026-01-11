#!/usr/bin/env python3
"""
ESP32 MPU6050 3D Visualizer
Real-time 3D STL model visualization with serial IMU data from ESP32

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
import math
import sys

# Configuration
SERIAL_PORT = 'COM3'          # Change to your ESP32 serial port
BAUD_RATE = 115200            # ESP32 uses 115200 baud
STL_FILE = 'model.STL'        # Path to your 3D model
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
FPS = 60

class IMUData:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.temp = 0.0
        self.lock = threading.Lock()

    def update(self, roll, pitch, yaw, temp=0.0):
        with self.lock:
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw
            self.temp = temp

    def get_angles(self):
        with self.lock:
            return self.roll, self.pitch, self.yaw, self.temp

class SerialReader(threading.Thread):
    def __init__(self, port, baudrate, imu_data):
        threading.Thread.__init__(self, daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.imu_data = imu_data
        self.running = True
        self.ser = None
        self.last_error_time = 0

    def run(self):
        while self.running:
            try:
                if self.ser is None:
                    print(f"[ESP32] Connecting to {self.port} at {self.baudrate} baud...")
                    self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                    print(f"[ESP32] Connected!")
                
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            data = json.loads(line)
                            self.imu_data.update(
                                data.get('roll', 0),
                                data.get('pitch', 0),
                                data.get('yaw', 0),
                                data.get('temp', 0)
                            )
                            print(f"[ESP32] Roll: {data['roll']:.2f}°, Pitch: {data['pitch']:.2f}°, Yaw: {data['yaw']:.2f}°, Temp: {data['temp']:.1f}°C")
                        except json.JSONDecodeError:
                            pass
            except serial.SerialException as e:
                if time.time() - self.last_error_time > 5:  # Log error once every 5 seconds
                    print(f"[ESP32 ERROR] {str(e)}")
                    self.last_error_time = time.time()
                if self.ser:
                    self.ser.close()
                    self.ser = None
                time.sleep(1)
            except Exception as e:
                print(f"[ESP32 ERROR] Unexpected error: {str(e)}")
                if self.ser:
                    self.ser.close()
                    self.ser = None
                time.sleep(1)

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()

def load_stl(filename):
    """Load STL mesh file"""
    try:
        stl_mesh = mesh.Mesh.from_file(filename)
        print(f"✓ Loaded STL: {filename}")
        return stl_mesh
    except FileNotFoundError:
        print(f"ERROR: STL file not found: {filename}")
        return None

def render_mesh(stl_mesh):
    """Render the STL mesh"""
    glBegin(GL_TRIANGLES)
    glColor3f(0.2, 0.5, 0.9)  # Blue color
    
    for triangle in stl_mesh.vectors:
        for vertex in triangle:
            glVertex3f(vertex[0], vertex[1], vertex[2])
    
    glEnd()

def main():
    # Initialize Pygame and OpenGL
    pygame.init()
    display = (WINDOW_WIDTH, WINDOW_HEIGHT)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption('ESP32 MPU6050 3D Visualizer')
    
    # OpenGL setup
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    
    gluPerspective(45, (WINDOW_WIDTH / WINDOW_HEIGHT), 0.1, 500.0)
    glTranslatef(0, 0, -50)
    
    # Light setup
    glLight(GL_LIGHT0, GL_POSITION, (50, 50, 50, 0))
    glLight(GL_LIGHT0, GL_AMBIENT, (0.2, 0.2, 0.2, 1))
    glLight(GL_LIGHT0, GL_DIFFUSE, (0.8, 0.8, 0.8, 1))
    
    # Load STL model
    stl_mesh = load_stl(STL_FILE)
    if stl_mesh is None:
        print("Failed to load STL file. Using placeholder.")
        stl_mesh = None
    
    # Initialize IMU data and serial reader
    imu_data = IMUData()
    reader = SerialReader(SERIAL_PORT, BAUD_RATE, imu_data)
    reader.start()
    
    clock = pygame.time.Clock()
    yaw_enabled = True
    frame_count = 0
    start_time = time.time()
    
    print("\n[ESP32 Visualizer] Starting 3D visualization...")
    print("[ESP32 Visualizer] Press 'Z' to toggle yaw rotation")
    print("[ESP32 Visualizer] Press 'ESC' to exit\n")
    
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
                    print(f"[ESP32] Yaw rotation: {'ENABLED' if yaw_enabled else 'DISABLED'}")
        
        # Clear screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0, -50)
        
        # Get IMU angles
        roll, pitch, yaw, temp = imu_data.get_angles()
        
        # Apply rotations based on IMU data
        glRotatef(pitch, 1.0, 0.0, 0.0)     # Pitch around X-axis
        glRotatef(roll, 0.0, 0.0, 1.0)      # Roll around Z-axis
        if yaw_enabled:
            glRotatef(yaw, 0.0, 1.0, 0.0)   # Yaw around Y-axis
        
        # Render mesh
        if stl_mesh:
            render_mesh(stl_mesh)
        else:
            # Render a simple cube as placeholder
            from OpenGL.GL import GL_QUADS
            glColor3f(0.5, 0.5, 0.5)
            glBegin(GL_QUADS)
            # Front
            glVertex3f(-1, -1, 1)
            glVertex3f(1, -1, 1)
            glVertex3f(1, 1, 1)
            glVertex3f(-1, 1, 1)
            glEnd()
        
        # Display info
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        
        glDisable(GL_LIGHTING)
        glColor3f(1, 1, 1)
        # Info would be displayed here if using text rendering
        glEnable(GL_LIGHTING)
        
        glPopMatrix()
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        
        pygame.display.flip()
        clock.tick(FPS)
        frame_count += 1
        
        # Print FPS every second
        elapsed = time.time() - start_time
        if int(elapsed) > 0 and frame_count % FPS == 0:
            print(f"[ESP32] FPS: {frame_count / elapsed:.1f} | Roll: {roll:6.2f}° | Pitch: {pitch:6.2f}° | Yaw: {yaw:6.2f}° | Temp: {temp:5.1f}°C")
    
    # Cleanup
    reader.stop()
    pygame.quit()
    print("\n[ESP32 Visualizer] Closing...")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"[ESP32 ERROR] {str(e)}")
        sys.exit(1)
