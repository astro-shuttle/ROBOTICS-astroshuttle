#!/usr/bin/env python3
"""
TEST20-1: Compass and Ultrasonic Sensor Test

This program is designed to validate:
1. The QMC5883L compass readings, ensuring accurate heading (in degrees).
2. The distance readings from both left and right ultrasonic sensors.

Pinout for testing on the Raspberry Pi:
-----------------------------------------
QMC5883L Compass:
    - SDA: Connect to GPIO2 (SDA, physical pin 3)
    - SCL: Connect to GPIO3 (SCL, physical pin 5)
    - VCC: 3.3V or 5V (check your module specs)
    - GND: Ground

Ultrasonic Sensors:
  Left Sensor:
    - TRIG: Connect to GPIO17 (physical pin 11)
    - ECHO: Connect to GPIO27 (physical pin 13)
  Right Sensor:
    - TRIG: Connect to GPIO22 (physical pin 15)
    - ECHO: Connect to GPIO23 (physical pin 16)
-----------------------------------------

Press the 's' key to exit the test loop.
"""

import smbus2
import time
import math
import sys
import select
import termios
import tty
import RPi.GPIO as GPIO

# --- Terminal Setup for Key Detection ---
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

# --- I2C Setup for QMC5883L Compass ---
bus = smbus2.SMBus(1)
QMC5883L_ADDR = 0x0D

# Calibration constants (adjust these as needed)
minX, maxX = -248, 2161
minY, maxY = -23, 2687

# --- GPIO Pins for Ultrasonic Sensors ---
TRIG_LEFT = 17
ECHO_LEFT = 27
TRIG_RIGHT = 22
ECHO_RIGHT = 23

# --- Initialize GPIO ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)

# --- Helper Functions ---

def read_compass():
    """Read raw compass values from QMC5883L."""
    try:
        data = bus.read_i2c_block_data(QMC5883L_ADDR, 0x00, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        if x > 32767:
            x -= 65536
        if y > 32767:
            y -= 65536
        return x, y
    except Exception as e:
        print("Error reading compass:", e)
        return 0, 0

def calibrate(x, y):
    """Apply calibration offsets to raw compass data."""
    xCal = x - ((minX + maxX) / 2)
    yCal = y - ((minY + maxY) / 2)
    return xCal, yCal

def get_heading(x, y):
    """Calculate heading in degrees from calibrated data."""
    heading = math.atan2(y, x) * 180.0 / math.pi
    if heading < 0:
        heading += 360.0
    return heading

def measure_distance(trig_pin, echo_pin):
    """Measure distance using an ultrasonic sensor."""
    GPIO.output(trig_pin, False)
    time.sleep(0.0002)
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)
    
    start_time = time.time()
    timeout = start_time + 0.04
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
        if start_time > timeout:
            return None

    end_time = time.time()
    timeout = end_time + 0.04
    while GPIO.input(echo_pin) == 1:
        end_time = time.time()
        if end_time > timeout:
            return None

    pulse_duration = end_time - start_time
    distance = pulse_duration * 34300 / 2.0  # distance in cm
    return distance

def get_ultrasonic_distances():
    """Return the distances measured by the left and right ultrasonic sensors."""
    left_distance = measure_distance(TRIG_LEFT, ECHO_LEFT)
    right_distance = measure_distance(TRIG_RIGHT, ECHO_RIGHT)
    return left_distance, right_distance

# --- Main Test Loop ---
def main():
    print("TEST20-1: Compass and Ultrasonic Sensor Test")
    print("Press 's' to stop the test.\n")
    
    try:
        while True:
            # Read and compute compass heading
            x, y = read_compass()
            xCal, yCal = calibrate(x, y)
            heading = get_heading(xCal, yCal)
            
            # Read ultrasonic sensor distances
            left_dist, right_dist = get_ultrasonic_distances()
            
            # Print sensor readings
            print(f"Compass Heading: {heading:.1f}° | Left Ultrasonic: {left_dist if left_dist is not None else 'N/A'} cm | Right Ultrasonic: {right_dist if right_dist is not None else 'N/A'} cm")
            
            # Check for 's' key to stop the test
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 's':
                    print("Stop key pressed. Exiting test.")
                    break
            
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Exiting test.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        GPIO.cleanup()
        print("TEST20-1 terminated cleanly.")

if __name__ == "__main__":
    main()
