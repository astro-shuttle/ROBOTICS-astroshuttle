import smbus2
import time
import math
import serial
import sys
import select
import termios
import tty

#part of s to stop command
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)  # Set terminal to raw mode so we don't need Enter


# Adjust Later
steering_time = 0.4
HEADING_THRESHOLD = 90  # Adjust this threshold based on tuning

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

# Constants for calibration
minX, maxX = -248, 2161
minY, maxY = -23, 2687

# Initialize I2C bus for QMC5883L
bus = smbus2.SMBus(1)

# Compass Address
QMC5883L_ADDR = 0x0D

# Initialize serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Serial connection established on", SERIAL_PORT)
except Exception as e:
    print("Failed to open serial port:", e)
    exit()

########################################################
#                    HELPER FUNCTIONS                  #
########################################################

def send_command(command):
    """Send a command string to the Arduino."""
    cmd = command.strip() + "\n"
    ser.write(cmd.encode('utf-8'))
    print("Sent:", command)

def read_response():
    """Read any available output from the Arduino."""
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print("Arduino:", line)

def read_compass():
    """Read raw compass values from QMC5883L"""
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
        return 0, 0  # Return default values if there's an error

def calibrate(x, y):
    """Apply calibration offsets"""
    xCal = x - ((minX + maxX) / 2)
    yCal = y - ((minY + maxY) / 2)
    return xCal, yCal

def get_heading(x, y):
    """Calculate heading in degrees"""
    heading = math.atan2(y, x) * 180.0 / math.pi
    if heading < 0:
        heading += 360.0
    return heading

def map_value(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another"""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


time.sleep(1)  # Delay for startup
print("Starting setup...")

# Get initial heading
x, y = read_compass()
xCal, yCal = calibrate(x, y)
targetHeading = get_heading(xCal, yCal)
print(f"Target Heading: {targetHeading}")

try:
    
    while True:
        x, y = read_compass()
        xCal, yCal = calibrate(x, y)
        heading = get_heading(xCal, yCal)
        print(f"Current Heading: {heading}")
    
    # Calculate heading difference
        diff = heading - targetHeading
        if diff < -180:
            diff += 360
        if diff > 180:
            diff -= 360
    
    # Map heading difference to servo position
        servo_position = map_value(abs(diff), 0, 180, 0, 180)
        
        if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == 's':
                    print("Stop key pressed. Exiting loop.")
                    break
    
    # Check if correction is needed
        if abs(diff) > 30:
            adjust_time = steering_time * (abs(diff) / HEADING_THRESHOLD)
        
            if diff > 0:
                print("TURNING LEFT")
                send_command(f"steerL {servo_position}")
                time.sleep(adjust_time)
                send_command("steerZ")
            else:
                print("TURNING RIGHT")
                send_command(f"steerR {servo_position}")
                time.sleep(adjust_time)
                send_command("steerZ")
        
            time.sleep(1)
        
            

        
    
        time.sleep(0.5)
    
finally:
    termios.tcsetattr(fd,termios.TCSADRAIN, old_settings)
    print("Program Terminated cleanly.")
