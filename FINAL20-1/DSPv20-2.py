import smbus2
import time
import math
import serial
import sys
import select
import termios
import tty

# Terminal setup for key detection (raw mode)
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

# --- Configuration Constants ---
steering_time = 0.4              # Base steering time (seconds)
HEADING_THRESHOLD = 90           # Threshold for scaling steering correction
STOP_DIFF_THRESHOLD = 30         # Correction only if deviation > 30°

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

# Calibration constants for compass (adjust as needed)
minX, maxX = -248, 2161
minY, maxY = -23, 2687

# I2C setup for QMC5883L compass
bus = smbus2.SMBus(1)
QMC5883L_ADDR = 0x0D

# --- Initialize Serial ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Serial connection established on", SERIAL_PORT)
except Exception as e:
    print("Failed to open serial port:", e)
    exit()

########################################################
#               HELPER FUNCTIONS                       #
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

def map_value(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another."""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def get_smoothed_heading(samples=5, delay=0.05):
    """
    Get a smoothed heading by averaging multiple compass readings.
    Uses vector averaging to correctly handle angular wrap-around.
    """
    headings = []
    for _ in range(samples):
        x, y = read_compass()
        # Skip invalid readings
        if x == 0 and y == 0:
            continue
        xCal, yCal = calibrate(x, y)
        headings.append(get_heading(xCal, yCal))
        time.sleep(delay)
    if not headings:
        return None
    # Vector average to handle circular mean
    sin_sum = sum(math.sin(math.radians(h)) for h in headings)
    cos_sum = sum(math.cos(math.radians(h)) for h in headings)
    avg_angle = math.degrees(math.atan2(sin_sum, cos_sum))
    if avg_angle < 0:
        avg_angle += 360
    return avg_angle

# --- (Optional) PID Controller Placeholder ---
# For future refinement, you might implement a PID controller to improve stability:
# Kp, Ki, Kd = <tuned values>
# Then compute:
#    error = diff, update integral and derivative, and determine correction.
# For now, we continue using a proportional scaling of steering_time.

time.sleep(1)  # Startup delay
print("Starting setup...")

# Set the initial target heading using smoothed readings
targetHeading = get_smoothed_heading()
if targetHeading is None:
    print("Failed to obtain valid compass readings. Exiting.")
    exit()
print(f"Initial Target Heading: {targetHeading}")

########################################################
#                        MAIN LOOP                     #
########################################################

try:
    while True:
        # Get smoothed current heading
        current_heading = get_smoothed_heading()
        if current_heading is None:
            print("Invalid compass data; skipping iteration.")
            continue
        print(f"Current Heading: {current_heading}")
    
        # Calculate heading difference and normalize to [-180, 180]
        diff = current_heading - targetHeading
        if diff < -180:
            diff += 360
        if diff > 180:
            diff -= 360
    
        # Map the absolute difference to a servo position value (0-180)
        servo_position = map_value(abs(diff), 0, 180, 0, 180)
        
        # Check for stop key input
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 's':
                print("Stop key pressed. Exiting loop.")
                break
    
        # If correction is needed
        if abs(diff) > STOP_DIFF_THRESHOLD:
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
            # Short delay to allow correction to settle
            time.sleep(1)
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stop signal received.")

finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print("Program Terminated cleanly.")
