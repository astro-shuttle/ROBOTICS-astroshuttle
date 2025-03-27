import RPi.GPIO as GPIO
import time
import serial

# GPIO PINS for ULTRASONIC
TRIG_LEFT = 17
ECHO_LEFT = 27
TRIG_RIGHT = 22
ECHO_RIGHT = 23

# SERIAL INFO
SERIAL_PORT = '/dev/ttyUSB0'  # may change with lidar, etc.
BAUD_RATE = 9600

# DISTANCE THRESHOLD (for deciding if path is “clear”)
CLEAR_DISTANCE = 25.0

# Sweep increments: more steps = finer search, but slower
PIVOT_STEPS = 12 

# CONSTANTS FOR SCORING
MAX_OFFSET = 10.0       # maximum acceptable difference (cm) between left & right readings
PENALTY_FACTOR = 1.0    # penalty for imbalance (per cm difference)

# INITIALIZE GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)

# INITIALIZE SERIAL
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) 
    print(f"Serial connection established on {SERIAL_PORT}")
except Exception as e:
    print("Failed to open serial port:", e)
    exit()

# FUNCTIONS
def send_command(command):
    """
    Send a command string to the Arduino.
    For example: 'driveF', 'pivotL', 'pivotR', etc.
    """
    cmd = command.strip() + "\n"
    ser.write(cmd.encode('utf-8'))
    print("Sent:", command)

def read_response():
    """
    Non-blocking read of any available lines from the Arduino.
    """
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print("Arduino:", line)

def measure_distance(trig_pin, echo_pin):
    """
    Measure distance (in cm) using an ultrasonic sensor.
    Returns the distance in cm or None on error.
    """
    # Ensure trigger is low, then send a 10µs pulse.
    GPIO.output(trig_pin, False)
    time.sleep(0.0002)
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    # Wait for echo to start (rising edge)
    start_time = time.time()
    timeout = start_time + 0.04 
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
        if start_time > timeout:
            return None

    # Wait for echo to end (falling edge)
    end_time = time.time()
    timeout = end_time + 0.04
    while GPIO.input(echo_pin) == 1:
        end_time = time.time()
        if end_time > timeout:
            return None

    # Calculate distance (speed of sound: 34300 cm/s)
    pulse_duration = end_time - start_time
    distance = pulse_duration * 34300 / 2.0
    return distance

def get_ultrasonic_distances():
    """
    Returns a tuple (left_dist, right_dist) in cm.
    If a sensor read fails, returns None for that sensor.
    """
    left = measure_distance(TRIG_LEFT, ECHO_LEFT)
    right = measure_distance(TRIG_RIGHT, ECHO_RIGHT)
    return (left, right)

def path_is_clear(left_dist, right_dist, threshold=CLEAR_DISTANCE):
    """
    Returns True if both distances are above the threshold.
    """
    if left_dist is None or right_dist is None:
        return False
    return (left_dist > threshold and right_dist > threshold)

def servo_sweep_scan():
    """
    Perform a sweep by moving the sensors (or pivoting the chassis) incrementally
    left and right. Record the left and right distance measurements at each step.
    Then return to center and evaluate which angle (measured in steps from center)
    gives the best reading (high combined distance and balanced left/right values).
    
    Returns:
        best_angle (int): negative for left, positive for right, 0 for center.
    """
    measurements = []  # List to store tuples: (angle, left, right)
    
    # 0. Record center measurement (angle 0)
    left, right = get_ultrasonic_distances()
    measurements.append((0, left, right))
    print("Center measurement:", (0, left, right))
    time.sleep(0.2)
    
    # 1. Sweep left from center
    for i in range(1, PIVOT_STEPS + 1):
        send_command("pivotL")  # adjust sensor/rover left a small step
        time.sleep(0.3)
        send_command("S")       # stop movement
        time.sleep(0.2)
        left, right = get_ultrasonic_distances()
        measurements.append((-i, left, right))
        print(f"Left sweep step {-i} measurement:", (left, right))
    
    # 2. Return to center from left sweep
    for i in range(PIVOT_STEPS):
        send_command("pivotR")
        time.sleep(0.3)
        send_command("S")
        time.sleep(0.2)
    
    # 3. Sweep right from center
    for i in range(1, PIVOT_STEPS + 1):
        send_command("pivotR")
        time.sleep(0.3)
        send_command("S")
        time.sleep(0.2)
        left, right = get_ultrasonic_distances()
        measurements.append((i, left, right))
        print(f"Right sweep step {i} measurement:", (left, right))
    
    # 4. Return to center from right sweep
    for i in range(PIVOT_STEPS):
        send_command("pivotL")
        time.sleep(0.3)
        send_command("S")
        time.sleep(0.2)
    
    # 5. Evaluate all measurements using a scoring function.
    #    Score = (left + right) - (penalty * abs(left - right))
    best_angle = 0
    best_score = -1
    for angle, left, right in measurements:
        if left is None or right is None:
            continue  # skip invalid readings
        diff = abs(left - right)
        score = left + right - PENALTY_FACTOR * diff
        print(f"Angle {angle}: left {left:.1f} cm, right {right:.1f} cm, diff {diff:.1f}, score {score:.1f}")
        if score > best_score:
            best_score = score
            best_angle = angle
    
    print("All sweep measurements:", measurements)
    print("Best angle chosen:", best_angle, "with score:", best_score)
    return best_angle

def main():
    """
    Main loop:
    1. Drive forward briefly.
    2. Stop and perform a servo (or chassis pivot) sweep scan.
    3. Pivot to the best angle based on the sweep.
    4. Drive forward again.
    """
    try:
        while True:
            # Drive forward for a short burst
            print("Driving forward...")
            send_command("driveF")
            time.sleep(1.0)  # drive forward for 1 second
            send_command("S")
            read_response()
            time.sleep(0.5)
            
            # Perform sweep scan
            print("Performing servo sweep scan...")
            best_angle = servo_sweep_scan()
            
            # Pivot to the chosen best angle
            if best_angle < 0:
                print(f"Pivoting left {abs(best_angle)} step(s) to align.")
                for i in range(abs(best_angle)):
                    send_command("pivotL")
                    time.sleep(0.3)
                    send_command("S")
                    time.sleep(0.2)
            elif best_angle > 0:
                print(f"Pivoting right {best_angle} step(s) to align.")
                for i in range(best_angle):
                    send_command("pivotR")
                    time.sleep(0.3)
                    send_command("S")
                    time.sleep(0.2)
            else:
                print("Best angle is center; no pivot needed.")
            
            # Drive forward again after aligning
            print("Driving forward after alignment...")
            send_command("driveF")
            time.sleep(1.0)
            send_command("S")
            read_response()
            
            time.sleep(1.0)  # pause before the next cycle
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()
        send_command("S")
        ser.close()

if __name__ == "__main__":
    main()
