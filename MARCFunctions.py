# Built-in Python libraries, not visible in the CIRCUITPYTHON drive
import time
import busio
import board
import digitalio
import supervisor
import traceback
import math


# Python modules from the "lib" folder
from lib.marc import MARC
def seconds_since_boot():
    return f"{time.monotonic():.3f}"

# Initialize MARC with proper pins
print(seconds_since_boot() + " - Initializing MARC...")
marc = MARC(
    r_pins=(board.GP28, board.GP19), # Right motor pins
    l_pins=(board.GP17, board.GP18), # Left motor pins
    x_pins=(board.GP20, board.GP21),  # Extra motor pins
    servo_pins = (board.GP7, board.GP8, board.GP9, board.GP10)
    #dumpServo = servo_0
    #armServo = servo_1
)
uart1 = busio.UART(board.GP4, board.GP5, baudrate=115200)





#servo functions
def runDump(state):
    if state == "up":
        marc.move_single_servo(
            servo_name="0",
            angle=0
        )
        marc.wait(1)


    if state == "down":
        marc.move_single_servo(
            servo_name="0",
            angle=57
        )
        marc.wait(1)


def armOpen(state):
    if state == "open":
        marc.move_single_servo(
            servo_name='1',
            angle=0
        )
        marc.wait(1)


    if state == "closed":
        marc.move_single_servo(
            servo_name="1",
            angle=0
        )
        marc.wait(1)


#motor movement def

def spin(spinFac):
    # spinFac > 0 is CCW, spinFac < 0 is CW
    if spinFac > 0:
        # Counter-Clockwise: Right wheel is 1.2x stronger
        r_speed = spinFac * 1.2
        l_speed = spinFac
    else:
        # Clockwise (or stopped): Keep them equal
        r_speed = spinFac
        l_speed = spinFac
        
    marc.set_multiple_motor_throttles(
        motor_throttles=[r_speed, l_speed, 0], # Right, Left, Extra
        runtime = 1
    )

def forward(speedFac):
    #og speedFac was 0.5
    x = speedFac
    marc.set_multiple_motor_throttles(
        motor_throttles=[-x, x, 0],#Right, Left, Extra
        runtime = 1
    )


#LiDAR DEF
def scan():
    # If the buffer is getting too full, it's old data. Clear it.
    if uart1.in_waiting > 64:
        uart1.reset_input_buffer()
        return None

    while uart1.in_waiting >= 9:
        # Peek at the first byte
        byte1 = uart1.read(1)
        if byte1 == b'\x59':
            # Look for the second header byte
            byte2 = uart1.read(1)
            if byte2 == b'\x59':
                # We found the header! Read the remaining 7 bytes of the packet
                data = uart1.read(7)
                if len(data) == 7:
                    distance = data[0] + (data[1] << 8)
                    # Reliability check: TF-Luna distance shouldn't be 0 or 65535
                    if 0 < distance < 1200:
                        return distance
            else:
                # Byte 2 wasn't 0x59, so byte 1 wasn't a real header.
                # The loop continues to look for the next 0x59.
                continue
        else:
            # Not a header byte, keep looking
            continue
    return None

#Define different speeds (SpinFac) for different actions
Initial_sweep_speed = 0.8
forward_drive_speed = 1

#Define other tuning variables
BUFFER_SIZE = 5




#variable used to store distance:
distance_history = []

"""
Description: uses raw data, and calculates the average of the 5 points above and below it and adds it to a new list
Pre-Condition: window size, which indicates the number of points belwo and above an index to be averaged, and raw_list, which is the raw dataset
Post-condition: the smoothed dataset
"""
def smooth_data(raw_list, window_size=5):
    smoothed = []
    for i in range(len(raw_list)):
        # Calculate the average of the window around the current index
        start = max(0, i - window_size // 2)
        end = min(len(raw_list), i + window_size // 2 + 1)
        window = raw_list[start:end]
        smoothed.append(sum(window) / len(window))
    return smoothed

#
"""
Description: finds index of minimum of a smoothed dataset
Pre-Condition: a smoothed data set list
Post-condition: the index to the midpoint
"""
def find_minimum_distance(smoothed_list):
    # Find the index where the distance was minimum (bottom of the U)
    min_dist = min(smoothed_list)
    mid_index = smoothed_list.index(min_dist)

    return mid_index


"""
Description:
Used while driving
Calculates the slope (gradient) of the distance over the last few samples.
A negative value means distance is decreasing (moving toward can center).
A positive value means distance is increasing (drifting off the can).

Precondition: data_buffer, which is the number of most recent points to be used to calculate the derivative.

Postcondition: a float that indicates the linearized slope/trend
"""
def calculate_gradient(data_buffer):

    n = len(data_buffer)
    if n < 2:
        return 0

    # We use the indices as 'x' and distances as 'y'
    sum_x = sum(range(n))
    sum_y = sum(data_buffer)
    sum_xx = sum(i * i for i in range(n))
    sum_xy = sum(i * data_buffer[i] for i in range(n))

    # Standard linear regression slope formula:
    # slope = (n*sum_xy - sum_x*sum_y) / (n*sum_xx - sum_x**2)
    denominator = (n * sum_xx - sum_x**2)
    if denominator == 0:
        return 0

    slope = (n * sum_xy - sum_x * sum_y) / denominator
    return slope

"""
Description: updates the distance history based on buffer size
Pre-Condition: a new distance reading
Post-condition: no return variable, but removes the oldest value in the distance history based on buffer size
"""
def update_buffer(new_reading):
    distance_history.append(new_reading) # Add new data
    if len(distance_history) > BUFFER_SIZE:
        distance_history.pop(0) # Remove oldest data if we exceed the size















def initial_locate(timeout=10.0):
    print("Phase 1: Starting Gradient-Triggered Sweep...")
    raw_scan_data = []
    log_timestamps = []
    
    can_detected = False
    can_passed = False
    extra_scan_start = None
    
    # We'll use a small local window to check the exit gradient
    exit_buffer = []
    EXIT_WINDOW = 4 
    # A positive gradient of > 10cm/sample usually means we've hit the background
    EXIT_GRADIENT_THRESHOLD = 10.0 

    uart1.reset_input_buffer()
    start_time = time.monotonic()
    
    # Start spinning
    marc.set_multiple_motor_throttles([1.2*Initial_sweep_speed, Initial_sweep_speed, 0], runtime = 1)
    
    while (time.monotonic() - start_time) < timeout:
        d = scan()
        if d is not None:
            curr_time = time.monotonic() - start_time
            raw_scan_data.append(d)
            log_timestamps.append(curr_time)
            
            # Update local exit buffer
            exit_buffer.append(d)
            if len(exit_buffer) > EXIT_WINDOW:
                exit_buffer.pop(0)

            # 1. Detect entry (Simple drop in distance)
            if not can_detected and len(raw_scan_data) > 2:
                # If distance drops by more than 50cm suddenly, we found 'something'
                if (raw_scan_data[-2] - d) > 50: 
                    print(f"Target Entered View: {d}cm")
                    can_detected = True

            # 2. Detect exit (Gradient Analysis)
            if can_detected and not can_passed and len(exit_buffer) == EXIT_WINDOW:
                current_grad = calculate_gradient(exit_buffer)
                if current_grad > EXIT_GRADIENT_THRESHOLD:
                    print(f"Target Exit Detected (Grad: {current_grad:.1f}). Finishing scan...")
                    can_passed = True
                    extra_scan_start = time.monotonic()
        
        # 3. The 1-second "Padding"
        if can_passed and (time.monotonic() - extra_scan_start) > 1.0:
            break
            
        time.sleep(0.01)

    spin(0) # Emergency stop
    
    if not can_detected:
        print("Error: Target never entered field of view.")
        return False

    # --- Alignment Math ---
    smoothed = smooth_data(raw_scan_data)
    mid_index = find_minimum_distance(smoothed)
    
    actual_duration = log_timestamps[-1]
    total_samples = len(raw_scan_data)
    samples_to_reverse = total_samples - mid_index
    reverse_time = (samples_to_reverse / total_samples) * actual_duration
    
    print(f"Alignment: Midpoint at index {mid_index}. Reversing {reverse_time:.2f}s")
    
    marc.set_multiple_motor_throttles([-Initial_sweep_speed, -Initial_sweep_speed, 0], runtime = 1)
    time.sleep(reverse_time)
    spin(0)
    
    return True


# --- Fine-Tuned Constants ---
base_gradient = -0.8369  # Your calibrated value (cm/sample)
# MAX_PHYSICAL_DELTA: If distance increases by more than 4cm + your speed, it's a wall.
# We set this lower because at 20Hz, the rover moves very little between samples.
MAX_PHYSICAL_DELTA = 4.0 - base_gradient

BLIP_THRESHOLD = 30  # CM jump to ignore (sensor glitch)
MAX_BLIPS = 15       # At 20Hz, 15 samples is only 0.75 seconds of "lost" data.

# BUFFER_SIZE: Since you have 20Hz data, we can afford a larger buffer (10 samples)
# to get a very smooth gradient without much delay (0.5 seconds of history).
BUFFER_SIZE = 10

# WOBBLE_FREQ: Adjusted for 20Hz sampling to keep the "wiggle" visible.
WOBBLE_FREQ = 0.5

# CORRECTION_KP: How aggressively the rover turns back to center.
CORRECTION_KP = 0.3


def approach_optimized(stop_distance=22):
    global distance_history
    distance_history = [] 
    last_valid_dist = None
    blip_count = 0
    start_time = time.monotonic()
    last_steering_dir = 0 
    
    # TUNING: Time to rotate 90 degrees at search speed (e.g., 0.8)
    ninety_degree_time = 1.2 

    while True:
        d = scan() 
        if d is None: continue
        curr_time = time.monotonic() - start_time

        # --- 1. TARGET LOSS & RECOVERY ---
        if last_valid_dist is not None:
            if (d - last_valid_dist) > BLIP_THRESHOLD:
                blip_count += 1
                if blip_count > MAX_BLIPS:
                    print("!!! Target Lost: Starting Recovery Sweeps !!!")
                    # Try 90 deg one way, then 180 deg the other
                    success = recovery_sweep(last_steering_dir, ninety_degree_time)
                    
                    if success:
                        print("Re-acquired! Resuming approach...")
                        blip_count = 0
                        distance_history = [] 
                        continue # Back to the top of the while loop
                    else:
                        print("Total Loss: Returning to Phase 1 (Full Sweep)")
                        return "LOST" # This triggers the locate() loop to restart
                continue

        blip_count = 0
        last_valid_dist = d

        # --- 2. ARRIVAL & CONTROL (Same as before) ---
        if d <= stop_distance:
            marc.set_multiple_motor_throttles([0, 0, 0], runtime = 1)
            return "ARRIVED"

        current_speed = 0.7
        if d < 30:
            marc.set_multiple_motor_throttles([-current_speed, current_speed, 0], runtime = 1)
            continue

        update_buffer(d)
        if len(distance_history) >= BUFFER_SIZE:
            elapsed = time.monotonic() - start_time
            wiggle = math.sin(elapsed * WOBBLE_FREQ * 2 * math.pi) * (0.08 * (d / 100.0))
            net_slope = calculate_gradient(distance_history) - base_gradient
            bias = net_slope * CORRECTION_KP if abs(net_slope) > 0.05 else 0
            
            last_steering_dir = wiggle + bias 
            l_speed = current_speed - last_steering_dir
            r_speed = -(current_speed + last_steering_dir) 
            marc.set_multiple_motor_throttles([r_speed, l_speed, 0], runtime = 1)
        else:
            marc.set_multiple_motor_throttles([-current_speed, current_speed, 0], runtime = 1)

        time.sleep(0.05)

def recovery_sweep(last_dir, ninety_time):
    # Step 1: Rotate 90 degrees in the direction we were leaning
    search_speed = 0.8 if last_dir > 0 else -0.8
    print(f"Sweep 1: 90 degrees @ {search_speed}")
    if scan_for_target(search_speed, ninety_time):
        return True
        
    # Step 2: Rotate 180 degrees the OTHER way
    print(f"Sweep 2: 180 degrees @ {-search_speed}")
    # We use ninety_time * 2 to cover the full 180 degree arc
    if scan_for_target(-search_speed, ninety_time * 2):
        return True
        
    return False # If we reach here, both sweeps failed

def scan_for_target(speed, duration):
    """Spins and breaks immediately if the can is spotted."""
    start_search = time.monotonic()
    
    # APPLY THE 1.2x BOOST HERE FOR RECOVERY
    # If speed is positive, it's a CCW spin
    if speed > 0:
        r_motor = speed * 1.2
        l_motor = speed
    else:
        r_motor = speed
        l_motor = speed
        
    marc.set_multiple_motor_throttles([r_motor, l_motor, 0], runtime = 1) 
    
    while (time.monotonic() - start_search) < duration:
        d = scan()
        if d is not None and d < 100:
            print(f"Found something at {d}cm! Stopping.")
            marc.set_multiple_motor_throttles([0, 0, 0], runtime = 1)
            return True
        time.sleep(0.02)
    
    marc.set_multiple_motor_throttles([0, 0, 0], runtime = 1)
    return False

def dump_approach_log(log_data):
    print("\n--- APPROACH DATA LOG ---")
    print("Time (s) | Dist (cm) | Status/Bias")
    print("-" * 35)
    for entry in log_data:
        print(f"{entry[0]:.2f}     | {entry[1]:.1f}      | {entry[2]}")
    print("-" * 35)

def locate():
    while True:
        if not initial_locate(5.0):
            continue

        print("Closing the gap...")
        target_found = True
        last_d = None

        while True:
            d = scan()
            if d is None: continue

            # --- ROBUST GATING ---
            if last_d is not None:
                delta = d - last_d

                # Check 1: The "Impossible Leap"
                # If d increases by more than we could have moved, we hit the wall
                if delta > MAX_PHYSICAL_DELTA:
                    print(f"Target Lost (Jump: {delta:.1f}cm). Re-scanning...")
                    target_found = False
                    break

                # Check 2: The "Gradient Flip"
                # If we are driving forward but distance is INCREASING, we lost the 'dip'
                if delta > 2.0: # Small tolerance for sensor noise
                    print("Target Lost (Distance Increasing). Re-scanning...")
                    target_found = False
                    break

            last_d = d

            if d <= 60:
                forward(0)
                break

            forward(forward_drive_speed)
            time.sleep(0.1)

        if not target_found:
            continue

        # Proceed to Phase 2
        status = approach_optimized(stop_distance=15)

        if status == "ARRIVED":
            break
        elif status == "LOST":
            print("Approach failed. Restarting search...")


def calibrate_base_gradient(duration=3.0):
    print(f"Starting Calibration Sweep ({duration}s)...")
    readings = []
    timestamps = []

    # 1. Clear the UART buffer to get fresh data immediately
    uart1.reset_input_buffer()

    start_t = time.monotonic()
    forward(forward_drive_speed)

    while (time.monotonic() - start_t) < duration:
        d = scan()
        if d is not None:
            readings.append(d)
            timestamps.append(time.monotonic() - start_t) # Relative time
        time.sleep(0.05)

    forward(0) # Stop

    if not readings:
        print("No data collected. Check LiDAR wiring.")
        return None

    # 2. Calculate Stats
    hz = len(readings) / duration
    avg_grad = calculate_gradient(readings)

    # 3. Output Data List
    print("\n--- RAW DATA LOG ---")
    print("Index | Time (s) | Distance (cm)")
    print("-" * 30)
    for i in range(len(readings)):
        print(f"{i:03d}   | {timestamps[i]:.2f}     | {readings[i]}")

    print("-" * 30)
    print(f"Final Count: {len(readings)} samples")
    print(f"Actual HZ:   {hz:.2f} Hz")
    print(f"Base Grad:   {avg_grad:.4f} cm/sample")

    return readings

# To run it:
# my_data = calibrate_with_data_dump(3.0)

def driving_test():
    print("--- Rover Target Acquisition Test ---")
    print(f"Parameters: Base Speed: {forward_drive_speed}, Sweep Speed: {Initial_sweep_speed}")
    print(f"Base Gradient: {base_gradient}, Max Physical Delta: {MAX_PHYSICAL_DELTA}")

    # Wait for user to place the rover and clear the area
    print("System ready. Starting in 3 seconds... (Press Ctrl+C to abort)")
    time.sleep(3)

    try:
        # Start the State Machine
        locate()

        print("\n[SUCCESS] Rover reached the stop distance.")

    except KeyboardInterrupt:
        print("\n[ABORTED] User interrupted the test.")
    except Exception as e:
        print(f"\n[ERROR] System Failure: {e}")
    finally:
        # SAFETY FIRST: Always ensure motors are OFF when the script ends
        print("Safety Shutdown: Powering down motors.")
        marc.set_multiple_motor_throttles([0, 0, 0], runtime = 1)


