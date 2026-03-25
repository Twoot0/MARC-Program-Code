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
CORRECTION_KP = 0.2

# TUNING: Time to rotate 90 degrees at  search speed (e.g., 0.8)
ninety_degree_time = 3


def initial_locate(timeout=ninety_degree_time):
    print("Phase 1: Starting Multi-Directional Search with Can Validation...")
    
    # 1. Try Clockwise Sweep (90 degrees)
    found_cw, cw_data = scan_and_map(speed=-Initial_sweep_speed, timeout=timeout*1.5, low=20, high=150)
    if found_cw:
        # VALIDATION: Check if the object's physical width matches a can
        if is_it_a_can(cw_data):
            process_sweep_alignment(cw_data, Initial_sweep_speed)
            return True
        else:
            print("CW Object rejected: Width does not match a can. Continuing search...")

    print("Target not found CW (or rejected). Trying CCW...")

    # 2. Try Counter-Clockwise Sweep (180 degrees+)
    # We use a larger timeout here to ensure a full 360+ coverage if needed
    found_ccw, ccw_data = scan_and_map(speed=Initial_sweep_speed, timeout=timeout * 5, low=20, high=150)
    if found_ccw:
        # VALIDATION: Check if the object's physical width matches a can
        if is_it_a_can(ccw_data):
            process_sweep_alignment(ccw_data, -Initial_sweep_speed)
            return True
        else:
            print("CCW Object rejected: Width does not match a can.")

    print("Search failed: No can-sized objects detected.")
    return False
    
def process_sweep_alignment(data, reverse_speed):
    """Calculates the midpoint of a sweep and reverses to center on it."""
    raw_scan_data = data["distances"]
    log_timestamps = data["timestamps"]
    
    if not raw_scan_data:
        return

    # Use your existing smoothing and min-distance logic
    smoothed = smooth_data(raw_scan_data)
    mid_index = find_minimum_distance(smoothed)

    actual_duration = log_timestamps[-1]
    total_samples = len(raw_scan_data)
    samples_to_reverse = total_samples - mid_index
    reverse_time = (samples_to_reverse / total_samples) * actual_duration

    print(f"Alignment: Midpoint at index {mid_index}. Reversing {reverse_time:.2f}s")

    # Reverse at the specified speed (sign depends on which way we were just spinning)
    # If we were spinning CW (negative speed), reverse_speed should be positive (CCW)
    r_val = reverse_speed * 1.2 if reverse_speed > 0 else reverse_speed
    l_val = reverse_speed
    
    marc.set_multiple_motor_throttles([r_val, l_val, 0], runtime=1)
    time.sleep(reverse_time)
    spin(0)

# --- Safety Helper ---
def set_safe_throttles(r_raw, l_raw, runtime=None):
    """Caps motor values between -1.0 and 1.0 to prevent system failure."""
    r_safe = max(-1.0, min(1.0, r_raw))
    l_safe = max(-1.0, min(1.0, l_raw))
    # We use a tiny runtime or None to ensure software control
    marc.set_multiple_motor_throttles([r_safe, l_safe, 0], runtime=runtime)
    
def is_it_a_can(data):
    dists = data["distances"]
    # 1. How many samples were "on target" (e.g., within 10cm of the minimum)?
    min_d = min(dists)
    on_target_samples = [d for d in dists if abs(d - min_d) < 10]
    
    # 2. Estimate physical width in cm
    # (Assuming a 90-degree sweep took 'len(dists)' samples)
    angular_width_per_sample = 90 / len(dists)
    total_angle_radians = math.radians(len(on_target_samples) * angular_width_per_sample)
    
    # Chord length formula: Width = 2 * R * sin(theta/2)
    physical_width = 2 * min_d * math.sin(total_angle_radians / 2)
    
    print(f"Object Analysis: Distance={min_d:.1f}cm, Calc Width={physical_width:.1f}cm")
    
    # A standard soda can is ~6.6cm wide. 
    # Let's allow a range of 4cm to 12cm to account for noise.
    if 4 < physical_width < 12:
        return True
    return False

def approach_optimized(stop_distance=22):
    global distance_history
    distance_history = []
    last_valid_dist = None
    blip_count = 0
    start_time = time.monotonic()
    last_steering_dir = 0
    grace_period_until = 0

    while True:
        d = scan()
        if d is None: continue
        
        curr_time = time.monotonic()
        
        # --- 1. TARGET LOSS & RECOVERY ---
        if last_valid_dist is not None and curr_time > grace_period_until:
            if (d - last_valid_dist) > BLIP_THRESHOLD:
                blip_count += 1
                if blip_count > MAX_BLIPS:
                    print(f"!!! Target Lost. Last seen at {last_valid_dist:.1f}cm. Recovering...")
                    set_safe_throttles(0, 0)
                    time.sleep(0.5)
                    recovery_sweep(last_steering_dir, ninety_degree_time, last_valid_dist)
                    print("Recovery complete. Resuming approach...")
                    blip_count = 0
                    distance_history = []
                    grace_period_until = time.monotonic() + 1.5
                    continue 
                continue

        blip_count = 0
        last_valid_dist = d 
        
        # --- 2. ARRIVAL & CONTROL ---
        if d <= stop_distance:
            set_safe_throttles(0, 0)
            return "ARRIVED"

        current_speed = 0.7
        
        # Logic for very close range (no gradient math needed)
        if d < 30:
            set_safe_throttles(-current_speed, current_speed)
            continue

        update_buffer(d)
        if len(distance_history) >= BUFFER_SIZE:
            elapsed = time.monotonic() - start_time
            
            # --- MODIFIED WIGGLE LOGIC ---
            # If distance > 40cm, calculate wiggle. Else, set to 0.
            if d > 40:
                wiggle = math.sin(elapsed * WOBBLE_FREQ * 2 * math.pi) * (0.08 * (d / 100.0))
            else:
                wiggle = 0
            
            net_slope = calculate_gradient(distance_history) - base_gradient
            bias = net_slope * CORRECTION_KP if abs(net_slope) > 0.05 else 0

            last_steering_dir = wiggle + bias
            l_speed = current_speed - last_steering_dir
            r_speed = -(current_speed + last_steering_dir)
            
            set_safe_throttles(r_speed, l_speed)
        else:
            set_safe_throttles(-current_speed, current_speed)

        time.sleep(0.05)

def recovery_sweep(last_dir, ninety_time, expected_dist):
    """Internal recovery loop. Guaranteed to make a decision."""
    low, high = expected_dist - 30, expected_dist + 30

    while True: # Keep trying until we have data
        # 1. CW Pass
        found_cw, cw_data = scan_and_map(speed=-0.8, timeout=ninety_time, low=low, high=high)
        if found_cw:
            perform_centering_best_match(cw_data, 0.8, expected_dist)
            return

        # 2. CCW Pass (180 deg)
        found_ccw, ccw_data = scan_and_map(speed=0.8, timeout=ninety_time * 1.5, low=low, high=high)
        
        if ccw_data["distances"]:
            # Even if 'found_ccw' is false (not in window), we pick the best available point
            perform_centering_best_match(ccw_data, -0.8, expected_dist)
            return
        else:
            print("Total darkness. Blind nudge CCW and retrying sweep...")
            set_safe_throttles(-0.8, 0.8)
            time.sleep(0.6)
            set_safe_throttles(0, 0)

def scan_and_map(speed, timeout, low, high):
    raw_scan_data = []
    log_timestamps = []
    
    # Gradient tracking for exit detection
    exit_buffer = []
    EXIT_WINDOW = 4
    EXIT_GRADIENT_THRESHOLD = 10.0
    
    can_detected = False
    can_passed = False
    extra_scan_start = None
    
    set_safe_throttles(0, 0)
    time.sleep(0.5) 
    
    start_time = time.monotonic()
    r_val = speed * 1.2 if speed > 0 else speed
    l_val = speed
    
    set_safe_throttles(r_val, l_val)

    while (time.monotonic() - start_time) < timeout:
        d = scan()
        if d is not None and 15 < d < 180:
            curr_relative_time = time.monotonic() - start_time
            raw_scan_data.append(d)
            log_timestamps.append(curr_relative_time)
            
            # --- Exit Detection Logic ---
            exit_buffer.append(d)
            if len(exit_buffer) > EXIT_WINDOW:
                exit_buffer.pop(0)

            # 1. Detect if we are currently looking at the target
            if not can_detected and low < d < high:
                can_detected = True

            # 2. Detect if we have passed the target (Gradient spikes)
            if can_detected and not can_passed and len(exit_buffer) == EXIT_WINDOW:
                current_grad = calculate_gradient(exit_buffer)
                if current_grad > EXIT_GRADIENT_THRESHOLD:
                    print(f"Sweep Exit Detected (Grad: {current_grad:.1f}). Adding padding...")
                    can_passed = True
                    extra_scan_start = time.monotonic()

        # 3. The 0.5-second "Padding" stop
        if can_passed and (time.monotonic() - extra_scan_start) > 0.5:
            print("Padding complete. Stopping sweep.")
            break

        time.sleep(0.01)

    set_safe_throttles(0, 0)
    found_in_window = any(low < d < high for d in raw_scan_data)
    return found_in_window, {"distances": raw_scan_data, "timestamps": log_timestamps}

def perform_centering_best_match(data, reverse_speed, expected_dist):
    dists = data["distances"]
    times = data["timestamps"]
    
    # Mathematical 'Best Match' calculation
    best_idx = min(range(len(dists)), key=lambda i: abs(dists[i] - expected_dist))
    
    time_at_target = times[best_idx]
    total_duration = times[-1]
    reverse_time = total_duration - time_at_target

    print(f"--> Target estimate: {expected_dist}cm. Best match found: {dists[best_idx]}cm.")
    
    r_val = reverse_speed * 1.2 if reverse_speed > 0 else reverse_speed
    l_val = reverse_speed
    
    # Reverse to target
    set_safe_throttles(r_val, l_val)
    time.sleep(reverse_time)
    set_safe_throttles(0, 0)
    
def dump_approach_log(log_data):
    print("\n--- APPROACH DATA LOG ---")
    print("Time (s) | Dist (cm) | Status/Bias")
    print("-" * 35)
    for entry in log_data:
        print(f"{entry[0]:.2f}     | {entry[1]:.1f}      | {entry[2]}")
    print("-" * 35)

def locate():
    while True:
        # Phase 1: Spinning to find the general direction
        if not initial_locate(5.0):
            print("Searching for target...")
            continue

        print("Target identified. Handing over to Optimized Approach...")
        
        # We skip the 'Gap Closer' loop because approach_optimized 
        # is now smart enough to handle long distances (up to 150cm)
        # and it won't crash back to Phase 1 if it hits a 'delta' jump.
        
        status = approach_optimized(stop_distance=15)

        if status == "ARRIVED":
            print("MISSION SUCCESS: Target Reached.")
            break
        
        # Note: In the new version of approach_optimized, 
        # it will ALMOST NEVER return "LOST". It will just 
        # loop its own recovery until it finds the can.
        if status == "LOST":
            print("Critical Loss. Restarting full search...")
            continue

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

def rotation_test(button_obj, speed=0.8): # Add button_obj here
    """
    Rotates the rover at a set speed. 
    Stops immediately when the button on GP14 is pressed.
    """
    print("--- ROTATION TEST STARTING ---")
    print("Spinning... Press the GP14 button to stop.")
    
    spin(speed) 
    
    # Use the passed-in object
    while button_obj.value == True:
        time.sleep(0.01)
    
    spin(0)
    print("--- ROTATION TEST STOPPED BY USER ---")
