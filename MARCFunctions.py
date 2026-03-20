# Built-in Python libraries, not visible in the CIRCUITPYTHON drive
import time
import busio
import board
import digitalio
import supervisor
import traceback


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
    #og was +-0.5
    x = spinFac
    marc.set_multiple_motor_throttles(
        motor_throttles=[x, x, 0],#Right, Left, Extra
        #runtime = 1
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
Initial_sweep_speed = 0.3
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















def initial_locate(sweep_duration=5.0):
    print("Phase 1: Starting initial sweep...")
    raw_scan_data = []
    
    # 1. Sweep and Record
    start_time = time.monotonic()
    spin(Initial_sweep_speed)
    
    while (time.monotonic() - start_time) < sweep_duration:
        d = scan()
        if d is not None:
            raw_scan_data.append(d)
        time.sleep(0.01) # Small delay to prevent CPU hogging
    
    spin(0) # Stop spinning
    
    if not raw_scan_data:
        print("Error: No LiDAR data collected.")
        return False

    # 2. Process Data
    smoothed = smooth_data(raw_scan_data)
    mid_index = find_minimum_distance(smoothed)
    
    # 3. Calculate Reverse Spin
    # The percentage of the sweep we need to go back
    total_samples = len(raw_scan_data)
    samples_to_reverse = total_samples - mid_index
    reverse_time = (samples_to_reverse / total_samples) * sweep_duration
    
    print(f"Can found at index {mid_index}. Reversing for {reverse_time:.2f}s")
    
    # 4. Align to Center
    spin(-Initial_sweep_speed) # Spin in opposite direction
    time.sleep(reverse_time)
    spin(0)
    
    print("Phase 1 Complete: Aligned with can.")
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


def approach_optimized(stop_distance=15):
    global distance_history
    distance_history = [] # Reset buffer
    last_valid_dist = None
    blip_count = 0
    start_time = time.monotonic()

    while True:
        d = scan() # TF-Luna @ 3Hz
        if d is None: continue

        # --- 1. GATING LOGIC (Blip Protection) ---
        if last_valid_dist is not None:
            # If distance jumps up by more than BLIP_THRESHOLD, it's the wall
            if (d - last_valid_dist) > BLIP_THRESHOLD:
                blip_count += 1
                print(f"Blip detected! Count: {blip_count}")
                
                if blip_count > MAX_BLIPS:
                    print("Target Lost: Stopping for safety.")
                    marc.set_multiple_motor_throttles([0, 0, 0])
                    return "LOST"  # Return a status code instead of breaking
                
                # IMPORTANT: We do NOT update the buffer with the blip.
                # We skip the rest of this loop iteration to "hold" the last state.
                time.sleep(0.3)
                continue 
        
        # If we reach here, it's a valid reading
        blip_count = 0 
        last_valid_dist = d

        # Stop condition
        if d <= stop_distance: 
            marc.set_multiple_motor_throttles([0, 0, 0])
            print("Arrived at target!")
            return "ARRIVED"  # Return a status code for success
            
        update_buffer(d)
        
        # --- 2. CONTROL LOGIC ---
        if len(distance_history) >= BUFFER_SIZE:
            elapsed = time.monotonic() - start_time
            
            # Sinusoidal Wiggle
            # As d (distance) gets smaller, the wiggle amplitude gets smaller
            dynamic_amplitude = 0.12 * (d / 60.0) # Full wiggle at 60cm, nearly 0 at 0cm
            wiggle = math.sin(elapsed * WOBBLE_FREQ * 2 * math.pi) * dynamic_amplitude
            
            # Gradient Analysis
            current_slope = calculate_gradient(distance_history)
            net_slope = current_slope - base_gradient
            
            # Proportional Correction
            bias = 0
            if net_slope > 0.05: 
                bias = net_slope * CORRECTION_KP
            
            # Motor Output
            # Note: If bias is positive, it nudges the rover. 
            # You may need to flip the +/- on bias depending on which motor needs to slow down.
            marc.set_multiple_motor_throttles([
                forward_drive_speed + wiggle + bias,
                forward_drive_speed - wiggle - bias,
                0
            ])
        else:
            # Not enough data yet, drive straight
            marc.set_multiple_motor_throttles([forward_drive_speed, forward_drive_speed, 0])
        
        time.sleep(0.3)


def locate():
    while True:
        if not initial_locate(sweep_duration=5.0):
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
        marc.set_multiple_motor_throttles([0, 0, 0])


