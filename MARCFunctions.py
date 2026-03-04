# Built-in Python libraries, not visible in the CIRCUITPYTHON drive
import time
import board
import digitalio
import supervisor
import traceback


# Python modules from the "lib" folder
from lib.marc import MARC

def seconds_since_boot():
    return f"{time.monotonic():.3f}"


# Initializing inputs and outputs
led_pin = digitalio.DigitalInOut(board.LED)
led_pin.direction = digitalio.Direction.OUTPUT
led_pin.value = True # Turning on on-board LED
time.sleep(1)
led_pin.value = False # Turning on on-board LED
time.sleep(1)
led_pin.value = True # Turning on on-board LED
start_button = digitalio.DigitalInOut(board.GP14)
start_button.direction = digitalio.Direction.INPUT
start_button.pull = digitalio.Pull.UP

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

#movement function definitions
def runDump(state):
    if state == "up":
        marc.move_single_servo(
            servo_name="0",
            angle=0
        )
        marc.wait(2)


    if state == "down":
        marc.move_single_servo(
            servo_name="0",
            angle=90
        )
        marc.wait(2)


def armOpen(state):
    if state == "open":
        marc.move_single_servo(
            servo_name='1',
            angle=0
        )
        marc.wait(2)


    if state == "closed":
        marc.move_single_servo(
            servo_name="1",
            angle=0
        )
        marc.wait(2)

#LiDAR DEF
#make sure uart1 = busio.UART(board.GP4, board.GP5, baudrate=115200)
def scan():
    if uart1.in_waiting >= 9:   # Check if full packet available
        data = uart1.read(9)    # Read 9-byte TF-Luna packet
        
        if data is not None and len(data) == 9:
            # Check header bytes (0x59 0x59)
            if data[0] == 0x59 and data[1] == 0x59:
                # Distance (cm)
                distance = data[2] + (data[3] << 8)
                return distance
    return None

