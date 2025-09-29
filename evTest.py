#!/home/plaaspadda/skripsie/bin/python3
import evdev
from evdev import InputDevice, categorize, ecodes
from select import select

device = evdev.InputDevice('/dev/input/event21')
print("Using device:", device)

# Controller state
LJoyX = 0
RTrig = 0
X_BTN = 0
Y_BTN = 0
B_BTN = 0

while True:
    # Wait until there is input available (non-blocking)
    r, _, _ = select([device], [], [], 0.01)
    if device in r:
        for event in device.read():
            if event.type == ecodes.EV_ABS:
                if event.code == ecodes.ABS_X:
                    LJoyX = event.value
                elif event.code == ecodes.ABS_RZ:
                    RTrig = event.value
            elif event.type == ecodes.EV_KEY:
                if event.code == ecodes.BTN_NORTH:
                    X_BTN = event.value
                elif event.code == ecodes.BTN_WEST:
                    Y_BTN = event.value
                elif event.code == ecodes.BTN_EAST:
                    B_BTN = event.value

    # Print current state
    print(f"LJoyX: {LJoyX}, RTrig: {RTrig}, X: {X_BTN}, Y: {Y_BTN}, B: {B_BTN}")

