#!/home/plaaspadda/skripsie/bin/python3
import threading
import math
from inputs import get_gamepad

class Controller(object):
    Max_Trig_Val = math.pow(2, 8)
    Max_Joy_Val = math.pow(2, 15)

    def __init__(self):
        self.LJoyX = 0
        self.X_BTN = 0
        self.Y_BTN = 0
        self.B_BTN = 0
        self.RTrig = 0

    def GiveValue(self):
        leftX = self.LJoyX
        btn_X = self.X_BTN
        btn_Y = self.Y_BTN
        btn_B = self.B_BTN
        rightTrig = self.RTrig
        return leftX, btn_X, btn_Y, btn_B, rightTrig 

    def _monitor_Controller(self):
        events = get_gamepad()
        for event in events:
            if (event.code == 'ABS_X'):
                self.LJoyX = event.state / Controller.Max_Joy_Val
            elif (event.code == 'BTN_WEST'):
                self.Y_BTN = event.state
            elif (event.code == 'BTN_NORTH'):
                self.X_BTN = event.state
            elif (event.code == 'BTN_EAST'):
                self.B_BTN = event.state
            elif (event.code == 'ABS_RZ'):
                self.RTrig = event.state / Controller.Max_Trig_Val
            


if __name__=='__main__':
    controller = Controller()
    
    steer = "Middle"
    algo = "Off"
    gas = "Stop"

    while True:
        controller._monitor_Controller()
        LX, BX, BY, BB, RT = controller.GiveValue()
        print(f"LX: {LX:.2f}  X: {BX:.2f} B: {BB:.2f} RT: {RT:.2f}")
        if (RT > 0):
           gas = "Forward" 
        else:
            gas = "Stop"

        if (LX > 0):
            steer = "Right"
        elif (LX < 0):
            steer = "Left"
        elif (LX == 0):
            steer = "Left"

        if (BX>0):
            algo = "On"

        if (BB>0):
            algo = "Off"
        
        if (BY>0):
            break

        print(f"Steer: {steer} Algorithm: {algo} Rotation: {gas}")
