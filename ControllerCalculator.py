import math
import threading
from pyPS4Controller.controller import Controller

# Global variables that will be updated using the controller thread
leftJoystick_percentage = 0
rightJostick_percentage = 0
throttle_percentage = 0
gaitPatternID = 0

# this function will be called in the main loop, it will give the turnAngle and stepDistance based on the current controller inputs
def calcBodyMovement():
    """
    calcBodyMovement calculates the turn angle and step distance based on controller inputs

    :param throttle_percentage: The throttle input as a percentage (0.0 to 100.0)
    :param leftJoystick_percentage: The left joystick input as a percentage (-100.0 to 100.0)
    :param rightJostick_percentage: The right joystick input as a percentage (-100.0 to 100.0)

    :return: returns turnAngle in degrees and stepDistance in mm
    """
    maxTurnAngle = 35.0
    maxStepDistance = 160.0

    turnAngle = (rightJostick_percentage - leftJoystick_percentage) * maxTurnAngle/200
    stepDistance = throttle_percentage * maxStepDistance

    return turnAngle, stepDistance

class MyController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_x_press(self):
        global gaitPatternID
        gaitPatternID = 1

    def on_x_release(self):
        pass

    def on_triangle_press(self):
        pass

    def on_triangle_release(self):
        pass

    def on_circle_press(self):
        global gaitPatternID
        gaitPatternID = 2

    def on_circle_release(self):
        pass

    def on_square_press(self):
        global gaitPatternID
        gaitPatternID = 0

    def on_square_release(self):
        pass

    def on_L1_press(self):
        pass

    def on_L1_release(self):
        pass

    def on_L2_press(self, value):
        pass

    def on_L2_release(self):
        pass

    def on_R1_press(self):
        pass

    def on_R1_release(self):
        pass

    def on_R2_release(self):
        pass

    def on_up_arrow_press(self):
        pass

    def on_up_down_arrow_release(self):
        pass

    def on_down_arrow_press(self):
        pass

    def on_left_arrow_press(self):
        pass

    def on_left_right_arrow_release(self):
        pass

    def on_right_arrow_press(self):
        pass

    def on_L3_left(self, value):
        pass

    def on_L3_right(self, value):
        pass

    def on_L3_y_at_rest(self):
        pass

    def on_L3_x_at_rest(self):
        pass

    def on_L3_press(self):
        pass

    def on_L3_release(self):
        pass

    def on_R3_left(self, value):
        pass

    def on_R3_right(self, value):
        pass

    def on_R3_y_at_rest(self):
        pass

    def on_R3_x_at_rest(self):
        pass

    def on_R3_press(self):
        pass

    def on_R3_release(self):
        pass

    def on_options_press(self):
        pass

    def on_options_release(self):
        pass

    def on_share_press(self):
        pass

    def on_share_release(self):
        pass

    def on_playstation_button_press(self):
        pass

    def on_playstation_button_release(self):
        pass

    def on_L3_down(self, value):
       number = -(value/32767)*100
       global leftJoystick_percentage
       leftJoystick_percentage = round(number,2)
       print("L3_down: {} ".format(leftJoystick_percentage))
       
    def on_L3_up(self, value):
       number = -(value/32767)*100
       global leftJoystick_percentage
       leftJoystick_percentage = round(number,2)
       print("L3_up: {} ".format(leftJoystick_percentage))

    def on_R3_down(self, value):
       number = -(value/32767)*100
       global rightJoystick_percentage
       rightJoystick_percentage = round(number,2)
       print("R3_down: {} ".format(rightJoystick_percentage))
       
    def on_R3_up(self, value):
       number = -(value/32767)*100
       global rightJoystick_percentage
       rightJoystick_percentage = round(number,2)
       print("R3_up: {} ".format(rightJoystick_percentage))        
            
    def on_R2_press(self, value):
       number = (((value/32767)*100)+100)/2
       global throttle_percentage
       throttle_percentage = round(number,2)
       print("R2: {} ".format(throttle_percentage)) 

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)

def start_controller():
    controller.listen(timeout=60)

# Starts a separate thread to listen for controller inputs, this allows the main loop to run at the same time
threading.Thread(target=start_controller, daemon=True).start()

