import sys
import rospy
import signal
import time
import math
import Board as Board

# Success buzzer has 1 long "beep"
def success_buzz():
    Board.setBuzzer(1)
    time.sleep(0.75)
    Board.setBuzzer(0)
    
# Error buzzer has 3 short "beeps"
def error_buzz():
    Board.setBuzzer(1)
    time.sleep(0.25)
    Board.setBuzzer(0)
    time.sleep(0.25)
    Board.setBuzzer(1)
    time.sleep(0.25)
    Board.setBuzzer(0)
    time.sleep(0.25)
    Board.setBuzzer(1)
    time.sleep(0.25)
    Board.setBuzzer(0)

# Starts both fans for the extruder assembly
def fan_start_DC():
    Board.setMotor(2,100)
    Board.setMotor(3,100)

# Stops both fans for the extruder assembly
def fan_stop_DC():
    Board.setMotor(2,0)
    Board.setMotor(3,0)

# ======================================================================================
# Alternatively, we can use PWM headers 6, 7; voltage is based on battery/external power
# Will use DC for simplicity for now, if 5V is not enough (which will probably the the
# case), use PWM code instead:4

def fan_start_PWM():
    while fan_start == True: 
        Board.setPWMServoPulse(6,1000,100)
        Board.setPWMServoPulse(7,1000,100)
        
def fan_stop_PWM():
    global fan_start
    fan_start = False

# ======================================================================================

