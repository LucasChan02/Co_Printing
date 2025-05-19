#!/usr/bin/env python3
#import os
import sys
import time
import RPi.GPIO as GPIO
from BusServoCmd import *
from smbus2 import SMBus, i2c_msg
from rpi_ws281x import PixelStrip
from rpi_ws281x import Color as PixelColor

#phaner technology raspberrypi expansion board sdk#
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

__ADC_BAT_ADDR = 0
__SERVO_ADDR   = 21
__MOTOR_ADDR   = 31
__SERVO_ADDR_CMD  = 40

__motor_speed = [0, 0, 0, 0]
__servo_angle = [0, 0, 0, 0, 0, 0]
__servo_pulse = [0, 0, 0, 0, 0, 0]
__i2c = 1
__i2c_addr = 0x7A

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

__RGB_COUNT = 2
__RGB_PIN = 12
__RGB_FREQ_HZ = 800000
__RGB_DMA = 10
__RGB_BRIGHTNESS = 120
__RGB_CHANNEL = 0
__RGB_INVERT = False
RGB = PixelStrip(__RGB_COUNT, __RGB_PIN, __RGB_FREQ_HZ, __RGB_DMA, __RGB_INVERT, __RGB_BRIGHTNESS, __RGB_CHANNEL)
RGB.begin()
for i in range(RGB.numPixels()):
    RGB.setPixelColor(i, PixelColor(0,0,0))
    RGB.show()

def setMotor(index, speed):
    if index < 1 or index > 4:
        raise AttributeError("Invalid motor num: %d"%index)
    index = index - 1
    speed = 100 if speed > 100 else speed
    speed = -100 if speed < -100 else speed
    speed = -speed
    reg = __MOTOR_ADDR + index
    with SMBus(__i2c) as bus:
        msg = i2c_msg.write(__i2c_addr, [reg, speed.to_bytes(1, 'little', signed=True)[0]])
        bus.i2c_rdwr(msg)
        __motor_speed[index] = speed
    return __motor_speed[index]
    
def getMotor(index):
    if index < 1 or index > 4:
        raise AttributeError("Invalid motor num: %d"%index)
    index = index - 1
    return __motor_speed[index]

def setPWMServoAngle(index, angle):
    if servo_id < 1 or servo_id > 6:
        raise AttributeError("Invalid Servo ID: %d"%servo_id)
    index = servo_id - 1

    angle = 180 if angle > 180 else angle
    angle = 0 if angle < 0 else angle

    reg = __SERVO_ADDR + index

    with SMBus(__i2c) as bus:
        msg = i2c_msg.write(__i2c_addr, [reg, angle])
        bus.i2c_rdwr(msg)
        __servo_angle[index] = angle
        __servo_pulse[index] = int(((200 * angle) / 9) + 500)

    return __servo_angle[index]

def setPWMServoPulse(servo_id, pulse = 1500, use_time = 1000):
    if servo_id< 1 or servo_id > 6:
        raise AttributeError("Invalid Servo ID: %d" %servo_id)
    index = servo_id - 1

    pulse = 500 if pulse < 500 else pulse
    pulse = 2500 if pulse > 2500 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    buf = [__SERVO_ADDR_CMD, 1] + list(use_time.to_bytes(2, 'little')) + [servo_id,] + list(pulse.to_bytes(2, 'little'))

    with SMBus(__i2c) as bus:
        msg = i2c_msg.write(__i2c_addr, buf)
        bus.i2c_rdwr(msg)
        __servo_pulse[index] = pulse
        __servo_angle[index] = int((pulse - 500) * 0.09)

    return __servo_pulse[index]

def getPWMServoAngle(servo_id):
    if servo_id < 1 or servo_id > 6:
        raise AttributeError("Invalid Servo ID: %d"%servo_id)
    index = servo_id - 1
    return __servo_pulse[index]

def getPWMServoPulse(index):
    if servo_id < 1 or servo_id > 6:
        raise AttributeError("Invalid Servo ID: %d"%servo_id)
    index = servo_id - 1
    return __servo_pulse[index]
    
def getBattery():
    ret = 0
    with SMBus(__i2c) as bus:
        msg = i2c_msg.write(__i2c_addr, [__ADC_BAT_ADDR,])
        bus.i2c_rdwr(msg)
        read = i2c_msg.read(__i2c_addr, 2)
        bus.i2c_rdwr(read)
        ret = int.from_bytes(bytes(list(read)), 'little')
    return ret

# Buzzer Driver
def setBuzzer(st):
    GPIO.setup(6, GPIO.OUT)
    GPIO.output(6, st)

def setBusServoID(oldid, newid):
    """
    Configure the servo ID number, the factory default is 1
    :param oldid: original id, factory default is 1
    :param newid: new id

    """
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_ID_WRITE, newid)

def getBusServoID(id=None):
    """
    Read serial port servo id
    :param id: empty by default
    :return: Return the servo id
    """
    
    while True:
        if id is None:  # There can only be one servo on the bus
            serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ)
        else:
            serial_servo_read_cmd(id, LOBOT_SERVO_ID_READ)
        # Get Content
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ID_READ)
        if msg is not None:
            return msg

def setBusServoPulse(id, pulse, use_time):
    """
    Drive the serial port servo to the specified position
    :param id: the id of the servo to be driven
    :pulse: location
    :use_time: The time required for rotation
    """

    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, use_time)

def stopBusServo(id=None):
    '''
    Stop the servo operation
    :param id:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_STOP)

def setBusServoDeviation(id, d=0):
    """
    Adjustment deviation
    :param id: servo id
    :param d: deviation
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_ADJUST, d)

def saveBusServoDeviation(id):
    """
    Configuration deviation, power-off protection
    :param id: servo id
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_WRITE)

time_out = 50
def getBusServoDeviation(id):
    '''
    Read deviation value
    :param id: servo number
    :return:
    '''
    # Send read deviation command
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_READ)
        # get
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_OFFSET_READ)
        count += 1
        if msg is not None:
            return msg
        if count > time_out:
            return None

def setBusServoAngleLimit(id, low, high):
    '''
    
    Set the servo rotation range
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_WRITE, low, high)

def getBusServoAngleLimit(id):
    '''
    Read the servo rotation range
    :param id:
    :return: Return to origin 0: low bit 1: high bit
    '''
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_LIMIT_READ)
        if msg is not None:
            count = 0
            return msg

def setBusServoVinLimit(id, low, high):
    '''
    Set the servo voltage range
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_VIN_LIMIT_WRITE, low, high)

def getBusServoVinLimit(id):
    '''
    Read the servo rotation range
    :param id:
    :return: Return to origin 0: low bit 1: high bit
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_LIMIT_READ)
        if msg is not None:
            return msg

def setBusServoMaxTemp(id, m_temp):
    '''
    Set the maximum temperature alarm of the servo
    :param id:
    :param m_temp:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, m_temp)

def getBusServoTempLimit(id):
    '''
    Read the servo temperature alarm range
    :param id:
    :return:
    '''
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        if msg is not None:
            return msg

def getBusServoPulse(id):
    '''
    Read the current position of the servo
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_POS_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_POS_READ)
        if msg is not None:
            return msg

def getBusServoTemp(id):
    '''
    Read the servo temperature
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_READ)
        if msg is not None:
            return msg

def getBusServoVin(id):
    '''
    Read servo voltage
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_READ)
        if msg is not None:
            return msg

def restBusServoPulse(oldid):
    # Servo clear deviation and P value median（500）
    serial_servo_set_deviation(oldid, 0)    # Clear deviation
    time.sleep(0.1)
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_MOVE_TIME_WRITE, 500, 100)    #median

##Power outage
def unloadBusServo(id):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0)

##Read whether the power is off
def getBusServoLoadStatus(id):
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        if msg is not None:
            return msg

