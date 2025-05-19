#!/usr/bin/env python3
# encoding: utf-8
import time
import ctypes
import serial
import RPi.GPIO as GPIO
#Huaner Technology Bus Servo Communication#

LOBOT_SERVO_FRAME_HEADER         = 0x55
LOBOT_SERVO_MOVE_TIME_WRITE      = 1
LOBOT_SERVO_MOVE_TIME_READ       = 2
LOBOT_SERVO_MOVE_TIME_WAIT_WRITE = 7
LOBOT_SERVO_MOVE_TIME_WAIT_READ  = 8
LOBOT_SERVO_MOVE_START           = 11
LOBOT_SERVO_MOVE_STOP            = 12
LOBOT_SERVO_ID_WRITE             = 13
LOBOT_SERVO_ID_READ              = 14
LOBOT_SERVO_ANGLE_OFFSET_ADJUST  = 17
LOBOT_SERVO_ANGLE_OFFSET_WRITE   = 18
LOBOT_SERVO_ANGLE_OFFSET_READ    = 19
LOBOT_SERVO_ANGLE_LIMIT_WRITE    = 20
LOBOT_SERVO_ANGLE_LIMIT_READ     = 21
LOBOT_SERVO_VIN_LIMIT_WRITE      = 22
LOBOT_SERVO_VIN_LIMIT_READ       = 23
LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE = 24
LOBOT_SERVO_TEMP_MAX_LIMIT_READ  = 25
LOBOT_SERVO_TEMP_READ            = 26
LOBOT_SERVO_VIN_READ             = 27
LOBOT_SERVO_POS_READ             = 28
LOBOT_SERVO_OR_MOTOR_MODE_WRITE  = 29
LOBOT_SERVO_OR_MOTOR_MODE_READ   = 30
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31
LOBOT_SERVO_LOAD_OR_UNLOAD_READ  = 32
LOBOT_SERVO_LED_CTRL_WRITE       = 33
LOBOT_SERVO_LED_CTRL_READ        = 34
LOBOT_SERVO_LED_ERROR_WRITE      = 35
LOBOT_SERVO_LED_ERROR_READ       = 36

serialHandle = serial.Serial("/dev/ttyS0", 115200)  # Initialize the serial port, the baud rate is 115200

rx_pin = 4
tx_pin = 27

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

def portInit():  # Configure the IO port used
    GPIO.setup(rx_pin, GPIO.OUT)  # Configure RX_CON that is GPIO17 as output
    GPIO.output(rx_pin, 0)
    GPIO.setup(tx_pin, GPIO.OUT)  # Configure TX_CON that is GPIO27 as output
    GPIO.output(tx_pin, 1)

portInit()

def portWrite():  # Configure the single-wire serial port as output
    GPIO.output(tx_pin, 1)  # Pull TX_CON high which is GPIO27
    GPIO.output(rx_pin, 0)  # Pull RX_CON low which is GPIO17

def portRead():  # Configure the single-wire serial port as input
    GPIO.output(rx_pin, 1)  # Pull RX_CON high which is GPIO17
    GPIO.output(tx_pin, 0)  # Pull TX_CON low which is GPIO27

def portRest():
    time.sleep(0.1)
    serialHandle.close()
    GPIO.output(rx_pin, 1)
    GPIO.output(tx_pin, 1)
    serialHandle.open()
    time.sleep(0.1)

def checksum(buf):
    # Calculate checksum
    sum = 0x00
    for b in buf:  # Sum
        sum += b
    sum = sum - 0x55 - 0x55  # Remove the two 0x55 at the beginning of the command
    sum = ~sum  # Rebellion
    return sum & 0xff

def serial_serro_wirte_cmd(id=None, w_cmd=None, dat1=None, dat2=None):
    '''
    
    write command
    :param id:
    :param w_cmd:
    :param dat1:
    :param dat2:
    :return:
    '''
    portWrite()
    buf = bytearray(b'\x55\x55')  # Frame header
    buf.append(id)
    # Instruction length
    if dat1 is None and dat2 is None:
        buf.append(3)
    elif dat1 is not None and dat2 is None:
        buf.append(4)
    elif dat1 is not None and dat2 is not None:
        buf.append(7)

    buf.append(w_cmd)  # instruction
    # Write data
    if dat1 is None and dat2 is None:
        pass
    elif dat1 is not None and dat2 is None:
        buf.append(dat1 & 0xff)  # deviation
    elif dat1 is not None and dat2 is not None:
        buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])  # Divide the low 8 bits into the high 8 bits and put them into cache
        buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])  # Divide the low 8 bits into the high 8 bits and put them into cache
    # Checksum
    buf.append(checksum(buf))
    # for i in buf:
    #     print('%x' %i)
    serialHandle.write(buf)  # Send

def serial_servo_read_cmd(id=None, r_cmd=None):
    '''
    Send read command
    :param id:
    :param r_cmd:
    :param dat:
    :return:
    '''
    portWrite()
    buf = bytearray(b'\x55\x55')  # Frame header
    buf.append(id)
    buf.append(3)  # Instruction length
    buf.append(r_cmd)  # instruction
    buf.append(checksum(buf))  # Checksum
    serialHandle.write(buf)  # Send
    time.sleep(0.00034)

def serial_servo_get_rmsg(cmd):
    '''
    # Get the data of the specified read command
    :param cmd: read command
    :return: data
    '''
    serialHandle.flushInput()  # Clear receive cache
    portRead()  # Configure a single-wire serial port as input
    time.sleep(0.005)  # Delay briefly and wait for reception to complete
    count = serialHandle.inWaiting()    # Get the number of bytes in the receive buffer
    if count != 0:  # If the received data is not empty
        recv_data = serialHandle.read(count)  # Read received data
        # for i in recv_data:
        #     print('%#x' %ord(i))
        # Is it a read id command?
        try:
            if recv_data[0] == 0x55 and recv_data[1] == 0x55 and recv_data[4] == cmd:
                dat_len = recv_data[3]
                serialHandle.flushInput()  # Clear receive cache
                if dat_len == 4:
                    # print ctypes.c_int8(ord(recv_data[5])).value    # Convert to signed integer
                    return recv_data[5]
                elif dat_len == 5:
                    pos = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    return ctypes.c_int16(pos).value
                elif dat_len == 7:
                    pos1 = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    pos2 = 0xffff & (recv_data[7] | (0xff00 & (recv_data[8] << 8)))
                    return ctypes.c_int16(pos1).value, ctypes.c_int16(pos2).value
            else:
                return None
        except BaseException as e:
            print(e)
    else:
        serialHandle.flushInput()  # Clear receive cache
        return None
