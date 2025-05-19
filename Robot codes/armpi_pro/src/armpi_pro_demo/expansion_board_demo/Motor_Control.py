#!/usr/bin/python3
# coding=utf8
# Date:2022/06/30
import sys
import time
import signal
import Board as Board

print('''
**********************************************************
*******功能:幻尔科技树莓派扩展板，电机控制例程*********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close the program. If it fails, please try multiple times.！
----------------------------------------------------------
''')


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

#Processing before closing
def stop(signum, frame):
    Board.setMotor(2, 0)   # Motor No. 2 stops
    
signal.signal(signal.SIGINT, stop)

if __name__ == '__main__':
    Board.setMotor(2,100) # Motor No. 2, rotating at 100 speed
    time.sleep(2)         # Delay 2S
    Board.setMotor(2,50)  # Motor No. 2, rotating at 50 speed
    time.sleep(2)
    Board.setMotor(2,-100) # Motor No. 2, rotates at -100 speed
    time.sleep(2)
    Board.setMotor(2, 0)   # Motor No. 2 stops
    