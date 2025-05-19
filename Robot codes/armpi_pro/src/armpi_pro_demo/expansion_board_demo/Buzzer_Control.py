#!/usr/bin/python3
# coding=utf8
# Date:2022/06/30
import sys
import time
import signal
import Board as Board

print('''
**********************************************************
******功能:幻尔科技树莓派扩展板，蜂鸣器控制例程********
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
    Board.setBuzzer(0) # Turn off the buzzer

signal.signal(signal.SIGINT, stop)

if __name__ == '__main__':
    
    #蜂鸣器响0.5S
    Board.setBuzzer(1) # Set buzzer sound
    time.sleep(0.5)    # delay 0.5S
    Board.setBuzzer(0) # Turn off the buzzer
    
    
    