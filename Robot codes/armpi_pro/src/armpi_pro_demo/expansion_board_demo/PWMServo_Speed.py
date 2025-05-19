#!/usr/bin/python3
# coding=utf8
import sys
import time
import threading
import Board

print('''
**********************************************************
******功能:幻尔科技树莓派扩展板，PWM舵机变速例程*******
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
    
    
if __name__ == '__main__':
   for i in range(5): #循环5次
        Board.setPWMServoPulse(1, 500, 1000) # It takes 1000ms for the PWM servo of interface 1 to move to the 500 position.
        time.sleep(1)
        Board.setPWMServoPulse(1, 1500, 300) # It takes 300ms for the PWM servo of interface 1 to move to the 1500 position.
        time.sleep(1)
        Board.setPWMServoPulse(1, 500, 1000) # It takes 1000ms for the PWM servo of interface 1 to move to the 500 position.
        
