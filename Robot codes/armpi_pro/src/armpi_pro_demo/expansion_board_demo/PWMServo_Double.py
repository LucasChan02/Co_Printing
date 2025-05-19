#!/usr/bin/python3
# coding=utf8
import sys
import time
import Board as Board

print('''
**********************************************************
*****功能:幻尔科技树莓派扩展板，PWM多舵机控制例程******
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
    for i in range(5):
        Board.setPWMServoPulse(1, 1500, 1000) # It takes 1000ms for the No. 1 servo to turn to the 1500 position.
        time.sleep(1)
        Board.setPWMServoPulse(1, 2500, 1000) # It takes 1000ms for the No. 1 servo to move to the 2500 position.
        time.sleep(1)

        Board.setPWMServoPulse(2, 1500, 1000) # It takes 1000ms for the No. 2 servo to turn to the 1500 position.
        time.sleep(1)
        Board.setPWMServoPulse(2, 2500, 1000) # It takes 1000ms for the No. 2 servo to move to the 2500 position.
        time.sleep(1)











