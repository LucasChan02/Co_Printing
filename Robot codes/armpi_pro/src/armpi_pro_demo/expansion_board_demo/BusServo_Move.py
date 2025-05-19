import time
import Board

print('''
**********************************************************
*********
Function: Huaner Technology Raspberry Pi expansion board, serial port servo movement routine*************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close the program. If it fails, please try multiple times.！
----------------------------------------------------------
''')

while True:
	
# Parameters: Parameter 1: Servo id; Parameter 2: Position; Parameter 3: Running time
# The rotation range of the servo is 0-240 degrees, and the corresponding pulse width is 0-1000, that is, the range of parameter 2 is 0-1000

	Board.setBusServoPulse(1, 800, 1000) #Servo No. 6 turns to position 800, which takes 1000ms.
	time.sleep(0.5) #Delay 0.5s

	Board.setBusServoPulse(1, 200, 1000) #Servo No. 6 turns to position 200, which takes 1000ms.
	time.sleep(0.5) #Delay 0.5s
    
    
