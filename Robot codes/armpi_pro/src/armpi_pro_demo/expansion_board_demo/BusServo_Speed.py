import time
import Board

print('''
**********************************************************
***********功能:幻尔科技树莓派扩展板，串口舵机变速例程***********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 
Press Ctrl+C to close the program. If it fails, please try multiple times.！
----------------------------------------------------------
''')

while True:
	# Parameters: Parameter 1: Servo id; Parameter 2: Position; Parameter 3: Running time
	# The rotation range of the servo is 0-240 degrees, and the corresponding pulse width is 0-1000, that is, the range of parameter 2 is 0-1000

	Board.setBusServoPulse(2, 800, 1200) # The No. 2 servo rotates to position 800, which takes 1200ms.
	time.sleep(0.5) # Delay 0.5s

	Board.setBusServoPulse(2, 200, 300) # The No. 2 servo rotates to position 200, which takes 300ms.
	time.sleep(0.5) # Delay 0.5s
	     
