import time
import Board
import signal

print('''
**********************************************************
***********功能:幻尔科技树莓派扩展板，RGB灯点亮例程*************
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


start = True
#Processing before closing
def Stop(signum, frame):
    global start

    start = False
    print('Closed...')

#Turn off all lights first
Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))  
Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))  
Board.RGB.show()

signal.signal(signal.SIGINT, Stop)

while True: 
    Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0)) # Set the LED light RGB1 of the expansion board to red
    Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0)) # Set the LED light RGB2 of the expansion board to red
    Board.RGB.show()
    time.sleep(1)

    if not start:
        #all lights off
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))  
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))  
        Board.RGB.show()
        print('Closed')
        break
    
