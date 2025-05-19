import time
import Board
import signal

print('''
**********************************************************
**********功能:幻尔科技树莓派扩展板，RGB彩灯变化例程************
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
    #Set 2 lights to red, 0 is RGB1, 1 is RGB2
    Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
    Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
    Board.RGB.show()
    time.sleep(1)
    
    #Set 2 lights to green, 0 is RGB1, 1 is RGB2
    Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
    Board.RGB.show()
    time.sleep(1)
    
    #Set 2 lights to blue, 0 is RGB1, 1 is RGB2
    Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
    Board.RGB.show()
    time.sleep(1)  

    if not start:
        #all lights off
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()
        print('Closed')
        break
