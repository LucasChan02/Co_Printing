import time
import Board

print('''
**********************************************************
*******功能:幻尔科技树莓派扩展板，RGB灯闪烁例程********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close the program. If it fails, please try multiple times.！
----------------------------------------------------------
''')


def stop():         
    print('关闭中...')
    Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0)) # Turn off the LED light RGB1 of the expansion board
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0)) # Turn off the LED light RGB2 of the expansion board
    Board.RGB.show()

def start():
    Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0)) # Set the LED light RGB1 of the expansion board to red
    Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0)) # Set the LED light RGB2 of the expansion board to red
    Board.RGB.show()
    time.sleep(1)

    Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0)) # Turn off the LED light RGB1 of the expansion board
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0)) # Turn off the LED light RGB2 of the expansion board
    Board.RGB.show()
    time.sleep(1)   

    for i in range(3):
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0)) # Set the LED light RGB1 of the expansion board to red
        Board.RGB.show()
        time.sleep(0.5)
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0)) # Turn off the LED light RGB1 of the expansion board
        Board.RGB.show()
        time.sleep(0.5)
         
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0)) # Set the LED light RGB2 of the expansion board to red
        Board.RGB.show()
        time.sleep(0.5)
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0)) # Turn off the LED light RGB2 of the expansion board
        Board.RGB.show()
        time.sleep(0.5)

    Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0)) # Set the LED light RGB1 of the expansion board to red
    Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0)) # Set the LED light RGB2 of the expansion board to red
    Board.RGB.show()
    time.sleep(1)

    Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0)) # Turn off the LED light RGB1 of the expansion board
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0)) # Turn off the LED light RGB2 of the expansion board
    Board.RGB.show()
    time.sleep(1)   


if __name__ == '__main__': 
    Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0)) # Turn off the LED light RGB1 of the expansion board
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0)) # Turn off the LED light RGB2 of the expansion board
    Board.RGB.show()
    time.sleep(1)	 
    while True:
        try:
            start()	
        except KeyboardInterrupt:
            stop()
            print('Closed')
            break
    
