import time
import Board
import signal



start = True
#关闭前处理
def Stop(signum, frame):
    global start

    start = False
    print('关闭中...')

#先将所有灯关闭
Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))  
Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))  
Board.RGB.show()

signal.signal(signal.SIGINT, Stop)

while True: 
    Board.RGB.setPixelColor(0, Board.PixelColor(0,255, 0)) # 将扩展板的LED灯RGB1设置为红色
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0)) # 将扩展板的LED灯RGB2设置为红色
    Board.RGB.show()
    time.sleep(1)

    if not start:
        #所有灯关闭
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))  
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))  
        Board.RGB.show()
        print('已关闭')
        break
    
