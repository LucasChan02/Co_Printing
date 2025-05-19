import time
import Board

print('''
**********************************************************
*********Function: Huaner Technology Raspberry Pi expansion board, serial port servo reading status routine**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close the program. If it fails, please try multiple times.！
----------------------------------------------------------
''')

def getBusServoStatus():
    Pulse = Board.getBusServoPulse(6) # Get the position information of servo No. 6
    Temp = Board.getBusServoTemp(6)   # Get the temperature information of servo No. 6
    Vin = Board.getBusServoVin(6)     # Get the voltage information of servo No. 6
    print('Pulse: {}\nTemp:  {}\nVin:   {}\n'.format(Pulse, Temp, Vin)) # Print status information
    time.sleep(0.5) # Time-lapse for easy viewing

   
Board.setBusServoPulse(6, 500, 1000) # It takes 1000ms for the No. 6 servo to turn to the 500 position.
time.sleep(1) #Delay 1s
getBusServoStatus() #Read bus servo status
 
