from gpiozero import Button
import time
from gpiozero import DigitalOutputDevice
from gpiozero import PWMOutputDevice


PWMA = PWMOutputDevice(18)
AIN1 = DigitalOutputDevice(22)
AIN2 = DigitalOutputDevice(27)

PWMB = PWMOutputDevice(23)
BIN1 = DigitalOutputDevice(25)
BIN2 = DigitalOutputDevice(24)


SW1 = Button(5, pull_up=False )
SW2 = Button(6, pull_up=False )
SW3 = Button(13, pull_up=False )
SW4 = Button(19, pull_up=False )

oldSw = [0,0,0,0]
newSw = [0,0,0,0]

class goStop:
    def frontGo():
        AIN1.value = 0
        AIN2.value = 1
        PWMA.value = 0.5 # 0.0~1.0 speed
        BIN1.value = 0
        BIN2.value = 1
        PWMB.value = 0.5 # 0.0~1.0 speed
    def frontStop():
        AIN1.value = 0
        AIN2.value = 1
        PWMA.value = 0.0 # 0.0~1.0 speed
        BIN1.value = 0
        BIN2.value = 1
        PWMB.value = 0.0 # 0.0~1.0 speed
    def backGo():
        AIN1.value = 1
        AIN2.value = 0
        PWMA.value = 0.5 # 0.0~1.0 speed
        BIN1.value = 1
        BIN2.value = 0
        PWMB.value = 0.5 # 0.0~1.0 speed
    def backStop():
        AIN1.value = 1
        AIN2.value = 0
        PWMA.value = 0.0 # 0.0~1.0 speed
        BIN1.value = 1
        BIN2.value = 0
        PWMB.value = 0.0 # 0.0~1.0 speed
    def leftGo():
        AIN1.value = 1
        AIN2.value = 0
        PWMA.value = 0.5 # 0.0~1.0 speed
        BIN1.value = 0
        BIN2.value = 1
        PWMB.value = 0.5 # 0.0~1.0 speed
    def leftStop():
        AIN1.value = 1
        AIN2.value = 0
        PWMA.value = 0.0 # 0.0~1.0 speed
        BIN1.value = 0
        BIN2.value = 1
        PWMB.value = 0.0 # 0.0~1.0 speed
    def rightGo():
        AIN1.value = 0
        AIN2.value = 1
        PWMA.value = 0.5 # 0.0~1.0 speed
        BIN1.value = 1
        BIN2.value = 0
        PWMB.value = 0.5 # 0.0~1.0 speed
    def rightStop():
        AIN1.value = 0
        AIN2.value = 1
        PWMA.value = 0.0 # 0.0~1.0 speed
        BIN1.value = 1
        BIN2.value = 0
        PWMB.value = 0.0 # 0.0~1.0 speed



try:
    while True:
        newSw[0] = SW1.is_pressed
        if newSw[0] != oldSw[0]:
            oldSw[0] = newSw[0]
            
            if newSw[0]:
                print("SW1 click")
                goStop.frontGo()
            else:
                goStop.frontStop()      
        
        
        newSw[1] = SW2.is_pressed
        if newSw[1] != oldSw[1]:
            oldSw[1] = newSw[1]
            
            if newSw[1] == 1:
                print("SW2 click")
                goStop.rightGo()
            else:
                goStop.rightStop()
            
            time.sleep(0.2)
            
        newSw[2] = SW3.is_pressed
        if newSw[2] != oldSw[2]:
            oldSw[2] = newSw[2]
            
            if newSw[2] == 1:
                print("SW3 click")
                goStop.leftGo()
            else:
                goStop.leftStop()
            
            time.sleep(0.2)
            
        newSw[3] = SW4.is_pressed
        if newSw[3] != oldSw[3]:
            oldSw[3] = newSw[3]
            
            if newSw[3] == 1:
                print("SW4 click")
                goStop.backGo()
            else:
                goStop.backStop()
            
            time.sleep(0.2)

except KeyboardInterrupt:
    pass
