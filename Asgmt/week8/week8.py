import threading
import serial
import time
from gpiozero import DigitalOutputDevice
from gpiozero import PWMOutputDevice

bleSerial = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1.0)

gData = ""

PWMA = PWMOutputDevice(18)
AIN1 = DigitalOutputDevice(22)
AIN2 = DigitalOutputDevice(27)

PWMB = PWMOutputDevice(23)
BIN1 = DigitalOutputDevice(25)
BIN2 = DigitalOutputDevice(24)

def go():
    AIN1.value = 0
    AIN2.value = 1
    PWMA.value = 0.5 # 0.0~1.0 speed
    BIN1.value = 0
    BIN2.value = 1
    PWMB.value = 0.5 # 0.0~1.0 speed

def back():
    AIN1.value = 1
    AIN2.value = 0
    PWMA.value = 0.5 # 0.0~1.0 speed
    BIN1.value = 1
    BIN2.value = 0
    PWMB.value = 0.5 # 0.0~1.0 speed

def left():
    AIN1.value = 1
    AIN2.value = 0
    PWMA.value = 0.5 # 0.0~1.0 speed
    BIN1.value = 0
    BIN2.value = 1
    PWMB.value = 0.5 # 0.0~1.0 speed
    
def right():
    AIN1.value = 0
    AIN2.value = 1
    PWMA.value = 0.5 # 0.0~1.0 speed
    BIN1.value = 1
    BIN2.value = 0
    PWMB.value = 0.5 # 0.0~1.0 speed
    
def stop():
    AIN1.value = 0
    AIN2.value = 1
    PWMA.value = 0.0 # 0.0~1.0 speed
    BIN1.value = 1
    BIN2.value = 0
    PWMB.value = 0.0 # 0.0~1.0 speed
    
def serial_thread():
    global gData
    while True:
        data = bleSerial.readline()
        data = data.decode()
        gData = data
        
def main():
    global gData
    try:
        while True:
            if gData.find("go") >= 0:
                gData = ""
                print("ok go")
                go()
            elif gData.find("back") >= 0:
                gData = ""
                print("ok back")
                back()
            elif gData.find("left") >= 0:
                gData = ""
                print("ok left")
                left()
            elif gData.find("right") >= 0:
                gData = ""
                print("ok right")
                right()
            elif gData.find("stop") >= 0:
                gData = ""
                print("ok stop")
                stop()

    except KeyboardInterrupt:
        pass
    
if __name__ == '__main__':
    task1 = threading.Thread(target = serial_thread)
    task1.start()
    main()
    bleSerial.close()
    PWMA.value = 0.0
    PWMB.value = 0.0