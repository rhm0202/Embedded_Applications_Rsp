from gpiozero import DigitalOutputDevice
from gpiozero import PWMOutputDevice

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