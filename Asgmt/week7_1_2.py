from gpiozero import Button
import time

SW1 = Button(5, pull_up=False )

oldSw = 0
newSw = 0
cnt = 0

try:
    while True:
        newSw = SW1.is_pressed
        if newSw != oldSw:
            cnt = cnt + 1
            print(f"click, {cnt}")
            time.sleep(0.2)
                
        oldSw = newSw

except KeyboardInterrupt:
    pass

