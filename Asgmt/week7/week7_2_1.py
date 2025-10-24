from gpiozero import  TonalBuzzer
import time

BUZZER = TonalBuzzer(12)

try:
    while True:
        BUZZER.play(262)
        time.sleep(0.2)
        
        BUZZER.stop()
        time.sleep(0.1)
        
        BUZZER.play(394)
        time.sleep(0.2)
        
        BUZZER.stop()
        time.sleep(0.1)
        
        BUZZER.play(330)
        time.sleep(0.2)
        
        BUZZER.stop()
        time.sleep(0.1)
        
        BUZZER.play(349)
        time.sleep(0.2)
        
        BUZZER.stop()
        time.sleep(0.1)
        
        BUZZER.play(292)
        time.sleep(0.2)
        
        BUZZER.stop()
        time.sleep(0.1)
        
        BUZZER.play(440)
        time.sleep(0.2)
        
        BUZZER.stop()
        time.sleep(0.1)
        
        BUZZER.play(494)
        time.sleep(0.2)
        
        BUZZER.stop()
        time.sleep(0.1)
        
        BUZZER.play(523)
        time.sleep(0.2)
        
        BUZZER.stop()
        time.sleep(0.1)
        
        BUZZER.stop()
        time.sleep(2.0)
        
        

except KeyboardInterrupt:
    pass

BUZZER.stop()
