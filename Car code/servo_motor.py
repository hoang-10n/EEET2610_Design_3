import RPi.GPIO as GPIO
import time
import sys
import tkinter as tk

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
pin_number = 40
GPIO.setup(pin_number, GPIO.OUT)  
frequency_hertz = 50
pwm = GPIO.PWM(pin_number, frequency_hertz)
pwm.start(0)

level = 0

def keyInput(event):
    print ('Key: ', event.char)
    keyPress = event.char
    global level
    if keyPress.lower() == 'w':
        pwm.ChangeDutyCycle(2.5)
        time.sleep(0.5)
    elif keyPress.lower() == 's':
        pwm.ChangeDutyCycle(12.5)
        time.sleep(0.5)
    elif keyPress.lower() == 'a':
        level += 2.5
        if (level >= 12.5): level = 12.5
        pwm.ChangeDutyCycle(level)
        time.sleep(0.5)
    elif keyPress.lower() == 'd':
        level -= 2.5
        if (level <= 2.5): level = 2.5
        pwm.ChangeDutyCycle(level)
        time.sleep(0.5)
    else:
        pass

command = tk.Tk()
command.bind('<KeyPress>', keyInput)
command.mainloop()
