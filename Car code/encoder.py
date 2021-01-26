import RPi.GPIO as GPIO
from datetime import datetime as time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(16, GPIO.IN, pull_up_down = GPIO.PUD_UP) # encoder A
GPIO.setup(18, GPIO.IN, pull_up_down = GPIO.PUD_UP) # encoder B

stateLast = GPIO.input(18)
rotationCount = 0
stateCount = 0
stateCountTotal = 0

timeLast = time.now()
statesPerRotation = 21 # change this one

try:
    while True:
        stateCurrent = GPIO.input(18)
        if stateCurrent != stateLast:
            if stateCurrent != GPIO.input(16):
                stateCountTotal += 1
            else:
                stateCountTotal -= 1
            stateLast = stateCurrent

        timeNow = time.now()
        if ((timeNow - timeLast).total_seconds() >= 1):
            rpm = stateCountTotal/20
            print("RPM %f" % rpm)
            speed = rpm * 0.0054
            print("Speed %f" % speed)
            stateCountTotal = 0
            timeLast = timeNow

except KeyboardInterrupt:
    GPIO.cleanup()

# TODO velocity = 0.8m/s = 150 RPM