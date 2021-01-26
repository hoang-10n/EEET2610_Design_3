import threading
import os

gapDetected = False
gateTime = 0
objectPassed = 0
frontDistance = 0
dutyCycle = 0
pwmFreq = 100
rangeFront = 0
rangeWall = 0
pulse_start = 0
pulse_end = 0

def task1():
    import RPi.GPIO as GPIO
    import time
    from datetime import timedelta
    import sys

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)


    EN1 = 11
    IN1 = 13 
    IN2 = 15
    EN2 = 23
    IN3 = 19
    IN4 = 21

    trigFront = 29
    echoFront = 31
    trigRight = 26
    echoRight = 24

    infraredSensorFront = 35
    infraredSensorRightFront = 37
    infraredSensorRightRear = 33

    GPIO.setup(EN1, GPIO.OUT)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(EN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)

    GPIO.setup(trigFront, GPIO.OUT)
    GPIO.setup(echoFront, GPIO.IN)
    GPIO.setup(trigRight, GPIO.OUT)
    GPIO.setup(echoRight, GPIO.IN)

    GPIO.setup(infraredSensorFront, GPIO.IN)
    GPIO.setup(infraredSensorRightFront, GPIO.IN)
    GPIO.setup(infraredSensorRightRear, GPIO.IN)

    EN1 = GPIO.PWM(EN1, pwmFreq)
    EN2 = GPIO.PWM(EN2, pwmFreq)
    EN1.start(0)
    EN2.start(0)

    def runMotor(motor, direction):
        in1 = GPIO.HIGH
        in2 = GPIO.LOW

        if (direction == 1):
            in1 = GPIO.LOW
            in2 = GPIO.HIGH

        if (motor == 0):
            GPIO.output(IN1, in1)
            GPIO.output(IN2, in2)

        elif (motor == 1):
            GPIO.output(IN3, in1)
            GPIO.output(IN4, in2)
            
    def forward(dutyCycle):
        EN1.ChangeDutyCycle(dutyCycle)
        EN2.ChangeDutyCycle(dutyCycle)
        runMotor(0, 0)
        runMotor(1, 0)

    def reverse(dutyCycle):
        EN1.ChangeDutyCycle(dutyCycle)
        EN2.ChangeDutyCycle(dutyCycle)
        runMotor(0, 1)
        runMotor(1, 1)
        
    def turnLeft(dutyCycle):
        EN1.ChangeDutyCycle(dutyCycle)
        EN2.ChangeDutyCycle(dutyCycle)
        runMotor(0, 1)
        runMotor(1, 0)

    def turnRight(dutyCycle):
        EN1.ChangeDutyCycle(dutyCycle)
        EN2.ChangeDutyCycle(dutyCycle)
        runMotor(0, 0)
        runMotor(1, 1)

    def motorStop():
        EN1.ChangeDutyCycle(0)
        EN2.ChangeDutyCycle(0)

    def ping(Trig, Echo):
        GPIO.output(Trig, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(Trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(Trig, GPIO.LOW)
        pulse_start = time.time()
        timeout = pulse_start + 0.04
        
        while GPIO.input(Echo) == 0 and pulse_start < timeout:
            pulse_start = time.time()
            pulse_end = time.time()
            
        while GPIO.input(Echo) == 1 and pulse_end < timeout:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance_cm = (pulse_duration / 2) * 343 * 100
        distance_cm = round(distance_cm, 2)
        return distance_cm

    def readRangeFront():
        rangeFront = ping(trigFront, echoFront)
        return rangeFront

    def readRangeRight():
        rangeWall = ping(trigRight, echoRight)
        return rangeWall

    def checkIRFront():
        return GPIO.input(infraredSensorFront) == GPIO.LOW

    def checkIRRightFront():
        return GPIO.input(infraredSensorRightFront) == GPIO.LOW

    def checkIRRightRear():
        return GPIO.input(infraredSensorRightRear) == GPIO.LOW

    # def checkFrontWall(): # TODO change this method so that it won't depend on the batteries
    #     frontDistance = readRangeFront()
    #     if (GPIO.input(infraredSensorFront) == GPIO.LOW):
    #         motorStop()
    #         time.sleep(0.1)
    #         # if (GPIO.input(infraredSensorFront) == GPIO.LOW):
                
    #         print("Wall ahead")

    #         while (checkIRFront()):
    #             turnLeft(40)
    #             time.sleep(0.2)
    #             # forward(15)
    #             # time.sleep(0.1)
    #         while (readRangeFront() < 29):
    #             # if (checkIRFront()):
    #                 # turnLeft(40)
    #                 # time.sleep(0.1)
    #                 # break
    #             print("Fir %f" % readRangeRight())
    #             turnLeft(40)
    #             time.sleep(0.2)
    #         turnLeft(40)
    #         time.sleep(0.3)
    #         if (not checkIRRightRear()):
    #             print("POP")
    #             turnLeft(40)
    #             time.sleep(0.2)

    def checkFrontWall():
        frontDistance = readRangeFront()
        if (GPIO.input(infraredSensorFront) == GPIO.LOW):
            motorStop()
            time.sleep(0.1)
            reverse(10)
            time.sleep(0.2)
                
            print("Wall ahead")

            while (checkIRFront()):
                turnLeft(40) #55
                time.sleep(0.2)
            while (not checkIRRightFront()):
                turnLeft(40)
                time.sleep(0.1)
    #         while (not checkIRRightFront()):
    #             turnLeft(33)
    #             time.sleep(0.1)

    def checkFrontWall2():
        frontDistance = readRangeFront()
        if (GPIO.input(infraredSensorFront) == GPIO.LOW):
    #         forward(15)
            motorStop()
            time.sleep(0.1)
            reverse(25)
            time.sleep(0.2)
                
            print("Wall ahead")

            while (checkIRFront()):
                turnLeft(33)
                time.sleep(0.2)
            while (not checkIRRightRear()):
                turnLeft(33)
                time.sleep(0.1)
            global gateTime
            gateTime = time.time()


    def checkGate():
        while (readRangeFront() > 20):
            motorStop()
            time.sleep(0.5)
        while (readRangeFront() < 20):
            motorStop()
            time.sleep(0.5)
        global objectPassed
        global gateTime
        objectPassed += 1
        gateTime = time.time()

    def checkObstacle(): # TODO ver 1 of this function
        global frontDistance
        suddenChange = readRangeFront()
        if (suddenChange > 50 or checkIRFront()):
            frontDistance = suddenChange
            return
        print("%f %f" % (frontDistance, suddenChange))
        if (suddenChange < 20):
            # if (frontDistance - suddenChange > 15):
            #     print("Obstacle")
            while (suddenChange < 20):
                suddenChange = readRangeFront()
                motorStop()
                time.sleep(1)
            print("NO obstacle")
            global objectPassed
            objectPassed += 1
                # TODO uncommnent this
        frontDistance = readRangeFront()

    def checkFront(): # TODO ver 1
        global objectPassed
        global gateTime
        if (objectPassed == 0):
            checkFrontWall()
            checkObstacle() #TODO change back to checkObstacle
        elif (objectPassed == 1):
            checkFrontWall2()
            if ((timedelta(seconds=time.time() - gateTime).total_seconds() > 3 or readRangeFront() < 15) and (gateTime is not 0)): # running duration before reaching the gate
                checkGate()

    def checkRightWall():
        global gapDetected
        if (not checkIRRightFront()):
            if (readRangeRight() > 15 and readRangeRight() < 29): # the sonic detects wall but it is too far
                
                print("Follow right")
                
                motorStop()
                time.sleep(0.1)
                while (not checkIRRightFront()):
                    if (readRangeFront() < 20): break
                    if (readRangeRight() > 29 or checkIRRightRear()):
                        break
                    turnRight(33)
                    time.sleep(0.2)
                    forward(15)
                    time.sleep(0.3)
                turnLeft(33)
                time.sleep(0.1)

            elif (readRangeRight() > 20): # gap
                if (not gapDetected):
                    gapDetected = True
                else:
                    return
        elif (checkIRRightFront()):
            if (readRangeRight() < 12):

                print("Follow left")
                
                motorStop()
                time.sleep(0.1)
                while (readRangeRight() < 12):
                    if (readRangeFront() in range (10, 20)): break
                    turnLeft(33)
                    time.sleep(0.2)
                    forward(15)
                    time.sleep(0.3)
                turnRight(33)
                time.sleep(0.1)
            # else:
            #     forward(15)
            #     time.sleep(0.3)

    def checkGap():
        pass
        # TODO these are the states to cross the gap
        """
        state 1: RF IR not detecting (turn right based on the function above)
        state 2: RF IR + sonic not detecting
        state 3: All three not detecting
        state 4: RF IR detecting
        state 5: RF IR + sonic detecting
        """
    def autonomy():
        try:
            while True:
                global objectPassed
                print(objectPassed)
                checkFront()
                # checkFrontWall()
                checkRightWall()
                forward(15)
                time.sleep(0.1)
                if (objectPassed == 2):
                    global gateTime
                    if (timedelta(seconds=time.time() - gateTime).total_seconds() > 3):
                        break

                
        except KeyboardInterrupt:
            GPIO.cleanup()
            sys.exit(0)

    autonomy()


def task2():
    import base64
    import cv2
    import paho.mqtt.publish as publish

    broker_location = "192.168.4.1"

    # context = zmq.Context()
    # footage_socket = context.socket(zmq.PUB)
    # footage_socket.connect('tcp://localhost:5555')

    camera = cv2.VideoCapture(0)  # init the camera

    while True:
        try:
            grabbed, frame = camera.read()  # grab the current frame
            frame = cv2.resize(frame, (640, 480))  # resize the frame
            encoded, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer)
            # footage_socket.send(jpg_as_text)
            publish.single("Design3/Camera", jpg_as_text, hostname = broker_location)

        except KeyboardInterrupt:
            camera.release()
            cv2.destroyAllWindows()
            break
    


if __name__ == "__main__":
    # creating threads
    t1 = threading.Thread(target=task1, name='Car_Run')
    t2 = threading.Thread(target=task2, name='Streaming')

    # starting threads
    t1.start()
    t2.start()

    # wait until all threads finish
    t1.join()
    t2.join()


