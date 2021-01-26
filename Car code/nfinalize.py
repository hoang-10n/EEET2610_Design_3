import threading, os, time, sys
import RPi.GPIO as GPIO
import paho.mqtt.publish as publish
import paho.mqtt.client as client
from ina219 import INA219, DeviceRangeError
from datetime import datetime as datetime
from datetime import timedelta
import cv2
import numpy as np

BROKER_LOCATION = "192.168.4.1"

gap_detected = False
auto_mode = False
# object_detection = False
direction = ""
gate_time = 0
object_passed = 0
frontDistance = 0
pwmFreq = 100
speed = 0
detecting = False

def on_message_control(client, userdata, message):
    msg = message.payload.decode('utf-8')
    # print(msg)
    global auto_mode
    global direction
    if (msg == "auto" or msg == "manual"):
        auto_mode = True if msg == "auto" else False
        print(auto_mode)
    else:
        if (not auto_mode):
            direction = msg
            print(direction)
        else:
            direction = ""

class Wifi:
# self.stream(message.payload)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected")
        msg_channel = "Design3/" + self.channel
        self.client.subscribe(msg_channel)
    
    def __init__(self, channel):
        self.channel = channel
        self.client = client.Client()
        self.client.on_connect = self.on_connect
        # self.client.on_message = on_message_camera
        self.client.connect("192.168.4.1")
        self.client.loop_start()

    def set_on_message(self, function):
        self.client.on_message = function

def findCube(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    ## mask of green (36,25,25) ~ (86, 255,255)
    # mask = cv2.inRange(hsv, (36, 25, 25), (86, 255,255))
    mask = cv2.inRange(hsv, (70,40,40), (125,255,255))

    ## slice the green
    imask = mask>0
    rg = np.zeros_like(rgb, np.uint8)
    rg[imask] = rgb[imask]

    MORPH = 7
    gray = cv2.cvtColor(rg, cv2.COLOR_BGR2GRAY)

    gray = cv2.bilateralFilter(gray, 1, 10, 120)

    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    edge = cv2.Canny(gray, 60, 100)

    kernel1 = np.ones((3, 3), np.uint8)

    edges = cv2.dilate(edge, kernel1, iterations=1)
    edges = cv2.dilate(edges, kernel1, iterations=1)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH, MORPH))

    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    contours, hieriarchy = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cont in contours:
        bbox = cv2.boundingRect(cont)

        x, y, w, h = bbox

        area = cv2.contourArea(cont)
        if area > 1000:
            arc_len = cv2.arcLength(cont, True)

            approx = cv2.approxPolyDP(cont, 0.01 * arc_len, True)

            rbox = cv2.minAreaRect(cont)
            (cx, cy), (cw, ch), rot_angle = rbox
            if area - 1500 < cw * ch < area + 1500 and len(approx) == 4:
                cv2.rectangle(rgb, (x, y), (x + w, y + h), (255, 0, 0), 1, 16)
                # cv2.drawContours(rgb, [approx], -1, (25, 0, 0), -1)
                # print(approx)

            if (len(approx) in range(4, 8)):
                # cv2.drawContours(rgb, [approx], -1, (255, 0, 0), -1)
                cv2.putText(rgb, "Cube", ((int)(cx) - 20, (int)(cy) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 2)
                print("Cube !!!")
                publish.single("Design3/Object", "cube", hostname = BROKER_LOCATION)
                break
            else:
                publish.single("Design3/Object", "cylinder", hostname = BROKER_LOCATION)
                print("Not cube !!")
                print("Cylinder !!")
                break

    # cv2.imshow('edges', edges)
    # cv2.imshow('rgb', rgb)
    if cv2.waitKey(99) & 0xFF == ord('c'):
        pass


def findCircle(rgb):
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

    gray = cv2.medianBlur(gray, 5)

    rows = gray.shape[0]
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                               param1=150, param2=30,
                               minRadius=50, maxRadius=100)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(rgb, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(rgb, center, radius, (255, 0, 255), -1)
            cv2.putText(rgb, "Sphere", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (255, 255, 255), 2)
        publish.single("Design3/Object", "sphere", hostname = BROKER_LOCATION)
        print("Sphere !!!")
        # cv2.imshow("detected circles", rgb)
        if cv2.waitKey(99) & 0xFF == ord('c'):
            pass

    else:
        print("Not sphere !!!")

        findCube(rgb)

def detect(img):
    findCircle(img)

def PowerConsumption(): 
    SHUNT_OHMS = 0.1
    MAX_EXPECTED_AMPS = 2.0
    ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS)
    ina.configure(ina.RANGE_16V)

    def read_ina219():
        try:
            # print('Bus Voltage: {0:0.2f}V'.format(ina.voltage()))
            # print('Bus Current: {0:0.2f}mA'.format(ina.current()))
            # print('Power: {0:0.2f}mW'.format(ina.power()))
            # print('Shunt Voltage: {0:0.2f}mV\n'.format(ina.shunt_voltage()))
            publish.single("Design3/Power", '{0:0.2f}mW'.format(ina.power()), hostname = BROKER_LOCATION)
        except DeviceRangeError as e:
            print(e)

    while 1:
        read_ina219()
        time.sleep(1)


def Speed():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # encoder A
    GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # encoder B

    stateLast = GPIO.input(18)
    stateCountTotal = 0

    timeLast = datetime.now()

    try:
        while True:
            stateCurrent = GPIO.input(18)
            if stateCurrent != stateLast:
                if stateCurrent != GPIO.input(16):
                    stateCountTotal += 1
                else:
                    stateCountTotal -= 1
                stateLast = stateCurrent

            timeNow = datetime.now()
            if ((timeNow - timeLast).total_seconds() >= 1):
                global speed
                rpm = stateCountTotal / 20
                # print("RPM %f" % rpm)
                speed = rpm * 0.0054
                # print("Speed %f" % speed)
                publish.single("Design3/Odometer", '%dcm/s' % (speed * 100), hostname = BROKER_LOCATION)
                stateCountTotal = 0
                timeLast = timeNow

    except KeyboardInterrupt:
        GPIO.cleanup()


def RunCar():

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    EN1 = 11
    IN1 = 13 
    IN2 = 15
    EN2 = 23
    IN3 = 19
    IN4 = 21

    TRIGFRONT = 29
    ECHOFRONT = 31
    TRIGRIGHT = 26
    ECHORIGHT = 24

    IRFRONT = 35
    IRRIGHTFRONT = 37
    IRRIGHTREAR = 33

    SERVOMOTOR = 40

    GPIO.setup(TRIGFRONT, GPIO.OUT)
    GPIO.setup(ECHOFRONT, GPIO.IN)
    GPIO.setup(TRIGRIGHT, GPIO.OUT)
    GPIO.setup(ECHORIGHT, GPIO.IN)

    GPIO.setup(IRFRONT, GPIO.IN)
    GPIO.setup(IRRIGHTFRONT, GPIO.IN)
    GPIO.setup(IRRIGHTREAR, GPIO.IN)

    GPIO.setup(EN1, GPIO.OUT)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(EN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)

    EN1 = GPIO.PWM(EN1, pwmFreq)
    EN2 = GPIO.PWM(EN2, pwmFreq)
    EN1.start(0)
    EN2.start(0)

    GPIO.setup(SERVOMOTOR, GPIO.OUT)  
    frequency_hertz = 50
    pwm = GPIO.PWM(SERVOMOTOR, frequency_hertz)
    pwm.start(0)

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
        global speed
        int_speed = int(speed * 100 * 0.1)
        publish.single("Design3/Position", str(int_speed), hostname = BROKER_LOCATION)        
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

        pulse_end = time.time()
        
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
        rangeFront = ping(TRIGFRONT, ECHOFRONT)
        return rangeFront

    def readRangeRight():
        rangeWall = ping(TRIGRIGHT, ECHORIGHT)
        return rangeWall

    def checkIRFront():
        return GPIO.input(IRFRONT) == GPIO.LOW

    def checkIRRightFront():
        return GPIO.input(IRRIGHTFRONT) == GPIO.LOW

    def checkIRRightRear():
        return GPIO.input(IRRIGHTREAR) == GPIO.LOW

    def checkFrontWall():
        frontDistance = readRangeFront()
        if (GPIO.input(IRFRONT) == GPIO.LOW):
            motorStop()
            time.sleep(0.1)
            reverse(15)
            time.sleep(0.2)
                
            print("Wall ahead")

            while (checkIRFront()):
                turnLeft(40) #55
                time.sleep(0.2)
            while (not checkIRRightFront()):
                turnLeft(40)
                time.sleep(0.1)

            publish.single("Design3/Orientation", "-90", hostname = BROKER_LOCATION)

    #         while (not checkIRRightFront()):
    #             turnLeft(33)
    #             time.sleep(0.1)

    def checkFrontWall2():
        frontDistance = readRangeFront()
        if (GPIO.input(IRFRONT) == GPIO.LOW):
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
            global gate_time
            gate_time = time.time()

            publish.single("Design3/Orientation", "-90", hostname = BROKER_LOCATION)


    def checkGate():
        while (readRangeFront() > 20):
            motorStop()
            time.sleep(0.5)
        while (readRangeFront() < 20):
            motorStop()
            time.sleep(0.5)
        global object_passed
        global gate_time
        object_passed += 1
        gate_time = time.time()

    def checkObstacle():
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
            global object_passed
            object_passed += 1
        frontDistance = readRangeFront()

    def checkFront():
        global object_passed
        global gate_time
        if (object_passed == 0):
            checkFrontWall()
            checkObstacle()
        elif (object_passed == 1):
            checkFrontWall2()
            if ((timedelta(seconds=time.time() - gate_time).total_seconds() > 2.5 or readRangeFront() < 15) and (gate_time is not 0)): # TODO running duration before reaching the gate
                checkGate()

    def checkRightWall():
        global gap_detected
        if (not checkIRRightFront()):
            if (readRangeRight() > 15 and readRangeRight() < 29): # the sonic detects wall but it is too far
                
                print("Follow right")
                
                motorStop()
                time.sleep(0.1)
                while (not checkIRRightFront()):
                    if (readRangeFront() < 20 or checkFront()): break
                    if (readRangeRight() > 29 or checkIRRightRear()):
                        break
                    turnRight(33)
                    time.sleep(0.2)
                    forward(15)
                    time.sleep(0.2)
                turnLeft(33)
                time.sleep(0.1)

            elif (readRangeRight() > 20): # gap
                if (not gap_detected):
                    gap_detected = True
                else:
                    return
        elif (checkIRRightFront()):
            if (readRangeRight() < 12):

                print("Follow left")
                
                motorStop()
                time.sleep(0.1)
                while (readRangeRight() < 12):
                    if (readRangeFront() in range (10, 20) or checkFront()): break
                    turnLeft(33)
                    time.sleep(0.2)
                    forward(15)
                    time.sleep(0.2)
                turnRight(33)
                time.sleep(0.1)
            # else:
            #     forward(15)
            #     time.sleep(0.3)

    def autonomy():
        try:
            global auto_mode
            while (auto_mode):
                if (readRangeFront() < 20):
                    publish.single("Design3/Wall", "f", hostname = BROKER_LOCATION)
                elif (readRangeRight() < 20):
                    publish.single("Design3/Wall", "r", hostname = BROKER_LOCATION)
                global object_passed
                print(object_passed)
                checkFront()
                # checkFrontWall()
                checkRightWall()
                forward(15)
                time.sleep(0.1)
                if (object_passed == 2):
                    global gate_time
                    if (timedelta(seconds=time.time() - gate_time).total_seconds() > 2):
                        while(True):
                            global detecting
                            detecting = True
                            motorStop()
                            time.sleep(0.1)
                            if (not auto_mode):
                                break

            control = Wifi("Control")
            control.set_on_message(on_message_control)

            level = 0

            while (not auto_mode):
                if (readRangeFront() < 20):
                    publish.single("Design3/Wall", "f", hostname = BROKER_LOCATION)
                elif (readRangeRight() < 20):
                    publish.single("Design3/Wall", "r", hostname = BROKER_LOCATION)
                global direction
                print (direction)
                if (direction == ""):
                    motorStop()
                    time.sleep(0.1)
                elif (direction == "forward"):
                    forward(15)
                    time.sleep(0.2)
                    direction = ""
                elif (direction == "reverse"):
                    reverse(15)
                    time.sleep(0.2)
                    direction = ""
                elif (direction == "right"):
                    turnRight(30)
                    time.sleep(0.2)
                    direction = ""
                elif (direction == "left"):
                    turnLeft(30)
                    time.sleep(0.2)
                    direction = ""

                #TODO forklift
                elif (direction == "down"):
                    direction = ""
                    level += 2.5
                    if (level >= 12.5): level = 12.5
                    pwm.ChangeDutyCycle(level)
                    time.sleep(0.8)
                    pwm.ChangeDutyCycle(0)
                elif (direction == "up"):
                    direction = ""
                    level -= 2.5
                    if (level <= 2.5): level = 2.5
                    pwm.ChangeDutyCycle(level)
                    time.sleep(0.8)
                    pwm.ChangeDutyCycle(0)
                
                
        except KeyboardInterrupt:
            # pwm.ChangeDutyCycle(12.5)
            # time.sleep(0.5)
            GPIO.cleanup()
            sys.exit(0)

    autonomy()


def Stream():
    import base64

    camera = cv2.VideoCapture(0)  # init the camera
    t1.join(0)
    while True:
        try:
            grabbed, frame = camera.read()  # grab the current frame
            global detecting
            if (detecting): 
                detect(frame)
            # if (1):
            #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            #     ## mask of green (36,25,25) ~ (86, 255,255)
            #     # mask = cv2.inRange(hsv, (36, 25, 25), (86, 255,255))
            #     mask = cv2.inRange(hsv, (70,40,40), (125,255,255))

            #     ## slice the green
            #     if (1 in mask):
            #         global object_detection
            #         object_detection = True
            

            frame = cv2.resize(frame, (640, 480))  # resize the frame
            encoded, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer)
            # footage_socket.send(jpg_as_text)
            publish.single("Design3/Camera", jpg_as_text, hostname = BROKER_LOCATION)

        except KeyboardInterrupt:
            camera.release()
            cv2.destroyAllWindows()
            break

    # TODO velocity = 0.8m/s = 150 RPM


if __name__ == "__main__":
    # creating threads
    t1 = threading.Thread(target=PowerConsumption, name='PowerConsumption')
    t2 = threading.Thread(target=Speed, name='Speed')
    t3 = threading.Thread(target=RunCar, name= 'RunCar')
    t4 = threading.Thread(target=Stream, name= 'Streaming')

    # starting threads
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    # wait until all threads finish
    t1.join()
    t2.join()
    t3.join()
    t4.join()

