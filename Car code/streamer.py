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