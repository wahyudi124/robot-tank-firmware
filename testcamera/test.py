# Importing Libraries
import cv2 as cv
import paho.mqtt.client as mqtt
import base64
import time


# mqtt broker configs
MQTT_BROKER = "103.84.207.210"
MQTT_SEND = "esp32/cam_0"

# Object to capture the frames
cap = cv.VideoCapture(0)

# Phao-MQTT Clinet
client = mqtt.Client()

# Establishing Connection with the Broker
client.connect(MQTT_BROKER)

try:
    while True:
        start = time.time()
        # Read Frame
        _, frame = cap.read()
        frame= cv.resize(frame, (240, 190)) 
        # Encoding the Frame
        _, buffer = cv.imencode('.jpg', frame)
        # Converting into encoded bytes
        jpg_as_text = base64.b64encode(buffer)
        # Publishig the Frame on the Topic home/server
        client.publish(MQTT_SEND, "data:image/jpg;base64,{}".format(jpg_as_text.decode("utf-8") ))
        end = time.time()
        t = end - start
        fps = 1/t
        time.sleep(0.06)
        print(fps)
        
except:
    cap.release()
    client.disconnect()
    print("\nNow you can restart fresh")