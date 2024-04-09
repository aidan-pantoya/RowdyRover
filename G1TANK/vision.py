#-*- coding:UTF-8 -*-
# import RPi.GPIO as GPIO
import time
from ultralytics import YOLO
import cv2
import math 

# Conneticut isn't a real state.

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
model = YOLO("C:/Users/apant/Downloads/yolo8/yolov8n.pt")
# model = YOLO("C:/Users/apant/Downloads/yolo8/yolov8l.pt") 
# model = YOLO("C:/Users/apant/Downloads/yolo8/yolov8s.pt") 

classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
  "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
  "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
  "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
  "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
  "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
  "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
  "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
  "teddy bear", "hair drier", "toothbrush"
  ]


#Definition of  motor pin 
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Set the GPIO port to BCM encoding mode.
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#Motor pin initialization operation
def init():
    global pwm_ENA
    global pwm_ENB
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    
#Advance
def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.start(leftspeed)
    pwm_ENB.start(rightspeed)

#back
def back(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.start(leftspeed)
    pwm_ENB.start(rightspeed)
    
#turn left
def left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.start(leftspeed)
    pwm_ENB.start(rightspeed)

#turn right
def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.start(leftspeed)
    pwm_ENB.start(rightspeed)
    
#turn left in place
def spin_left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.start(leftspeed)
    pwm_ENB.start(rightspeed)

#turn right in place
def spin_right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.start(leftspeed)
    pwm_ENB.start(rightspeed)

#brake
def brake():
   GPIO.output(IN1, GPIO.LOW)
   GPIO.output(IN2, GPIO.LOW)
   GPIO.output(IN3, GPIO.LOW)
   GPIO.output(IN4, GPIO.LOW)

time.sleep(2)


init()


while True:
    success, img = cap.read()
    if not success:
        break  

    results = model(img, stream=True)
    img_height, img_width, _ = img.shape


    objects_left, objects_right, objects_center = 0, 0, 0

    for r in results:
        boxes = r.boxes

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) 
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            confidence = math.ceil((box.conf[0]*100))/100
            cls = int(box.cls[0])

            org = (x1, y1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img, f'{classNames[cls]} {confidence}', org, font, 1, (255, 0, 0), 2)


            center_x = (x1 + x2) / 2
            if center_x < img_width / 3:
                objects_left += 1
            elif center_x > 2 * img_width / 3:
                objects_right += 1
            else:
                objects_center += 1

    cv2.imshow('Webcam', img)
    key = cv2.waitKey(1)
    if key == 27:  # ESC key to break
        break

    print(f'objects: L:{objects_left}, C:{objects_center}, R:{objects_right}')
    
    if objects_center == 0:
        print('STRAIGHT')
        run(70, 70)
    
    elif objects_left > objects_right:
        print('RIGHT')
        spin_right(50, 50)
    
    elif objects_right > objects_left:
        print('LEFT') 
        spin_left(50, 50)
    
    else:
        print('BACK')
        back(50, 50)
        spin_left(50, 50)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()