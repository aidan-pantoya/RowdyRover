
#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
from ultralytics import YOLO
import cv2
import math 
import threading


model = YOLO("C:/Users/apant/Downloads/RowdyNav/yolov8n.pt")

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

#Definition of  motor pins 
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Definition of  key
key = 8

#Definition of  ultrasonic module pins
EchoPin = 0
TrigPin = 1

#Definition of RGB module pins
LED_R = 22
LED_G = 27
LED_B = 24

#Definition of servo pin
ServoPin = 23

#Set the GPIO port to BCM encoding mode
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#Motor pins are initialized into output mode
#Key pin is initialized into input mode
#Ultrasonic pin,RGB pin,servo pin initialization
def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(key,GPIO.IN)
    GPIO.setup(EchoPin,GPIO.IN)
    GPIO.setup(TrigPin,GPIO.OUT)
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(ServoPin, GPIO.OUT)
    #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)

    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)
    
#advance
def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#back
def back(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)
    
#turn left
def left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#trun right 
def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)
    
#turn left in place
def spin_left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#turn right in place
def spin_right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#brake
def brake():
   GPIO.output(IN1, GPIO.LOW)
   GPIO.output(IN2, GPIO.LOW)
   GPIO.output(IN3, GPIO.LOW)
   GPIO.output(IN4, GPIO.LOW)

def Distance():
    GPIO.output(TrigPin,GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(TrigPin,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin,GPIO.LOW)

    t3 = time.time()


    while not GPIO.input(EchoPin):
        t4 = time.time()
        if (t4 - t3) > 0.03 :
            return -1

    t1 = time.time()

    while GPIO.input(EchoPin):
        t5 = time.time()
        if(t5 - t1) > 0.03 :
            return -1

    t2 = time.time()
    time.sleep(0.01)
#    print "distance is %d " % (((t2 - t1)* 340 / 2) * 100)
    return ((t2 - t1)* 340 / 2) * 100

def Distance_test():
    num = 0
    ultrasonic = []
    while num < 5:
            distance = Distance()
            while int(distance) == -1 :
                distance = Distance()
                print("Tdistance is %f"%(distance) )
            while (int(distance) >= 500 or int(distance) == 0) :
                distance = Distance()
                print("Edistance is %f"%(distance) )
            ultrasonic.append(distance)
            num = num + 1
            time.sleep(0.01)
    print(ultrasonic)
    distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3])/3
    print("distance is %f"%(distance) ) 
    return distance
    
def servo_appointed_detection(pos):
    for i in range(18):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)   
        
def get_frame():
    global last_frame
    cap = cv2.VideoCapture(0)
    cap.set(3, 600)
    cap.set(4, 500)
    cap.set(5, 30)  # Set frame
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 10)  # Set brightness. Range: -64 to 64
    cap.set(cv2.CAP_PROP_CONTRAST, 15)  # Set contrast. Range: -64 to 64

    while True:
        ret,frame = cap.read()
        if not ret:
            break
        with frame_locker:
            last_frame = frame
    cap.release()

last_frame = None
frame_locker = threading.Lock()

try:
    init()
    grabThread = threading.Thread(target=get_frame)
    grabThread.start()

    while True:
        time.sleep(0.1)
        brake()

        with frame_locker:
            img = last_frame
        
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

        print(f'objects: L:{objects_left}, C:{objects_center}, R:{objects_right}')
    
        servo_appointed_detection(90)
        distance = Distance_test()
        if distance > 60 and objects_center == 0:
            run(35, 35)

        elif 40 <= distance and distance <= 60 and objects_center == 0:
            run(15, 15)

        elif distance < 40 or objects_center > 0:
            GPIO.output(LED_R, GPIO.HIGH)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.LOW)
            back(20, 20)
            time.sleep(0.08)
            brake()
            
            servo_appointed_detection(30)
            time.sleep(0.8)
            rightdistance = Distance_test()
          
            servo_appointed_detection(150)
            time.sleep(0.8)
            leftdistance = Distance_test()

            servo_appointed_detection(90)
            time.sleep(0.8)
            frontdistance = Distance_test()
         
            if leftdistance < 40 and rightdistance < 40 and frontdistance < 40:
                #Magenta
                GPIO.output(LED_R, GPIO.HIGH)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.HIGH)
                spin_right(45, 45)
                time.sleep(0.58)
            elif leftdistance >= rightdistance and objects_right > objects_left:
            #Blue
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.HIGH)
                spin_left(45, 45)
                time.sleep(0.28)
            elif leftdistance <= rightdistance and objects_left > objects_right:
            #Magenta
                GPIO.output(LED_R, GPIO.HIGH)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.HIGH)
                spin_right(45, 45)
                time.sleep(0.28)

            elif leftdistance >= rightdistance or objects_right > objects_left:
            #Blue
                back(50, 50)
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.HIGH)
                spin_left(45, 45)
                time.sleep(0.28)

            elif leftdistance <= rightdistance or objects_left > objects_right:
            #Magenta
                back(50, 50)
                GPIO.output(LED_R, GPIO.HIGH)
                GPIO.output(LED_G, GPIO.LOW)
                GPIO.output(LED_B, GPIO.HIGH)
                spin_right(45, 45)
                time.sleep(0.28)

except KeyboardInterrupt:
    pass


pwm_ENA.stop()
pwm_ENB.stop()
pwm_servo.stop()
GPIO.cleanup()

cv2.destroyAllWindows()