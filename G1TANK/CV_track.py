#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import cv2
import threading
import enum
import numpy as np

#Definition of motor pins
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13


#Initialize the servo angle
ServoLeftRightPos = 90


#Servo pin definition
ServoLeftRightPin = 11

#Set the GPIO port to BCM encoding mode.
GPIO.setmode(GPIO.BCM)

GPIO.setwarnings(False)


#All servo reset
def servo_init():
    servoflag = 0
    servoinitpos = 90
    if servoflag != servoinitpos:        
        leftrightservo_appointed_detection(servoinitpos)
        time.sleep(0.5)
        pwm_LeftRightServo.ChangeDutyCycle(0)   

#The servo rotates left or right to the specified angle
def leftrightservo_appointed_detection(pos): 
    for i in range(1):   
        pwm_LeftRightServo.ChangeDutyCycle(2.5 + 10 * pos/180)
        time.sleep(0.02)                            
        #pwm_LeftRightServo.ChangeDutyCycle(0)  

    
#servo stop
def servo_stop():
    pwm_LeftRightServo.ChangeDutyCycle(0)  

#Camera servo moving left
def servo_left():
    global ServoLeftRightPos
    pos = ServoLeftRightPos
    leftrightservo_appointed_detection(pos)
    #time.sleep(0.10)
    pos += 0.7
    ServoLeftRightPos = pos
    if ServoLeftRightPos >= 180:
        ServoLeftRightPos = 180

#Camera servo moving right
def servo_right():
    global ServoLeftRightPos
    pos = ServoLeftRightPos
    leftrightservo_appointed_detection(pos)
    #time.sleep(0.10)
    pos -= 0.7 
    ServoLeftRightPos = pos
    if ServoLeftRightPos <= 0:
        ServoLeftRightPos =  0


#Motor pins are initialized into output mode
def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_LeftRightServo
    GPIO.setup(ServoLeftRightPin, GPIO.OUT)
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    pwm_LeftRightServo = GPIO.PWM(ServoLeftRightPin, 50)
    pwm_LeftRightServo.start(0)
   #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
	
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

#turn right
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
    
# Function to stop the rover
def stop_rover():
    brake()
    time.sleep(1)  # Stop for 1 second
    line_timer = time.time()  # Reset the timer
    detected_lines.clear()  # Clear the list of detected lines
    
def mask_top_corners(image):
    height, width = image.shape[:2]
    mask = np.ones((height, width), dtype=np.uint8) * 255
    corner_width = int(0.4 * width)
    triangle1 = np.array([(0, 0), (corner_width, 0), (0, corner_width)])
    triangle2 = np.array([(width, 0), (width - corner_width, 0), (width, corner_width)])
    cv2.drawContours(mask, [triangle1], 0, 0, -1)
    cv2.drawContours(mask, [triangle2], 0, 0, -1)
    top_height = int(0.5 * height)
    cv2.rectangle(mask, (0, 0), (width, top_height), 0, -1)
    bottom_height = int(0.9 * height)
    cv2.rectangle(mask, (0, bottom_height), (width, height), 0, -1)
    return mask

def process_degs(degs,tryway,looking):
    val = int(degs/90 * 30)
    if val < 0:
        val = -val
    val = max(20,val)
    val = min(val,30)
    print('speed: ',val)
    if degs == 1000:
        if looking > 4:
            brake()
            time.sleep(0.5)
            global detected_lines
            global expectedRows
            global whichside
            if expectedRows <= len(detected_lines):
                print('turn around')
                spin_right(30,30)
                time.sleep(2.4)
                return tryway,0

            elif whichside == True:
                whichside = False
                r_o_l = 1000
                servo_init()
                print('Turning right')
                run(30,30)
                time.sleep(1.4)
                spin_right(30,30)
                time.sleep(1.2)
                leftrightservo_appointed_detection(47)
                run(30,30)
                time.sleep(0.6)
                brake()
                time.sleep(1)
                while r_o_l > 12 or r_o_l < -12:
                    r_o_l = process_contours()
                    if r_o_l < 30 and r_o_l > -30:
                        run(30,30)
                        time.sleep(0.05)
                        brake()
                        time.sleep(0.05)
                    else:
                        run(30,30)
                        time.sleep(0.2)
                        brake()
                        time.sleep(0.2)
                servo_init()
                spin_right(30,30)
                time.sleep(0.8)
                brake()
                return tryway,0
               
            elif whichside == False:
                whichside = True
                r_o_l = 1000
                servo_init()
                print('Turning left')
                run(30,30)
                time.sleep(1.4)
                spin_left(30,30)
                time.sleep(1.2)
                leftrightservo_appointed_detection(270)
                run(30,30)
                time.sleep(0.6)
                brake()
                time.sleep(1)
                while r_o_l > 12 or r_o_l < -12:
                    r_o_l = process_contours()
                    if r_o_l < 30 and r_o_l > -30:
                        run(30,30)
                        time.sleep(0.05)
                        brake()
                        time.sleep(0.05)
                    else:
                        run(30,30)
                        time.sleep(0.2)
                        brake()
                        time.sleep(0.2)
                servo_init()
                spin_left(30,30)
                time.sleep(1.1)
                brake()
                return tryway,0
        elif tryway == True:
            print('looking')
            looking +=1
            spin_right(30,30)
            time.sleep(0.05)
            return False, looking
        elif tryway == False:
            print('looking')
            looking +=1
            spin_left(30,30)
            time.sleep(0.05)
            return True, looking

    elif degs < 8 and degs > -8:
        run(val,val)
        time.sleep(0.12)
    elif degs < 0:
        spin_right(val,val)
        time.sleep(0.04)
    elif degs > 0:
        spin_left(val,val)
        time.sleep(0.04) 
    return tryway, 0

def process_contours():
    global last_frame, runner
    while runner:
        with frame_locker:
            frame = last_frame
        if frame is not None:
            image = last_frame
            height,width,_ = image.shape
            img = image
            image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            image = cv2.GaussianBlur(image, (7, 7), 0)
            _, image = cv2.threshold(image, 90, 255, cv2.THRESH_BINARY_INV)

            corner_mask = mask_top_corners(image) 
            final_mask = cv2.bitwise_and(image, corner_mask)
            kernel = np.ones((11, 11), np.uint8)
            mask_cleaned = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)
            image = cv2.morphologyEx(mask_cleaned, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            final_cont = 0
            for contour in contours:
                contour_area = cv2.contourArea(contour)
                if int(width*height*0.01) < contour_area < int(width*height*.4):
                    contourA = contour_area
                    final_cont = contour
                    break
            
            for contour in contours:
                contour_area = cv2.contourArea(contour)
                if int(width*height*0.01) < contour_area < int(width*height*.4):
                    if contour_area > contourA:
                        contourA = contour_area
                        final_cont = contour
            try:
                mask = np.zeros_like(image)
                cv2.drawContours(mask, [final_cont], -1, (255, 255, 255), thickness=cv2.FILLED)
                # Find the centroid of the contour
                M = cv2.moments(final_cont)
                if M["m00"] != 0:
                   cX = int(M["m10"] / M["m00"])
                   cY = int(M["m01"] / M["m00"])
                else:
                   cX, cY = 0, 0

                # Draw a circle at the centroid on the original image
                img_with_circle = cv2.circle(img.copy(), (cX, cY), 5, (0, 0, 255), -1)

                # Show the image with the contour and centroid
                # cv2.imshow("Image with path and center", img_with_circle)

                cv2.imshow('mask',mask)

                cv2.waitKey(1)
                # cv2.destroyAllWindows()

                r_o_l = int(width / 2) - cX
                r_o_l = r_o_l * 90 / width
                if r_o_l > 0:
                    print("Go Left degs:", r_o_l)
                elif r_o_l < 0:
                    print('Go Right degs:', r_o_l)
                else:
                    print("Go Straight")
            except:
                r_o_l = 1000
            return r_o_l
    return 1000


def get_frame():
    global last_frame, runner
    cap = cv2.VideoCapture(-1)

    cap.set(3,600)
    cap.set(4,500)
    cap.set(5,30) # Set frame
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 40) # Set brightness. Range: -64 to 64
    cap.set(cv2.CAP_PROP_CONTRAST, 40) # Set contrast. Range: -64 to 64
    while runner:
        ret,frame = cap.read()
        if not ret:
            break
        with frame_locker:
            last_frame = frame
    cap.release()

detected_lines = []
tryway = False 
whichside = True # True to turn Right, first then alternate after
looking = 0
last_frame = None
runner = True
how_do_you_feel = True
frame_locker = threading.Lock()
init()
servo_init()

# servo_left()
# servo_right()

expectedRows = input('How many rows? (as an integer)')
while how_do_you_feel == True:
    try:    
        grabThread = threading.Thread(target=get_frame)
        grabThread.start()
        r_o_l = process_contours()
        tryway, looking = process_degs(r_o_l, tryway, looking)
        cv2.imshow("Camera Feed", last_frame)
        brake()
        # time.sleep(1)
    except KeyboardInterrupt:
        how_do_you_feel = False
        runner = False

cv2.destroyAllWindows()
print detected_lines
pwm_ENA.stop()
pwm_ENB.stop()
pwm_LeftRightServo.stop()
GPIO.cleanup()  