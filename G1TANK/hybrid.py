#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import cv2
import threading
import enum
import numpy as np
"""
if turn_right:
        while all([TrackSensorLeftValue1, TrackSensorLeftValue2, TrackSensorRightValue1, TrackSensorRightValue2]):
            spin_right(50, 25)
            time.sleep(0.1)
            TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
            TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
            TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
            TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)
    elif not turn_right:
         while all([TrackSensorLeftValue1, TrackSensorLeftValue2, TrackSensorRightValue1, TrackSensorRightValue2]):
            spin_left(25,50)
            time.sleep(0.1)
            TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
            TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
            TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
            TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)
"""

#Definition of motor pins
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Definition of button
key = 8

#Initialize the servo angle
ServoLeftRightPos = 90

#Servo pin definition
ServoLeftRightPin = 11

#TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1  =  3   #The first tracking infrared sensor pin on the left is connected to  BCM port 3 of Raspberry pi
TrackSensorLeftPin2  =  5   #The second tracking infrared sensor pin on the left is connected to  BCM port 5 of Raspberry pi
TrackSensorRightPin1 =  4    #The first tracking infrared sensor pin on the right is connected to  BCM port 4 of Raspberry pi
TrackSensorRightPin2 =  18   #The second tracking infrared sensor pin on the right is connected to  BCMport 18 of Raspberry pi

#Set the GPIO port to BCM encoding mode.
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#All servo reset
def servo_init():
    servoflag = 0
    servoinitpos = 90
    if servoflag != servoinitpos:        
        leftrightservo_appointed_detection(servoinitpos)
        time.sleep(0.5)
        pwm_LeftRightServo.ChangeDutyCycle(0)   
        
#Motor pins are initialized into output mode
#Key pin is initialized into input mode
#Track sensor module pins are initialized into input mode
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
    GPIO.setup(key,GPIO.IN)
    GPIO.setup(TrackSensorLeftPin1,GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2,GPIO.IN)
    GPIO.setup(TrackSensorRightPin1,GPIO.IN)
    GPIO.setup(TrackSensorRightPin2,GPIO.IN)
   #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    
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


#advance
def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)
    
#advance on timer
def run_timer(leftspeed, rightspeed):
    TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
    TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
    TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
    TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)
    while all([TrackSensorLeftValue1, TrackSensorLeftValue2, TrackSensorRightValue1, TrackSensorRightValue2]):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(0.1)
        TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
        TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
        TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
        TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)

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

#turn left in place
def spin_left_find(leftspeed, rightspeed):
    print("Spin left find")
    TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)
    while TrackSensorRightValue2:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(0.1)
        TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)

#turn right in place
def spin_right_find(leftspeed, rightspeed):
    print("Spin right find")
    TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
    while TrackSensorLeftValue2:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(0.1)
        TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)

#turn right in place
def spin_180():
    print("Spin 180")
    TrackSensorLeftValue2 = GPIO.input(TrackSensorLeftPin2)
    TrackSensorLeftValue1 = GPIO.input(TrackSensorLeftPin1)
    while TrackSensorLeftValue2 and TrackSensorLeftValue1:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(50)
        pwm_ENB.ChangeDutyCycle(50)
        time.sleep(0.1)
        TrackSensorLeftValue2 = GPIO.input(TrackSensorLeftPin2)
        TrackSensorLeftValue1 = GPIO.input(TrackSensorLeftPin1)

#brake
def brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

#Button detection
def key_scan():
    while GPIO.input(key):
        pass
    while not GPIO.input(key):
        time.sleep(0.01)
        if not GPIO.input(key):
            time.sleep(0.01)
    while not GPIO.input(key):
        pass
    
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
            
            if turn_right:
                turn_right = False
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
               
            elif not turn_right:
                turn_right = True
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
    
def return_to_start(target_line):
    # Continue to the end of the current line
    run(50, 50)
    time.sleep(2)  # Adjust the time based on your rover's speed and environment

    # Perform a 180-degree turn
    spin_left(50, 50)
    time.sleep(2)  # Adjust the time based on your rover's speed and environment

    # Go back down the lines until the target line is reached
    for line in reversed(detected_lines):
        if line == target_line:
            resume_following = True
            break  # Stop when the target line is reached
        elif line is not None:
            if line.startswith('row'):
                # Handle the line based on your specific logic
                print("Processing line:",line)

        # Adjust the time based on your rover's speed and environment
        time.sleep(2)

    brake()


detected_lines = [] # List to store detected lines
#The try/except statement is used to detect errors in the try block.
#the except statement catches the exception information and processes it.

while True:
    try:
        expectedRows = int(input('Enter number of rows: '))
        if expectedRows <= 0:
            raise ValueError
        break
    except ValueError:
        print("Invalid value entered")
#delay 2s   
time.sleep(2)

# Initialize a timer and lines variable
line_timer = time.time()
stop_signal = False  # Flag to indicate the stop signal
turn_right = True # What turn to take next
detected_lines = []
tryway = False 
looking = 0
last_frame = None
runner = True
how_do_you_feel = True
frame_locker = threading.Lock()
init()
servo_init()
while True:
    try:
        expectedRows = int(input('Enter number of rows: '))
        if expectedRows <= 0:
            raise ValueError
        break
    except ValueError:
        print("Invalid value entered")

try:
    grabThread = threading.Thread(target=get_frame)
    grabThread.start()
    while not stop_signal:
        #When the black line is detected, the corresponding indicator of the tracking module is on, and the port level is LOW.
        #When the black line is not detected, the corresponding indicator of the tracking module is off, and the port level is HIGH.
        TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
        TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
        TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
        TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)

        if not all([TrackSensorLeftValue1, TrackSensorLeftValue2, TrackSensorRightValue1, TrackSensorRightValue2]):
            #4 tracking pins level status
            # 0 0 X 0
            # 1 0 X 0
            # 0 1 X 0
            #Turn right in place,speed is 100,delay 80ms
            #Handle right acute angle and right right angle
            if (TrackSensorLeftValue1 == False or TrackSensorLeftValue2 == False) and  TrackSensorRightValue2 == False:
                spin_right(35, 35)
                time.sleep(0.08)
    
            #4 tracking pins level status
            # 0 X 0 0       
            # 0 X 0 1 
            # 0 X 1 0       
            #Turn right in place,speed is 100,delay 80ms   
            #Handle left acute angle and left right angle 
            elif TrackSensorLeftValue1 == False and (TrackSensorRightValue1 == False or  TrackSensorRightValue2 == False):
                spin_left(35, 35)
                time.sleep(0.08)
    
            # 0 X X X
            #Left_sensor1 detected black line
            elif TrackSensorLeftValue1 == False:
                spin_left(35, 35)
        
            # X X X 0
            #Right_sensor2 detected black line
            elif TrackSensorRightValue2 == False:
                spin_right(35, 35)
    
            #4 tracking pins level status
            # X 0 1 X
            elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == True:
                left(0,30)
    
            #4 tracking pins level status
            # X 1 0 X  
            elif TrackSensorLeftValue2 == True and TrackSensorRightValue1 == False:
                right(30, 0)
    
            #4 tracking pins level status
            # X 0 0 X
            elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False:
                run(40, 40)
    
            # Check if the stop signal is received
            if GPIO.input(key) == 0:
                stop_signal = True
                stop_rover()
                break

        elif all([TrackSensorLeftValue1, TrackSensorLeftValue2, TrackSensorRightValue1, TrackSensorRightValue2]):
            detected_lines.append('row{}'.format(len(detected_lines) + 1))
            if len(detected_lines) >= expectedRows:
                print('performing 180')
                expectedRows = expectedRows * 2
                spin_180() #spin_left_find(30,30)
                spin_180()
                turn_right = not turn_right
            elif turn_right:
                r_o_l = process_contours()
                tryway, looking = process_degs(r_o_l, tryway, looking)
                cv2.imshow("Camera Feed", last_frame)
                spin_right(30,30)
                time.sleep(1.2)
                run_timer(30, 30)
                run(30,30)
                time.sleep(0.2)
                print("Spin right find")
                spin_right_find(40,35)
            elif not turn_right:
                r_o_l = process_contours()
                tryway, looking = process_degs(r_o_l, tryway, looking)
                cv2.imshow("Camera Feed", last_frame)
                spin_left(30,30)
                time.sleep(1.2)
                run_timer(30, 30)
                run(30,30)
                print("Spin left find")
                time.sleep(0.2)
                spin_left_find(35,40)
            turn_right = not turn_right
        #When the level of 4 pins are 1 1 1 1 , the car keeps the previous running state.     
except KeyboardInterrupt:
    pass

pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()

