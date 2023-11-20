import cv2
import math
import time
import numpy as np
#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

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
    
def find_row():
    centerBool = True
    if turn_right:
        leftrightservo_appointed_detection(200) # Could be 180
    if not turn_right:
        leftrightservo_appointed_detection(0)
    while centerBool:
        ret, frame = image.read()
        centerBool  = valCheck([240, 10], frame)
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(30)
        pwm_ENB.ChangeDutyCycle(30)
        time.sleep(0.1)

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
def spin_left_find(r2, r2Pos):
    print("Spin left find")
    while r2:
        ret, frame = image.read()
        r2  = valCheck(r2Pos, frame)
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(30)
        pwm_ENB.ChangeDutyCycle(30)
        time.sleep(0.1)

#turn right in place
def spin_right_find(l2, l2Pos):
    print("Spin right find")
    while l2:
        ret, frame = image.read()
        l2  = valCheck(l2Pos, frame)
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(30)
        pwm_ENB.ChangeDutyCycle(30)
        time.sleep(0.1)

#turn right in place
def spin_90_left():
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

def valCheck(arr, frame):
    if any(frame[arr[1], arr[0]] < 50):
        return False
    else:
        return True
    
#The servo rotates left or right to the specified angle
def leftrightservo_appointed_detection(pos): 
    for i in range(1):   
        pwm_LeftRightServo.ChangeDutyCycle(2.5 + 10 * pos/180)
        time.sleep(0.02)                            
        #pwm_LeftRightServo.ChangeDutyCycle(0)  

detected_lines = [] # List to store detected lines
#The try/except statement is used to detect errors in the try block.
#the except statement catches the exception information and processes it.

while True:
    try:
        expectedRows = int(input('How many rows? (as an integer)'))
        if expectedRows <= 0:
            raise ValueError
        break
    except ValueError:
        print("Incorrect value entered")
#delay 2s   
time.sleep(2)

# Initialize a timer and lines variable
line_timer = time.time()
stop_signal = False  # Flag to indicate the stop signal
turn_right = True # What turn to take next

image = cv2.VideoCapture(0)

image.set(3, 600)
image.set(4, 500)
image.set(5, 30)  # Set frame
image.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
image.set(cv2.CAP_PROP_BRIGHTNESS, 40)  # Set brightness. Range: -64 to 64
image.set(cv2.CAP_PROP_CONTRAST, 40)  # Set contrast. Range: -64 to 64
image.set(cv2.CAP_PROP_EXPOSURE, 156)  # Set exposure. Range: 1 to 5000
count = 0


try:
    init()
    #key_scan()
    while not stop_signal:
        ret, frame = image.read()
        
        if ret:
            #When the black line is detected, the corresponding indicator of the tracking module is on, and the port level is LOW.
            #When the black line is not detected, the corresponding indicator of the tracking module is off, and the port level is HIGH.
            
            # Extract specific pixel values (adjust these coordinates accordingly)
            # Order ov values from left to right: left1 left2 right2 right1
            # 1 = outer
            # 2 = inner
            height, width = frame.shape[:2] # Webcam 640w x 480h
            left1Loc = (160, 470)
            left2Loc = (220, 470)
            right2Loc = (260, 470)
            right1Loc = (320, 470)
            
            #print("Position: ", left1Loc, left2Loc, right2Loc, right1Loc)

            # Draw circles at extracted pixel locations
            color = (0, 0, 255)
            cv2.circle(frame, left1Loc, 1, color, 1)
            cv2.circle(frame, left2Loc, 1, color, 1)
            cv2.circle(frame, right1Loc, 1, color, 1)
            cv2.circle(frame, right2Loc, 1, color, 1)
            
            left1LocValue = frame[left1Loc[0], left1Loc[1]]
            left2LocValue = frame[left2Loc[0], left2Loc[1]]
            right1LocValue = frame[right1Loc[0], right1Loc[1]]
            right2LocValue = frame[right2Loc[0], right2Loc[1]]
            
            print("Frame ", count)
            print("Value at locations: ", left1LocValue, left2LocValue, right2LocValue, right1LocValue)
            
            TrackSensorLeftValue1  = valCheck(left1Loc, frame)
            TrackSensorLeftValue2  = valCheck(left2Loc, frame)
            TrackSensorRightValue1 = valCheck(right1Loc, frame)
            TrackSensorRightValue2 = valCheck(right2Loc, frame)            
            
            # Display the frame
            cv2.imshow('Frame with Points', frame)
            #time.sleep(2)
            count += 1

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

            # end of line
            elif all([TrackSensorLeftValue1, TrackSensorLeftValue2, TrackSensorRightValue1, TrackSensorRightValue2]):
                # Add what row we just traversed
                detected_lines.append('row{}'.format(len(detected_lines) + 1))
                
                # End of field
                if len(detected_lines) >= expectedRows:
                    print('performing 180')
                    expectedRows = expectedRows * 2
                    spin_90_left() #spin_left_find(30,30)
                    spin_90_left()
                    turn_right = not turn_right
                    
                # Find row to right
                elif turn_right:
                    # Go forward a bit
                    run(30,30)
                    time.sleep(0.2)
                    
                    # Turn right
                    spin_right(30,30)
                    time.sleep(1.2)
                    
                    # Run until find line
                    find_row()
                    
                    # Go a bit past line
                    run(30,30)
                    time.sleep(0.2)
                    
                    # Spin right to find line again
                    print("Spin right find")
                    spin_right_find(TrackSensorLeftValue2, left2LocValue)
                    
                # Find row to left
                elif not turn_right:
                    # Go forward a bit
                    spin_left(30,30)
                    time.sleep(1.2)
                    
                    find_row()
                    run(30,30)
                    print("Spin left find")
                    time.sleep(0.2)
                    spin_left_find(TrackSensorRightValue2, right2LocValue)
                turn_right = not turn_right
            #When the level of 4 pins are 1 1 1 1 , the car keeps the previous running state.   
        # Exit the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    pass

GPIO.cleanup()

image.release()
cv2.destroyAllWindows()
