#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

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

#Motor pins are initialized into output mode
#Key pin is initialized into input mode
#Track sensor module pins are initialized into input mode
def init():
    global pwm_ENA
    global pwm_ENB
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
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
def lineFollow():
    expectedRows = input('How many rows? (as an integer)')
    #delay 2s   
    time.sleep(2)

    # Initialize a timer and lines variable
    line_timer = time.time()
    global detected_lines
    stop_signal = False  # Flag to indicate the stop signal
    turn_right = True # What turn to take next


    try:
        init()
        #key_scan()
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
                    spin_right(30,30)
                    time.sleep(1.2)
                    run_timer(30, 30)
                    run(30,30)
                    time.sleep(0.2)
                    print("Spin right find")
                    spin_right_find(40,35)
                elif not turn_right:
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

lineFollow()
print detected_lines
#pwm_ENA.stop()
#pwm_ENB.stop()
GPIO.cleanup()

