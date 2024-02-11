#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import cv2
import threading
import enum
import numpy as np
import pyrealsense2 as rs

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
    
def make_points(image, line):
    slope, intercept = line
    y1 = int(image.shape[0])
    y2 = int(y1*3/5)         
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return [[x1, y1, x2, y2]]

def average_slope_intercept(image, lines):
    left_fit    = []
    right_fit   = []
    if lines is None:
        return None
    for line in lines:
        for x1, y1, x2, y2 in line:
            fit = np.polyfit((x1,x2), (y1,y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0: 
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
    left_fit_average  = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)
    left_line  = make_points(image, left_fit_average)
    right_line = make_points(image, right_fit_average)
    averaged_lines = [left_line, right_line]
    return averaged_lines

def display_lines(img,lines):
    line_image = np.zeros_like(img)
    lnr = 0
    lns = 0
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image,(x1,y1),(x2,y2),(255,255,255),10)
                lnr += 1
                lns += (x1+x2)/2

    h,w = line_image.shape[:2]

    cv2.line(line_image,(int(w/2),y1),(int(w/2),y2),(0,0,255),5)
    cv2.line(line_image,(int(lns/lnr),y1),(int(lns/lnr),y2),(255,0,0),5)

    return line_image, w/2,lns/lnr

def region_of_interest(canny):
    height = canny.shape[0]
    width = canny.shape[1]
    mask = np.zeros_like(canny)
    triangle = np.array([[
    (int(0.0*width), int(height)),
    (int(width/2), 0),
    (int(1.0*width), int(height)),]], np.int32)
    cv2.fillPoly(mask, triangle, 255)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image

# def region_of_interest(canny):
#     height = canny.shape[0]
#     width = canny.shape[1]
#     print(width)
#     print(height)
#     mask = np.zeros_like(canny)

#     triangle = np.array([[
#     (int((200*width)/1280), height),
#     (int((550*width)/1280), int(250*height/720)),
#     (int((1100*width)/1280), height),]], np.int32)

#     cv2.fillPoly(mask, triangle, 255)
#     masked_image = cv2.bitwise_and(canny, mask)
#     return masked_image

def test_lines(cropped_canny,lnthrsh,frame):
    lines = cv2.HoughLinesP(cropped_canny, 2, np.pi/180, lnthrsh, np.array([]), minLineLength=40,maxLineGap=50)
    return average_slope_intercept(frame, lines)

def process_degs(degs):
    val = int(degs/90 * 30)
    if val < 0:
        val = -val
    val = max(20,val)
    val = min(val,40)
    print('speed: ',val)
    if degs < 8 and degs > -8:
        run(val,val)
        time.sleep(0.12)
    elif degs < 0:
        spin_right(val,val)
        time.sleep(0.04)
    elif degs > 0:
        spin_left(val,val)
        time.sleep(0.04) 

def process_contours():
    global last_frame, runner
    while runner:
        with frame_locker:
            frame = last_frame
        if frame is not None:
            good = 0
            cnt += 1
            img = cv2.resize(frame,(600,300))
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            kernel = 5
            blur = cv2.GaussianBlur(gray,(kernel, kernel),0)
            canny = cv2.Canny(gray, 50, 150)
            # cv2.imshow('canny', canny)
            # cv2.imwrite(f"C:/Users/apant/Documents/Capstone/LaneVid/cannyimg_{str(cnt).zfill(8)}.png",canny)
            cropped_canny = region_of_interest(canny)
            lnthrsh = 80
            width = cropped_canny.shape[1]
            while good ==0 and lnthrsh > 10:
                try:
                    average_lines = test_lines(cropped_canny,lnthrsh, img)
                    line_image, midx,avgX = display_lines(img, average_lines)
                    good = 1
                except:
                    lnthrsh -= 7
                    good = 0
                    print('retry')

            deg = (avgX - midx) * 90 / (midx*2)
            if deg > 90:
                deg = deg % 90
            elif deg < -90:
                deg = deg % -90

            # combo_image = cv2.addWeighted(img, 0.8, line_image, 1, 1)
            # cv2.imshow("result", combo_image)
            # cv2.imwrite(f"C:/Users/apant/Documents/Capstone/LaneVid/comboimg_{str(cnt).zfill(8)}.png",combo_image)

            print(f'Degs: {deg}')
            return deg
        else:
            print('No Frame Received...')
            return 0

def get_frame():
    global last_frame, runner
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while runner:

            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color_image = np.asanyarray(color_frame.get_data())
            # cv2.imshow('RealSense', color_image)
            # cv2.waitKey(1)
            with frame_locker:
                last_frame = color_image

    finally:
        pipeline.stop()

last_frame = None
runner = True
how_do_you_feel = True
frame_locker = threading.Lock()
init()
servo_init()

# servo_left()
# servo_right()

grabThread = threading.Thread(target=get_frame)
grabThread.start()

while how_do_you_feel:
    try:    
        r_o_l = process_contours()
        process_degs(r_o_l)
        cv2.imshow("Camera Feed", last_frame)
        brake()
        time.sleep(1)
    except KeyboardInterrupt:
        how_do_you_feel = False
        runner = False

cv2.destroyAllWindows()
print(detected_lines)
pwm_ENA.stop()
pwm_ENB.stop()
pwm_LeftRightServo.stop()
GPIO.cleanup()  