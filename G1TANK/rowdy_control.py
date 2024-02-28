#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import pyrealsense2 as rs
import numpy as np
import cv2

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
GPIO.setwarnings(True)

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

def get_ply_pointcloud(name='test_ply'):
    pc = rs.pointcloud()
    points = rs.points()

    pipe = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth)
    pipe.start(config)
    colorizer = rs.colorizer()

    try:
        frames = pipe.wait_for_frames()
        colorized = colorizer.process(frames)
        ply = rs.save_to_ply(f"{name}.ply")
        ply.set_option(rs.save_to_ply.option_ply_binary, False)
        ply.set_option(rs.save_to_ply.option_ply_normals, True)
        print(f"Saving to {name}.ply...")
        ply.process(colorized)
        print("Done")
    finally:
        pipe.stop()


# Configure depth and color streams
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
    print("Requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
ContinueProg = True

n = 'n'
y = 'y'
w = 'w'
s = 's'
d = 'd'
a = 'a'
x = 'x'
i = 'i'
b = 'b'
init()
print('Wait a moment... Camera view loading....')

cnt = 0
while ContinueProg:
    try:
        cnt +=1
    
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        resized_color_image = cv2.resize(color_image, dsize=(600, 300), interpolation=cv2.INTER_AREA)
        resized_depth_colormap = cv2.resize(depth_colormap, dsize=(600, 300), interpolation=cv2.INTER_AREA)
        images = np.hstack((resized_color_image, resized_depth_colormap))
        
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

        print("Commands:")
        print("x: Stop")
        print("w: Forward")
        print("d: Right")
        print("a: Left")
        print("s: Back")
        print("c: Stop and save image")
        print("b: Stop and save depth")
        print("p: Stop and save PLY cloud")

        u_input = input("Please enter a command: ")
        print("You entered:", u_input)
        if u_input == 'w':
            run(70,70)
        elif u_input == 'd':
            right(50,50)
        elif u_input == 'a':
            left(50,50)
        elif u_input == 'x':
            brake()
        elif u_input == 's':
            back(50,50)  
        elif u_input == 'c':
            brake() 
            cv2.imwrite(resized_color_image,f'/color_{cnt}.png')
        elif u_input == 'b':
            brake()
            cv2.imwrite(resized_depth_image,f'/depth_{cnt}.png')
        elif u_input == 'p':
            brake()
            time.sleep(1)
            get_ply_pointcloud(f'point_{cnt}')
        else:
            print('No command: '+str(u_input))
            print('try again...')
            brake() 

    except KeyboardInterrupt:
        ContinueProg = False

pipeline.stop()

pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()