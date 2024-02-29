#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

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

#The try/except statement is used to detect errors in the try block.
#the except statement catches the exception information and processes it.
try:
    n = 'n'
    y = 'y'
    w = 'w'
    s = 's'
    d = 'd'
    a = 'a'
    x = 'x'
    init()
  
    while True:
        print("Commands:")
        print("s: Stop")
        print("w: Forward")
        print("d: Right")
        print("a: Left")
        print("x: Back")

       
        u_input = input("Please enter a command: ")
        print("You entered:", u_input)
        if u_input == 'w':
            run(70,70)
        elif u_input == 'd':
            right(50,50)
        elif u_input == 'a':
            left(50,50)
        elif u_input == 's':
            brake()
        elif u_input == 'x':
            back(50,50)  
       
except KeyboardInterrupt:
    pass
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()

