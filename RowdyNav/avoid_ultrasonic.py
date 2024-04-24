def avoid():
    #-*- coding:UTF-8 -*-
    import RPi.GPIO as GPIO
    import time
    import signal

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
        distance = (ultrasonic[1] + ultrasonic[2]+ ultrasonic[3])/3

        print("distance is %f"%(distance) ) 
        return distance

    def Distance_test_b():
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
        distance1 = ultrasonic[1] 

        distance2 = ultrasonic[2]  

        distance3 = ultrasonic[3]

        print("distance is %f"%((distance1+distance2+distance3)/3) ) 
        return distance1, distance2, distance3
        
    #The servo rotates to the specified angle
    def servo_appointed_detection(pos):
        for i in range(18):
            pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)   
            

    def servo_color_carstate():
      #red
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
        elif leftdistance >= rightdistance:
        #Blue
            GPIO.output(LED_R, GPIO.LOW)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.HIGH)
            spin_left(45, 45)
            time.sleep(0.28)
        elif leftdistance <= rightdistance:
        #Magenta
            GPIO.output(LED_R, GPIO.HIGH)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.HIGH)
            spin_right(45, 45)
            time.sleep(0.28)
            
    time.sleep(2)

    def t_h(signum,frame):
        raise TimeoutError('ttt')

    time.sleep(2)
    U_input = 'nothin'
    signal.signal(signal.SIGALRM,t_h)
    close = 0
    print("Enter 'Q' to end.")
    try:
        init()
        while U_input != 'Q':
            # signal.alarm(1)
            # try:
            #     U_input = input()
            #     signal.alarm(0)
            # except:
            #     U_input = '747'

            servo_appointed_detection(90)
            distance11,distance22,distance33 = 0,0,0
            for xx in range(1,4):
                distance1,distance2,distance3 = Distance_test_b()
                distance11 += distance1
                distance22 += distance2
                distance33 += distance3

            distance11 /=xx
            distance22 /=xx
            distance33 /=xx
            print(f"{distance11}, {distance22}, {distance33}")

            # if distance11 < 45 or distance22<45 or distance33 < 45:
            #     close +=1

            if distance11 > 42 and distance22 > 42 and distance33 > 42: #and close<3:
            # if close < 3:
                run(30, 30)
            # elif 42 <= distance <= 60:
                # run(15, 15)
            # elif distance < 42:
            elif distance11 < 30 or distance22 < 30 or distance33 < 30: 
                back(30,30)
                time.sleep(0.2)
                servo_color_carstate()
            else:    
                close =0
                servo_color_carstate()
           
    except KeyboardInterrupt:
        pass
    pwm_ENA.stop()
    pwm_ENB.stop()
    pwm_servo.stop()
    GPIO.cleanup()
