def manual():
    import RPi.GPIO as GPIO
    import time
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

    u_input = 'what even is materials engineering'

    print('Camera setting up...')        
    image = cv2.VideoCapture(-1)

    image.set(3, 600)
    image.set(4, 500)
    image.set(5, 30)  # Set frame
    image.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    image.set(cv2.CAP_PROP_BRIGHTNESS, 20)  # Set brightness. Range: -64 to 64
    image.set(cv2.CAP_PROP_CONTRAST, 20)  # Set contrast. Range: -64 to 64
    imgcnt = 0
    print('')
    print('')
    try:
        n = 'n'
        y = 'y'
        w = 'w'
        s = 's'
        d = 'd'
        a = 'a'
        x = 'x'
        init()
      
        while u_input != "Q":
            ret, frame = image.read()
            if ret:
                frame = cv2.resize(frame,(450,300))
                imgcnt +=1
                cv2.imshow("Live View",frame)
            print('')
            print("Commands:")
            print("w: Forward")
            print("d: Right Spin")
            print("a: Left Spin")
            print("s: Back")
            print("x: save image")
            print("Q: Quit")
            print("else: Stop")

           
            u_input = input("Please enter a command: ")
            print("You entered:", u_input)
            if u_input == 'w':
                run(5,5)
            elif u_input == 'd':
                spin_right(8,8)
            elif u_input == 'a':
                spin_left(8,8)
            elif u_input == 's':
                back(7,7)  
            elif u_input == 'x':
                cv2.imwrite('/home/ubuntu/images/img_'+str(imgcnt)+'.png',frame)
                brake()
            else:
                brake()

            cv2.waitKey(1)
           
    except KeyboardInterrupt:
        pass
    pwm_ENA.stop()
    pwm_ENB.stop()
    GPIO.cleanup()
    image.release()
    cv2.destroyAllWindows()

manual()