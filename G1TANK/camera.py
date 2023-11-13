import cv2
import threading
import time
import enum

def bgr8_to_jpeg(value, quality=75):
    return bytes(cv2.imencode('.jpg', value)[1])

# Specifies which camera to use, can be found through the terminal command:
# ls /dev/
image = cv2.VideoCapture(0)
# Can also do:
# cap = cv2.VideoCapture("[videoPath]")
# This allows us to use a video instead of a camera

# width = 1280
# height = 960
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

image.set(3,600)
image.set(4,500)
image.set(5,30) # Set frame
image.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
image.set(cv2.CAP_PROP_BRIGHTNESS, 40) # Set brightness. Range: -64 to 64
image.set(cv2.CAP_PROP_CONTRAST, 40) # Set contrast. Range: -64 to 64
image.set(cv2.CAP_PROP_EXPOSURE, 156) # Set exposure. Range: 1 to 5000

if image.isOpened():
    ret, frame = image.read() # Read camera data
else:
    ret = False
    
while ret:
    try:
        cv2.imshow("Camera Feed", frame)
        ret, frame = image.read()
        c = cv2.waitKey(10)
        #time.sleep(1)
    except KeyboardInterrupt:
       break 
   
image.release() # After using object, we need to release it
cv2.destroyAllWindows()
