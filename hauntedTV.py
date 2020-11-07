import cv2
import time
import numpy as np
import sys
import RPi.GPIO as GPIO

#timers
timeTV = 5
timeGhost = 25
timeNoGhost = 10


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(12, GPIO.IN)# Remote Button B - Take picture
GPIO.setup(16, GPIO.IN)# Remote button D - Armed & Ready

GPIO.setup(22, GPIO.OUT)# RED
GPIO.setup(24, GPIO.OUT)# GREEN
GPIO.setup(26, GPIO.OUT)# BLUE

GPIO.setup(32, GPIO.OUT)# Relay - TV on/off



#If you want to use a custom background you can use this function to resize it.
def resize(dst,img):
    width = img.shape[1]
    height = img.shape[0]
    dim = (width, height)
    resized = cv2.resize(dst, dim, interpolation = cv2.INTER_AREA)
    return resized


presenceTriggered = False
cooldown = False #cooldown state after the haunt is over
ghostVisible = False #toggles the visiblity of the ghost

GPIO.output(32, GPIO.HIGH) #Turn TV on at the beginning for setup mode.


window_name = "window"
face_cascade = cv2.CascadeClassifier('/home/pi/Documents/haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

#inialize fullscreen
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

#capture video from the camera if you want to test this on your computer, change 0 with 1
cam = cv2.VideoCapture(0)

#custom resolution for CRT TV
cam.set(3,768)
cam.set(4,576)

success, ref_img = cam.read() #reference image taken for background removal
success2, bg = cam.read()
success3, bg2 = cam.read()
flag = 0


#star the timers
coolDownStart = time.time() #this will reset after the haunt is done.
faceTimeStart = time.time() #the time it takes to detect a new face.
ghostTime = time.time()
faceFreqCounter = 0 #face frequency counter to filter out false positives

while True:

    ret, img = cam.read()
    if ret:
        img=cv2.flip(img,0) #flips the image vertically
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detected = face_cascade.detectMultiScale(gray, 1.3, 5)
        #erase the '#' below if you want to detect eyes instead of faces
        #detected = eye_cascade.detectMultiScale(gray)

        #If the setup is armed, execute facedetection.
        if flag==3:
            if not cooldown: #the cooldown timer prevents the facedetection from running
                for (x,y,w,h) in detected:
                    #draw rectangles around faces/eyes for debugging
                    #cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

                    faceTimeEnd = time.time()
                    elapsedFaceTime = faceTimeEnd - faceTimeStart

                    if (elapsedFaceTime<1):
                        faceFreqCounter+=1
                        print(faceFreqCounter)

                    if (elapsedFaceTime>5):
                        faceFreqCounter=0

                    faceTimeStart = time.time()

                faceDetectionElapsed=time.time()- faceTimeStart

                if(faceFreqCounter>timeTV) & (not ghostVisible) & (not presenceTriggered):
                    GPIO.output(32, GPIO.HIGH)
                if(faceFreqCounter>timeGhost) & (faceDetectionElapsed>0.5) & (not ghostVisible):

                    ghostVisible = True
                    faceFreqCounter=0
                    bg = bgHaunted

                if(ghostVisible & (faceFreqCounter>timeNoGhost) & (faceDetectionElapsed>0.5)) & (not presenceTriggered):
                    print("NO GHOST")
                    ghostTime = time.time()
                    ghostVisible = False
                    faceFreqCounter=0
                    presenceTriggered = True
                    bg=bg2

            end2 = time.time()
            elapsedTime2 = end2 - ghostTime
            print(elapsedTime2)
            if (elapsedTime2>5) and presenceTriggered:
                GPIO.output(32, GPIO.LOW)
                presenceTriggered = False
                cooldown = True
            if (elapsedTime2>20) and cooldown:
                print("cooldown done")
                faceFreqCounter=0
                cooldown = False


        #background removal based on https://github.com/misbah4064/backgroundRemoval
        #success, img = cam.read()

        if flag==0:
                ref_img = img
                flag=1

        if flag==2:
                success3, bgHaunted = cam.read()
                flag=1
        # create a mask
        diff1=cv2.subtract(img,ref_img)
        diff2=cv2.subtract(ref_img,img)
        diff = diff1+diff2
        #orig value =13
        diff[abs(diff)<20.0]=0
        gray = cv2.cvtColor(diff.astype(np.uint8), cv2.COLOR_BGR2GRAY)
        gray[np.abs(gray) < 10] = 0
        fgmask = gray.astype(np.uint8)
        fgmask[fgmask>0]=255
        #invert the mask
        fgmask_inv = cv2.bitwise_not(fgmask)
        #use the masks to extract the relevant parts from FG and BG
        fgimg = cv2.bitwise_and(img,img,mask = fgmask)
        bgimg = cv2.bitwise_and(bg,bg,mask = fgmask_inv)
        #combine both the BG and the FG images
        dst = cv2.add(fgimg,bgimg)
        dst=cv2.flip(dst, 0)
        cv2.imshow(window_name,dst)




        key = cv2.waitKey(5) & 0xFF
        if ord('q') == key:
                break
        if ord('a') == key or (GPIO.input(16) == GPIO.HIGH):
                flag = 3
                print("Setup armed and ready!")
                GPIO.output(24, GPIO.LOW)
                GPIO.output(22, GPIO.HIGH)
                GPIO.output(32, GPIO.LOW)

        if (ord('h') == key) or (GPIO.input(12) == GPIO.HIGH):
                flag = 2
                print("Haunted Picture Taken!")
                GPIO.output(24, GPIO.HIGH)
                GPIO.output(22, GPIO.LOW)
        elif ord('d') == key:
                flag = 1
                print("Background Captured")
        elif ord('r') == key:
                flag = 0
                print("Ready to Capture new Background")
        elif ord('t') == key:
                bg = bgHaunted
                print("Image is now haunted!")


GPIO.cleanup()
cv2.destroyAllWindows()
cam.release()
