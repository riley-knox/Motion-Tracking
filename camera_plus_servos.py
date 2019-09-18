#import python3 print function
from __future__ import print_function
#import necessary libraries
import serial
import numpy as np
import cv2
import time

#open serial port that closes after 10 seconds
serial = serial.Serial('/dev/ttyACM0',timeout = 10)

#bit codes representing servo channels 0 and 2 on the Maestro board (use 0 and 2 to give header plugs more space)
#first channel is left/right angle ("pan"); second channel is up/down angle ("tilt")
servoChannels = ['\x00','\x02']

#servo constants for calculating angle position from microsecond position; [m,b] format
servoConstants = [[10.04, 592],[9.69, 496]]

#bitmask codes for finding least and most significant bit
#127 = 00000001111111; 16256 = 11111110000000
bitmask = [127,16256]

#set threshold limits for green ball
#limits found using script downloaded from: https://github.com/opencv/opencv/blob/3.4/samples/cpp/tutorial_code/ImgProc/Threshold_inRange.cpp
H_range = (40, 60)
S_range = (40, 160)
V_range = (80, 255)

#define function to obtain servo angular positions
def getPosition():
    angles = []                         #create empty list to hold angle values
    for i in range(len(servoChannels)):     #obtain angle measurement for 'i' servos
        serial.write('\x90'+servoChannels[i])   #send serial command to retrieve positions
        LSB = ord(serial.read(1))       #obtain LSB as numerical equivalent of read ASCII character
        MSB = ord(serial.read(1))       #obtain MSB as numerical equivalent of read ASCII character
        MSB = MSB << 8                  #bitshift MSB 8 spots left
        pos_bit = MSB|LSB               #bitwise MSB and LSB addition; returns position as quarter-microsecond measurement
        pos = pos_bit/4                 #convert position to microsecond
        angle = (pos - servoConstants[i][1])/servoConstants[i][0]   #convert microsecond position to angle position
        angles.append(angle)            #add servo 'i' angular measurement to list of angles
    return angles                       #return angle list as output variable

#define function converting entered angle to hexadecimal bits
def angletohex(angle,m,b):
    position = b + m*angle         #determine servo position on microsecond scale
    sec = int(round(4*position))        #calculate quarter-microsecond value
    #generate 7-bit least significant bit and most significant bit from 14-bit number
    LSB = sec & bitmask[0]
    MSB = sec & bitmask[1]
    MSB = MSB >> 7                      #bitshift the MSB seven bits to the right (i.e. remove the 7 trailing zeros)
    #generate hexadecimal bits from 7-bit binary strings
    MSBhex = chr(MSB)
    LSBhex = chr(LSB)
    bitList = [LSBhex,MSBhex]           #create data list containing LSB and MSB
    return bitList                      #output bit data list

#establish angle-per-frame max value
anglePerFrame = 5

#initialize video capture
capture = cv2.VideoCapture(2)

#window height & width
windowWidth = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
windowHeight = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

#window center
widthCenter = windowWidth/2
heightCenter = windowHeight/2

while True:
    #designate exit command
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    #determine angular positions
    livePosition = getPosition()
    panPosition = livePosition[0]
    tiltPosition = livePosition[1]

    #read captured image for display
    ret, frame = capture.read()

    #convert image from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #thresholding
    frame_threshold = cv2.inRange(hsv, (H_range[0], S_range[0], V_range[0]), (H_range[1], S_range[1], V_range[1]))

    #Gaussian blur
    blur = cv2.GaussianBlur(frame_threshold,(5,5),0)

    #erosion/dilation
    img_erosion = cv2.erode(blur, np.ones((5,5)), iterations = 1)
    img_dilation = cv2.dilate(img_erosion, np.ones((3,3)), iterations = 5)

    #find contours in blurred image
    image, contours, hierarchy = cv2.findContours(blur,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

    #isolate largest contour
    contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
    biggest_contour = max(contours, key = cv2.contourArea)

    #find smallest circle enclosing largest contour
    (x,y), radius = cv2.minEnclosingCircle(biggest_contour)
    centerX = int(x)
    centerY = int(y)
    radius = int(radius)

    #draw circle on webcam image
    circleImg = cv2.circle(frame, (centerX,centerY), radius, (255,0,0), 2)

    #draw lines indicating center of circle
    cv2.line(circleImg,(centerX,0),(centerX,480),(0,0,255),2)
    cv2.line(circleImg,(0,centerY),(640,centerY),(0,0,255),2)

    #show image with tracking circle
    cv2.imshow('Tracking',circleImg)

    #determine how to move the camera to line up image center with ball center
    #horizontal assessment first
    if centerX > widthCenter:               #if the ball center is away from the frame center
        directionX = 'left'                 #set direction to move
    elif centerX < widthCenter:
        directionX = 'right'
    else:                                   #if not away from the center
        directionX = 'none'                 #don't move
    #vertical assessment next
    if centerY > heightCenter:
        directionY = 'up'
    elif centerY < heightCenter:
        directionY = 'down'
    else:
        directionY = 'none'

    #establish proportional angle control
    #angle changes by smaller amount as the ball center gets closer to the frame center
    percentX = ((abs(centerX-widthCenter))/widthCenter)**2
    percentY = ((abs(centerY-heightCenter))/heightCenter)**2
    proportionalX = anglePerFrame*percentX
    proportionalY = anglePerFrame*percentY

    #change "pan" angle by proportional amount in the necessary direction
    if directionX == 'left':
        panPosition += proportionalX
    elif directionX == 'right':
        panPosition -= proportionalX
    else:
        pass

    #change "tilt" angle by proportional amount in the necessary direction
    if directionY == 'up':
        tiltPosition -= proportionalY
    elif directionY == 'down':
        tiltPosition += proportionalY
    else:
        pass

    #generate hexadecimal MSB and LSB for the "pan" and "tilt" angles
    panBits = angletohex(panPosition,servoConstants[0][0],servoConstants[0][1])
    tiltBits = angletohex(tiltPosition,servoConstants[1][0],servoConstants[1][1])

    #empty list to hold bits for each angle
    bitStrings = []

    #add pan and tilt angle bits to angle bit data list
    bitStrings.append(['\x84',servoChannels[0],panBits[0],panBits[1]])
    bitStrings.append(['\x84',servoChannels[1],tiltBits[0],tiltBits[1]])

    #activate servo in pan dimension
    serial.write(bitStrings[0])
    #activate servo in tilt dimension
    serial.write(bitStrings[1])

#end video capture
capture.release()
cv2.destroyAllWindows()
