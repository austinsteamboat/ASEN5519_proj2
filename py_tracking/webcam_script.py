import cv2
import numpy as np
def nothing(x):
    pass
cap = cv2.VideoCapture(0)
# Create a black image, a window
img = np.zeros((300,300,3), np.uint8)
cv2.namedWindow('image')
cv2.createTrackbar('H_Low','image',0,255,nothing)
cv2.createTrackbar('S_Low','image',0,255,nothing)
cv2.createTrackbar('V_Low','image',0,255,nothing)
cv2.createTrackbar('H_High','image',0,255,nothing)
cv2.createTrackbar('S_High','image',0,255,nothing)
cv2.createTrackbar('V_High','image',0,255,nothing)
kernel = np.ones((5,5),np.uint8)
while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # create trackbars for color change
    
    hl = cv2.getTrackbarPos('H_Low','image')
    sl = cv2.getTrackbarPos('S_Low','image')
    vl = cv2.getTrackbarPos('V_Low','image')
    hh = cv2.getTrackbarPos('H_High','image')
    sh = cv2.getTrackbarPos('S_High','image')
    vh = cv2.getTrackbarPos('V_High','image')
    lower_blue = np.array([hl,sl,vl])
    upper_blue = np.array([hh,sh,vh])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_erode = cv2.erode(mask,kernel,iterations = 5)
    mask_dilate = cv2.dilate(mask_erode,kernel,iterations = 1)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    cv2.imshow('frame',hsv)
    cv2.imshow('mask',mask_dilate)
    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
