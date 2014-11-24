import curses
import cv2
import numpy as np


def nothing(x):
    pass
  
def curses_print(string, line, col):
	"""
	Function to do a simple curses print.
	"""

	#Check for bad inputs
	if col > 1 or col < 0:
		return

	if line > 22 or line < 0:
		return

	#Print to screen using curses
	if col == 0:
		screen.addstr(line, 0, string)
	if col == 1:
		screen.addstr(line, 40, string)

	screen.refresh()



cap = cv2.VideoCapture(0)

cv2.namedWindow('res')
"""
cv2.createTrackbar('H_lower','res',157,255, nothing)
cv2.createTrackbar('S_lower','res',149,255,nothing)
cv2.createTrackbar('V_lower','res',67,255,nothing)
cv2.createTrackbar('H_upper','res',186,255,nothing)
cv2.createTrackbar('S_upper','res',255,255,nothing)
cv2.createTrackbar('V_upper','res',255,255,nothing)
"""
H_lower = 157
S_lower = 149
V_lower = 67
    
H_upper = 186
S_upper = 255
V_upper = 255

#Setup the screen for curses
screen = curses.initscr()
screen.clear()
screen.refresh()

while(True):
    # Capture frame-by-frame
    
    ret, frame = cap.read()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    """
    H_lower = cv2.getTrackbarPos('H_lower','res',)
    S_lower = cv2.getTrackbarPos('S_lower','res')
    V_lower = cv2.getTrackbarPos('V_lower','res')
    
    H_upper = cv2.getTrackbarPos('H_upper','res')
    S_upper = cv2.getTrackbarPos('S_upper','res')
    V_upper = cv2.getTrackbarPos('V_upper','res')
    """
    lower = np.array([H_lower, S_lower, V_lower], dtype=np.uint8)
    upper = np.array([H_upper,S_upper,V_upper], dtype=np.uint8)

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower, upper)
    
    kernel_open = np.ones((4,4),np.uint8)
    kernel_close = np.ones((25,25),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
    
    cv2.imshow('mask',mask)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
        
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    try:
        cnt = contours[0]
    
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
    
        moment = cv2.moments(cnt)
        cx = int(moment['m10']/moment['m00'])
        cy = int(moment['m01']/moment['m00'])
        
        min_con_x = min(x[0][0] for x in cnt)
        min_con_y = min(y[0][1] for y in cnt)
        max_con_x = max(x[0][0] for x in cnt)
        max_con_y = max(y[0][1] for y in cnt)
        
        out_of_bounds = False
        if min_con_x == 1 or min_con_y == 1 or max_con_x == 638 or max_con_y == 478:
			out_of_bounds = True


    except:
        area = None
        perimeter = None
        cx = None
        cy = None
        min_con_x = None
        min_con_y = None
        max_con_x = None
        max_con_y = None
        out_of_bounds = None
    
    #print len(contours)
    screen.clear()
    screen.refresh()
	
    curses_print("Image information:",0,0)
    curses_print("Area of contour:      " + str(area),2,0)
    curses_print("Perimeter of contour: " + str(perimeter),3,0)
    curses_print("Number of objects:    " + str(len(contours)),5,0)
    curses_print("Center X:             " + str(cx),7,0)
    curses_print("Center Y:             " + str(cy),8,0)
    curses_print("Min Contour X:        " + str(min_con_x),10,0)
    curses_print("Min Contour Y:        " + str(min_con_y),11,0)
    curses_print("Max Contour X:        " + str(max_con_x),12,0)
    curses_print("Max Contour Y:        " + str(max_con_y),13,0)
    curses_print("Out of bounds:        " + str(out_of_bounds),14,0)
    
    cv2.imshow('frame',frame)
    cv2.imshow('res',res)

    # Display the resulting frame
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
