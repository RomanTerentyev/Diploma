#!/usr/bin/python
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist

def nothing(x):
    pass

rospy.init_node('mto')

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
msg = Twist()

pt = 0
mmin = np.array([255, 255, 255])
mmax = np.array([0, 0, 0])
fl = 0

# mouse callback function
def masking(event,x,y,flags,param):
    global pt,mmin,mmax
    if event == cv2.EVENT_LBUTTONDBLCLK:
        for g in range(3):
	    for h in range(3):
		pt = (imgtr[y+g-1,x+h-1])
		for i in range(3):
		    if pt[i] < mmin[i]:
			mmin[i] = pt[i]
		    if pt[i] > mmax[i]:
			mmax[i] = pt[i]

cap = cv2.VideoCapture(1)
cv2.namedWindow('imagetrans')
cv2.setMouseCallback('imagetrans',masking)

while(1):
    timer = cv2.getTickCount()
    flag, img = cap.read()
    #imgtr = img
    #imgtr = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    imgtr = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    imgtr = cv2.GaussianBlur(imgtr,(5,5),0)
    mask = cv2.inRange(imgtr, mmin, mmax)

    # morphology    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # contours
    _,contours,hierarchy = cv2.findContours(mask,1,1)
    if fl == 1:
	try:
	    max_area = 0
	    pos = 0
	    for i in contours:
		area = cv2.contourArea(i)
		if area > max_area:
			max_area = area
			pos = i
	    hull = cv2.convexHull(pos)
	    cv2.drawContours(img,[hull],-1,(0,0,0),2)
	    moments = cv2.moments(pos, 1)
	    dM01 = moments['m01']
	    dM10 = moments['m10']
	    dArea = moments['m00']
	    #print(dArea)
	    cx = int(dM10 / dArea)
	    cy = int(dM01 / dArea)
	    cv2.circle(img, (cx, cy), 5, (0,0,0), -1)
	    
	    '''if dArea < 4000:
		msg.linear.x = 0
		msg.angular.z = 0.1'''
	    if dArea > 50000:
		msg.linear.x = 0
		#msg.angular.z = 0
	    elif cx<290:
		msg.angular.z = 0.1
		#msg.linear.x = 0
	    elif cx>350:
		msg.angular.z = -0.1
		#msg.linear.x = 0
	    elif 290<cx<350:
		msg.angular.z = 0
		msg.linear.x = 0.01
    
	    pub.publish(msg)
	except:
	    print('OpenCV_Error')
	    continue

    fps = int(cv2.getTickFrequency() / (cv2.getTickCount() - timer))
    cv2.putText(imgtr, "FPS : " + str(fps), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,0), 2)
    cv2.imshow('image',img)
    cv2.imshow('imagetrans',imgtr)
    cv2.imshow('mask',mask)

    k = cv2.waitKey(1) & 0xFF
    if k == 32:
	fl = 1
    elif k == ord('r'):
	fl = 0
	mmin = np.array([255, 255, 255])
	mmax = np.array([0, 0, 0])
    elif k == 27:
	break

cap.release()
cv2.destroyAllWindows()
