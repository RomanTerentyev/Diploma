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

x = np.array([0, 0, 0])
y = np.array([0, 0, 0])
fl = 0

img = cv2.imread('/home/administry/catkin_ws/src/diploma/scrabble.jpeg', cv2.IMREAD_GRAYSCALE) # queryiamge
cap = cv2.VideoCapture(1)

# Features
#Feat = cv2.xfeatures2d.SIFT_create()
#Feat = cv2.xfeatures2d.SURF_create(300)
Feat = cv2.ORB_create(nfeatures=1500)
kp_image, desc_image = Feat.detectAndCompute(img, None)

# Feature matching
#Matcher = cv2.BFMatcher()
Matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

while True:
    timer = cv2.getTickCount()
    _, frame = cap.read()
    grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # trainimage

    kp_grayframe, desc_grayframe = Feat.detectAndCompute(grayframe, None)
    matches = Matcher.knnMatch(desc_image, desc_grayframe, k=2)

    good_points = []
    for m, n in matches:
        if m.distance < 0.6*n.distance:
            good_points.append(m)

    img3 = cv2.drawMatches(img, kp_image, grayframe, kp_grayframe, good_points, grayframe)

    # Homography
    fps = int(cv2.getTickFrequency() / (cv2.getTickCount() - timer))
    cv2.putText(grayframe, "FPS : " + str(fps), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,0), 2)
    if len(good_points) > 9:
        query_pts = np.float32([kp_image[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
        train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)

        matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)

        # Perspective transform
        h, w = img.shape
        pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
        try:
	    dst = np.int32(cv2.perspectiveTransform(pts, matrix))
	    for i in range(3):
		x[i] = dst[i][0][0]
		y[i] = dst[i][0][1]
	    a = ((x[1]-x[0])**2 + (y[1]-y[0])**2)**0.5
	    b = ((x[1]-x[2])**2 + (y[1]-y[2])**2)**0.5
	    dArea = a*b
	    #print(dArea)
	    cx = int((x[0]+x[2])/2)
	    cy = int((y[0]+y[2])/2)
	    cv2.circle(grayframe, (cx, cy), 5, (0,0,0), -1)
	    
	    if fl == 1:
		'''if dArea < 4000:
		    msg.linear.x = 0
		    msg.angular.z = 0.05'''
		if dArea > 50000:
		    msg.linear.x = 0
		   # msg.angular.z = 0
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
	    print('TransformError')
	    continue

        homography = cv2.polylines(grayframe, [dst], True, (0, 0, 0), 2)
        cv2.imshow("Homography", homography)
    else:
        cv2.imshow("Homography", grayframe)

    #cv2.imshow("img3", img3)

    key = cv2.waitKey(1)
    if key == 32:
	fl = 1
    elif key == 27:
        break

cap.release()
cv2.destroyAllWindows()