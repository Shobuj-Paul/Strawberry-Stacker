#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215,167]], dtype=float32)}

	Detected_ArUco_markers = {}
	## enter your code here ##
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
	parameters = aruco.DetectorParameters_create()
	#These (x,y)-coordinates represent the top-left, top-right, bottom-right, and bottom-left corners of the ArUco tag
	corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
	if ids is not None:
		ids = ids.flatten()
		Detected_ArUco_markers = dict(zip(ids,corners))
		for key in Detected_ArUco_markers:
			temp = []
			Detected_ArUco_markers[key] = Detected_ArUco_markers[key].tolist()
			for i in range(0,4):
				Detected_ArUco_markers[key][0][i] = list(map(int,Detected_ArUco_markers[key][0][i]))
				temp.append(Detected_ArUco_markers[key][0][i])
			Detected_ArUco_markers[key] = temp
			Detected_ArUco_markers[key] = np.array(Detected_ArUco_markers[key])
	return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}

	ArUco_marker_angles = {}
	## enter your code here ##
	angles = []
	for key in Detected_ArUco_markers:
		top_left = Detected_ArUco_markers[key][0]
		top_right = Detected_ArUco_markers[key][1]
		bottom_right = Detected_ArUco_markers[key][2]
		bottom_left = Detected_ArUco_markers[key][3]
		center = np.array([int((top_left[0]+bottom_left[0]+bottom_right[0]+top_right[0])/4), int((top_left[1]+bottom_left[1]+bottom_right[1]+top_right[1])/4)])
		top_centre = (top_left + top_right)/2
		ref = np.array([[0,0], [640,0]])
		angle_ref = int(math.degrees(math.atan2((ref[1][1]-ref[0][1]),(ref[1][0]-ref[0][0]))))
		angle_prime = int(math.degrees(math.atan2((top_centre[1]-center[1]), top_centre[0]-center[0])))
		angle = angle_ref - angle_prime
		if angle<0:
			angle = 360 - abs(angle)
			angles.append(angle)
		elif angle == 360:
			angle = 0
			angles.append(angle)
		else:
			angles.append((angle))
	ids = Detected_ArUco_markers.keys()
	ArUco_marker_angles = dict(zip(ids, angles))
	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement	
	
	## enter your code here ##
	for key in Detected_ArUco_markers:
		top_left = Detected_ArUco_markers[key][0]
		top_right = Detected_ArUco_markers[key][1]
		bottom_right = Detected_ArUco_markers[key][2]
		bottom_left = Detected_ArUco_markers[key][3]
		center = np.array([int((top_left[0]+bottom_left[0]+bottom_right[0]+top_right[0])/4), int((top_left[1]+bottom_left[1]+bottom_right[1]+top_right[1])/4)])
		top_centre = list(map(int, (top_left + top_right)/2))
		img = cv2.circle(img, (center[0],center[1]), 5, (0,0,255), -1)
		img = cv2.circle(img, (top_left[0],top_left[1]), 5, (125,125,125), -1)
		img = cv2.circle(img, (bottom_left[0],bottom_left[1]), 5, (255,255,255), -1)
		img = cv2.circle(img, (bottom_right[0],bottom_right[1]), 5, (180,105,255), -1)
		img = cv2.circle(img, (top_right[0],top_right[1]), 5, (0,255,0), -1)
		img = cv2.line(img, (center[0],center[1]), (top_centre[0],top_centre[1]), (255,0,0), 3)
		text = "%d" % ArUco_marker_angles[key]
		idx = "%d" % key
		img = cv2.putText(img, text, (center[0]-80,center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2, cv2.LINE_AA)
		img = cv2.putText(img, idx, (center[0]+20,center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2, cv2.LINE_AA)
	return img
