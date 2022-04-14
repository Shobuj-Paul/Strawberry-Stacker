#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
from std_msgs.msg import String
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import *

class image_proc:
    def __init__(self):
        self.img = np.empty([]) # This will contain your image frame from camera
        self.bridge = CvBridge() #cv_bridge object
        self.id, self.x, self.y, self.yaw = 0.0, 0.0, 0.0, 0.0
        self.corners = np.empty([])

    # Callback function of camera topic
    def image_callback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
            self.id, self.x, self.y, self.yaw = self.calculate_pose()
        except CvBridgeError as e:
            print(e)
            return

    def detect_aruco(self):
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, idx, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        if idx != None:
            idx = idx[0][0]
            corners = corners[0][0]
        return idx, corners

    def calculate_pose(self):
        idx, corners = self.detect_aruco()
        if not len(corners)==0:
            top_left = corners[0]
            top_right = corners[1]
            bottom_right = corners[2]
            bottom_left = corners[3]
            top_centre = (top_right+top_left)/2
            center = (top_left+top_right+bottom_right+bottom_left)/4
            height, width, _ = self.img.shape
            ref = np.array([[0,height], [width,height]])
            angle_ref = math.degrees(math.atan2((ref[1][1]-ref[0][1]),(ref[1][0]-ref[0][0])))
            angle_prime = math.degrees(math.atan2((top_centre[1]-center[1]), top_centre[0]-center[0]))
            angle = angle_ref - angle_prime
            if angle<0:
                angle = 360 - abs(angle)
            elif angle == 360:
                angle = 0
            angle = round(angle, 2)
            return idx, center[0], center[1], angle
        else:
            return idx, 0.0, 0.0, 0.0

class edrone:
    def __init__(self, num):
        self.num = num
        if self.num == 1:
            self.drone = 'edrone1/'
        elif self.num == 0:
            self.drone = 'edrone0/'

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service(self.drone + 'mavros/cmd/arming')  # Waiting until the service starts 
        try:
            armService = rospy.ServiceProxy(self.drone + 'mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly declare other service proxies 

    def setDisarm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service(self.drone + 'mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(self.drone + 'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)
   
    def offboard_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD and print fail message on failure
        rospy.wait_for_service(self.drone + 'mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.drone + 'mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s"%e)

    def activate_gripper(self, activate):
        # Call /activate_gripper to activate the gripper and print fail message on failure
        rospy.wait_for_service(self.drone + 'activate_gripper')
        try:
            gripperService = rospy.ServiceProxy(self.drone + 'activate_gripper', Gripper)
            gripperService(activate)
        except rospy.ServiceException as e:
            print ("service gripper call failed: %s"%e)
    
    def land_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to LAND and print fail message on failure
        rospy.wait_for_service(self.drone + 'mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.drone + 'mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s"%e)

class stateMoniter:
    def __init__(self):
        # Instantiate a setpoints message
        self.state = State()
        self.pos = PoseStamped()
        self.gripper = String()
        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    def posCb(self, msg):
        # Callback function for topic /mavros/local_position/pose
        self.pos = msg
    
    def gripperCb(self, msg):
        # Callback function for topic /gripper_check
        self.gripper = msg.data

def main():
    rospy.init_node('multidrone')

    stateMt1 = stateMoniter()
    stateMt2 = stateMoniter()
    edrone1 = edrone(1)
    edrone2 = edrone(2)
    aruco1 = image_proc()
    aruco2 = image_proc()

    #Initialise subscribers
    rospy.Subscriber('/'+ edrone1.drone +'mavros/state', State, stateMt1.stateCb)
    rospy.Subscriber('/'+ edrone1.drone +'mavros/local_position/pose', PoseStamped, stateMt1.posCb)
    rospy.Subscriber('/'+ edrone1.drone +'gripper_check', String, stateMt1.gripperCb)
    rospy.Subscriber('/'+ edrone1.drone +'/camera/image_raw', Image, aruco1.image_callback) #Subscribing to the camera topic

    rospy.Subscriber('/'+ edrone2.drone +'mavros/state', State, stateMt2.stateCb)
    rospy.Subscriber('/'+ edrone2.drone +'mavros/local_position/pose', PoseStamped, stateMt2.posCb)
    rospy.Subscriber('/'+ edrone2.drone +'gripper_check', String, stateMt2.gripperCb)
    rospy.Subscriber('/'+ edrone2.drone +'camera/image_raw', Image, aruco2.image_callback) #Subscribing to the camera topic


if __name__=='__main__':
    try: main()
    except rospy.ROSInterruptException: pass