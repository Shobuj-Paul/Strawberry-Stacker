#!/usr/bin/env python3

'''
This python file runs a ROS-node of name pick_n_place_aruco which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here (https://docs.px4.io/master/en/flight_modes/offboard.html) to understand more about offboard mode 
This node publishes and subscribes the following topics:

    Subscriptions                            Services to be called                                 Publications                                          				
    /mavros/state                            /mavros/cmd/arming                                    /mavros/setpoint_position/local                       
    /mavros/local_position/pose              /mavros/set_mode                                      /mavros/setpoint_velocity/cmd_vel                        
    /gripper_check                           /activate_gripper                                                                              
    /eDrone/camera/image_raw     
'''

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
            #self.id, self.corners = self.detect_aruco()
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

class PickNPlace:
    def __init__(self):
        pass

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting until the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly declare other service proxies 

    def setDisarm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)
   
    def offboard_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s"%e)

    def activate_gripper(self, activate):
        # Call /activate_gripper to activate the gripper and print fail message on failure
        rospy.wait_for_service('/activate_gripper')
        try:
            gripperService = rospy.ServiceProxy('/activate_gripper', Gripper)
            gripperService(activate)
        except rospy.ServiceException as e:
            print ("service gripper call failed: %s"%e)
    
    def land_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to LAND and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
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
    rospy.init_node('pick_n_place_aruco', anonymous=True)

    stateMt = stateMoniter()
    pick_n_place = PickNPlace()
    aruco = image_proc()

    #Initialize publishers
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    #Specify rate
    rate = rospy.Rate(10) # 10hz

    #Make a list of setpoints
    setpoints = [[0.0, 0.0, 3.0], [9.0, 0.0, 3.0], [9.0, 0.0,0.0], [9.0, 0.0, 3.0], [0.0, 0.0, 3.0], [0.0, 0.0, 0.0]]

    #Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    temp_pos = PoseStamped()
    temp_pos.pose.position.x = 0
    temp_pos.pose.position.y = 0
    temp_pos.pose.position.z = 0

    #Set velocity here
    vel = TwistStamped()
    vel.twist.linear.x = 0.05
    vel.twist.linear.y = 0.05
    vel.twist.linear.z = 0.01

    vel_img = TwistStamped()
    vel_img.twist.linear.y = 0.0
    vel_img.twist.linear.x = 0.0
    vel_img.twist.linear.z = 0.0

    #Initialise subscribers
    rospy.Subscriber('mavros/state', State, stateMt.stateCb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, stateMt.posCb)
    rospy.Subscriber('/gripper_check', String, stateMt.gripperCb)
    rospy.Subscriber('/eDrone/camera/image_raw', Image, aruco.image_callback) #Subscribing to the camera topic

    # Send some dummy setpoints before starting offboard mode
    for _ in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()

    # Switching the state to offboard mode
    while not stateMt.state.mode=="OFFBOARD":
        pick_n_place.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")

    # Arming the drone
    while not stateMt.state.armed:
        pick_n_place.setArm()
        rate.sleep()
    print("Armed!!")

    #Set distance allowance 
    allowance = 0.2
    k_vel = 0.001

    # Publish the setpoints 
    while not rospy.is_shutdown():
        for i in range(len(setpoints)):
            pos.pose.position.x = setpoints[i][0]
            pos.pose.position.y = setpoints[i][1]
            pos.pose.position.z = setpoints[i][2]
            while not ((stateMt.pos.pose.position.x - setpoints[i][0])**2 + (stateMt.pos.pose.position.y - setpoints[i][1])**2 + (stateMt.pos.pose.position.z - setpoints[i][2])**2)**0.5 < allowance:
                local_pos_pub.publish(pos)
                local_vel_pub.publish(vel)
                rate.sleep()
                if not aruco.id == None:
                    while not ((aruco.x/100 - 2)**2 + (aruco.y/100 - 2)**2)**0.5 < 0.25:
                        vel_img.twist.linear.x = k_vel*(2.0 - aruco.x/100)
                        vel_img.twist.linear.y = k_vel*(2.0 - aruco.y/100)
                        vel_img.twist.linear.z = 0.0
                        local_vel_pub.publish(vel_img)
                        rate.sleep()
                    while not stateMt.state.mode=='AUTO.LAND':
                        pick_n_place.land_set_mode()
                        rate.sleep()
                    for _ in range(100):
                        if stateMt.gripper == 'True':
                            pick_n_place.activate_gripper(True)
                            print("Gripper Activated")
                        rate.sleep()
                    for _ in range(100):
                        local_pos_pub.publish(temp_pos)
                        rate.sleep()
                    while not stateMt.state.mode=='OFFBOARD':
                        pick_n_place.offboard_set_mode()
                        rate.sleep()
            if i==3:
                while not stateMt.state.mode=='AUTO.LAND':
                    pick_n_place.land_set_mode()
                    rate.sleep()
                for _ in range(100):
                    if stateMt.gripper == 'True':
                        pick_n_place.activate_gripper(False)
                        print("Gripper Deactivated")
                    rate.sleep()
                for _ in range(100):
                    local_pos_pub.publish(temp_pos)
                    rate.sleep()
                while not stateMt.state.mode=='OFFBOARD':
                    pick_n_place.offboard_set_mode()
                    rate.sleep()
            print("Reached setpoint")
            
        #Landing
        pick_n_place.land_set_mode()
        #Disarm the drone
        while stateMt.state.armed:
            pick_n_place.setDisarm()
            rate.sleep()
        print("Disarmed!!")
        break

if __name__=='__main__':
    try: main()
    except rospy.ROSInterruptException: pass