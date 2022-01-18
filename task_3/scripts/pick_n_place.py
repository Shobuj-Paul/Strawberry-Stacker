#!/usr/bin/env python

'''
This python file runs a ROS-node of name pick_n_place which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here (https://docs.px4.io/master/en/flight_modes/offboard.html) to understand more about offboard mode 
This node publishes and subscribes the following topics:

    Services to be called                    Publications                                          Subscriptions				
    /mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
    /activate_gripper                                                                              /gripper_check
    
'''

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_ros_link_attacher.srv import *

class PickNPlace:
    def __init__(self):
        rospy.init_node('pick_n_place', anonymous=True)

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
        self.state = State()
        self.pos = PoseStamped()
        self.gripper = String()
        # Instantiate a setpoints message
        
    def stateCb(self, msg):
        self.state = msg
        # Callback function for topic /mavros/state

    # Create more callback functions for other subscribers
    def posCb(self, msg):
        # Callback function for topic /mavros/local_position/pose
        self.pos = msg
    
    def gripperCb(self, msg):
        # Callback function for topic /gripper_check
        self.gripper = msg.data

def main():
    stateMt = stateMoniter()
    pick_n_place = PickNPlace()

    #Initialize publishers
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    #Specify rate
    rate = rospy.Rate(10) # 10hz

    #Make a list of setpoints
    setpoints = [[0.0, 0.0, 3.0], [3.0, 0.0, 3.0], [3.0, 0.0,0.0], [3.0, 0.0, 3.0], [3.0, 3.0, 3.0], [3.0, 3.0, 0.0], [3.0, 3.0, 3.0], [0.0, 0.0, 3.0], [0.0, 0.0, 0.0]]

    #Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    temp_pose = PoseStamped()
    temp_pose.pose.position.x = 0
    temp_pose.pose.position.y = 0
    temp_pose.pose.position.z = 0

    #Set velocity here
    vel = TwistStamped()
    vel.twist.linear.x = 0.1
    vel.twist.linear.y = 0.1
    vel.twist.linear.z = 0.05

    #Initialise subscribers
    rospy.Subscriber('mavros/state', State, stateMt.stateCb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, stateMt.posCb)
    rospy.Subscriber('/gripper_check', String, stateMt.gripperCb)

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
            print("Reached setpoint")
            if i == 2:
                while not stateMt.state.mode == 'AUTO.LAND':
                    local_pos_pub.publish(pos)
                    if ((stateMt.pos.pose.position.x - setpoints[i][0])**2 + (stateMt.pos.pose.position.y - setpoints[i][1])**2)**0.5 < 0.05:
                        pick_n_place.land_set_mode()
                    rate.sleep()
                for _ in range(100):
                    if stateMt.gripper == "True":             
                        pick_n_place.activate_gripper(True)
                    rate.sleep()
                print("Gripper activated")
                for _ in range(100):
                    local_pos_pub.publish(temp_pose)
                    rate.sleep()
                while not stateMt.state.mode=="OFFBOARD":
                    pick_n_place.offboard_set_mode()
                    rate.sleep()
            elif i == 5:
                while not stateMt.state.mode == 'AUTO.LAND':
                    local_pos_pub.publish(pos)
                    if ((stateMt.pos.pose.position.x - setpoints[i][0])**2 + (stateMt.pos.pose.position.y - setpoints[i][1])**2)**0.5 < 0.05:
                        pick_n_place.land_set_mode()
                    rate.sleep()
                for _ in range(100):
                    if stateMt.gripper == "True":
                        pick_n_place.activate_gripper(False)
                    rate.sleep()
                print("Gripper deactivated")
                for _ in range(100):
                    local_pos_pub.publish(temp_pose)
                    rate.sleep()
                while not stateMt.state.mode=="OFFBOARD":
                    pick_n_place.offboard_set_mode()
                    rate.sleep()
        #Landing
        pick_n_place.land_set_mode()
        #Disarm the drone
        while stateMt.state.armed:
            pick_n_place.setDisarm()
            rate.sleep()
        print("Disarmed!!")
        break

if __name__=='__main__':
    main()