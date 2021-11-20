#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name waypoint_Mission which sends the waypoints to drone
in mission mode.
This node publishes and subscribes the following topics:

    Services to be called         Subscriptions				
    /mavros/cmd/arming            /mavros/state
    /mavros/set_mode
    /mavros/mission/push
    /mavros/mission/pull
'''

import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class Modes:
    def __init__(self):
        pass

# Calling the rosservices

    # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting until the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

   
    def auto_set_mode(self):

        # Call /mavros/set_mode to set the mode the drone to AUTO.MISSION
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.MISSION')
        except rospy.ServiceException as e:
            print ("Service set_mode call failed: %s"%e)
    
    def wpPush(self,index,wps):
        # Call /mavros/mission/push to push the waypoints
        # and print fail message on failure
        rospy.wait_for_service('mavros/mission/push')
        try:
            waypointService = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            waypointService(start_index=index, waypoints=wps)
        except rospy.ServiceException as e:
            print ("Service waypoint push call failed: %s"%e)
    
    def wpPull(self):
        # Call /mavros/mission/pull to pull the waypoints
        # and print fail message on failure
        rospy.wait_for_service('mavros/mission/pull')
        try:
            waypointService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
            waypointService()
        except rospy.ServiceException as e:
            print ("Service waypoint pull call failed: %s"%e)
   
class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.pos = PoseStamped()
        self.pos.pose.position.x = 0
        self.pos.pose.position.y = 0
        self.pos.pose.position.z = 2

        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

class wpMissionCnt:

    def __init__(self):
        self.wp =Waypoint()
        
    def setWaypoints(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
        self.wp.frame =frame #  FRAME_GLOBAL_REL_ALT = 3 for more visit http://docs.ros.org/api/mavros_msgs/html/msg/Waypoint.html
        self.wp.command = command #VTOL TAKEOFF = 84,NAV_WAYPOINT = 16, TAKE_OFF=22 for checking out other parameters go to https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/CommandCode.msg'''
        self.wp.is_current= is_current
        self.wp.autocontinue = autocontinue # enable taking and following upcoming waypoints automatically 
        self.wp.param1=param1 # To know more about these params, visit https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
        self.wp.param2=param2
        self.wp.param3=param3
        self.wp.param4=param4
        self.wp.x_lat= x_lat 
        self.wp.y_long=y_long
        self.wp.z_alt= z_alt #relative altitude.

        return self.wp


def main():

    rospy.init_node('waypoint_Mission', anonymous=True) #Initialise rosnode
    rate = rospy.Rate(20.0)

    stateMt = stateMoniter()
    md = Modes()
    
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    wayp0 = wpMissionCnt()
    wayp1 = wpMissionCnt()
    wayp2 = wpMissionCnt()
    wayp3 = wpMissionCnt()
    wayp4 = wpMissionCnt()
    # Add more waypoints here

    
    wps = [] #List to store waypoints
    
    w = wayp0.setWaypoints(3,22,True,True,0.0,0.0,0.0,float('nan'),19.134641,72.911706,10)
    wps.append(w)

    w = wayp1.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),19.134617,72.911886,10)
    wps.append(w)

    w = wayp2.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),19.134434,72.911817,10)
    wps.append(w)

    w = wayp3.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),19.134423,72.911763,10)
    wps.append(w)

    w = wayp4.setWaypoints(3,21,False,True,0.0,0.0,0.0,float('nan'),19.134423,72.911763,0)
    wps.append(w)

    print (wps)
    md.wpPush(0,wps)


    # Arming the drone
    while not stateMt.state.armed:
        md.setArm()
        rate.sleep()
        print("ARM!!")

    # Switching the state to auto mode
    while not stateMt.state.mode=="AUTO.MISSION":
        md.auto_set_mode()
        rate.sleep()
        print ("AUTO.MISSION")

    # Pulling the waypoints from the drone
    md.wpPull()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass