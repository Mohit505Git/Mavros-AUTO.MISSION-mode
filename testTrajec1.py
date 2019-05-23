#!/usr/bin/env python
# ROS python API
import rospy
import math
import time
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller():
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 3.0
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP
		
        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 3.0)

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

	# Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y
        self.sp.yaw_rate = 0.5
    
    def cntTakeOff(self,height):
        self.sp.position.x = 0
        self.sp.position.y = 0
        self.sp.position.z = height

    def goto(self,gotoX,gotoY,gotoZ,gotoYaw):    
        self.sp.position.x = gotoX
        self.sp.position.y = gotoY
        self.sp.position.z = gotoZ
        self.sp.yaw = gotoYaw
        # error is evaluated to check if the position is reached or not
        error = math.sqrt(pow(self.local_pos.x - gotoX,2) + pow(self.local_pos.y - gotoY,2) + pow(self.local_pos.z - gotoZ,2))
        if error <0.15: 
            self.dstFlag = 1 # succesfully reached the desired state
        else:
            self.dstFlag = 0


    def x_dir(self):
    	self.sp.position.x = self.local_pos.x + dVal
    	self.sp.position.y = self.local_pos.y

    def neg_x_dir(self):
    	self.sp.position.x = self.local_pos.x - 5
    	self.sp.position.y = self.local_pos.y

    def y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y + 5

    def neg_y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y - 5


# Main function
def main():
    print "Welcome to VTOL flight TASK1"
    print "Let's plan the mission __________ Fill the following information to start the sub-mission"
    h=raw_input("Enter takeoff Height:-")    


    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1


    # activate OFFBOARD mode
    modes.setOffboardMode()

    x1,y1,z1 = 2,9,4 #waypoint1
    x2,y2,z2 = 7,2,4 #waypoint2

    while not rospy.is_shutdown():
        cnt.goto(0,0,int(h) ,0)
        
        sp_pub.publish(cnt.sp)
        rate.sleep()

        if cnt.dstFlag:
            print "TakeOff Succesful"
            break

    while not rospy.is_shutdown():
        cnt.goto(x1,y1,z1,0)
        
        sp_pub.publish(cnt.sp)
        rate.sleep()

        if cnt.dstFlag:
            print "succesfully reached",(x1,y1,z1)
            break

    while not rospy.is_shutdown():
        cnt.goto(x2,y2,z2,0 )
        
        sp_pub.publish(cnt.sp)
        rate.sleep()

        if cnt.dstFlag:
            print "succesfully reached",(x2,y2,z2)
            break

    # ROS main loop
    while not rospy.is_shutdown():
    	# cnt.updateSp()

        cnt.goto(0,0,0,0)

    	sp_pub.publish(cnt.sp)
    	rate.sleep()
        if cnt.dstFlag:
            print "sub-mission complete. RTL"
            break


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass