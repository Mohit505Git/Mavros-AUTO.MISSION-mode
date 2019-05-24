#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class Modes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def auto_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            # setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.set_mode.request.custom_mode)
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="AUTO.MISSION")
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e

    def wpPush(self,wps):
        rospy.wait_for_service('mavros/mission/push')
        try:
            wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush,persistent=True)
            wpPushService(start_index=0,waypoints=wps)
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e


class wpMissionCnt:
    
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        self.wp = Waypoint()


    def stateCb(self, msg):
        self.state = msg

    def setWaypoints1(self):
        self.wp.frame ="FRAME_LOCAL_NED"
        self.wp.command = "NAV_TAKEOFF"
        self.wp.is_current= True
        self.wp.autocontinue = True
        # self.wp.param1=
        # self.wp.param2=
        # self.wp.param3=
        # self.wp.param4=
        self.wp.x_lat= 20
        self.wp.y_long= 20 
        self.wp.z_alt= 3
        return self.wp

    def setWaypoints2(self):
        self.wp.frame ="FRAME_LOCAL_NED"
        self.wp.command = "NAV_RETURN_TO_LAUNCH"
        self.wp.is_current= False
        self.wp.autocontinue = True
        # self.wp.param1=
        # self.wp.param2=
        # self.wp.param3=
        # self.wp.param4=
        self.wp.x_lat= 0
        self.wp.y_long= 0 
        self.wp.z_alt= 0
        return self.wp


def main():
    rospy.init_node('waypointMission', anonymous=True)
    rate = rospy.Rate(20.0)

    md = Modes()
    wayp = wpMissionCnt()
    wps = []
    wps.append(wayp.setWaypoints1())
    # wps.append(wayp.setWaypoints2())
    md.wpPush(wps)
    rospy.Subscriber("/mavros/state",State, wayp.stateCb)


    while not wayp.state.armed:
        md.setArm()
        rate.sleep()
    md.auto_set_mode()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass