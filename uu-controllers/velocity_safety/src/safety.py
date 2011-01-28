#!/usr/bin/env python
import roslib; roslib.load_manifest('velocity_safety')
import sys

from sensor_msgs.msg import JointState

import rospy
import numpy as np
import math

#
#   Add in a /emergency subscriber to cut out
#
rospy.loginfo("Starting velocity safety node")
currentJointStates = JointState()
pub = rospy.Publisher('/schunk/target_vc/joint_states', JointState) # publish if ok on this topic


def callbackJointStates(data):
    thr = 0.5
    global currentJointStates
    currentJointStates = data
    #checkit
    ok = True
    i=-1
    for jointvel in data.velocity:
        i=i+1
        if math.fabs(jointvel)>0.8:
            ok=False
	if math.isnan(jointvel):
	    ok = False
	#if jointvel > thr:
	#	joint_vel = thr
	#elif jointvel < -thr:
	#	joint_vel = -thr
	#ok = True
    if not ok:
	rospy.loginfo("Unsafe velocity, cutting to zero")
        #print "=========="
        #print "Not OK - publish zeroes"
        #print "=========="
#        data.velocity[:] = 0
        data.velocity = [0.0]*len(data.velocity)
#        for jointvel in data.velocity:
#            jointvel=0
#    print data
    else:
	print "BUTB: ", data.velocity
	
    pub.publish(data)


def safetynode():
    rospy.init_node('velocity_safety')
    rospy.Subscriber("/schunk/target_vel_safe/joint_states", JointState, callbackJointStates)
       
    print "Ready"
    
    rospy.spin()

if __name__ == "__main__":
    safetynode()


    
    
    
