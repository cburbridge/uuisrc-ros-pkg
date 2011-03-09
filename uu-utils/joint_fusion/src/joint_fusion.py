#! /usr/bin/env python

import roslib
roslib.load_manifest("joint_fusion")
import rospy
from sensor_msgs.msg import JointState

joint_1 = None
joint_2 = None

def source1_cb(data):
    global joint_1
    joint_1 = data

def source2_cb(data):
    global joint_2
    joint_2 = data

if __name__ == "__main__":
    rospy.init_node("joint_fusion")
    rospy.loginfo("Joint Fusion started")
    
    hz = rospy.get_param("rate", 10)
    rospy.Subscriber("source1", JointState, source1_cb)
    rospy.Subscriber("source2", JointState, source2_cb)
    
    pub_comb = rospy.Publisher("joint_states", JointState)
    
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():        
        rate.sleep()
        if joint_1 is None:
            rospy.logdebug("Waiting for the first source")
            continue
        if joint_2 is None:
            rospy.logdebug("Waiting for the second source")
            continue
        
#        time_to_use = joint_1.header.stamp
        time_to_use = rospy.Time.now()
        
        #combined
        msg = JointState()
        msg.header.stamp = time_to_use
        msg.name = joint_1.name + joint_2.name
        msg.position = joint_1.position + joint_2.position
        msg.velocity = joint_1.velocity + joint_2.velocity
        msg.effort = (0,0,0,0,0) + joint_2.effort
        
#        rospy.loginfo("%d %d %d %d"%(len(msg.name), len(msg.position), len(msg.velocity), len(msg.effort)))
        
        pub_comb.publish(msg)
        