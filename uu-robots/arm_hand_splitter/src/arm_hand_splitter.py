#! /usr/bin/env python

import roslib
roslib.load_manifest("arm_hand_splitter")
import rospy
from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

class SplitterCombiner(object):
    def __init__(self):
        self.wrj1 = 0
        self.wrj2 = 0
        
        rospy.Subscriber("hand_joint_states", JointState, self.hand_callback)
        rospy.Subscriber("arm_trajectory_state",JointTrajectoryControllerState, self.arm_callback)
        rospy.Subscriber("command", JointTrajectory, self.trajectory_cb)
        
        self.state_publisher = rospy.Publisher("state", JointTrajectoryControllerState)
        self.traj_publisher = rospy.Publisher("out_command", JointTrajectory)
        
    def hand_callback(self, state):
        #Assume WRJ1 and WRJ2 are the last two elements
        if state.name[-1] != "WRJ2":
            rospy.logfatal("Error:, joint name is %s, expected WRJ2"%(state.name[-1]) )
            return
        self.wrj1 = state.position[-1]
        
        if state.name[-2] != "WRJ1":
            rospy.logfatal("Error:, joint name is %s, expected WRJ1"%(state.name[-2]) )
            return
        self.wrj2 = state.position[-2]
        
    def arm_callback(self, state):
        state.joint_names += ("WRJ1",)
        state.actual.positions += (self.wrj1,)
        state.actual.velocities += (0,)
#        state.actual.accelerations += (0,)
        state.desired.positions += (self.wrj1,)
        state.desired.velocities += (0,)
#        state.desired.accelerations += (0,)
        state.error.positions += (self.wrj1,)
        state.error.velocities += (0,)
        
        state.joint_names += ("WRJ2",)
        state.actual.positions += (self.wrj2,)
        state.actual.velocities += (0,)
#        state.actual.accelerations += (0,)
        state.desired.positions += (self.wrj2,)
        state.desired.velocities += (0,)
#        state.desired.accelerations += (0,)
        state.error.positions += (self.wrj2,)
        state.error.velocities += (0,)
        
        self.state_publisher.publish(state)
    
    def trajectory_cb(self, traj):
        rospy.loginfo("Received a new trajectory, beginning the split..")
        newtraj_arm = JointTrajectory()
        newtraj_arm.header.stamp = traj.header.stamp
        newtraj_arm.header.frame_id = traj.header.frame_id
        
        for name in traj.joint_names:
            if name[:5] == "Joint":
                newtraj_arm.joint_names += (name,)
        
        for point in traj.points:
            newpoint = JointTrajectoryPoint()
            for i, name in enumerate(traj.joint_names):
                if name[:5] == "Joint":
                    newpoint.positions += (point.positions[i],)
                    newpoint.velocities += (point.velocities[i],)
                    newpoint.accelerations += (point.accelerations[i],)
                    newpoint.time_from_start = point.time_from_start
                    
                    
            newtraj_arm.points += (newpoint,)
        
#        rospy.loginfo("Sending trajectory:\n" + str(newtraj_arm))
        rospy.loginfo("Publishing the arm trajectory")
        self.traj_publisher.publish(newtraj_arm)
        
        
if __name__ == "__main__":
    rospy.init_node("arm_hand_splitter")
    rospy.loginfo("arm_hand_splitter fusion started")
    
    splitter = SplitterCombiner()
    
    rospy.spin()
        