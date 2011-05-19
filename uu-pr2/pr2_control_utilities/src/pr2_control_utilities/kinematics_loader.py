#! /usr/bin/env python

PKG="pr2_control_utilities"
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
#import load
from tf import transformations
#from math import pi

from pr2_joint_mover import RobotState
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
#from sensor_msgs.msg import JointState

#import actionlib

class KinematicsLoader(object):
    '''
    A convenience class to read the IK values of the robot and issue commands.
    '''
    def __init__(self, robot_state):
        '''        
        @param robot_state: a RobotState instance.
        '''
        self.listener = tf.TransformListener()
        
        rospy.loginfo("Waiting for right arm ik service")
        rospy.wait_for_service("pr2_right_arm_kinematics/get_ik")
        self.right_ik = rospy.ServiceProxy("pr2_right_arm_kinematics/get_ik",GetPositionIK)
        
        rospy.loginfo("Waiting for left arm ik service")
        rospy.wait_for_service("pr2_left_arm_kinematics/get_ik")
        self.left_ik = rospy.ServiceProxy("pr2_left_arm_kinematics/get_ik",GetPositionIK)
        rospy.loginfo("IKs are ok")
        
        self.robot_state = robot_state
    
    def get_right_arm_pose_euler(self):
        '''
        Get the position / euler angles of the right wrist
        '''
        self.listener.waitForTransform("base_link","r_wrist_roll_link", 
                                       rospy.Time(), rospy.Duration(2.0))
        position, quaternion = self.listener.lookupTransform("/base_link","r_wrist_roll_link", 
                                       rospy.Time())
        
        euler = transformations.euler_from_quaternion(quaternion)
        rospy.loginfo("Right arm:\nPosition: %s\nQuaterion: %s\nEuler: %s", 
                      str(position), str(quaternion), str(euler))
        
        return position, euler
   
    def get_left_arm_pose_euler(self):
        '''
        Get the position / euler angles of the left wrist
        '''        
        self.listener.waitForTransform("base_link","l_wrist_roll_link", 
                                       rospy.Time(), rospy.Duration(2.0))
        position, quaternion = self.listener.lookupTransform("/base_link","l_wrist_roll_link", 
                                       rospy.Time())
        
        euler = transformations.euler_from_quaternion(quaternion)
        rospy.loginfo("Left arm:\nPosition: %s\nQuaterion: %s\nEuler: %s", 
                      str(position), str(quaternion), str(euler))
        
        return position, euler
        
    def set_right_arm_pose_euler(self, position, angles):        
        '''
        Move the right arm to a position in euler space
        @param position: x,y,z
        @param angles: roll pitch yaw
        '''
        quaternion = transformations.quaternion_from_euler(angles[0], angles[1], angles[2])
        return self.set_right_arm_pose(position, quaternion) 
    
    def set_left_arm_pose(self, position, orientation):
        '''
        Move the left arm to a position
        @param position: x,y,z
        @param angles: quaternion
        '''        
        
        req = GetPositionIKRequest()
        req.timeout = rospy.Duration(5.0)
        req.ik_request.ik_link_name = "l_wrist_roll_link"
        
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = position[0]
        req.ik_request.pose_stamped.pose.position.y = position[1]
        req.ik_request.pose_stamped.pose.position.z = position[2]
        
        req.ik_request.pose_stamped.pose.orientation.x = orientation[0]
        req.ik_request.pose_stamped.pose.orientation.y = orientation[1]
        req.ik_request.pose_stamped.pose.orientation.z = orientation[2]
        req.ik_request.pose_stamped.pose.orientation.w = orientation[3]
        
        req.ik_request.ik_seed_state.joint_state.position = self.robot_state.left_arm_positions
        req.ik_request.ik_seed_state.joint_state.name = self.robot_state.left_joint_names
        
        res = self.left_ik(req)
        if res.error_code.val != 1:
            rospy.logerr("IK error, code is %d" % res.error_code.val)
            return None
        
        joints = res.solution.joint_state.position
        return joints
    
    def set_left_arm_pose_euler(self, position, angles):
        '''
        Move the right arm to a position in euler space
        @param position: x,y,z
        @param angles: roll pitch yaw
        '''
        quaternion = transformations.quaternion_from_euler(angles[0], angles[1], angles[2])
        return self.set_left_arm_pose(position, quaternion) 
    
    def set_right_arm_pose(self, position, orientation):
        '''
        Move the right arm to a position
        @param position: x,y,z
        @param angles: quaternion
        '''                
        req = GetPositionIKRequest()
        req.timeout = rospy.Duration(5.0)
        req.ik_request.ik_link_name = "r_wrist_roll_link"
        
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = position[0]
        req.ik_request.pose_stamped.pose.position.y = position[1]
        req.ik_request.pose_stamped.pose.position.z = position[2]
        
        req.ik_request.pose_stamped.pose.orientation.x = orientation[0]
        req.ik_request.pose_stamped.pose.orientation.y = orientation[1]
        req.ik_request.pose_stamped.pose.orientation.z = orientation[2]
        req.ik_request.pose_stamped.pose.orientation.w = orientation[3]
        
        req.ik_request.ik_seed_state.joint_state.position = self.robot_state.right_arm_positions
        req.ik_request.ik_seed_state.joint_state.name = self.robot_state.right_joint_names
        
        res = self.right_ik(req)        
        if res.error_code.val != 1:
            rospy.logerr("IK error, code is %d" % res.error_code.val)
            return None
        
        joints = res.solution.joint_state.position        
        return joints
    
if __name__ == "__main__":
    rospy.init_node(PKG, anonymous=True)
    robot_state = RobotState()
    kinematics = KinematicsLoader(robot_state)
    pos, angles = kinematics.get_left_arm_pose_euler()
    
    print "Left arm"
    print "Pos: ", pos
    print "Orientation: ", angles
    
        