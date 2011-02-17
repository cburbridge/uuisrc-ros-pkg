#! /usr/bin/env python

import roslib 
roslib.load_manifest("schunkarm_server")

import rospy
import actionlib
import schunkarm_server.msg
import kinematics_msgs.srv
import kinematics_msgs.msg
from sensor_msgs.msg import JointState
import random

class ArmServer(object):
    _feedback = schunkarm_server.msg.movearmActionFeedback()
    _result = schunkarm_server.msg.movearmActionResult()

    def __init__(self, name):
        self._action_name = name
        self._jointstate = None
        self._goal = None
        self._a_server = actionlib.SimpleActionServer(self._action_name, 
                                                      schunkarm_server.msg.movearmAction)
        self._end_effector = rospy.get_param("tip_name")
        
        rospy.Subscriber("joint_states", JointState, self.callbackJointStates)
        self._commander = rospy.Publisher("target_pc/joint_states", JointState)
        self._jointstate = rospy.wait_for_message("joint_states", JointState, 1.0)
        
        rospy.loginfo("Waiting for inverse kinematics services...")
        rospy.wait_for_service('get_ik')
        rospy.loginfo("Kinematics services alive and kicking")
#        rospy.wait_for_service("/get_ik_solver_info")        
        
        self._inverseKinematics = rospy.ServiceProxy('get_ik', kinematics_msgs.srv.GetPositionIK)
                
        
        self._a_server.register_goal_callback(self.goal_received)
        self._a_server.start()
        

    def goal_received(self):
        goal = self._a_server.accept_new_goal()
        self._goal = goal
        rospy.logdebug("Received goal: %s"%(str(goal)) )
        
        burb_counter = 0
        found = False
        
        while burb_counter < 10:
            req = self.create_ik_request()        
            rospy.logdebug("About to ask for " + str(req))
            resp = self._inverseKinematics(req)
            if resp.error_code.val == 1:
                found = True               
                break
            burb_counter += 1
        
        if found:
            rospy.logdebug("Found a solution after %d attempts"%burb_counter)
            rospy.logdebug("Response: " + str(resp) )
            self._commander.publish(resp.solution.joint_state)
        else:
            rospy.logdebug("No solution you moron!")
        
        
        
    def create_ik_request(self):
        req = kinematics_msgs.srv.GetPositionIKRequest()
        
        req.timeout = rospy.Duration(1.)
        req.ik_request.ik_link_name = self._end_effector
        req.ik_request.pose_stamped = self._goal.goal_pose
        
        for i in xrange(len(self._jointstate.name)):
            req.ik_request.ik_seed_state.joint_state.name.append( self._jointstate.name[i] )
            req.ik_request.ik_seed_state.joint_state.position.append( random.uniform(-1.5, 1.5) )            
        
        return req
        
        
    def callbackJointStates(self, data):
        self._jointstate = data
        rospy.logdebug("Received joint state: " + str(data))
        


if __name__ == '__main__':
    rospy.init_node('schunkarm_server', log_level=rospy.DEBUG)
    ArmServer(rospy.get_name())
    rospy.spin()
