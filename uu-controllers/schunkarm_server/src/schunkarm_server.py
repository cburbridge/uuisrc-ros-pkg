#! /usr/bin/env python

import roslib 
roslib.load_manifest("schunkarm_server")

import rospy
import actionlib
import schunkarm_server.msg

class ArmServer(object):
    _feedback = schunkarm_server.msg.movearmActionFeedback()
    _result = schunkarm_server.msg.movearmActionResult()

    def __init__(self, name):
        self._action_name = name
        self._a_server = actionlib.SimpleActionServer(self._action_name, 
                                                      schunkarm_server.msg.movearmAction)
        
        self._a_server.register_goal_callback(self.goal_received)
        self._a_server.start()
        

    def goal_received(self):
        goal = self._a_server.accept_new_goal()
        rospy.logdebug("Received goal: %s"%(str(goal)) )


if __name__ == '__main__':
    rospy.init_node('schunkarm_server', log_level=rospy.DEBUG)
    ArmServer(rospy.get_name())
    rospy.spin()
