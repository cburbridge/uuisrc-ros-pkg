#! /usr/bin/env python

import roslib; roslib.load_manifest('schunkarm_server')
import rospy

import actionlib
import schunkarm_server.msg

def main():
    client =  actionlib.SimpleActionClient("schunkarm_server", schunkarm_server.msg.movearmAction)
    client.wait_for_server()
    
    goal = schunkarm_server.msg.movearmGoal()
    print "Goal: ", goal
    client.send_goal(goal)
    
    client.wait_for_result()
    return client.get_result() 

if __name__ == "__main__":
    try:
        rospy.init_node("test_server")
        res = main()
        rospy.loginfo("Result: %s"%(str(res)))
    except rospy.ROSInterruptException:
        rospy.loginfo("exiting")