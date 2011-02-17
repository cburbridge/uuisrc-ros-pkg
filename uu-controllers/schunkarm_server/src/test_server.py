#! /usr/bin/env python

import roslib; roslib.load_manifest('schunkarm_server')
import rospy

import actionlib
import schunkarm_server.msg
import random

def main():
    client =  actionlib.SimpleActionClient("schunkarm_server", schunkarm_server.msg.movearmAction)
    client.wait_for_server()
    
    goal = schunkarm_server.msg.movearmGoal()
    goal.goal_pose.header.stamp = rospy.Time.now()
    goal.goal_pose.header.frame_id = "/schunk/position/PAM112_BaseConector"
    
    goal.goal_pose.pose.position.x = -0.51
    goal.goal_pose.pose.position.y = -0.000501
#    goal.goal_pose.pose.position.z = 0.6252

#    goal.goal_pose.pose.position.x = random.uniform(0,0.5)
#    goal.goal_pose.pose.position.y = random.uniform(-0.1,0.1)
    goal.goal_pose.pose.position.z = random.uniform(0.3,0.8)
    
    goal.goal_pose.pose.orientation.x = -0.000236
    goal.goal_pose.pose.orientation.y = 0.73538
    goal.goal_pose.pose.orientation.z = 0.000226
    goal.goal_pose.pose.orientation.w = -0.677654
      
    
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