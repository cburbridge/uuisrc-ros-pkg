#! /usr/bin/env python
# Copyright (c) 2010, Lorenzo Riano.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#         * Redistributions of source code must retain the above copyright
#             notice, this list of conditions and the following disclaimer.
#         * Redistributions in binary form must reproduce the above copyright
#             notice, this list of conditions and the following disclaimer in the
#             documentation and/or other materials provided with the distribution.
#         * Neither the name of the Lorenzo Riano. nor the names of its
#             contributors may be used to endorse or promote products derived from
#             this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Lorenzo Riano <lorenzo.riano@gmail.com> 

import roslib
roslib.load_manifest("pr2_control_utilities")
import rospy
import actionlib
from move_arm_msgs.msg import MoveArmAction, MoveArmGoal
from actionlib import SimpleActionClient
from motion_planning_msgs.msg import PositionConstraint, OrientationConstraint

class PR2MoveArm(object):
    def __init__(self, ik, joint_mover):
        self.move_right_arm_client = SimpleActionClient("move_right_arm", MoveArmAction)
        self.move_right_arm_client.wait_for_server()
        
        self.move_left_arm_client = SimpleActionClient("move_left_arm", MoveArmAction)
        self.move_left_arm_client.wait_for_server()
        
        self.ik = ik
        self.joint_mover = joint_mover
        
        
    def move_right_arm(self, position, orientation, frame_id,  waiting_time):
        goal = MoveArmGoal()
        goal.motion_plan_request.group_name = "right_arm"
        goal.motion_plan_request.num_planning_attempts = 10
        goal.motion_plan_request.planner_id = ""
        goal.planner_service_name = "ompl_planning/plan_kinematic_path"
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(waiting_time / 10.);
        
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = frame_id
        position_constraint.link_name = "r_wrist_roll_link"
        
        position_constraint.position.x = position[0]
        position_constraint.position.y = position[1]
        position_constraint.position.z = position[2]
        position_constraint.constraint_region_shape.type = position_constraint.constraint_region_shape.BOX
        tolerance = 2 * 0.02
        position_constraint.constraint_region_shape.dimensions = [tolerance, tolerance, tolerance]
        
        position_constraint.constraint_region_orientation.x = 0.
        position_constraint.constraint_region_orientation.y = 0.
        position_constraint.constraint_region_orientation.z = 0.
        position_constraint.constraint_region_orientation.w = 1.        
        position_constraint.weight = 1.0
        
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = frame_id
        orientation_constraint.link_name = "r_wrist_roll_link"
#        orientation_constraint.type = dunno!
        orientation_constraint.orientation.x = orientation[0]
        orientation_constraint.orientation.y = orientation[1]
        orientation_constraint.orientation.z = orientation[2]
        orientation_constraint.orientation.w = orientation[3]
        
        orientation_constraint.absolute_roll_tolerance = 0.04
        orientation_constraint.absolute_pitch_tolerance = 0.04
        orientation_constraint.absolute_yaw_tolerance = 0.04
        orientation_constraint.weight = 1.0
        
        goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
        goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)
        
        goal.disable_collision_monitoring = True
        
#        rospy.loginfo("Goal: " + str(goal))
        state = self.move_right_arm_client.send_goal_and_wait(goal, rospy.Duration(waiting_time))
        if state == actionlib.GoalStatus.SUCCEEDED:
            return True
        else:
            return False
        
    def move_right_arm_non_collision(self, position, orientation, frame_id, ignore_errors = False):
        current_pose_stamped = self.ik.run_fk(self.joint_mover.robot_state.right_arm_positions,
                                              "r_wrist_roll_link")
        end_pose = self.ik.lists_to_pose_stamped(position, 
                                            orientation, 
                                            frame_id, 
                                            frame_id)
        trajectory, error_codes = self.ik.check_cartesian_path(current_pose_stamped, 
                                                     end_pose, 
                                                     self.joint_mover.robot_state.right_arm_positions, 
                                                     collision_aware = 0,
                                                     num_steps=10)
        if (not ignore_errors) and any( (e == 3 for e in error_codes) ):
            rospy.logerr("IK returns error codes: %s"%str(error_codes))
            return False
        self.joint_mover.execute_trajectory(trajectory, "right", True)
        return True
        
if __name__ == "__main__":
    rospy.init_node('trytest', anonymous=True)
    move_arm = PR2MoveArm()
    
    position = (0.55 - 0.1, -0.188, 0)
    orientation = (0., 0., 0., 1.)
    
    if move_arm.move_right_arm(position, orientation, "/torso_lift_link", 120.):
        rospy.loginfo("OK")
    else:
        rospy.loginfo("bad")
            
        
        