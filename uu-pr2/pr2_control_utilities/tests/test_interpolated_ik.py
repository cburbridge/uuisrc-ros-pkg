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

# Author: Lorenzo Riano <lorenzo.riano@gmail.com> (based on code from Jon Scholz)

import roslib
roslib.load_manifest("pr2_control_utilities")
import rospy
from interpolated_ik_motion_planner import ik_utilities
import pr2_control_utilities
import copy

if __name__ == "__main__":
    
    rospy.init_node('trytest', anonymous=True)
    state = pr2_control_utilities.RobotState()
    mover = pr2_control_utilities.PR2JointMover(state)
    ik = ik_utilities.IKUtilities("right")
    
    angles = state.right_arm_positions
    link = "r_wrist_roll_link"
    
    pose = ik.run_fk(angles, link)
    
    newpose = copy.deepcopy(pose)
    newpose.pose.position.x += 0.3
    newpose.pose.position.z += 0.0
    
    newpose.pose.orientation.x = 0
    newpose.pose.orientation.y = 0
    newpose.pose.orientation.z = 0
    newpose.pose.orientation.w = 1
    
    startpos = ik.pose_stamped_to_lists(pose, pose.header.frame_id)
    goalpos = ik.pose_stamped_to_lists(newpose, newpose.header.frame_id)
    
#    rospy.loginfo("Current pose is: %s"%str( pose))
#    rospy.loginfo("New pose is %s"%str(newpose))
    
    trajectory, error_codes = ik.check_cartesian_path(pose, 
                                                     newpose, 
                                                     state.right_arm_positions, 
                                                     collision_aware = 0,
                                                     num_steps=10)
    
    newtraj = []
    for t,e in zip(trajectory, error_codes):
        if e != 3:
            newtraj.append(t)
    mover.execute_trajectory(newtraj, "right", True)
    
#    error_codes = [1 if x ==3 else 0 for x in error_codes]
#    if sum(error_codes) == 0:
#        mover.execute_trajectory(trajectory, "right", True)
#    else:
#        rospy.logerr("Error codes: " + str(error_codes))
    
    current_pose = ik.run_fk(state.right_arm_positions, link)
    rospy.loginfo("Current position is \n%s, \nwanted is\n%s"%(str(current_pose), str(newpose)))
    
    rospy.signal_shutdown("now")
    rospy.sleep(0.5)
