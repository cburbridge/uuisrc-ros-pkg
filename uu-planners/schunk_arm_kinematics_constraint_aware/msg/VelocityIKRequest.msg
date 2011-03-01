# A Velocity IK request message
# The name of the link for which we are computing IK
string ik_link_name

# The twist of the link
geometry_msgs/Twist twist

# Here specify the starting positions 
# of joints/links on the robot.
motion_planning_msgs/RobotState robot_state
