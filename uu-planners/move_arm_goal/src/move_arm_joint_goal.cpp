#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm_msgs/MoveArmAction.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;

  std::string servername = "move_schunk_hand";
//  std::string servername = "move_schunk";
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm(servername,true);

  ROS_INFO("Waiting for server %s", servername.c_str());
  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  move_arm_msgs::MoveArmGoal goalB;
  std::vector<std::string> names(7);
//  std::vector<std::string> names(5);
  names[0] = "Joint0";
  names[1] = "Joint1";
  names[2] = "Joint2";
  names[3] = "Joint3";
  names[4] = "Joint4";
  names[5] = "WRJ1";
  names[6] = "WRJ2";
//  names[5] = "Joint5";
//  names[6] = "Joint6";

  goalB.motion_plan_request.group_name = "schunk_hand";
//  goalB.motion_plan_request.group_name = "schunk";
  goalB.motion_plan_request.num_planning_attempts = 2;
  goalB.motion_plan_request.allowed_planning_time = ros::Duration(10.0);
  goalB.disable_collision_monitoring = 0;

  goalB.motion_plan_request.planner_id= std::string("SBLkConfig");
  goalB.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalB.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

  for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.05;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.05;
  }

//  goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -0.884;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[1].position = 0.5236;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[2].position = -0.2698;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.2618;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[4].position = -0.4642;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.1745;
//  goalB.motion_plan_request.goal_constraints.joint_constraints[6].position = -0.0;

  goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -0.00;
  goalB.motion_plan_request.goal_constraints.joint_constraints[1].position = -0.80;
  goalB.motion_plan_request.goal_constraints.joint_constraints[2].position = -0.00;
  goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = 0.8;
  goalB.motion_plan_request.goal_constraints.joint_constraints[4].position = -0.0;
  goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = 0.0;
  goalB.motion_plan_request.goal_constraints.joint_constraints[6].position = 0.0;

  if (nh.ok())
  {
    bool finished_within_time = false;

//    ROS_INFO_STREAM("About to send: "<<goalB<<"\n");
    move_arm.sendGoal(goalB);
    finished_within_time = move_arm.waitForResult(ros::Duration(60.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  ros::shutdown();
}
