#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <move_arm_msgs/MoveArmAction.h>
#include <move_arm_msgs/utils.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <geometry_msgs/PoseStamped.h>

bool test_ik(ros::NodeHandle& nh, const geometry_msgs::Pose& pose ) {

	ROS_INFO("Querying original ik service");
	ros::service::waitForService("/schunk_kinematics/get_ik");
	ros::ServiceClient ik_client = nh.serviceClient<kinematics_msgs::GetPositionIK>("/schunk_kinematics/get_ik");

	kinematics_msgs::GetPositionIK::Request request;
	kinematics_msgs::GetPositionIK::Response response;

	request.ik_request.pose_stamped.header.frame_id = "/ScitosBase";
	request.ik_request.pose_stamped.pose = pose;

	request.ik_request.ik_seed_state.joint_state.position.resize(7);
	request.ik_request.ik_seed_state.joint_state.name.resize(7);

	request.ik_request.ik_seed_state.joint_state.position[0] = 0;
	request.ik_request.ik_seed_state.joint_state.name[0] = "Joint0";
	request.ik_request.ik_seed_state.joint_state.position[1] = 0;
	request.ik_request.ik_seed_state.joint_state.name[1] = "Joint1";
	request.ik_request.ik_seed_state.joint_state.position[2] = 0.00;
	request.ik_request.ik_seed_state.joint_state.name[2] = "Joint2";
	request.ik_request.ik_seed_state.joint_state.position[3] = 0.35;
	request.ik_request.ik_seed_state.joint_state.name[3] = "Joint3";
	request.ik_request.ik_seed_state.joint_state.position[4] = 0;
	request.ik_request.ik_seed_state.joint_state.name[4] = "Joint4";

	request.ik_request.ik_seed_state.joint_state.position[5] = 0.0;
	request.ik_request.ik_seed_state.joint_state.name[5] = "WR1";
	request.ik_request.ik_seed_state.joint_state.position[6] = 0.0;
	request.ik_request.ik_seed_state.joint_state.name[6] = "WR2";

//	request.ik_request.ik_seed_state.joint_state.position[0] = 0;
//	request.ik_request.ik_seed_state.joint_state.position[1] = 0;
//	request.ik_request.ik_seed_state.joint_state.position[2] = 0;
//	request.ik_request.ik_seed_state.joint_state.position[3] = 0;
//	request.ik_request.ik_seed_state.joint_state.position[4] = 0;
//	request.ik_request.ik_seed_state.joint_state.position[5] = 0;
//	request.ik_request.ik_seed_state.joint_state.position[6] = 0;

	if(ik_client.call(request, response))	  {
	    if(response.error_code.val == response.error_code.SUCCESS) {
	      for(unsigned int i=0; i < response.solution.joint_state.name.size(); i ++)
	        ROS_INFO("Joint: %s %f",response.solution.joint_state.name[i].c_str(),response.solution.joint_state.position[i]);
	      return true;
	    }
	    else {
	      ROS_ERROR("Inverse kinematics failed");
	      return false;
	    }
	  }
	  else {
	    ROS_ERROR("Inverse kinematics service call failed");
	    return false;
	}

}

int main(int argc, char **argv){
	ros::init (argc, argv, "move_arm_pose_goal_test");
	ros::NodeHandle nh;

	move_arm_msgs::MoveArmGoal goalA;

	goalA.motion_plan_request.group_name = "schunk_hand";
	goalA.motion_plan_request.num_planning_attempts = 1;
	goalA.motion_plan_request.planner_id = std::string("SBLkConfig");
	goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalA.motion_plan_request.allowed_planning_time = ros::Duration(2.0);

	motion_planning_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "/ScitosBase";
	desired_pose.link_name = "PAM100";
	desired_pose.pose.position.x = 0.46;
	desired_pose.pose.position.y = 0.00;
	desired_pose.pose.position.z = 1.14;

	desired_pose.pose.orientation.x = 0.71;
	desired_pose.pose.orientation.y = 0.0;
	desired_pose.pose.orientation.z = 0.71;
	desired_pose.pose.orientation.w = 0.0;

	desired_pose.absolute_position_tolerance.x = 0.02;
	desired_pose.absolute_position_tolerance.y = 0.02;
	desired_pose.absolute_position_tolerance.z = 0.02;

	desired_pose.absolute_roll_tolerance = 0.04;
	desired_pose.absolute_pitch_tolerance = 0.04;
	desired_pose.absolute_yaw_tolerance = 0.04;

	move_arm_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

	if (!test_ik(nh, desired_pose.pose)) {
		ROS_ERROR("No kinematic solution!");
		ros::shutdown();
		return -1;
	}
	std::string servername = "move_schunk_hand";
	ROS_INFO("Waiting for server %s", servername.c_str());
	actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm(servername,true);
	move_arm.waitForServer();
	ROS_INFO("Connected to server");

	if (nh.ok())
	{
		bool finished_within_time = false;
		move_arm.sendGoal(goalA);
		finished_within_time = move_arm.waitForResult(ros::Duration(20.0));
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
