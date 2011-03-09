//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include <schunk_arm_kinematics_constraint_aware/schunk_arm_ik_solver.h>

using namespace Eigen;

Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p)
{
  Eigen::Matrix4f b = Eigen::Matrix4f::Identity();
  for(int i=0; i < 3; i++)
  {
    for(int j=0; j<3; j++)
    {
      b(i,j) = p.M(i,j);
    }
    b(i,3) = p.p(i);
  }
  return b;
}


SchunkArmIKSolver::SchunkArmIKSolver(const urdf::Model &robot_model,
                               const std::string &root_frame_name,
                               const std::string &tip_frame_name,
                               const double &search_discretization_angle, 
                               const int &free_angle,
                               boost::shared_ptr<KDL::ChainIkSolverPos> solver,
                               kinematics_msgs::KinematicSolverInfo solver_info)
{
	solver_ = solver;
	solver_info_ = solver_info;
	search_discretization_angle_ = search_discretization_angle;
	free_angle_ = free_angle;
	root_frame_name_ = root_frame_name;

  active_ = true;
}
//
//void SchunkArmIKSolver::getSolverInfo(kinematics_msgs::KinematicSolverInfo &response)
//{
//  pr2_arm_ik_.getSolverInfo(response);
//}

int SchunkArmIKSolver::CartToJnt(const KDL::JntArray& q_init,
                              const KDL::Frame& p_in, 
                              KDL::JntArray &q_out)
{
	ROS_INFO("PEZZ DEBUG: %d -- %d", q_init.rows(), q_out.rows());
	int ret = solver_->CartToJnt(q_init, p_in, q_out);
	if (ret < 0) {
		ROS_WARN("Original KDL fails, what can I do?");
	}
	return ret;
};

int SchunkArmIKSolver::CartToJnt(const KDL::JntArray& q_init,
                              const KDL::Frame& p_in,
                              std::vector<KDL::JntArray> &q_out)
{
  KDL::JntArray q;

  q.resize(7); //TODO: fix this 7 !!
  q_out.clear();

  bool found = CartToJnt(q_init, p_in, q);
  if (!found) {
	  return NO_IK_SOLUTION;
  }
  else {
	  q_out.push_back(q);
	  return 1;
  }

}

bool SchunkArmIKSolver::getCount(int &count,
                              const int &max_count, 
                              const int &min_count)
{
  if(count > 0)
  {
    if(-count >= min_count)
    {   
      count = -count;
      return true;
    }
    else if(count+1 <= max_count)
    {
      count = count+1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(1-count <= max_count)
    {
      count = 1-count;
      return true;
    }
    else if(count-1 >= min_count)
    {
      count = count -1;
      return true;
    }
    else
      return false;
  }
}

int SchunkArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in,
                                    const KDL::Frame& p_in, 
                                    std::vector<KDL::JntArray> &q_out, 
                                    const double &timeout)
{
	KDL::JntArray q_init = q_in;
	Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
	double initial_guess = q_init(free_angle_);

	ros::Time start_time = ros::Time::now();
	double loop_time = 0;
	int count = 0;

	int num_positive_increments = (int)((solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
	int num_negative_increments = (int)((initial_guess-solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
	ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,solver_info_.limits[free_angle_].max_position,solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
	while(loop_time < timeout)
	{
		if(CartToJnt(q_init,p_in,q_out) > 0)
			return 1;
		if(!getCount(count,num_positive_increments,-num_negative_increments))
			return -1;
		q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
		ROS_DEBUG("%d, %f",count,q_init(free_angle_));
		loop_time = (ros::Time::now()-start_time).toSec();
	}
	if(loop_time >= timeout)
	{
		ROS_DEBUG("IK Timed out in %f seconds",timeout);
		return TIMED_OUT;
	}
	else
	{
		ROS_DEBUG("No IK solution was found");
		return NO_IK_SOLUTION;
	}
	return NO_IK_SOLUTION;
}

int SchunkArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in,
                                    const KDL::Frame& p_in, 
                                    KDL::JntArray &q_out, 
                                    const double &timeout)
{
  KDL::JntArray q_init = q_in;
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,solver_info_.limits[free_angle_].max_position,solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_DEBUG("IK Timed out in %f seconds",timeout);
    return TIMED_OUT;
  }
  else
  {
    ROS_DEBUG("No IK solution was found");
    return NO_IK_SOLUTION;
  }
  return NO_IK_SOLUTION;
}

int SchunkArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in,
                                    const KDL::Frame& p_in, 
                                    KDL::JntArray &q_out, 
                                    const double &timeout, 
                                    motion_planning_msgs::ArmNavigationErrorCodes &error_code,
                                    const boost::function<void(const KDL::JntArray&,const KDL::Frame&,motion_planning_msgs::ArmNavigationErrorCodes &)> &desired_pose_callback,
                                    const boost::function<void(const KDL::JntArray&,const KDL::Frame&,motion_planning_msgs::ArmNavigationErrorCodes &)> &solution_callback)
{
	ROS_INFO("Searching for a solution in %f seconds", timeout);
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  KDL::JntArray q_init = q_in;
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,solver_info_.limits[free_angle_].max_position,solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);

  if(!desired_pose_callback.empty())
    desired_pose_callback(q_init,p_in,error_code);
  if(error_code.val != error_code.SUCCESS)
  {
    return -1;
  }
  bool callback_check = true;
  if(solution_callback.empty())
    callback_check = false;

  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) >= 0)
    {
      if(callback_check)
      {
        solution_callback(q_out,p_in,error_code);
        if(error_code.val == error_code.SUCCESS)
        {
          return 1;
        }
      }
      else
      {
        error_code.val = error_code.SUCCESS;
        return 1;
      }
    }
    if(!getCount(count,num_positive_increments,-num_negative_increments))
    {
      error_code.val = error_code.NO_IK_SOLUTION;
      return -1;
    }
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("Redundancy search, index:%d, free angle value: %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_INFO("IK Timed out in %f seconds",timeout);
    error_code.val = error_code.TIMED_OUT;
  }
  else
  {
    ROS_INFO("No IK solution was found");
    error_code.val = error_code.NO_IK_SOLUTION;
  }
  return -1;
}

std::string SchunkArmIKSolver::getFrameId()
{
  return root_frame_name_;
}
