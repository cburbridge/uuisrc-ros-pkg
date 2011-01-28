/*
 * move.cpp
 *
 *  Created on: 20 Oct 2010
 *      Author: chris
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <schunk_kinematics/GetVelocityIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/KinematicSolverInfo.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <cmath>

class Callbacks {

public:
	sensor_msgs::JointState jointstates;
	bool ready;

	Callbacks() {
		ready = false;
	}

	void jointsCallback(const sensor_msgs::JointState::ConstPtr& data) {
		//		printf("Got some joint states.\n");
		jointstates = *(data.get());
		ready = true;
	}
};

int main(int argc, char** argv) {
    
    double target_x = 0.0;
    double target_y = 0.0;
    double target_z = 0.0;
    
    
    ros::init(argc, argv, "arm_track");
    ros::NodeHandle n;
    ros::Rate r(27);
    ros::Publisher vel_pub = n.advertise<sensor_msgs::JointState> (
		    "/schunk/target_vel_safe/joint_states", 1);

    tf::TransformListener transformer;
    geometry_msgs::PoseStamped target_pose;
    
    target_pose.pose.position.x = target_x;
    target_pose.pose.position.y = target_y;
    target_pose.pose.position.z = target_z;
    
    target_pose.pose.orientation.x = 0;
    target_pose.pose.orientation.y = 0;
    target_pose.pose.orientation.z = 0;
    target_pose.pose.orientation.w = 1;
    
    target_pose.header.frame_id = "/head";
//     target_pose.header.frame_id = "/schunk/position/PAM112_BaseConector";
    
    Callbacks callbacks;

    ros::Subscriber joints = n.subscribe("/schunk/position/joint_states", 10,
		    &Callbacks::jointsCallback, &callbacks);


    ROS_DEBUG("Waiting for some joint state info\n");
    while (callbacks.ready == false) {
	    ros::spinOnce();
	    r.sleep();
    }

    ros::ServiceClient ik_service = n.serviceClient<
		    schunk_kinematics::GetVelocityIK> ("/schunk_kinematics/get_ik_vel");
    
    ros::ServiceClient fk_service = n.serviceClient<
		    kinematics_msgs::GetPositionFK> ("/schunk_kinematics/get_fk");

    ROS_DEBUG("Waiting for the kineamtics service...\n");
    ik_service.waitForExistence(ros::Duration(-1));
    fk_service.waitForExistence(ros::Duration(-1));
    

    schunk_kinematics::GetVelocityIK ik_msg;
    ik_msg.request.timeout = ros::Duration(10);
    ik_msg.request.ik_request.ik_link_name = "GripperBox";


    ik_msg.request.ik_request.twist.angular.z = 0;
    ik_msg.request.ik_request.twist.angular.y = 0;
    ik_msg.request.ik_request.twist.angular.x = 0;
    
    float velocity[3];
    velocity[0]=velocity[1]=velocity[2]=0;
    
    while (ros::ok()) {
   
	std::string err;
	//Getting the target point in the arm frame of reference	
	target_pose.header.stamp = ros::Time::now();
	
	bool waiting_result = transformer.waitForTransform("/schunk/position/GripperBox", "/head", 
					    target_pose.header.stamp, ros::Duration(1),
					    ros::Duration(0.01), &err);
	if (!waiting_result) {
	    std::cerr<<"Wait for transform err: "<<err<<"\n";
	    continue;
	}
	
	geometry_msgs::PoseStamped result;	
	transformer.transformPose("/schunk/position/GripperBox",target_pose,result);
	
	double gripper_desired_roll = -atan2(result.pose.position.y, result.pose.position.z);
	double gripper_desired_pitch = atan2(result.pose.position.x, result.pose.position.z);
// 	double gripper_desired_pitch = 0;
	double gripper_desired_yaw = 0;	
	
	
	ROS_DEBUG("From the gripper point of view (roll,pitch,yaw): %.2f, %.2f, %.2f", 
						     gripper_desired_roll, 
						     gripper_desired_pitch,
						     gripper_desired_yaw);
	
//	geometry_msgs::QuaternionStamped rot_request, rot_result;
//	rot_request.header.frame_id = "/schunk/position/GripperBox";
//	rot_request.header.stamp = target_pose.header.stamp;
	
//	tf::Quaternion rot_quaternion;
//	rot_quaternion.setRPY(gripper_desired_roll, gripper_desired_pitch, gripper_desired_yaw);
	
//	rot_request.quaternion.x=rot_quaternion.getX();
//	rot_request.quaternion.y=rot_quaternion.getY();
//	rot_request.quaternion.z=rot_quaternion.getZ();
//	rot_request.quaternion.w=rot_quaternion.getW();
	
	tf::Vector3 rotation(gripper_desired_roll, gripper_desired_pitch, gripper_desired_yaw);
	tf::StampedTransform transform;
	transformer.waitForTransform("/schunk/position/PAM112_BaseConector", "/schunk/position/GripperBox", 
					    target_pose.header.stamp, ros::Duration(5.0));
	transformer.lookupTransform("/schunk/position/PAM112_BaseConector", "/schunk/position/GripperBox", target_pose.header.stamp , transform);
	tf::Vector3 result_vector = transform.getBasis() * rotation;
	
	
//	transformer.waitForTransform("/schunk/position/GripperBox", "/schunk/position/PAM112_BaseConector", 
//					    rot_request.header.stamp, ros::Duration(1.0));
//	transformer.transformQuaternion("/schunk/position/PAM112_BaseConector",rot_request,rot_result);

	//getting the velocity vector
	
//	ROS_INFO("Target pose (x,y,z): %.2f, %.2f, %.2f", result.pose.position.x, 
//						     result.pose.position.y, result.pose.position.z);
						     
	ROS_DEBUG("Target anglulars (x,y,z): %.2f, %.2f, %.2f", result_vector.x(), 
						     result_vector.y(),result_vector.z());

//	tf::Quaternion q(rot_result.quaternion.x, rot_result.quaternion.y,
//		       rot_result.quaternion.z, rot_result.quaternion.w);
	
//	double gripper_roll, gripper_pitch, gripper_yaw	       ;
//	btMatrix3x3(q).getRPY(gripper_roll, gripper_pitch, gripper_yaw);
	
//	ROS_INFO("Gripper orientation: (roll,pitch,yaw): %.2f, %.2f, %.2f", 
//						     gripper_roll, 
//						     gripper_pitch,
//						     gripper_yaw);	


	double gripper_roll, gripper_pitch, gripper_yaw;
	gripper_roll = result_vector.x();
	gripper_pitch = result_vector.y();
	gripper_yaw = result_vector.z();
	
	double scale = 1;
	
	ik_msg.request.ik_request.twist.linear.x = 0;
	ik_msg.request.ik_request.twist.linear.y = 0;
	ik_msg.request.ik_request.twist.linear.z = 0;
	
	ik_msg.request.ik_request.twist.angular.x = gripper_roll * scale;
	ik_msg.request.ik_request.twist.angular.y = gripper_pitch * scale;
	ik_msg.request.ik_request.twist.angular.z = gripper_yaw * scale;
	
	ROS_INFO("Moving at vels (roll,pitch,yaw): %.2f, %.2f, %.2f", gripper_roll, 
			gripper_pitch, gripper_yaw);

	ik_msg.request.ik_request.robot_state.joint_state = callbacks.jointstates;


	if (ik_service.call(ik_msg)) {
		//		ROS_INFO("Sum: %ld", (long int)ik_msg.response.sum);
	} else {
	    while (! ik_service.waitForExistence(ros::Duration(5))) {
		ROS_DEBUG("Waiting for IK service");
	    }
	}

	vel_pub.publish(ik_msg.response.solution.joint_state);
// 	ROS_DEBUG("");


	r.sleep();
	ros::spinOnce();
    }

}

