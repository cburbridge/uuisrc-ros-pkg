/*
 * ros_player.cpp
 *
 *  Created on: 22 Jul 2010
 *      Author: chris burbridge
 *
 *  Interfaces ROS with Player by connecting to player server on robot, and
 *  publishing/subscribing the necessary things on the ROS side.
 *
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <player-2.1/libplayerc++/playerc++.h>
#include "player_interface/GoTo.h"
#include "player_interface/GoToEnable.h"

struct DriveCommandCallback {

	float lv, wv;

	DriveCommandCallback() {
		lv=wv=0;
	}

	/**
	 *  The callback that gets done when ROS receives velocity commands
	 */
	void driveCommandCallback(const geometry_msgs::TwistConstPtr& msg) {
		ROS_DEBUG("Received some speeds [%f %f]", msg->linear.x, msg->angular.z);

		// Store the speeds for use in main()
		lv = msg->linear.x;
		wv = msg->angular.z;

	}

};

struct DriveCommandPezz {

	float lv, wv;
	PlayerCc::Position2dProxy * m_driver;

	DriveCommandPezz(PlayerCc::Position2dProxy* driver) {
		
		lv=wv=0;
		m_driver = driver;
	}

	/**
	 *  The callback that gets done when ROS receives velocity commands
	 */
	void driveCommandCallback(const geometry_msgs::TwistConstPtr& msg) {
		ROS_DEBUG("Received some speeds [%f %f]", msg->linear.x, msg->angular.z);

		// Store the speeds for use in main()
		lv = msg->linear.x;
		wv = msg->angular.z;
		m_driver->SetSpeed(lv,wv);

	}

};

struct GotoService {
	float goalX, goalY, goalTh;
	PlayerCc::PlannerProxy planner;


	GotoService(PlayerCc::PlayerClient &client) : planner(&client) {

	}

	bool gotoCallback(player_interface::GoTo::Request  &req,
			             player_interface::GoTo::Response &res){
		planner.SetGoalPose(req.x, req.y, req.th);
		return true;
	}

	bool enable(player_interface::GoToEnable::Request &req,
						player_interface::GoToEnable::Response &res) {
		planner.SetEnable(req.enable);
		return true;
	}
};


int main(int argc, char** argv) {

	PlayerCc::PlayerClient playerclient("foyle", 6665);
	PlayerCc::Position2dProxy position(&playerclient, 20);
	PlayerCc::LaserProxy laser(&playerclient);
	PlayerCc::PtzProxy ptz(&playerclient);

	// Set the data mode to PULL
	playerclient.SetDataMode(PLAYER_DATAMODE_PULL);
	playerclient.SetReplaceRule(true,-1,-1,-1);

	playerclient.Read();

// 	DriveCommandCallback driveCommand;
	DriveCommandPezz driveCommand(&position);
// 	GotoService gotoservice(playerclient);

	ros::init(argc, argv, "ros_player"); // init the ros node as "ros_player"
	ros::NodeHandle node;
	ros::Subscriber commandSubscriber = node.subscribe("cmd_vel", 100, &DriveCommandPezz::driveCommandCallback, &driveCommand);
	ros::Publisher odomPublisher = node.advertise<nav_msgs::Odometry> ("odom", 50);
	ros::Publisher laserscanPublisher = node.advertise<sensor_msgs::LaserScan>("base_scan", 50);
// 	ros::ServiceServer gotoServer = node.advertiseService("GoTo", &GotoService::gotoCallback, &gotoservice);
// 	ros::ServiceServer gotoEnableServer = node.advertiseService("GoToEnable", &GotoService::enable, &gotoservice);


	tf::TransformBroadcaster odom_broadcaster;
	tf::TransformBroadcaster pantilt_broadcaster;



	// The odometry position and velocities of the robot
	double x, y, th, vx, vy, vth;

	// Times for publishing
	ros::Time currentTime, lastTime;
	currentTime = ros::Time::now();
	lastTime = ros::Time::now();


	// The transform for the PanTilt head
	geometry_msgs::TransformStamped head_trans;
//	geometry_msgs::Quaternion head_quat = tf::createQuaternionMsgFromYaw(th);



	// The main loop
	while (node.ok()) {
		currentTime = ros::Time::now();

		ros::spinOnce();

		// Set the commanded velocities
		//------------------------------
//		position.SetSpeed(driveCommand.lv, driveCommand.wv);
		playerclient.Read();

		// Publish the odometry on ROS
		//-----------------------------
		if (position.IsFresh()) {
//			printf("Got some odometry data.\n");
			// Read the odometry from player
			x = position.GetXPos();;
			y = position.GetYPos();
			th = position.GetYaw();
			vx = position.GetXSpeed();
			vy = position.GetYSpeed();
			vth= position.GetYawSpeed();

			// since all odometry is 6DOF we'll need a quaternion created from yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

			//first, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = currentTime;
			odom_trans.header.frame_id = "/odom";
			odom_trans.child_frame_id = "/base_link";

			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = currentTime;
			odom.header.frame_id = "/odom";
			odom.child_frame_id = "/base_link";

			//set the position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			//set the velocity
			odom.twist.twist.linear.x = vx;
			odom.twist.twist.linear.y = vy;
			odom.twist.twist.angular.z = vth;

			//publish the message
			odomPublisher.publish(odom);
			position.NotFresh();
		}

		// Publish the Laser
		//-------------------
		//
		if (laser.IsFresh()) {
			// Laser parameters
//			printf("Got some laser data.\n");
			sensor_msgs::LaserScan scan;
			scan.header.frame_id = "/laser_frame";
			scan.angle_min = laser.GetMinAngle();
			scan.angle_max = laser.GetMaxAngle();
			scan.angle_increment = laser.GetScanRes();
			scan.time_increment = (1.0 /  10.0 )/ laser.GetCount() ; // the time between individual point measures; 10.0 should be laser.GetScanningFrequency()
			scan.range_min = 0.0;
			scan.range_max = 8.0;
			scan.set_ranges_size(laser.GetCount());
			scan.set_intensities_size(laser.GetCount());
			for(unsigned int i = 0; i < laser.GetCount(); ++i){
			  scan.ranges[i] = laser[i];
			  scan.intensities[i] = laser.GetIntensity(i);
			}
			scan.header.stamp = currentTime;
			laserscanPublisher.publish(scan);
			laser.NotFresh();
		}

		// Publish the PTZ head frame
		if (ptz.IsFresh()) {
			// Got some fresh head data.
			// compose the transform and publish it.
//			printf("Got some PTZ data.\n");
			head_trans.header.stamp = currentTime;
			head_trans.header.frame_id = "/head_link";
			head_trans.child_frame_id = "/head_frame";

			head_trans.transform.translation.x = 0;//-0.1;
			head_trans.transform.translation.y = 0;
			head_trans.transform.translation.z = 0;//1.30;	// How high above the base unit is the head
			head_trans.transform.rotation=
					tf::createQuaternionMsgFromRollPitchYaw(0,0-ptz.GetTilt(),ptz.GetPan());
//			head_trans.transform.rotation.y = 0.0;
//			head_trans.transform.rotation.z = 1.0;
//			head_trans.transform.rotation.w = 0.0;

			pantilt_broadcaster.sendTransform(head_trans);

			ptz.NotFresh();
		}


		lastTime = currentTime;

	}
}
