/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: pointcloud_online_viewer.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

// ROS core
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <pcl_ros/subscriber.h>
#include <terminal_tools/print.h>
#include <terminal_tools/parse.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;
using terminal_tools::print_highlight;
using terminal_tools::parse_argument;

unsigned int frame_no = 0;
unsigned int bigcloud_no = 0;
tf::TransformListener* listener;
tf::StampedTransform transform_res;
pcl::PointCloud<pcl::PointXYZRGB> bigcloud;

void
  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("PointCloud with %d data points (%s), stamp %f, and frame %s.", msg->width * msg->height, 
	   pcl::getFieldsList (*msg).c_str (),
	   msg->header.stamp.toSec (), 
	   msg->header.frame_id.c_str ()); 
  
  ros::Time now = ros::Time::now();
//   tf::StampedTransform tranform;

  try{
    listener->waitForTransform("/vicon/world", msg->header.frame_id,
						  now,ros::Duration(5));
						  
    listener->lookupTransform("/vicon/world", msg->header.frame_id,  now, transform_res);
    
  }
    catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  
    tf::Vector3 pos = transform_res.getOrigin();
    if (pos.x() == 0 && pos.y() == 0 && pos.z() == 0) {
	return;
    }
    
    sensor_msgs::PointCloud msg_point1;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, msg_point1);

  listener->transformPointCloud("/vicon/world",msg_point1,msg_point1);
  
  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  sensor_msgs::PointCloud2 tmp;
  sensor_msgs::convertPointCloudToPointCloud2(msg_point1, tmp);
  pcl::fromROSMsg (tmp, cloud);
  
  bigcloud.header = cloud.header;
  bigcloud += cloud;
  if (!(frame_no %100) ) {
      ROS_INFO("Saving bigcloud %d", bigcloud_no);
      std::stringstream filename;
      filename<<"bigcloud_"<<bigcloud_no<<".pcd";
      pcl::io::savePCDFileASCII(filename.str(), bigcloud);
      bigcloud = pcl::PointCloud<pcl::PointXYZRGB>();
      bigcloud_no++;
  }
  frame_no++;  
}

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "pointcloud_save");
  ros::NodeHandle nh;

  listener = new tf::TransformListener();
  sleep(2);
  // Get the queue size from the command line
  int queue_size = 1;
  parse_argument (argc, argv, "-qsize", queue_size);
  print_highlight ("Using a queue size of %d\n", queue_size);

  // Create a ROS subscriber
  ros::Subscriber sub = nh.subscribe ("/rgbd/points2", queue_size, cloud_cb);
  while (frame_no != 150000)
	  ros::spinOnce();
 
  return (0);
}
/* ]--- */
