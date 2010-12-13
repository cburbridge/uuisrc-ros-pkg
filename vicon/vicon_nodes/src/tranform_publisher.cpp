/*
 *  Vicon tranform publisher 
 *
 *  Communicates with Vicon NX server, and publishes a model tranformation over ROS
 *  Should work with any number of segments, but only tested with one so far.
 *
 *  Copyright (C) 2010, Chris Burbridge <cburbridge@gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "libvicon/ViconBaseStream.h"
#include <stdio.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv) {
	ros::init(argc, argv, "transform_publisher");
	ros::NodeHandle n;
	ros::Publisher marker_publisher = n.advertise<visualization_msgs::MarkerArray> (
			"vicon_object_array", 1);


	double framerate = 30;
	std::string server;
	int port;
	bool stream;
	std::string modelfile;
	std::string subject;


	// Get vicon server IP and port from the parameter server
	if (!n.getParam("/vicon/vicon_ip", server)) {
		ROS_WARN("No server name found on parameter server, assuming 192.168.1.1");
		server="192.168.1.1";
	}
	if (!n.getParam("/vicon/vicon_port", port)) {
		ROS_WARN("No port number found on parameter server, assuming 800");
		port=800;
	}
	if (!n.getParam("/vicon/vicon_freq", framerate)) {
		ROS_WARN("No framerate found on parameter server, asssuming 30");
		framerate=30;
	}
	if (!n.getParam("/vicon/stream", stream)) {
		ROS_WARN("No stream status flag, assuming streaming.");
		stream=true;
	}
	if (!n.getParam("/vicon/modelfile", modelfile)) {
		ROS_FATAL("No model file specified.");
		return 0;
	}
	if (!n.getParam("/vicon/subject", subject)) {
		ROS_FATAL("No subject specified.");
		return 0;
	}

	ros::Rate r(framerate);
	ViconBase *vicon;

	if (stream)
		vicon = new ViconBaseStream (modelfile.c_str(), subject.c_str(), server.c_str(),port);
	else
		vicon = new ViconBase(modelfile.c_str(), subject.c_str(), server.c_str(),port);

	vicon->printStructure();
	vicon->requestValuesUpdate();

    tf::TransformBroadcaster trans_broadcaster;


    std::vector< geometry_msgs::TransformStamped > vicon_transforms;
	visualization_msgs::MarkerArray markers;
    char segmentname[1000];
	for (uint i=0; i<vicon->Segments.size(); i++) {
		 geometry_msgs::TransformStamped trans;
		 trans.header.frame_id="/vicon/world";
		 printf("/vicon/%s/%s\n",vicon->subjectName.c_str(), vicon->Segments[i].Name.c_str());
		 sprintf(segmentname,"/vicon/%s/%s",vicon->subjectName.c_str(), vicon->Segments[i].Name.c_str());
		 trans.child_frame_id=std::string(segmentname);
		 vicon_transforms.push_back(trans);

		 visualization_msgs::Marker markerline;
		 markerline.header.frame_id=std::string(segmentname);
		 markerline.lifetime = ros::Duration(2.0/framerate); // live for two frames
		 markerline.ns=std::string(segmentname);
		 markerline.id=i*1000;
		 markerline.type = visualization_msgs::Marker::LINE_STRIP;
		 markerline.scale.x=0.01;
		 markerline.color.r=0;
		 markerline.color.g=155;
		 markerline.color.b=155;
		 markerline.color.a=255;

		 for (uint j=0; j<vicon->Segments[i].Markers.size(); j++) {
			 visualization_msgs::Marker marker;
			 marker.header.frame_id=std::string(segmentname);
			 marker.lifetime = ros::Duration(2.0/framerate); // live for two frames
			 marker.ns=std::string(segmentname);
			 marker.id=i*100+j;
			 marker.type = visualization_msgs::Marker::SPHERE;
			 marker.scale.x=0.03;
			 marker.scale.y=0.03;
			 marker.scale.z=0.03;
			 marker.color.r=0;
			 marker.color.a=255;

			 marker.pose.position.x=vicon->Segments[i].Markers[j].modelX / 1000.0;
			 marker.pose.position.y=vicon->Segments[i].Markers[j].modelY / 1000.0;
			 marker.pose.position.z=vicon->Segments[i].Markers[j].modelZ / 1000.0;
			 marker.pose.orientation.x=0;
			 marker.pose.orientation.y=0;
			 marker.pose.orientation.z=0;
			 marker.pose.orientation.w=1;

			 marker.action = visualization_msgs::Marker::ADD;
			 markers.markers.push_back(marker);

			 markerline.points.push_back(marker.pose.position);
		 }
		 markers.markers.push_back(markerline);

	}
	long seeq=0;



	printf("Going live.\n");
	uint k=0;
	while (ros::ok()) {
		vicon->requestValuesUpdate();
		k=0;
		for (uint i=0; i<vicon->Segments.size(); i++) {

			vicon_transforms[i].header.seq=seeq;
			vicon_transforms[i].header.stamp=ros::Time::now();

			vicon_transforms[i].transform.translation.x=vicon->Segments[i].X / 1000.0;
			vicon_transforms[i].transform.translation.y=vicon->Segments[i].Y / 1000.0;
			vicon_transforms[i].transform.translation.z=vicon->Segments[i].Z / 1000.0;

			vicon_transforms[i].transform.rotation.x=vicon->Segments[i].QuaternionX;
			vicon_transforms[i].transform.rotation.y=vicon->Segments[i].QuaternionY;
			vicon_transforms[i].transform.rotation.z=vicon->Segments[i].QuaternionZ;
			vicon_transforms[i].transform.rotation.w=vicon->Segments[i].QuaternionW;

			trans_broadcaster.sendTransform(vicon_transforms[i]);

			for (uint j=0; j<vicon->Segments[i].Markers.size(); j++) {

				if (vicon->Segments[i].Markers[j].visible ) {
					markers.markers[k].color.g=255;
					markers.markers[k].color.b=255;
				} else {
					markers.markers[k].color.b=0;
					markers.markers[k].color.g=255;
				}

				markers.markers[k].header.stamp=ros::Time::now();

				k++;
			 }
			marker_publisher.publish(markers);

			markers.markers[k].header.stamp=ros::Time::now();
			k++; // skip the lines trip marker

		}
		seeq++;
		r.sleep();
	}
}

