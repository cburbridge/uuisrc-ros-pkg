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

#include <player-2.1/libplayerc++/playerc++.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <camera_calibration_parsers/parse.h>
#include <string>


// The name of the camera
std::string camera_name;
std::string param_file;


// The camera info
sensor_msgs::CameraInfo camera_info;

bool set_info(sensor_msgs::SetCameraInfo::Request  &req,
         sensor_msgs::SetCameraInfo::Response &res )
{
	res.success=1;
	res.status_message="Camera params saved.";
	ROS_INFO("Set the info on the camera.");


	camera_calibration_parsers::writeCalibration(param_file, camera_name, req.camera_info);
	camera_calibration_parsers::readCalibration(param_file, camera_name, camera_info);

	return true;
}


int main(int argc, char** argv) {

	// First argument to the node is the name of the camera.
	camera_name.assign(argv[1]);
	param_file.assign(argv[1]);
	param_file.append("_params.yaml");
	printf("Running config file \"%s\"\n",param_file.c_str());

	// Load the camera data file for the calibration parameters
	// TODO: If not existing yet then it crashes :-|
	camera_calibration_parsers::readCalibration(param_file, camera_name, camera_info);
	printf("Parameters loaded.\n");

	PlayerCc::PlayerClient playerclient("foyle", 6665);
	PlayerCc::CameraProxy camera(&playerclient,0);

	// Set the data mode to PULL
	playerclient.SetDataMode(PLAYER_DATAMODE_PULL);
	playerclient.SetReplaceRule(true,-1,-1,-1);

	playerclient.Read();


	ros::init(argc, argv, "player_camera");
	ros::NodeHandle node;
	image_transport::ImageTransport it(node);
	image_transport::Publisher cameraPublisher = it.advertise("/camera/image", 1);
	ros::Publisher info_pub = node.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 50);
	ros::ServiceServer camera_service = node.advertiseService("/camera/set_camera_info", set_info);


	// Times for publishing
	ros::Time currentTime, lastTime;
	currentTime = ros::Time::now();
	lastTime = ros::Time::now();


	sensor_msgs::Image::Ptr image(new sensor_msgs::Image);
	unsigned int msgCount=0;

	// The main loop
	while (node.ok()) {
		currentTime = ros::Time::now();

		ros::spinOnce();

		playerclient.Read();
		camera.Decompress();	//TODO: What if it isn't compressed?

		// Publish the camera image
//		if (camera.IsValid()) {	// NOTE: Not working on robotlab3 machine?
//		if (camera.IsFresh()) {	// NOTE: Not working on robotlab3 machine?
			// Got some fresh camera data.

			image->width=camera.GetWidth();
			image->height=camera.GetHeight();
			// TODO: Hard coded for rgb images, need to generify
			image->encoding = "rgb8";
			image->step=image->width*3;
			image->data.resize(camera.GetImageSize());
			camera.GetImage(&(image->data[0]));
			image->header.frame_id="/camera_frame";
			image->header.seq=msgCount;
			image->header.stamp=currentTime;

//			camera_info.height=image->height;
//			camera_info.width=image->width;
			camera_info.header=image->header;
//			camera_info.
//			image->data.asign


			cameraPublisher.publish(image);
			info_pub.publish(camera_info);
			msgCount++;
//		}

		lastTime = currentTime;

	}
}
