#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

int main(int argc, char** argv)
{
	ros::init (argc, argv, "empty_cloud");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud> ("empty_cloud", 1);

	sensor_msgs::PointCloud msg;
	msg.header.frame_id = "/ScitosBase";

	//Let's just add a single point to make everybody happy

	double depth = 0.4;

	for (double v = -1.0; v<1.0; v += 0.01) {
		geometry_msgs::Point32 point;
		point.x = depth;
		point.y = -v;
		point.z = 1.37;
		msg.points.push_back(point);
	}
	for (double v = -1.0; v<1.0; v += 0.01) {
		geometry_msgs::Point32 point;
		point.x = depth;
		point.y = -v;
		point.z = 0.97;
		msg.points.push_back(point);
	}
	for (double v = .77; v<1.47; v += 0.01) {
		geometry_msgs::Point32 point;
		point.x = depth;
		point.y = -0.25;
		point.z = v;
		msg.points.push_back(point);
	}
	for (double v = .77; v<1.47; v += 0.01) {
		geometry_msgs::Point32 point;
		point.x = depth;
		point.y = 0.25;
		point.z = v;
		msg.points.push_back(point);
	}

	ros::Rate loop_rate(4);
	while (nh.ok())
	{


		msg.header.stamp = ros::Time::now ();
		pub.publish (msg);
		ros::spinOnce ();
		loop_rate.sleep ();
	}
}
