#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>

class LaserScanReceiver {

    private:
	ros::NodeHandle n;
	std::string base_tf;
	std::string laser_topic;
	laser_geometry::LaserProjection projector;
	tf::TransformListener listener;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
	tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;
	std::vector<float> obstacle_x;
	std::vector<float> obstacle_y;
	unsigned int laser_count;

	ros::Publisher pcl_pub;
    
    public:
	LaserScanReceiver(ros::NodeHandle node, std::string tf, std::string topic) :
          n(node),
          base_tf(tf),
          laser_topic(topic),
          laser_sub(n, topic, 10),
          laser_notifier(laser_sub,listener, tf, 10)

        {
	    laser_notifier.registerCallback(&LaserScanReceiver::scanCallback, this);
	    laser_notifier.setTolerance(ros::Duration(0.2));
	    laser_count = 0;
	    
	    pcl_pub = node.advertise<sensor_msgs::PointCloud>("laser_pcl",1);

	}

      
	void scanCallback (const sensor_msgs::LaserScan scan_in)  {
	    ROS_DEBUG("LaserScanReceiver: I have %d ranges", scan_in.ranges.size());
	    sensor_msgs::PointCloud cloud;
	    try {
		projector.transformLaserScanToPointCloud(
			      	  base_tf.c_str(),scan_in, cloud,listener);
	    }
	    catch (tf::TransformException& e)    {
	    	ROS_ERROR("%s", e.what());
	    	return;
	    }
	    
	    obstacle_x.clear();
	    obstacle_y.clear();
	    obstacle_x.reserve(cloud.points.size());
	    obstacle_y.reserve(cloud.points.size());
	    
	    for (std::vector<geometry_msgs::Point32>::iterator i=cloud.points.begin(); i!=cloud.points.end(); i++) {
		obstacle_x.push_back(i->x);
		obstacle_y.push_back(i->y);
	    }
	    
	    laser_count = obstacle_x.size();
	    pcl_pub.publish(cloud);
	 }
	 
	 float* get_obstacles_x() {
	     return &obstacle_x[0];
	 }
	 float* get_obstacles_y() {
	     return &obstacle_y[0];
	 }
	 
	 unsigned int get_laser_count() {
	     return laser_count;
	 }
};

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "laser_2_pointcloud");
    ros::NodeHandle node;

    std::string base_tf;
    std::string laser_topic;

    node.param("/laser_to_pointcloud/base_tf", base_tf, std::string("/odom"));
    node.param("/laser_to_pointcloud/laser_topic", laser_topic, std::string("/base_scan"));

    LaserScanReceiver laser(node, base_tf, laser_topic);
    
    ros::Rate loop_rate(10);
    while (node.ok()) {
		ros::spinOnce();
		loop_rate.sleep();
    }
    
}
