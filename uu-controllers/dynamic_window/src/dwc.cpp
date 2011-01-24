#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <vector>
#include <nav_msgs/Odometry.h>

#include "dynamicwindow.h"

const char* laser_topic = "base_scan";
const char* base_tf = "base_link";

class PoseReceiver {
    public:
	PoseReceiver(ros::NodeHandle node) : n(node) {
	    sub = n.subscribe("/odom", 1, &PoseReceiver::pose_received, this);
	}
    
	void pose_received(const nav_msgs::Odometry& pose) {
	    v = pose.twist.twist.linear.x;
	    w = pose.twist.twist.angular.z;
	    ROS_DEBUG("Robot vel is %f %f", v, w);
	}
    
    public:
	float v;
	float w;
    
    private:
	ros::NodeHandle n;
	ros::Subscriber sub;

    
};

class LaserScanReceiver {

    private:
	ros::NodeHandle n;
	laser_geometry::LaserProjection projector;
	tf::TransformListener listener;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
	tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;
	std::vector<float> obstacle_x;
	std::vector<float> obstacle_y;
	unsigned int laser_count;

	ros::Publisher debug_pub;

    
    public:
	LaserScanReceiver(ros::NodeHandle node) :
          n(node),
          laser_sub(n, laser_topic, 10),
          laser_notifier(laser_sub,listener, base_tf, 10)
        {
	    laser_notifier.registerCallback(&LaserScanReceiver::scanCallback, this);
	    laser_notifier.setTolerance(ros::Duration(0.2));
	    laser_count = 0;
	    
	    debug_pub = node.advertise<sensor_msgs::PointCloud>("debug_point_cloud",1);

	}

      
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)  {
// 	    ROS_INFO("LaserScanReceiver: I have %d ranges", scan_in->ranges.size());
	    sensor_msgs::PointCloud cloud;
// 	    ROS_INFO("PIPPO");
	    try {
		projector.transformLaserScanToPointCloud(
			      	  base_tf,*scan_in, cloud,listener);
	    }
	    catch (tf::TransformException& e)    {
		ROS_ERROR("%s", e.what());
		return;
	    }
// 	    ROS_INFO("EXIT PIPPO");
	    
	    obstacle_x.clear();
	    obstacle_y.clear();
	    obstacle_x.reserve(cloud.points.size());
	    obstacle_y.reserve(cloud.points.size());
	    
	    for (std::vector<geometry_msgs::Point32>::iterator i=cloud.points.begin(); i!=cloud.points.end(); i++) {
		obstacle_x.push_back(i->x);
		obstacle_y.push_back(i->y);
	    }
	    
	    laser_count = obstacle_x.size();
// 	    ROS_INFO("LaserScanReceiver: I have %d points", laser_count);
	    
	    debug_pub.publish(cloud);
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
	 
	 void test_scan() {
	     double min = 1000;
	     double d;
	     
	     for (unsigned int i=0; i<get_laser_count(); i++) {
		 d = sqrt(get_obstacles_x()[i]*get_obstacles_x()[i] + get_obstacles_y()[i]*get_obstacles_y()[i]);
		 if (d < min) {
		     min = d;
		 }
	     }
	     ROS_INFO("LASER POINTCLOUD: min distance is %f", min);
	 }
};

class CommandsReceiver {
    public:
	CommandsReceiver(ros::NodeHandle node) : n(node) {
	    commandSubscriber = n.subscribe("cmd_vel_dwc", 10, &CommandsReceiver::driveCommandCallback, this);
	    desired_v = 0;
	    desired_w = 0;
	    new_command = false;
	    
	}
	void driveCommandCallback(const geometry_msgs::TwistConstPtr& msg) {
		ROS_INFO("Received some speeds [%f %f]", msg->linear.x, msg->angular.z);
		desired_v = msg->linear.x;
		desired_w = msg->angular.z;		
		new_command = true;
	}
	
    public:
	bool new_command;
	float desired_v;
	float desired_w;
    private:
	ros::NodeHandle n;
	ros::Subscriber commandSubscriber;
	
};

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "dwc");
    ros::NodeHandle node;
    ros::Publisher commander_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    
    LaserScanReceiver laser(node);
    ROS_INFO("LaserScanReceiver created");
    PoseReceiver pose(node);
    ROS_INFO("PoseReceiver created");
    CommandsReceiver commands(node);
    ROS_INFO("CommandsReceiver created");
    DynamicWindow dw;
    
    
    double alpha = 0.3;
    dw.setAlpha(alpha);	
    double beta = 0.4;
    dw.setBeta(beta);	
    double gamma = 0.3;
    dw.setGamma(gamma);	
    double wpmax = DynamicWindow::deg2rad(100);
    dw.setWpmax(wpmax);
    double wmin = DynamicWindow::deg2rad(-90);
    dw.setWmin(wmin);
    double wmax = DynamicWindow::deg2rad(90);
    dw.setWmax(wmax);
    double wpbrake = DynamicWindow::deg2rad(100);
    dw.setWpbrake(wpbrake);
    double vmax = 0.5;
    dw.setVmax(vmax);
    double vpmax = 1.0;
    dw.setVpmax(vpmax);
    double vpbrake = 0.3;
    dw.setVpbrake(vpbrake);
    double tcmax = 0.2;
    dw.setTime_c(tcmax);
    double laser_max = 4.0;
    dw.setLaser_max(laser_max);
    double radius = 0.368;
    dw.setRadius(radius);
    

    
    
    ros::Rate loop_rate(1.0 / tcmax);
   
    while (node.ok()) {
	ros::spinOnce();
	
	if (commands.new_command) {
	    double v_found = 0;
	    double w_found = 0;
	    
// 	    commands.new_command = false;
	    dw.setVmax(commands.desired_v);	
	    dw.setLaser_count( laser.get_laser_count() );
// 	    ROS_INFO("I've got %d points", laser.get_laser_count());
	    
// 	    ROS_INFO("Entering motion_model_sampling_time");
	    dw.motion_model_sampling_time(tcmax - 0.02, pose.v, pose.w, commands.desired_w, laser.get_obstacles_x(), laser.get_obstacles_y(), v_found, w_found);
// 	    ROS_INFO("Exit motion_model_sampling_time");
	    
	    geometry_msgs::Twist msg;
	    msg.linear.x = v_found;
	    msg.angular.z = w_found;
// 	    ROS_INFO("DWC: output speeds: %f %f", v_found, w_found);
// 	    ROS_INFO("Diff speeds: %f %f", v_found - commands.desired_v,w_found - commands.desired_w);
	    commander_pub.publish(msg);
	    
	}
	
	loop_rate.sleep();
    }
    
}