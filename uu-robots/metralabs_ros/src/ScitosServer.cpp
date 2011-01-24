#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "metralabs_ros/movePosition.h"
#include "metralabs_ros/idAndFloat.h"
#include "sensor_msgs/JointState.h"
#include "urdf/model.h"
#include "metralabs_ros/SchunkStatus.h"
#include "ScitosBase.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

//#include "PowerCube5.h"
#include "PowerCube.h"

class SchunkServer {
private:
	PowerCube m_powerCube;
	sensor_msgs::JointState m_currentJointState;
	metralabs_ros::SchunkStatus m_schunkStatus;
	ros::NodeHandle m_node;
	std::vector<urdf::Joint*> m_joints;
	urdf::Model m_armModel;	// A parsing of the model description
	ros::Publisher m_currentJointStatePublisher;
	ros::Publisher m_schunkStatusPublisher;
	std::map<std::string, unsigned int> m_nameToNumber;
public:

	SchunkServer(ros::NodeHandle &node) : m_node(node) {
		init();
	}

	void init() {
		// Initialise the arm model parser and find out what non-fixed joins are present
		m_armModel.initParam("schunk_description");
		std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator mapElement;
		for (mapElement = m_armModel.joints_.begin(); mapElement!=m_armModel.joints_.end(); mapElement++) {
			if ((*mapElement).second.get()->type != urdf::Joint::FIXED)
				m_joints.push_back((*mapElement).second.get() ); // shared pointer mantaged?
		}
		ROS_INFO("URDF specifies %d non-fixed joints.", m_joints.size() );

		// Initialise the powercube
		m_powerCube.init();

		// Check that the required modules are present.
		ROS_WARN("Didn't check that the modules in the description match modules in reality.");

		// Set up the joint state publisher with the joint names
		// This assumes that the joints are ordered on the robot in the same order as
		// the URDF!!!
		m_currentJointState.name.resize(m_joints.size());
		m_currentJointState.position.resize(m_joints.size());
		for (unsigned int i=0;i<m_joints.size();i++) {
			m_currentJointState.name[i] = m_joints[i]->name;
			m_nameToNumber[m_joints[i]->name] = i;
			ROS_INFO("%d is mapping to %s", i, m_joints[i]->name.c_str());
		}
		m_currentJointStatePublisher = m_node.advertise<sensor_msgs::JointState>("/schunk/position/joint_states", 1);

		// Set up the schunk status publisher
		m_schunkStatusPublisher = m_node.advertise<metralabs_ros::SchunkStatus>("/schunk/status",1);
		for (uint i=0; i<m_joints.size();i++) {
			metralabs_ros::SchunkJointStatus status;
			status.jointName = m_joints[i]->name;
			m_schunkStatus.joints.push_back(status);
		}
		ROS_INFO("Ready");

	}

	void cb_emergency(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("emergency");
		m_powerCube.pc_emergency_stop();
	}

	void cb_stop(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("stop");
		m_powerCube.pc_normal_stop();
	}

	void cb_firstRef(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("first ref");
		m_powerCube.pc_first_ref();
	}

	void cb_ack(const std_msgs::Int8::ConstPtr& id) 	{
		ROS_INFO("cb_ack: [%d]", id->data);
		m_powerCube.pc_ack(id->data);
	}

	void cb_ackAll(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("cb_ackall");
		m_powerCube.pc_ack();
	}

	void cb_ref(const std_msgs::Int8::ConstPtr& id)	{
		ROS_INFO("cb_ref: [%d]", id->data);
		m_powerCube.pc_ref(id->data);
	}

	void cb_refAll(const std_msgs::Bool::ConstPtr& dummy)	{
		ROS_INFO("cb_refall");
		m_powerCube.pc_ref();
	}

	void cb_current(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_current: [%d, %f]", data->id, data->value);
		m_powerCube.pc_set_current(data->id, data->value);
	}

	void cb_currentsMaxAll(const std_msgs::Bool::ConstPtr& i)	{
		ROS_INFO("cb_currentsMax");
		m_powerCube.pc_set_currents_max();
	}

	void cb_movePosition(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_movePosition: [%d, %f]", data->id, data->value);
		m_powerCube.pc_move_position(data->id, data->value);
	}

//	void cb_movePositions(const metralabs_ros::Float32MultiArray::ConstPtr& data)
//	{
//		ROS_INFO("cb_movePositions");
//	}

	void cb_moveVelocity(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_moveVelocity: [%d, %f]", data->id, data->value);
		m_powerCube.pc_move_velocity(data->id, data->value);
	}

	void cb_targetVelocity(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_targetVelocity: [%d, %f]", data->id, data->value);
		m_powerCube.pc_set_target_velocity(data->id, data->value);
	}

	void cb_targetAcceleration(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_targetAcceleration: [%d, %f]", data->id, data->value);
		m_powerCube.pc_set_target_acceleration(data->id, data->value);
	}

//	void cb_startPosition(const std_msgs::Int8::ConstPtr& id)	{
//		ROS_INFO("cb_startPosition: [%d]", id->data);
//		m_powerCube.pc_start_position(id->data);
//	}

	void publishCurrentJointState () {
		m_currentJointState.header.stamp = ros::Time::now();
		for (unsigned int i=0;i<m_currentJointState.name.size(); i++) {
			m_currentJointState.position[i]=m_powerCube.mManipulator.getModules().at(i).status_pos/180.0 * M_PI;
		}
		m_currentJointStatePublisher.publish(m_currentJointState);
	}

	void publishSchunkStatus() {
		std::vector<metralabs_ros::SchunkJointStatus>::iterator i;
		uint moduleNumber;
		for (i=m_schunkStatus.joints.begin(); i != m_schunkStatus.joints.end(); i++) {
			moduleNumber=m_nameToNumber[(*i).jointName];
			m_powerCube.getModuleStatus(moduleNumber,&((*i).referenced),
													&((*i).moving),
													&((*i).progMode),
													&((*i).warning),
													&((*i).error),
													&((*i).brake),
													&((*i).moveEnd),
													&((*i).posReached),
													&((*i).errorCode),
													&((*i).current));
		}
		m_schunkStatusPublisher.publish(m_schunkStatus);
	}

	void targetJointStateCallbackPositionControl(const sensor_msgs::JointState::ConstPtr& data) {
		// The "names" member says how many joints, the other members may be empty.
		// Not all the joints have to be specified in the message
		// and not all types must be filled

		for (uint i=0;i<data.get()->name.size();i++) {
			m_powerCube.pc_set_currents_max();
			if (data.get()->position.size()!=0)
				m_powerCube.pc_move_position(m_nameToNumber[data.get()->name[i]],(data.get()->position[i])/M_PI * 180.0);
			if (data.get()->velocity.size()!=0)
				m_powerCube.pc_set_target_velocity(m_nameToNumber[data.get()->name[i]],(data.get()->velocity[i])/M_PI * 180.0);
			if (data.get()->effort.size()!=0)
				m_powerCube.pc_set_current(m_nameToNumber[data.get()->name[i]],(data.get()->effort[i]));

		}
	}

	void targetJointStateCallbackVelocityControl(const sensor_msgs::JointState::ConstPtr& data) {
		// The "names" member says how many joints, the other members may be empty.
		// Not all the joints have to be specified in the message
		// and not all types must be filled
//		m_powerCube.pc_ack();
		for (uint i=0;i<data.get()->name.size();i++) {

//			m_powerCube.pc_set_currents_max();

//			if (data.get()->position.size()!=0)
//				m_powerCube.pc_move_position(m_nameToNumber[data.get()->name[i]],(data.get()->position[i])/M_PI * 180.0);
			if (data.get()->velocity.size()!=0)
				m_powerCube.pc_move_velocity(m_nameToNumber[data.get()->name[i]],(data.get()->velocity[i])/M_PI * 180.0);
			if (data.get()->effort.size()!=0)
				m_powerCube.pc_set_current(m_nameToNumber[data.get()->name[i]],(data.get()->effort[i]));

		}
	}

};

class RosScitosBase {
    
    public:
	RosScitosBase(ros::NodeHandle& n, ScitosBase* base) : m_node(n) {
	    
	    m_base = base;
	    m_odomPublisher = m_node.advertise<nav_msgs::Odometry> ("odom", 50);
	    m_commandSubscriber = m_node.subscribe("cmd_vel", 100, &RosScitosBase::driveCommandCallback, this);
	}
	
	void loop() {
	    // The odometry position and velocities of the robot
	    double x, y, th, vx, vth;
	    m_base->get_odometry(x,y,th,vx,vth);
	    
	    //useless comment
	    m_base->loop();
	    
	    ros::Time currentTime = ros::Time::now();
	    // since all odometry is 6DOF we'll need a quaternion created from yaw
	    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	    //first, we'll publish the transform over tf
	    geometry_msgs::TransformStamped odom_trans;
	    odom_trans.header.stamp = currentTime;
	    odom_trans.header.frame_id = "/odom";
	    odom_trans.child_frame_id = "/schunk/position/ScitosBase";

	    odom_trans.transform.translation.x = x;
	    odom_trans.transform.translation.y = y;
	    odom_trans.transform.translation.z = 0.0;
	    odom_trans.transform.rotation = odom_quat;

	    //send the transform
	    m_odom_broadcaster.sendTransform(odom_trans);

	    //next, we'll publish the odometry message over ROS
	    nav_msgs::Odometry odom;
	    odom.header.stamp = currentTime;
	    odom.header.frame_id = "/odom";
	    odom.child_frame_id = "/schunk/position/ScitosBase";

	    //set the position
	    odom.pose.pose.position.x = x;
	    odom.pose.pose.position.y = y;
	    odom.pose.pose.position.z = 0.0;
	    odom.pose.pose.orientation = odom_quat;

	    //set the velocity
	    odom.twist.twist.linear.x = vx;
	    odom.twist.twist.linear.y = 0;
	    odom.twist.twist.angular.z = vth;

	    //publish the message
	    m_odomPublisher.publish(odom);
	}
    
    private:	
	ros::NodeHandle m_node;
	ScitosBase* m_base;
	tf::TransformBroadcaster m_odom_broadcaster;
	ros::Publisher m_odomPublisher;
	ros::Subscriber m_commandSubscriber;
    
    private:	
	void driveCommandCallback(const geometry_msgs::TwistConstPtr& msg) {
		ROS_DEBUG("Received some speeds [%f %f]", msg->linear.x, msg->angular.z);

		m_base->set_velocity(msg->linear.x, msg->angular.z);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "metralabs_ros");
	ros::NodeHandle n;


	SchunkServer server(n);
	ScitosBase base("/opt/MetraLabs/MLRobotic/etc/config/SCITOS-G5_without_Head_config.xml", argc, argv);
	RosScitosBase ros_scitos(n, &base);

	/*
	 * /schunk/position/joint_state -> publishes joint state for kinematics modules
	 * /schunk/target_pc/joint_state -> for received position control commands
	 * /schunk/target_vc/joint_state  <- for receiving velocity control commands
	 * /schunk/status -> to publish all the statuses
	 */

	ros::Subscriber targetJointStateSubscriberPositionControl = n.subscribe("/schunk/target_pc/joint_states", 1, &SchunkServer::targetJointStateCallbackPositionControl, &server);
	ros::Subscriber targetJointStateSubscriberVelocityControl = n.subscribe("/schunk/target_vc/joint_states", 1, &SchunkServer::targetJointStateCallbackVelocityControl, &server);


	ros::Subscriber emergency = n.subscribe("/emergency", 1, &SchunkServer::cb_emergency, &server);
	ros::Subscriber stop = n.subscribe("/stop", 1, &SchunkServer::cb_stop, &server);
	ros::Subscriber firstRef = n.subscribe("/firstRef", 1, &SchunkServer::cb_firstRef, &server);
	ros::Subscriber ack = n.subscribe("/ack", 1, &SchunkServer::cb_ack, &server);
	ros::Subscriber ackAll = n.subscribe("/ackAll", 1, &SchunkServer::cb_ackAll, &server);
	ros::Subscriber ref = n.subscribe("/ref", 1, &SchunkServer::cb_ref, &server);
	ros::Subscriber refAll = n.subscribe("/refAll", 1, &SchunkServer::cb_refAll, &server);
	ros::Subscriber current = n.subscribe("/current", 1, &SchunkServer::cb_current, &server);
	ros::Subscriber currentsMaxAll = n.subscribe("/currentsMaxAll", 1, &SchunkServer::cb_currentsMaxAll, &server);
	ros::Subscriber movePosition = n.subscribe("/movePosition", 1, &SchunkServer::cb_movePosition, &server);
//	n.subscribe("/movePositions", 1, &C_Callbacks::cb_movePositions, &listener);
	ros::Subscriber moveVelocity = n.subscribe("/moveVelocity", 1, &SchunkServer::cb_moveVelocity, &server);
	ros::Subscriber targetVelocity = n.subscribe("/targetVelocity", 1, &SchunkServer::cb_targetVelocity, &server);
	ros::Subscriber targetAcceleration = n.subscribe("/targetAcceleration", 1, &SchunkServer::cb_targetAcceleration, &server);
//	ros::Subscriber startPosition = n.subscribe("/startPosition", 1, &PubsAndSubs::cb_startPosition, &services);
  
	ros::Rate loop_rate(30);

	while (n.ok()) {
		ros::spinOnce();

		server.publishCurrentJointState();
		server.publishSchunkStatus();

		ros_scitos.loop();
		
		// This will adjust as needed per iteration
		loop_rate.sleep();

	}

	return 0;
}

