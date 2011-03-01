// bsd license blah blah
#include <cstring>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/KinematicSolverInfo.h>
#include "schunk_arm_kinematics_constraint_aware/GetVelocityIK.h"

#include "schunk_arm_kinematics_constraint_aware/schunk_arm_kinematics.h"

using std::string;

static const std::string IK_SERVICE = "get_ik";
static const std::string IK_VEL_SERVICE = "get_ik_vel";
static const std::string FK_SERVICE = "get_fk";
static const std::string IK_INFO_SERVICE = "get_ik_solver_info";
static const std::string FK_INFO_SERVICE = "get_fk_solver_info";

SchunkArmKinematics::SchunkArmKinematics(): nh_private ("~") {
}

bool SchunkArmKinematics::init() {
    // Get URDF XML
    std::string urdf_xml, full_urdf_xml;
    nh.param("urdf_xml",urdf_xml,std::string("robot_description"));
    nh.searchParam(urdf_xml,full_urdf_xml);
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result;
    if (!nh.getParam(full_urdf_xml, result)) {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return false;
    }

    // Get Root and Tip From Parameter Service
     if (!nh.getParam("/schunk_kinematics/root_name", root_name)) {
         ROS_FATAL("GenericIK: No root name found on parameter server");
         return false;
     }
     if (!nh.getParam("/schunk_kinematics/tip_name", tip_name)) {
         ROS_FATAL("GenericIK: No tip name found on parameter server");
         return false;
     }

//    root_name="PAM112_BaseConector"; // dirty :-)
//    tip_name="PAM100";

    // Load and Read Models
    if (!loadModel(result)) {
        ROS_FATAL("Could not load models!");
        return false;
    }

    // Get Solver Parameters
    int maxIterations;
    double epsilon;
    double lambda;

    nh_private.param("maxIterations", maxIterations, 1000);
    nh_private.param("epsilon", epsilon, 1e-2);
    nh_private.param("lambda", lambda, 0.01);


    ROS_INFO("IK Solver, maxIterations: %d, epsilon: %f, lambda: %f", maxIterations, epsilon, lambda);

    // Build Solvers
    fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
    
    ik_solver_vel = new KDL::ChainIkSolverVel_wdls(chain, epsilon, maxIterations);

    Eigen::MatrixXd jointWeights(7,7);
    for (uint i =0; i<7; i++)
	for (uint j=0; j<7; j++)
	    jointWeights(i,j) = 0.0;
    
    jointWeights(0,0) = 1.0;
    jointWeights(1,1) = 1.0;
    jointWeights(2,2) = 1.0;
    jointWeights(3,3) = 1.0;
    jointWeights(4,4) = 1.0;
    jointWeights(5,5) = 1.0;
    jointWeights(6,6) = 1.0;    
//     

    
    dynamic_cast<KDL::ChainIkSolverVel_wdls*>(ik_solver_vel)->setLambda(lambda);
    dynamic_cast<KDL::ChainIkSolverVel_wdls*>(ik_solver_vel)->setWeightJS(jointWeights);

    Eigen::MatrixXd coordsWeights(6,6);
    for (uint i =0; i<6; i++)
	for (uint j=0; j<6; j++)
	    coordsWeights(i,j) = 0.0;
    
    coordsWeights(0,0) = 1.0;
    coordsWeights(1,1) = 1.0;
    coordsWeights(2,2) = 1.0;
    coordsWeights(3,3) = 1.0;
    coordsWeights(4,4) = 1.0;
    coordsWeights(5,5) = 1.0;

    dynamic_cast<KDL::ChainIkSolverVel_wdls*>(ik_solver_vel)->setWeightTS(coordsWeights);
    
    
//    std::cerr<<"Pezzotto: damping factor is "<<lambda<<" eps is "<<eps<<"\n";
//    ROS_INFO("Setting damping factor to %f", lambda);

    
    ik_solver_pos = new KDL::ChainIkSolverPos_NR_JL(chain, joint_min, joint_max,
            *fk_solver, *ik_solver_vel, maxIterations, epsilon);

    ROS_INFO("Advertising services");
    fk_service = nh_private.advertiseService(FK_SERVICE,&SchunkArmKinematics::getPositionFK,this);
    ik_service = nh_private.advertiseService(IK_SERVICE,&SchunkArmKinematics::getPositionIK,this);
    ik_vel_service = nh_private.advertiseService(IK_VEL_SERVICE,&SchunkArmKinematics::getVelocityIK,this);
    ik_solver_info_service = nh_private.advertiseService(IK_INFO_SERVICE,&SchunkArmKinematics::getIKSolverInfo,this);
    fk_solver_info_service = nh_private.advertiseService(FK_INFO_SERVICE,&SchunkArmKinematics::getFKSolverInfo,this);

    return true;
}

bool SchunkArmKinematics::loadModel(const std::string xml) {
    urdf::Model robot_model;
    KDL::Tree tree;

    if (!robot_model.initString(xml)) {
        ROS_FATAL("Could not initialize robot model");
        return -1;
    }
    if (!kdl_parser::treeFromString(xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        return false;
    }
    if (!tree.getChain(root_name, tip_name, chain)) {
        ROS_ERROR("Could not initialize chain object");
        return false;
    }

    if (!readJoints(robot_model)) {
        ROS_FATAL("Could not read information about the joints");
        return false;
    }

    return true;
}

bool SchunkArmKinematics::readJoints(urdf::Model &robot_model) {
    num_joints = 0;
    num_links = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;

    info.link_names.push_back(link->name);

    while (link && link->name != root_name) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (!joint) {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            return false;
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
            num_joints++;
        }
        link = robot_model.getLink(link->getParent()->name);
        num_links++;
        info.link_names.insert(info.link_names.begin(), link->name);
    }

    joint_min.resize(num_joints);
    joint_max.resize(num_joints);
    info.joint_names.resize(num_joints);
    info.limits.resize(num_joints);

    link = robot_model.getLink(tip_name);
    unsigned int i = 0;
    while (link && i < num_joints) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

            float lower, upper;
            int hasLimits;
            if ( joint->type != urdf::Joint::CONTINUOUS ) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
                hasLimits = 1;
            } else {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
            }
            int index = num_joints - i -1;

            joint_min.data[index] = lower;
            joint_max.data[index] = upper;
            info.joint_names[index] = joint->name;
            info.limits[index].joint_name = joint->name;
            info.limits[index].has_position_limits = hasLimits;
            info.limits[index].min_position = lower;
            info.limits[index].max_position = upper;
            i++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    return true;
}


int SchunkArmKinematics::getJointIndex(const std::string &name) {
    for (unsigned int i=0; i < info.joint_names.size(); i++) {
        if (info.joint_names[i] == name)
            return i;
    }
    return -1;
}

int SchunkArmKinematics::getKDLSegmentIndex(const std::string &name) {
    int i=0; 
    while (i < (int)chain.getNrOfSegments()) {
        if (chain.getSegment(i).getName() == name) {
            return i+1;
        }
        i++;
    }
    return -1;
}


bool SchunkArmKinematics::getPositionIK(kinematics_msgs::GetPositionIK::Request &request,
                               kinematics_msgs::GetPositionIK::Response &response) {

    geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;
    tf::Stamped<tf::Pose> transform;
    tf::poseStampedMsgToTF( pose_msg_in, transform );

    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(num_joints);
    for (unsigned int i=0; i < num_joints; i++) {
        int tmp_index = getJointIndex(request.ik_request.ik_seed_state.joint_state.name[i]);
        if (tmp_index >=0) {
            jnt_pos_in(tmp_index) = request.ik_request.ik_seed_state.joint_state.position[i];
        } else {
            ROS_ERROR("i: %d, No joint index for %s",i,request.ik_request.ik_seed_state.joint_state.name[i].c_str());
        }
    }

    // populate F_dest from tf::Transform parameter
    KDL::Frame F_dest;
    tf::TransformTFToKDL(transform, F_dest);

    int ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);

    if (ik_valid >= 0) {
        response.solution.joint_state.name = info.joint_names;
        response.solution.joint_state.position.resize(num_joints);
        for (unsigned int i=0; i < num_joints; i++) {
            response.solution.joint_state.position[i] = jnt_pos_out(i);
            ROS_DEBUG("IK Solution: %s %d: %f",response.solution.joint_state.name[i].c_str(),i,jnt_pos_out(i));
        }
        response.error_code.val = response.error_code.SUCCESS;
        return true;
    } else {
        ROS_DEBUG("An IK solution could not be found");
        response.error_code.val = response.error_code.NO_IK_SOLUTION;
        return true;
    }
}


/**
 *   Get the inverse kinematics with velocity info - added by Chris
 */
bool SchunkArmKinematics::getVelocityIK(schunk_arm_kinematics_constraint_aware::GetVelocityIK::Request &request,
		schunk_arm_kinematics_constraint_aware::GetVelocityIK::Response &response) {

//    geometry_msgs::TwistStamped pose_msg_in = request.ik_request.twist;
//    tf::Stamped<tf::Pose> transform;
//    tf::poseStampedMsgToTF( pose_msg_in, transform );

	ROS_INFO("In velocity kinematixs funtion.");
    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(num_joints);
    for (unsigned int i=0; i < num_joints; i++) {
        int tmp_index = getJointIndex(request.ik_request.robot_state.joint_state.name[i]);
        if (tmp_index >=0) {
            jnt_pos_in(tmp_index) = request.ik_request.robot_state.joint_state.position[i];
        } else {
            ROS_ERROR("i: %d, No joint index for %s",i,request.ik_request.robot_state.joint_state.name[i].c_str());
        }
    }

	ROS_INFO("Populated the current jnt array");


    // populate F_dest from tf::Transform parameter
//    KDL::Frame F_dest;
    KDL::Twist T_dest;
//    tf::TransformTFToKDL(transform, F_dest);
    tf::TwistMsgToKDL(request.ik_request.twist,T_dest);

	ROS_INFO("About to call the IK solver (twist = (%f,%f,%f))", T_dest.rot.data[0],
			T_dest.rot.data[1],
			T_dest.rot.data[2]);

    //virtual int 	CartToJnt (const JntArray &q_in, const Twist &v_in, JntArray &qdot_out)
	jnt_pos_out.resize(jnt_pos_in.rows());
    int ik_valid = ik_solver_vel->CartToJnt(jnt_pos_in, T_dest, jnt_pos_out);

	ROS_INFO("Solver done.");

    if (ik_valid >= 0) {
        response.solution.joint_state.name = info.joint_names;
        response.solution.joint_state.velocity.resize(num_joints);
        for (unsigned int i=0; i < num_joints; i++) {
            response.solution.joint_state.velocity[i] = jnt_pos_out(i);
            ROS_INFO("IK Solution: %s %d: %f",response.solution.joint_state.name[i].c_str(),i,jnt_pos_out(i));
        }
        response.error_code.val = response.error_code.SUCCESS;
        return true;
    } else {
        ROS_DEBUG("An IK solution could not be found");
        response.error_code.val = response.error_code.NO_IK_SOLUTION;
        return true;
    }
}

bool SchunkArmKinematics::getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request,
                                 kinematics_msgs::GetKinematicSolverInfo::Response &response) {
    response.kinematic_solver_info = info;
    return true;
}

bool SchunkArmKinematics::getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request,
                                 kinematics_msgs::GetKinematicSolverInfo::Response &response) {
    response.kinematic_solver_info = info;
    return true;
}

bool SchunkArmKinematics::getPositionFK(kinematics_msgs::GetPositionFK::Request &request,
                               kinematics_msgs::GetPositionFK::Response &response) {
    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    jnt_pos_in.resize(num_joints);
    for (unsigned int i=0; i < num_joints; i++) {
        int tmp_index = getJointIndex(request.robot_state.joint_state.name[i]);
        if (tmp_index >=0)
            jnt_pos_in(tmp_index) = request.robot_state.joint_state.position[i];
    }

    response.pose_stamped.resize(request.fk_link_names.size());
    response.fk_link_names.resize(request.fk_link_names.size());

    bool valid = true;
    for (unsigned int i=0; i < request.fk_link_names.size(); i++) {
        int segmentIndex = getKDLSegmentIndex(request.fk_link_names[i]);
        ROS_DEBUG("End effector index: %d",segmentIndex);
        ROS_DEBUG("Chain indices: %d",chain.getNrOfSegments());
        if (fk_solver->JntToCart(jnt_pos_in,p_out,segmentIndex) >=0) {
            tf_pose.frame_id_ = root_name;
            tf_pose.stamp_ = ros::Time();
            tf::PoseKDLToTF(p_out,tf_pose);
            try {
            	ROS_INFO("transform from frame %s",root_name.c_str());
            	ROS_INFO("          to frame %s",request.header.frame_id.c_str());
//                tf_listener.transformPose(request.header.frame_id,tf_pose,tf_pose);
            } catch (...) {
                ROS_ERROR("Could not transform FK pose to frame: %s",request.header.frame_id.c_str());
                response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
                return false;
            }
            tf::poseStampedTFToMsg(tf_pose,pose);
            response.pose_stamped[i] = pose;
            response.fk_link_names[i] = request.fk_link_names[i];
            response.error_code.val = response.error_code.SUCCESS;
        } else {
            ROS_ERROR("Could not compute FK for %s",request.fk_link_names[i].c_str());
            response.error_code.val = response.error_code.NO_FK_SOLUTION;
            valid = false;
        }
    }
    return true;
}



