/**     Person Grabber
 *
 *  This node publishes a pointcloud of points corresponding to a user. The skeleton tracking
 *  library provides the person extraction. Only the points for one user are published, multiple
 *  users are not currently tracked.
 *
 *  A lot of this code is borrowed from openni_camera, and openni_tracker. 
 *
 *  ---------------------------------------------------------------------
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
 *  ---------------------------------------------------------------------
 *
**/
//@todo This is really annoying me! 32-bit? Not sure....
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <boost/thread/mutex.hpp>


#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include "pcl/filters/voxel_grid.h"


using std::string;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue; // Blue channel
    unsigned char Green; // Green channel
    unsigned char Red; // Red channel
    unsigned char Alpha; // Alpha channel
  };
  float float_value;
  long long_value;
} RGBValue;

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie);


#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
	}


#define AVG(a,b) (((int)(a) + (int)(b)) >> 1)
#define AVG3(a,b,c) (((int)(a) + (int)(b) + (int)(c)) / 3)
#define AVG4(a,b,c,d) (((int)(a) + (int)(b) + (int)(c) + (int)(d)) >> 2)
#define WAVG4(a,b,c,d,x,y)  ( ( ((int)(a) + (int)(b)) * (int)(x) + ((int)(c) + (int)(d)) * (int)(y) ) / ( 2 * ((int)(x) + (int(y))) ) )


// The main class 

class PersonGrabber {
public:
	ros::NodeHandle nh;
    ros::Publisher pub_depth_points2_;
    ros::Publisher pezz_publisher;
    /** \brief Approximate synchronization for XYZRGB point clouds. */
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
	typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
	boost::shared_ptr<Synchronizer> depth_rgb_sync_;

	xn::Context        context;
	xn::DepthGenerator depth_generator;
	xn::UserGenerator  user_generator;
    xn::ImageGenerator image_generator;
    XnStatus rc;
	XnBool bNeedPose;
	XnChar strPose[20];
	/** \brief PointCloud2 data. */
    sensor_msgs::PointCloud2 cloud2_;
    /** \brief Image data. */
    sensor_msgs::Image rgb_image;
    /**brief the distance between the IR projector and the IR camera*/
	XnDouble baseline_;

	/** \brief focal length in pixels for the IR camera in VGA resolution*/
	XnUInt64 depth_focal_length_VGA_;

	/** the value for shadow (occluded pixels) */
	XnUInt64 shadow_value_;

	/** the value for pixels without a valid disparity measurement */
	XnUInt64 no_sample_value_;

	static const double rgb_focal_length_VGA_ = 525;

    // The subsampling ratio of the point cloud
    // This is necessary as too much data was being published, and a low powered PC cant keep up with both tracking
    // and publishing...
	static const uint subsample = 8;

	int userId;

	string frame_id;
	string frame_points;
public:

    // The frame_id for publishing is set here as "/rgbd". 
	PersonGrabber(ros::NodeHandle &nodeHandle) : nh(nodeHandle), frame_id("/rgbd"), frame_points("/rgbd") {
		bNeedPose=FALSE;

		userId=-1;

		// Ros topics and bits
		pub_depth_points2_ = nh.advertise<PointCloud >("/rgbd/points2", 15);

		SyncPolicy sync_policy(4); // queue size
		/// @todo Set inter-message lower bound, age penalty, max interval to lower latency
		// Connect no inputs, we'll add messages manually
		depth_rgb_sync_.reset( new Synchronizer(sync_policy) );
		depth_rgb_sync_->registerCallback(boost::bind(&PersonGrabber::publishXYZRGBPointCloud, this, _1, _2));


		// Openni bits and bobs
		string configFilename = ros::package::getPath("person_grabber") + "/openni_tracker.xml";
		XnStatus nRetVal = context.InitFromXmlFile(configFilename.c_str());
		CHECK_RC(nRetVal, "InitFromXml");

		// ================ Depth Generator ==================
		nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth_generator);
		CHECK_RC(nRetVal, "Find depth generator");
		 XnDouble pixel_size;
		// Read parameters from the camera
		if (depth_generator.GetRealProperty ("ZPPS", pixel_size) != XN_STATUS_OK)
			ROS_ERROR ("[person_grabber] Could not read pixel size!");

		// pixel size @ VGA = pixel size @ SXGA x 2
		pixel_size *= 2.0;

		// focal length of IR camera in pixels for VGA resolution
		XnUInt64 depth_focal_length_VGA;
		if (depth_generator.GetIntProperty ("ZPD", depth_focal_length_VGA) != XN_STATUS_OK)
			ROS_ERROR ("[person_grabber] Could not read virtual plane distance!");

		if (depth_generator.GetRealProperty ("LDDIS", baseline_) != XN_STATUS_OK)
			ROS_ERROR ("[person_grabber] Could not read base line!");

		// baseline from cm -> meters
		baseline_ *= 0.01;

		//focal length from mm -> pixels (valid for 640x480)
		depth_focal_length_VGA_ = (double)depth_focal_length_VGA/pixel_size;

		if (depth_generator.GetIntProperty ("ShadowValue", shadow_value_) != XN_STATUS_OK)
			ROS_WARN ("[person_grabber] Could not read shadow value!");

		if (depth_generator.GetIntProperty ("NoSampleValue", no_sample_value_) != XN_STATUS_OK)
			ROS_WARN ("[person_grabber] Could not read no sample value!");




		// ================== User Generator =======================
		nRetVal = context.FindExistingNode(XN_NODE_TYPE_USER, user_generator);
		if (nRetVal != XN_STATUS_OK) {
			printf("creating user gen as not found.\n");

			nRetVal = user_generator.Create(context);
			CHECK_RC(nRetVal, "Find user generator");
		}

		if (!user_generator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
			printf("Supplied user generator doesn't support skeleton\n");
		}

		XnCallbackHandle hUserCallbacks;
		user_generator.RegisterUserCallbacks(User_NewUser, User_LostUser, this, hUserCallbacks);

		XnCallbackHandle hCalibrationCallbacks;
		user_generator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, this, hCalibrationCallbacks);

		if (user_generator.GetSkeletonCap().NeedPoseForCalibration()) {
			bNeedPose = TRUE;
			if (!user_generator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
				printf("Pose required, but not supported\n");
			}

			XnCallbackHandle hPoseCallbacks;
			XnStatus cb = user_generator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, this, hPoseCallbacks);
			CHECK_RC(cb,"Oh bother: ");
			if (cb != XN_STATUS_OK) {
				printf("Die you moron!\n");
				exit(-1);
			}

			user_generator.GetSkeletonCap().GetCalibrationPose(strPose);
		}

		user_generator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);


		// ============================ Image Generator =====================================
		if (!image_generator.IsValid())
		{
			rc = image_generator.Create (context);

			if (rc != XN_STATUS_OK)
			{
				ROS_ERROR ("[person_grabber] Failed to create ImageGenerator: %s", xnGetStatusString (rc));
			}
		}
		XnMapOutputMode mode;
		mode.nXRes = 640;
		mode.nYRes = 480;
		mode.nFPS  = 30;
		if (image_generator.SetMapOutputMode (mode) != XN_STATUS_OK)
		{
			ROS_ERROR("[person_grabber] Failed to set image output mode");
		}

		// Set up the Kinect ( no PrimeSense :-( )
		// InputFormat should be 6 = uncompressed Bayer for Kinect
		if (image_generator.SetIntProperty ("InputFormat", 6) != XN_STATUS_OK)
			ROS_ERROR ("[person_grabber] Error setting the image input format to Uncompressed 8-bit BAYER!");

		// RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
		if (depth_generator.SetIntProperty ("RegistrationType", 2) != XN_STATUS_OK)
			ROS_WARN ("[person_grabber] Error enabling registration!");

		// Grayscale: bypass debayering -> gives us bayer pattern!
		if (image_generator.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT )!= XN_STATUS_OK)
		{
			ROS_ERROR("[person_grabber] Failed to set image pixel format to 8bit-grayscale");
		}

		rc = depth_generator.GetAlternativeViewPointCap().SetViewPoint( image_generator );
		if (rc != XN_STATUS_OK)
		{
			ROS_ERROR ("[person_grabber::spin] Error in switching on depth stream registration: %s", xnGetStatusString (rc));
		}

		cloud2_.header.frame_id = frame_points; 
		cloud2_.height = XN_VGA_Y_RES / subsample;
		cloud2_.width  = XN_VGA_X_RES / subsample;
		cloud2_.fields.resize( 4 );
		cloud2_.fields[0].name = "x";
		cloud2_.fields[1].name = "y";
		cloud2_.fields[2].name = "z";
		cloud2_.fields[3].name = "rgb";
		int offset = 0;
		for (size_t s = 0; s < cloud2_.fields.size (); ++s, offset += sizeof(float))
		{
			cloud2_.fields[s].offset   = offset;
			cloud2_.fields[s].count    = 1;
			cloud2_.fields[s].datatype = sensor_msgs::PointField::FLOAT32;
		}
		cloud2_.point_step = offset;
		cloud2_.row_step   = cloud2_.point_step * cloud2_.width; /// @todo *offset?
		cloud2_.data.resize (cloud2_.row_step   * cloud2_.height);
		cloud2_.is_dense = false;

		// Assemble the color image data
		rgb_image.height = 480;
		rgb_image.width = 640;
		rgb_image.encoding = sensor_msgs::image_encodings::RGB8;
		rgb_image.data.resize (640 * 480 * 3);
		rgb_image.step = 640 * 3;


	}

	void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
	    static tf::TransformBroadcaster br;

	    XnSkeletonJointPosition joint_position;
	    user_generator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
	    double x =  -joint_position.position.X / 1000.0;
	    double y =  -joint_position.position.Y / 1000.0;	// do we want to flip the points 180 about  Z axis? no? ok then.
	    double z = joint_position.position.Z / 1000.0;

	    XnSkeletonJointOrientation joint_orientation;
	    user_generator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);
//	    user_generator.GetSkeletonCap().

	    XnFloat* m = joint_orientation.orientation.elements;
	    KDL::Rotation rotation(m[0], m[1], m[2],
	    					   m[3], m[4], m[5],
	    					   m[6], m[7], m[8]);
	    KDL::Rotation rotZ(-1, 0, 0,
	    					 0, 1, 0,
	    					 0, 0, 0);
//@todo check I need this rotation.
	    double qx, qy, qz, qw;
	    (rotation * rotZ).GetQuaternion(qx, qy, qz, qw);

	    tf::Transform transform;
	    transform.setOrigin(tf::Vector3(x, y, z));
	    transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
	    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
	}

	void publishTransforms() {
	    XnUserID users[15];
	    XnUInt16 users_count = 15;
	    user_generator.GetUsers(users, users_count);

	    for (int i = 0; i < users_count; ++i) {
	        XnUserID user = users[i];
	        if (!user_generator.GetSkeletonCap().IsTracking(user))
	            continue;


	        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
	        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
	        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

	        publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
	        publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
	        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

	        publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
	        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
	        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

	        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
	        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
	        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

	        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
	        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
	        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
	    }
	}

	void bayer2RGB ( const xn::ImageMetaData& bayer, sensor_msgs::Image& image, int method ) const
	{
	  if (bayer.XRes() == image.width && bayer.YRes() == image.height)
	  {
	    register const XnUInt8 *bayer_pixel = bayer.Data();
	    register unsigned yIdx, xIdx;

	    int line_step = image.width;
	    int line_step2 = image.width << 1;

	    int rgb_line_step  = line_step * 3;             // previous color line
	    register unsigned char *rgb_pixel = (unsigned char *)&image.data[0];

	    if (method == 1)
	    {
	      // first two pixel values for first two lines
	      // Bayer         0 1 2
	      //         0     G r g
	      // line_step     b g b
	      // line_step2    g r g

	      rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
	      rgb_pixel[1] = bayer_pixel[0];    // green pixel
	      rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

	      // Bayer         0 1 2
	      //         0     g R g
	      // line_step     b g b
	      // line_step2    g r g
	      //rgb_pixel[3] = bayer_pixel[1];
	      rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
	      rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

	      // BGBG line
	      // Bayer         0 1 2
	      //         0     g r g
	      // line_step     B g b
	      // line_step2    g r g
	      rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
	      rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
	      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

	      // pixel (1, 1)  0 1 2
	      //         0     g r g
	      // line_step     b G b
	      // line_step2    g r g
	      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
	      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
	      //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

	      rgb_pixel += 6;
	      bayer_pixel += 2;
	      // rest of the first two lines
	      for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
	      {
	        // GRGR line
	        // Bayer        -1 0 1 2
	        //           0   r G r g
	        //   line_step   g b g b
	        // line_step2    r g r g
	        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
	        rgb_pixel[1] = bayer_pixel[0];
	        rgb_pixel[2] = bayer_pixel[line_step + 1];

	        // Bayer        -1 0 1 2
	        //          0    r g R g
	        //  line_step    g b g b
	        // line_step2    r g r g
	        rgb_pixel[3] = bayer_pixel[1];
	        rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
	        rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

	        // BGBG line
	        // Bayer         -1 0 1 2
	        //         0      r g r g
	        // line_step      g B g b
	        // line_step2     r g r g
	        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
	        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
	        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

	        // Bayer         -1 0 1 2
	        //         0      r g r g
	        // line_step      g b G b
	        // line_step2     r g r g
	        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
	        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
	        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
	      }

	      // last two pixel values for first two lines
	      // GRGR line
	      // Bayer        -1 0 1
	      //           0   r G r
	      //   line_step   g b g
	      // line_step2    r g r
	      rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
	      rgb_pixel[1] = bayer_pixel[0];
	      rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

	      // Bayer        -1 0 1
	      //          0    r g R
	      //  line_step    g b g
	      // line_step2    r g r
	      rgb_pixel[3] = bayer_pixel[1];
	      rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
	      //rgb_pixel[5] = bayer_pixel[line_step];

	      // BGBG line
	      // Bayer        -1 0 1
	      //          0    r g r
	      //  line_step    g B g
	      // line_step2    r g r
	      rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
	      rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
	      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

	      // Bayer         -1 0 1
	      //         0      r g r
	      // line_step      g b G
	      // line_step2     r g r
	      rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
	      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
	      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

	      bayer_pixel += line_step + 2;
	      rgb_pixel += rgb_line_step + 6;

	      // main processing
	      for (yIdx = 2; yIdx < image.height-2; yIdx += 2)
	      {
	        // first two pixel values
	        // Bayer         0 1 2
	        //        -1     b g b
	        //         0     G r g
	        // line_step     b g b
	        // line_step2    g r g

	        rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
	        rgb_pixel[1] = bayer_pixel[0];    // green pixel
	        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] ); // blue;

	        // Bayer         0 1 2
	        //        -1     b g b
	        //         0     g R g
	        // line_step     b g b
	        // line_step2    g r g
	        //rgb_pixel[3] = bayer_pixel[1];
	        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
	        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

	        // BGBG line
	        // Bayer         0 1 2
	        //         0     g r g
	        // line_step     B g b
	        // line_step2    g r g
	        rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
	        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
	        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

	        // pixel (1, 1)  0 1 2
	        //         0     g r g
	        // line_step     b G b
	        // line_step2    g r g
	        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
	        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
	        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

	        rgb_pixel += 6;
	        bayer_pixel += 2;
	        // continue with rest of the line
	        for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
	        {
	          // GRGR line
	          // Bayer        -1 0 1 2
	          //          -1   g b g b
	          //           0   r G r g
	          //   line_step   g b g b
	          // line_step2    r g r g
	          rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
	          rgb_pixel[1] = bayer_pixel[0];
	          rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

	          // Bayer        -1 0 1 2
	          //          -1   g b g b
	          //          0    r g R g
	          //  line_step    g b g b
	          // line_step2    r g r g
	          rgb_pixel[3] = bayer_pixel[1];
	          rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
	          rgb_pixel[5] = AVG4( bayer_pixel[-line_step], bayer_pixel[2-line_step], bayer_pixel[line_step], bayer_pixel[line_step+2]);

	          // BGBG line
	          // Bayer         -1 0 1 2
	          //         -1     g b g b
	          //          0     r g r g
	          // line_step      g B g b
	          // line_step2     r g r g
	          rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1], bayer_pixel[line_step2+1], bayer_pixel[-1], bayer_pixel[line_step2-1] );
	          rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0], bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
	          rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

	          // Bayer         -1 0 1 2
	          //         -1     g b g b
	          //          0     r g r g
	          // line_step      g b G b
	          // line_step2     r g r g
	          rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
	          rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
	          rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
	        }

	        // last two pixels of the line
	        // last two pixel values for first two lines
	        // GRGR line
	        // Bayer        -1 0 1
	        //           0   r G r
	        //   line_step   g b g
	        // line_step2    r g r
	        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
	        rgb_pixel[1] = bayer_pixel[0];
	        rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

	        // Bayer        -1 0 1
	        //          0    r g R
	        //  line_step    g b g
	        // line_step2    r g r
	        rgb_pixel[3] = bayer_pixel[1];
	        rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
	        //rgb_pixel[5] = bayer_pixel[line_step];

	        // BGBG line
	        // Bayer        -1 0 1
	        //          0    r g r
	        //  line_step    g B g
	        // line_step2    r g r
	        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
	        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
	        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

	        // Bayer         -1 0 1
	        //         0      r g r
	        // line_step      g b G
	        // line_step2     r g r
	        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
	        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
	        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

	        bayer_pixel += line_step + 2;
	        rgb_pixel += rgb_line_step + 6;
	      }

	      //last two lines
	      // Bayer         0 1 2
	      //        -1     b g b
	      //         0     G r g
	      // line_step     b g b

	      rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
	      rgb_pixel[1] = bayer_pixel[0];    // green pixel
	      rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

	      // Bayer         0 1 2
	      //        -1     b g b
	      //         0     g R g
	      // line_step     b g b
	      //rgb_pixel[3] = bayer_pixel[1];
	      rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
	      rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

	      // BGBG line
	      // Bayer         0 1 2
	      //        -1     b g b
	      //         0     g r g
	      // line_step     B g b
	      //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
	      rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[0] , bayer_pixel[line_step+1] );
	      rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

	      // Bayer         0 1 2
	      //        -1     b g b
	      //         0     g r g
	      // line_step     b G b
	      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
	      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
	      rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

	      rgb_pixel += 6;
	      bayer_pixel += 2;
	      // rest of the last two lines
	      for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
	      {
	        // GRGR line
	        // Bayer       -1 0 1 2
	        //        -1    g b g b
	        //         0    r G r g
	        // line_step    g b g b
	        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
	        rgb_pixel[1] = bayer_pixel[0];
	        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

	        // Bayer       -1 0 1 2
	        //        -1    g b g b
	        //         0    r g R g
	        // line_step    g b g b
	        rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
	        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
	        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[-line_step+2] );

	        // BGBG line
	        // Bayer       -1 0 1 2
	        //        -1    g b g b
	        //         0    r g r g
	        // line_step    g B g b
	        rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[-1], bayer_pixel[1] );
	        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
	        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];


	        // Bayer       -1 0 1 2
	        //        -1    g b g b
	        //         0    r g r g
	        // line_step    g b G b
	        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
	        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
	        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
	      }

	      // last two pixel values for first two lines
	      // GRGR line
	      // Bayer       -1 0 1
	      //        -1    g b g
	      //         0    r G r
	      // line_step    g b g
	      rgb_pixel[rgb_line_step    ] = rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
	      rgb_pixel[1] = bayer_pixel[0];
	      rgb_pixel[5] = rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

	      // Bayer       -1 0 1
	      //        -1    g b g
	      //         0    r g R
	      // line_step    g b g
	      rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
	      rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[line_step+1], bayer_pixel[-line_step+1] );
	      //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

	      // BGBG line
	      // Bayer       -1 0 1
	      //        -1    g b g
	      //         0    r g r
	      // line_step    g B g
	      //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
	      rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
	      rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

	      // Bayer       -1 0 1
	      //        -1    g b g
	      //         0    r g r
	      // line_step    g b G
	      //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
	      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
	      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
	    }

	  }
	  else
	  {printf("hmm");
	    // get each or each 2nd pixel group to find rgb values!
	    register unsigned bayerXStep = bayer.XRes() / image.width;
	    register unsigned bayerYSkip = (bayer.YRes() / image.height - 1) * bayer.XRes();

	    // Downsampling and debayering at once
	    register const XnUInt8* bayer_buffer = bayer.Data();
	    register unsigned char* rgb_buffer = (unsigned char*)&image.data[0];

	    for( register unsigned yIdx = 0; yIdx < image.height; ++yIdx, bayer_buffer += bayerYSkip ) // skip a line
	    {
	      for( register unsigned xIdx = 0; xIdx < image.width; ++xIdx, rgb_buffer += 3, bayer_buffer += bayerXStep )
	      {
	        rgb_buffer[ 2 ] = bayer_buffer[ bayer.XRes() ];
	        rgb_buffer[ 1 ] = AVG( bayer_buffer[0], bayer_buffer[ bayer.XRes() + 1] );
	        rgb_buffer[ 0 ] = bayer_buffer[ 1 ];
	      }
	    }
	  }
	}

	void processDepth() {
//		printf("Doing a bit of depth.\n");
		// == The depth bit ===
		/// @todo Some sort of offset based on the hardware time
		ros::Time time = ros::Time::now();

		xn::DepthMetaData depth_md;
		depth_generator.GetMetaData( depth_md );

		sensor_msgs::ImagePtr depth_ptr = boost::make_shared<sensor_msgs::Image>();
		depth_ptr->header.stamp = time;
		depth_ptr->header.frame_id = std::string("/not/sure");//disp_image_.header.frame_id;
		depth_ptr->header.seq = depth_md.FrameID();
		sensor_msgs::fillImage(*depth_ptr, sensor_msgs::image_encodings::TYPE_16UC1,
							 depth_md.YRes(), depth_md.XRes(), depth_md.XRes() * sizeof(uint16_t),
							 (void*)depth_md.Data());
		depth_rgb_sync_->add<0>(depth_ptr);

	}

	void processRgb() {
		// === The Rgb bit ===
		ros::Time time = ros::Time::now();
		xn::ImageMetaData image_md;
		image_generator.GetMetaData( image_md );

		rgb_image.header.stamp   = time;
		rgb_image.header.seq      = image_md.FrameID();

		bayer2RGB( image_md, rgb_image, 1 );

		sensor_msgs::ImageConstPtr rgb_ptr = boost::make_shared<const sensor_msgs::Image> (rgb_image);

//		if (pub_depth_points2_.getNumSubscribers() > 0 && config_.point_cloud_type == OpenNI_XYZRGB)
			depth_rgb_sync_->add<1>(rgb_ptr);
	}

	void processUsers() {

	}

	void spin() {
		rc = context.StartGeneratingAll();
		CHECK_RC(rc, "StartGenerating");
		ros::Rate r(30);
		while (ros::ok()) {
			rc = depth_generator.GetAlternativeViewPointCap().SetViewPoint( image_generator );
			if (rc != XN_STATUS_OK)		{
				ROS_ERROR ("[person_grabber::spin] Error in switching on depth stream registration: %s", xnGetStatusString (rc));
				return;
			}

			context.WaitAndUpdateAll();
			if (depth_generator.IsDataNew())
				processDepth();
			if (image_generator.IsDataNew())
				processRgb();
			if (user_generator.IsDataNew()) {
				processUsers();
				publishTransforms();
			}

			r.sleep();
		}
	}


	void publishXYZRGBPointCloud ( const sensor_msgs::ImageConstPtr& depth_msg,
										 const sensor_msgs::ImageConstPtr& rgb_msg )
	{
		PointCloud::Ptr cloud_out = boost::make_shared<PointCloud> ();

		PointCloud filtered_cloud;

		cloud_out->header = cloud2_.header;
		cloud_out->height = cloud2_.height;
		cloud_out->width = cloud2_.width;
		cloud_out->is_dense = cloud2_.is_dense;

		cloud_out->points.resize(cloud_out->height * cloud_out->width);

		float bad_point = std::numeric_limits<float>::quiet_NaN ();
		int depth_idx = 0;
		float constant = 0.001 / rgb_focal_length_VGA_;

		unsigned depthStep = depth_msg->width / cloud_out->width;
		unsigned depthSkip = (depth_msg->height / cloud_out->height - 1) * depth_msg->width;

		int centerX = cloud2_.width >> 1;
		int centerY = cloud2_.height >> 1;
		constant *= depthStep;

		unsigned char* rgb_buffer = (unsigned char*)&rgb_image.data[0];
		RGBValue color;
		color.Alpha = 0;

		int color_idx = 0;
		unsigned colorStep = 3 * rgb_image.width / cloud_out->width;
		unsigned colorSkip = 3 * (rgb_image.height / cloud_out->height - 1) * rgb_image.width;

//		printf("Synced in tock two.\n");

		/// @todo Right now we always copy the raw depth data into a sensor_msgs/Image. In principle this
		/// isn't completely avoidable, as the synchronizer may wait for the next depth image before choosing
		/// the old data, which is overwritten by that point. In practice though we should optimize to avoid
		/// this almost always.
		const uint16_t* depth_md = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
//		printf("Synced in tock three\n");

		xn::SceneMetaData sceneMD;
		if (userId!=-1) {
			user_generator.GetUserPixels(userId, sceneMD);
//			user_generator.Ge
		} else
			return;

//		sceneMD.Data()

		for (int v = 0; v < (int)cloud_out->height; ++v, depth_idx += depthSkip, color_idx += colorSkip)
		{
			for (int u = 0; u < (int)cloud_out->width; ++u, depth_idx += depthStep, color_idx += colorStep)
			{
				pcl::PointXYZRGB& pt = (*cloud_out)(u, v);

				// Check the point is part of the user or scrap it
				if (sceneMD.Data()[depth_idx] != userId) {
					// not valid
					pt.x = bad_point;
					pt.y = bad_point;
					pt.z = bad_point;
					pt.rgb = bad_point;
					continue;
				}

				// Check for invalid measurements
				if (depth_md[depth_idx] == 0 ||
				  depth_md[depth_idx] == no_sample_value_ ||
				  depth_md[depth_idx] == shadow_value_)
				{
					// not valid
					pt.x = bad_point;
					pt.y = bad_point;
					pt.z = bad_point;
					pt.rgb = bad_point;
					continue;
				}

				// Fill in XYZ
				pt.x = - (u - centerX) * depth_md[depth_idx] * constant; // crazy?
				pt.y = (v - centerY) * depth_md[depth_idx] * constant;
				pt.z = depth_md[depth_idx] * 0.001;

				// Fill in color
				color.Red   = rgb_buffer[color_idx];
				color.Green = rgb_buffer[color_idx + 1];
				color.Blue  = rgb_buffer[color_idx + 2];
				pt.rgb = color.float_value;
			}
		}
//		printf("Synced in tock four.\n");

		cloud_out->header.stamp = std::max( depth_msg->header.stamp, rgb_msg->header.stamp );
//		cloud_out->header.frame_id = rgb_msg->header.frame_id; //why?
		cloud_out->header.seq = std::max( depth_msg->header.seq, rgb_msg->header.seq );


		pub_depth_points2_.publish (cloud_out);
//		printf("  I did it. Size=%d\n",cloud_out->points.size());
	}

	~PersonGrabber() {
		context.StopGeneratingAll();
//		context.Shutdown();
	}
};

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	printf("New User\n");
	PersonGrabber *grabber;
	grabber = (PersonGrabber*)pCookie;

	if (grabber->userId==-1) {
		if (grabber->bNeedPose)
			grabber->user_generator.GetPoseDetectionCap().StartPoseDetection(grabber->strPose, nId);
		else
			grabber->user_generator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
	printf("  ->ID = %d\n", nId);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	PersonGrabber *grabber;
	grabber = (PersonGrabber*)pCookie;

	printf("Lost user %d\n", nId);
	if (nId == grabber->userId)
		grabber->userId=-1;
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	printf("Calibration started for user %d\n", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	PersonGrabber *grabber;
	grabber = (PersonGrabber*)pCookie;
	if (bSuccess) {
		printf("Calibration complete, start tracking user %d\n", nId);
		grabber->userId=nId;
		grabber->user_generator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		printf("Calibration failed for user %d\n", nId);
		if (grabber->bNeedPose)
			grabber->user_generator.GetPoseDetectionCap().StartPoseDetection(grabber->strPose, nId);
		else
			grabber->user_generator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
	PersonGrabber *grabber;
	grabber = (PersonGrabber*)pCookie;
	printf("Pose %s detected for user %d\n", strPose, nId);
    grabber->user_generator.GetPoseDetectionCap().StopPoseDetection(nId);
    grabber->user_generator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "person_grabber");
    ros::NodeHandle nh;

    PersonGrabber pg(nh);
    pg.spin();


	return 0;
}
