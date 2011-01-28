/*********************************************************** 
 *  --- OpenSURF ---                                       *
 *  This library is distributed under the GNU GPL. Please   *
 *  use the contact form at http://www.chrisevansdev.com    *
 *  for more information.                                   *
 *                                                          *
 *  C. Evans, Research Into Robust Visual Features,         *
 *  MSc University of Bristol, 2008.                        *
 *                                                          *
 ************************************************************/

#include "surflib.h"
#include "kmeans.h"
#include "utils.h"
//#include "OpenSURF/iPoint.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

//-------------------------------------------------------
// In order to you use OpenSURF, the following illustrates
// some of the simple tasks you can do.  It takes only 1
// function call to extract described SURF features!
// Define PROCEDURE as:
//  - 1 and supply image path to run on static image
//  - 2 to capture from a webcam
//  - 3 to match find an object in an image (work in progress)
//  - 4 to display moving features (work in progress)
//  - 5 to show matches between static images
//  - 6 kMeans
//  - 7 custom surfing
#define PROCEDURE 7
uint Counter = 0;
int numOfIPoints;
string ImgFilename;
string gImageSubTopic;

ros::Publisher features;

class ImageConverter {
public:
	ImageConverter(ros::NodeHandle &n) : n_(n), it_(n_)
	{
		cvNamedWindow("Image window");
		image_sub_ = it_.subscribe(gImageSubTopic.c_str(), 1, &ImageConverter::imageCallback, this);
		//strSub = n.subscribe("/camera_sim/image_filename", 1, );
		// numOfIPoints = 20;
//		numOfIPoints = -1; // to publish all points
	}

	ImageConverter(ros::NodeHandle &n, int numOfIPoints) : n_(n), it_(n_)
	{
		cvNamedWindow("Image window");
		image_sub_ = it_.subscribe(gImageSubTopic.c_str(), 1, &ImageConverter::imageCallback, this);
		this->numOfIPoints = numOfIPoints;
	}

	~ImageConverter()
	{
		cvDestroyWindow("Image window");
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		// Image obtained from subscribed node and converted to IplImage
		IplImage *img = NULL;
		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2);
		try {
			img = bridge_.imgMsgToCv(msg_ptr, "bgr8"); // reads the image
			// Can I read something from another topic??
		}
		catch (sensor_msgs::CvBridgeException error) {
			ROS_ERROR("error");
		}

		// Declare Ipoints and other stuff
		IpVec ipts;

		// Detect and describe interest points in the image
		clock_t start = clock();

		// Basically the main bit of code is the following two statements, from here --
		surfDetDes(img, ipts, false, 5, 4, 2, 0.0004f); // default value is 0.0004f
		// publish the data to the node
		publishDescriptors(this->numOfIPoints, &features, &ipts);
		//save_descriptors_to_file(&ipts);
		//publishFilename;
		// -- to here

		// print the information on the screen
		//	printDescriptors(&ipts);
		// Draw the detected points
		drawIpoints(img, ipts);
		cvPutText(img, "filename", cvPoint(200, 290), &font, cvScalar(0, 255, 0, 0));
		// Display the result
		cvShowImage("Image window", img);

		// Uncomment the following few lines if you want to save the images. Folder "data" should exist in the folder from which the program is run.
		//		Counter++;
		//		stringstream filename;
		//		filename << "./data/image-" << Counter << ".jpg";
		//		cout << "Saving in " << filename.str() << endl;
		//		cvSaveImage(filename.str().c_str(), img);

		cvWaitKey(3);

		clock_t end = clock();
		std::cout<< "OpenSURF found: " << ipts.size() << " interest points" << std::endl;
		std::cout<< "OpenSURF took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;
		cout << endl;
	}
protected:
	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber strSub;
	sensor_msgs::CvBridge bridge_;
	int numOfIPoints;
};




//-------------------------------------------------------

int mainImage(void)
{
	// Declare Ipoints and other stuff
	IpVec ipts;
	IplImage *img=cvLoadImage("imgs/box.png");
//	IplImage *img=cvLoadImage("imgs/coke.jpg");

	// Detect and describe interest points in the image
	clock_t start = clock();
	surfDetDes(img, ipts, false, 5, 4, 2, 0.0004f);
	//  surfDetDes(img, ipts, false, 5, 4, 2, 0.005f);
	// publish the data to the node
	publishDescriptors(10, &features, &ipts);
	clock_t end = clock();

	std::cout<< "OpenSURF found: " << ipts.size() << " interest points" << std::endl;
	std::cout<< "OpenSURF took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;
//	printDescriptors(&ipts);

	// Draw the detected points
	drawIpoints(img, ipts);

	// Display the result
	showImage(img);

	return 0;
}

//-------------------------------------------------------

int mainVideo(void)
{
	// Initialise capture device
	CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
	if(!capture) error("No Capture");

	// Initialise video writer
	//cv::VideoWriter vw("c:\\out.avi", CV_FOURCC('D','I','V','X'),10,cvSize(320,240),1);
	//vw << img;

	// Create a window
	cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

	// Declare Ipoints and other stuff
	IpVec ipts;
	IplImage *img=NULL;

	// Main capture loop
	while( 1 )
	{
		// Grab frame from the capture source
		img = cvQueryFrame(capture);

		// Extract surf points
		surfDetDes(img, ipts, false, 4, 4, 2, 0.004f);

		// Draw the detected points
		drawIpoints(img, ipts);

		// Draw the FPS figure
		drawFPS(img);

		// Display the result
		cvShowImage("OpenSURF", img);

		// If ESC key pressed exit loop
		if( (cvWaitKey(10) & 255) == 27 ) break;
	}

	cvReleaseCapture( &capture );
	cvDestroyWindow( "OpenSURF" );
	return 0;
}


//-------------------------------------------------------


int mainMatch(void)
{
	// Initialise capture device
	CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
	if(!capture) error("No Capture");

	// Declare Ipoints and other stuff
	IpPairVec matches;
	IpVec ipts, ref_ipts;

	// This is the reference object we wish to find in video frame
	// Replace the line below with IplImage *img = cvLoadImage("imgs/object.jpg");
	// where object.jpg is the planar object to be located in the video
	IplImage *img = cvLoadImage("imgs/object.jpg");
	if (img == NULL) error("Need to load reference image in order to run matching procedure");
	CvPoint src_corners[4] = {{0,0}, {img->width,0}, {img->width, img->height}, {0, img->height}};
	CvPoint dst_corners[4];

	// Extract reference object Ipoints
	surfDetDes(img, ref_ipts, false, 3, 4, 3, 0.004f);
	drawIpoints(img, ref_ipts);
	showImage(img);

	// Create a window
	cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

	// Main capture loop
	while( true )
	{
		// Grab frame from the capture source
		img = cvQueryFrame(capture);

		// Detect and describe interest points in the frame
		surfDetDes(img, ipts, false, 3, 4, 3, 0.004f);

		// Fill match vector
		getMatches(ipts,ref_ipts,matches);

		// This call finds where the object corners should be in the frame
		if (translateCorners(matches, src_corners, dst_corners))
		{
			// Draw box around object
			for(int i = 0; i < 4; i++ )
			{
				CvPoint r1 = dst_corners[i%4];
				CvPoint r2 = dst_corners[(i+1)%4];
				cvLine( img, cvPoint(r1.x, r1.y),
						cvPoint(r2.x, r2.y), cvScalar(255,255,255), 3 );
			}

			for (unsigned int i = 0; i < matches.size(); ++i)
				drawIpoint(img, matches[i].first);
		}

		// Draw the FPS figure
		drawFPS(img);

		// Display the result
		cvShowImage("OpenSURF", img);

		// If ESC key pressed exit loop
		if( (cvWaitKey(10) & 255) == 27 ) break;
	}

	// Release the capture device
	cvReleaseCapture( &capture );
	cvDestroyWindow( "OpenSURF" );
	return 0;
}

//-------------------------------------------------------

int mainMotionPoints(void)
{
	// Initialise capture device
	CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
	if(!capture) error("No Capture");

	// Create a window
	cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

	// Declare Ipoints and other stuff
	IpVec ipts, old_ipts, motion;
	IpPairVec matches;
	IplImage *img;

	// Main capture loop
	while( 1 )
	{
		// Grab frame from the capture source
		img = cvQueryFrame(capture);

		// Detect and describe interest points in the image
		old_ipts = ipts;
		surfDetDes(img, ipts, true, 3, 4, 2, 0.0004f);

		// Fill match vector
		getMatches(ipts,old_ipts,matches);
		for (unsigned int i = 0; i < matches.size(); ++i)
		{
			const float & dx = matches[i].first.dx;
			const float & dy = matches[i].first.dy;
			float speed = sqrt(dx*dx+dy*dy);
			if (speed > 5 && speed < 30)
				drawIpoint(img, matches[i].first, 3);
		}

		// Display the result
		cvShowImage("OpenSURF", img);

		// If ESC key pressed exit loop
		if( (cvWaitKey(10) & 255) == 27 ) break;
	}

	// Release the capture device
	cvReleaseCapture( &capture );
	cvDestroyWindow( "OpenSURF" );
	return 0;
}


//-------------------------------------------------------

int mainStaticMatch()
{
	IplImage *img1, *img2;
	//  img1 = cvLoadImage("imgs/box.png");
	//  img2 = cvLoadImage("imgs/box_in_scene.png");
	img1 = cvLoadImage("imgs/coke.jpg");
	img2 = cvLoadImage("imgs/coke_pepsi.jpg");

	IpVec ipts1, ipts2;
	surfDetDes(img1,ipts1,false,4,4,2,0.0001f);
	surfDetDes(img2,ipts2,false,4,4,2,0.0001f);

	IpPairVec matches;
	getMatches(ipts1,ipts2,matches);

	for (unsigned int i = 0; i < matches.size(); ++i)
	{
		drawPoint(img1,matches[i].first);
		drawPoint(img2,matches[i].second);

		const int & w = img1->width;
		cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);
		cvLine(img2,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
	}

	std::cout<< "Matches: " << matches.size();

	cvNamedWindow("1", CV_WINDOW_AUTOSIZE );
	cvNamedWindow("2", CV_WINDOW_AUTOSIZE );
	cvShowImage("1", img1);
	cvShowImage("2",img2);
	cvWaitKey(0);

	return 0;
}

//-------------------------------------------------------

int mainKmeans(void)
{
	IplImage *img = cvLoadImage("imgs/img1.jpg");
	IpVec ipts;
	Kmeans km;

	// Get Ipoints
	surfDetDes(img,ipts,true,3,4,2,0.0006f);

	for (int repeat = 0; repeat < 10; ++repeat)
	{

		IplImage *img = cvLoadImage("imgs/img1.jpg");
		km.Run(&ipts, 5, true);
		drawPoints(img, km.clusters);

		for (unsigned int i = 0; i < ipts.size(); ++i)
		{
			cvLine(img, cvPoint(ipts[i].x,ipts[i].y), cvPoint(km.clusters[ipts[i].clusterIndex].x ,km.clusters[ipts[i].clusterIndex].y),cvScalar(255,255,255));
		}

		showImage(img);
	}

	return 0;
}

//-------------------------------------------------------

int main(int argc, char *argv[])
{
	numOfIPoints = argc == 1 ? -1 : atoi(argv[1]);
	// The following should be set from a command line option
	gImageSubTopic.assign("/camera_sim/image_raw");
	ros::init(argc, argv, "surf");
	ros::NodeHandle n;
	features = n.advertise<std_msgs::Float32MultiArray>("/vision/features", 1000, true);
//	if (PROCEDURE == 1) return mainImage();
//	else if (PROCEDURE == 2) return mainVideo();
//	else if (PROCEDURE == 3) return mainMatch();
//	else if (PROCEDURE == 4) return mainMotionPoints();
//	else if (PROCEDURE == 5) return mainStaticMatch();
//	else if (PROCEDURE == 6) return mainKmeans();
//	else if (PROCEDURE == 7) ImageConverter ic(n);
//	else return 0;
	ImageConverter ic(n, numOfIPoints);
	ros::spin();
}
