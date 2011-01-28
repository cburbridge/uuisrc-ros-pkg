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

#include <highgui.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <time.h>

#include "utils.h"

using namespace std;

//-------------------------------------------------------

static const int NCOLOURS = 8;
static const CvScalar COLOURS [] = {cvScalar(255,0,0), cvScalar(0,255,0), 
                                    cvScalar(0,0,255), cvScalar(255,255,0),
                                    cvScalar(0,255,255), cvScalar(255,0,255),
                                    cvScalar(255,255,255), cvScalar(0,0,0)};

//-------------------------------------------------------

//! Display error message and terminate program
void error(const char *msg) 
{
  cout << "\nError: " << msg;
  getchar();
  exit(0);
}

//-------------------------------------------------------

//! Show the provided image and wait for keypress
void showImage(const IplImage *img)
{
  cvNamedWindow("Surf", CV_WINDOW_AUTOSIZE); 
  cvShowImage("Surf", img);  
  cvWaitKey(3);
}

//-------------------------------------------------------

//! Show the provided image in titled window and wait for keypress
void showImage(char *title,const IplImage *img)
{
  cvNamedWindow(title, CV_WINDOW_AUTOSIZE); 
  cvShowImage(title, img);  
  cvWaitKey(3);
}

//-------------------------------------------------------

// Convert image to single channel 32F
IplImage *getGray(const IplImage *img)
{
  // Check we have been supplied a non-null img pointer
  if (!img) error("Unable to create grayscale image.  No image supplied");

  IplImage* gray8, * gray32;

  gray32 = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );

  if( img->nChannels == 1 )
    gray8 = (IplImage *) cvClone( img );
  else {
    gray8 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
    cvCvtColor( img, gray8, CV_BGR2GRAY );
  }

  cvConvertScale( gray8, gray32, 1.0 / 255.0, 0 );

  cvReleaseImage( &gray8 );
  return gray32;
}

//-------------------------------------------------------

//! Draw all the Ipoints in the provided vector
void drawIpoints(IplImage *img, vector<Ipoint> &ipts, int tailSize)
{
  Ipoint *ipt;
  float s, o;
  int r1, c1, r2, c2, lap;

  for(unsigned int i = 0; i < ipts.size(); i++) 
  {
    ipt = &ipts.at(i);
    s = (2.5f * ipt->scale);
    o = ipt->orientation;
    lap = ipt->laplacian;
    r1 = fRound(ipt->y);
    c1 = fRound(ipt->x);
    c2 = fRound(s * cos(o)) + c1;
    r2 = fRound(s * sin(o)) + r1;

    if (o) // Green line indicates orientation
      cvLine(img, cvPoint(c1, r1), cvPoint(c2, r2), cvScalar(0, 255, 0));
    else  // Green dot if using upright version
      cvCircle(img, cvPoint(c1,r1), 1, cvScalar(0, 255, 0),-1);

    if (lap == 1)
    { // Blue circles indicate dark blobs on light backgrounds
      cvCircle(img, cvPoint(c1,r1), fRound(s), cvScalar(255, 0, 0),1);
    }
    else if (lap == 0)
    { // Red circles indicate light blobs on dark backgrounds
      cvCircle(img, cvPoint(c1,r1), fRound(s), cvScalar(0, 0, 255),1);
    }
    else if (lap == 9)
    { // Red circles indicate light blobs on dark backgrounds
      cvCircle(img, cvPoint(c1,r1), fRound(s), cvScalar(0, 255, 0),1);
    }

    // Draw motion from ipoint dx and dy
    if (tailSize)
    {
      cvLine(img, cvPoint(c1,r1),
        cvPoint(int(c1+ipt->dx*tailSize), int(r1+ipt->dy*tailSize)),
        cvScalar(255,255,255), 1);
    }
  }
}

//-------------------------------------------------------

//! Draw a single feature on the image
void drawIpoint(IplImage *img, Ipoint &ipt, int tailSize)
{
  float s, o;
  int r1, c1, r2, c2, lap;

  s = (2.5f * ipt.scale);
  o = ipt.orientation;
  lap = ipt.laplacian;
  r1 = fRound(ipt.y);
  c1 = fRound(ipt.x);

  // Green line indicates orientation
  if (o) // Green line indicates orientation
  {
    c2 = fRound(s * cos(o)) + c1;
    r2 = fRound(s * sin(o)) + r1;
    cvLine(img, cvPoint(c1, r1), cvPoint(c2, r2), cvScalar(0, 255, 0));
  }
  else  // Green dot if using upright version
    cvCircle(img, cvPoint(c1,r1), 1, cvScalar(0, 255, 0),-1);

  if (lap >= 0)
  { // Blue circles indicate light blobs on dark backgrounds
    cvCircle(img, cvPoint(c1,r1), fRound(s), cvScalar(255, 0, 0),1);
  }
  else
  { // Red circles indicate light blobs on dark backgrounds
    cvCircle(img, cvPoint(c1,r1), fRound(s), cvScalar(0, 0, 255),1);
  }

  // Draw motion from ipoint dx and dy
  if (tailSize)
  {
    cvLine(img, cvPoint(c1,r1),
      cvPoint(int(c1+ipt.dx*tailSize), int(r1+ipt.dy*tailSize)),
      cvScalar(255,255,255), 1);
  }
}

//-------------------------------------------------------

//! Draw a single feature on the image
void drawPoint(IplImage *img, Ipoint &ipt)
{
  float s, o;
  int r1, c1;

  s = 3;
  o = ipt.orientation;
  r1 = fRound(ipt.y);
  c1 = fRound(ipt.x);

  cvCircle(img, cvPoint(c1,r1), fRound(s), COLOURS[ipt.clusterIndex%NCOLOURS], -1);
  cvCircle(img, cvPoint(c1,r1), fRound(s+1), COLOURS[(ipt.clusterIndex+1)%NCOLOURS], 2);

}

//-------------------------------------------------------

//! Draw a single feature on the image
void drawPoints(IplImage *img, vector<Ipoint> &ipts)
{
  float s, o;
  int r1, c1;

  for(unsigned int i = 0; i < ipts.size(); i++) 
  {
    s = 3;
    o = ipts[i].orientation;
    r1 = fRound(ipts[i].y);
    c1 = fRound(ipts[i].x);

    cvCircle(img, cvPoint(c1,r1), fRound(s), COLOURS[ipts[i].clusterIndex%NCOLOURS], -1);
    cvCircle(img, cvPoint(c1,r1), fRound(s+1), COLOURS[(ipts[i].clusterIndex+1)%NCOLOURS], 2);
  }
}

//-------------------------------------------------------

//! Draw descriptor windows around Ipoints in the provided vector
void drawWindows(IplImage *img, vector<Ipoint> &ipts)
{
  Ipoint *ipt;
  float s, o, cd, sd;
  int x, y;
  CvPoint2D32f src[4];

  for(unsigned int i = 0; i < ipts.size(); i++) 
  {
    ipt = &ipts.at(i);
    s = (10 * ipt->scale);
    o = ipt->orientation;
    y = fRound(ipt->y);
    x = fRound(ipt->x);
    cd = cos(o);
    sd = sin(o);

    src[0].x=sd*s+cd*s+x;   src[0].y=-cd*s+sd*s+y;
    src[1].x=sd*s+cd*-s+x;  src[1].y=-cd*s+sd*-s+y;
    src[2].x=sd*-s+cd*-s+x; src[2].y=-cd*-s+sd*-s+y;
    src[3].x=sd*-s+cd*s+x;  src[3].y=-cd*-s+sd*s+y;

    if (o) // Draw orientation line
      cvLine(img, cvPoint(x, y), 
      cvPoint(fRound(s*cd + x), fRound(s*sd + y)), cvScalar(0, 255, 0),1);
    else  // Green dot if using upright version
      cvCircle(img, cvPoint(x,y), 1, cvScalar(0, 255, 0),-1);


    // Draw box window around the point
    cvLine(img, cvPoint(fRound(src[0].x), fRound(src[0].y)), 
      cvPoint(fRound(src[1].x), fRound(src[1].y)), cvScalar(255, 0, 0),2);
    cvLine(img, cvPoint(fRound(src[1].x), fRound(src[1].y)), 
      cvPoint(fRound(src[2].x), fRound(src[2].y)), cvScalar(255, 0, 0),2);
    cvLine(img, cvPoint(fRound(src[2].x), fRound(src[2].y)), 
      cvPoint(fRound(src[3].x), fRound(src[3].y)), cvScalar(255, 0, 0),2);
    cvLine(img, cvPoint(fRound(src[3].x), fRound(src[3].y)), 
      cvPoint(fRound(src[0].x), fRound(src[0].y)), cvScalar(255, 0, 0),2);

  }
}

//-------------------------------------------------------

// Draw the FPS figure on the image (requires at least 2 calls)
void drawFPS(IplImage *img)
{
  static int counter = 0;
  static clock_t t;
  static float fps;
  char fps_text[20];
  CvFont font;
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1.0,1.0,0,2);

  // Add fps figure (every 10 frames)
  if (counter > 10)
  {
    fps = (10.0f/(clock()-t) * CLOCKS_PER_SEC);
    t=clock(); 
    counter = 0;
  }

  // Increment counter
  ++counter;

  // Get the figure as a string
  sprintf(fps_text,"FPS: %.2f",fps);

  // Draw the string on the image
  cvPutText (img,fps_text,cvPoint(10,25), &font, cvScalar(255,255,0));
}

//-------------------------------------------------------

//! Save the SURF features to file
void saveSurf(char *filename, vector<Ipoint> &ipts)
{
  ofstream outfile(filename);

  // output descriptor length
  outfile << "64\n";
  outfile << ipts.size() << "\n";

  // create output line as:  scale  x  y  des
  for(unsigned int i=0; i < ipts.size(); i++) 
  {
    outfile << ipts.at(i).scale << "  ";
    outfile << ipts.at(i).x << " ";
    outfile << ipts.at(i).y << " ";
    outfile << ipts.at(i).orientation << " ";
    outfile << ipts.at(i).laplacian << " ";
    outfile << ipts.at(i).scale << " ";
    for(int j=0; j<64; j++)
      outfile << ipts.at(i).descriptor[j] << " ";

    outfile << "\n";
  }

  outfile.close();
}

//-------------------------------------------------------

//! Load the SURF features from file
void loadSurf(char *filename, vector<Ipoint> &ipts)
{
  int descriptorLength, count;
  ifstream infile(filename);

  // clear the ipts vector first
  ipts.clear();

  // read descriptor length/number of ipoints
  infile >> descriptorLength;
  infile >> count;

  // for each ipoint
  for (int i = 0; i < count; i++) 
  {
    Ipoint ipt;

    // read vals
    infile >> ipt.scale; 
    infile >> ipt.x;
    infile >> ipt.y;
    infile >> ipt.orientation;
    infile >> ipt.laplacian;
    infile >> ipt.scale;

    // read descriptor components
    for (int j = 0; j < 64; j++)
      infile >> ipt.descriptor[j];

    ipts.push_back(ipt);

  }
}

//-------------------------------------------------------
//-------------------------------------------------------

int file_exists(char *filename)
{
	std::ifstream fp(filename, ifstream::in);
	if (fp) {
		fp.close();
		return 1;
	}
	return 0 ;
}


void save_descriptors_to_file(IpVec* ipts)
{
	if (file_exists("/home/yianni/temp/siddique_data/output.txt")) {
		return;
	}
	std::ofstream fp;
	fp.open ("/home/yianni/temp/siddique_data/output.txt");
	if (fp.is_open()) {
		fp << fixed;
//		fp << "Total size: " << ipts->size() << std::endl;
//		fp << "\tx\ty\tscale\torientation\tlaplacian\tdx\tdy\tclusterIndex\thessian" << std::endl;
		for (unsigned int i = 0; i < ipts->size(); i++) {
			//		for (int i = 0; i < 2; i++) {
//			fp << i + 1 << ")\t" \
//					<< round(ipts->at(i).x) << "\t" \
//					<< round(ipts->at(i).y) << "\t" \
//					<< round(ipts->at(i).scale) << "\t" \
//					<< round(ipts->at(i).orientation) << "\t\t" \
//					<< ipts->at(i).laplacian << "\t\t" \
//					<< round(ipts->at(i).dx) << "\t" \
//					<< round(ipts->at(i).dy) << "\t" \
//					<< ipts->at(i).clusterIndex << "\t" \
//					<< ipts->at(i).hessian << "\t" \
//					<< std::endl;
			//			fp << "\t";
			for (int j = 0; j < 64; j++) {
				fp << setprecision(4) << ipts->at(i).descriptor[j];
				if (j < 63)
					fp << ",";
//				if (ipts->at(i).descriptor[j] < min)
//					min = ipts->at(i).descriptor[j];
//				if (ipts->at(i).descriptor[j] > max)
//					max = ipts->at(i).descriptor[j];
			}
			fp << std::endl;
		}
	} else
		std::cout << "Unable to open file" << std::endl;
	fp.close();
}


/**
 * @param numOfIPointsTotal : -1 to publish all points
 * @param features : the ROS publisher
 * @param ipts : the vector of points
 */
void publishDescriptors(const int numOfIPointsTotal, ros::Publisher* features, IpVec* ipts)
{
	const int numOfIPoints = ((numOfIPointsTotal > 0) && (numOfIPointsTotal < ipts->size())) ? numOfIPointsTotal : ipts->size(); // can control number of points to publish
	const uint lengthOfIPointDescription = 73;
	const int arrayLength = numOfIPoints * lengthOfIPointDescription;
	std_msgs::Float32MultiArray iPoints;
	iPoints.layout.dim.push_back(std_msgs::MultiArrayDimension());
	iPoints.layout.dim[0].label = "numOfIPoints";
	iPoints.layout.dim[0].size = numOfIPoints;
	iPoints.layout.dim[0].stride = lengthOfIPointDescription * numOfIPoints;
	iPoints.layout.dim.push_back(std_msgs::MultiArrayDimension());
	iPoints.layout.dim[1].label = "lengthOfIPointDescription";
	iPoints.layout.dim[1].size = lengthOfIPointDescription;
	iPoints.layout.dim[1].stride = lengthOfIPointDescription;
	iPoints.set_data_size(arrayLength);
	int i, index = 0;
	for (i = 0; i < numOfIPoints; i++) {
		index = i * lengthOfIPointDescription;
		iPoints.data[index] = ipts->at(i).x;
		index++;
		iPoints.data[index] = ipts->at(i).y;
		index++;
		iPoints.data[index] = ipts->at(i).scale;
		index++;
		iPoints.data[index] = ipts->at(i).orientation;
		index++;
		iPoints.data[index] = ipts->at(i).laplacian;
		index++;
		iPoints.data[index] = ipts->at(i).dx;
		index++;
		iPoints.data[index] = ipts->at(i).dy;
		index++;
		iPoints.data[index] = ipts->at(i).clusterIndex;
		index++;
		iPoints.data[index] = ipts->at(i).hessian;
		index++;
		for (uint j = 0; j < 64; j++) {
			iPoints.data[index] = ipts->at(i).descriptor[j];
			index++;
		}
	}
	features->publish(iPoints);
}
