/*
 *  OcvImage.h
 *	A wrapper around opencv IplImage
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
 *
 */

#ifndef OCVIMAGE_H_
#define OCVIMAGE_H_
#include <cv.h>
#include <stdio.h>
#include <highgui.h>
#include <stdarg.h>

template<class T> class OcvImage {
  private:
	  bool allocated;
  public:
	  IplImage* iplimage;
	  OcvImage(IplImage* img=0) {iplimage=img;allocated=false;}
	  OcvImage(int width, int height, int depth, int channels) {
		  iplimage = cvCreateImage(cvSize(width,height),depth,channels);
		  cvSetZero(iplimage);
		  allocated=true;
	  }
	  OcvImage(OcvImage<T>& tocopy) {
		  allocated=tocopy.allocated;
		  iplimage=tocopy.iplimage;
	  }
	  ~OcvImage(){
		  if (allocated)
			  cvReleaseImage(&iplimage);
	  }
	  operator IplImage*() {
		  return iplimage;
	  }
	  void operator=(IplImage* img) {iplimage=img;}

	  inline void release() {
		  if (iplimage!=0) {
			  cvReleaseImage(&iplimage);
		  }
	  }

	  inline void clear() {
		  memset(iplimage->imageData,0,iplimage->imageSize);
	  }

	  inline T* operator[](const int rowIndx) {
		  return ((T *)(iplimage->imageData + rowIndx*iplimage->widthStep));
	  }

	  inline int width() {
		  return iplimage->width;
	  }

	  inline int height() {
		  return iplimage->height;
	  }

	  /// Convert the image to greyscale. Only works if the image is 3 channel 8U RGB
	  /// @param format should be the opencv conversion constant, eg CV_RGB2GRAY
	  inline void toGreyScale(int format) {
		  IplImage *conved = cvCreateImage(cvSize(iplimage->width,iplimage->height),IPL_DEPTH_8U,1);
		  cvCvtColor(iplimage,conved,format);
		  cvReleaseImage(&iplimage);
		  iplimage=conved;
	  }

	  inline void resize(int x, int y) {
		  IplImage *conved = cvCreateImage(cvSize(x,y),iplimage->depth,iplimage->nChannels);
		  cvResize(iplimage,conved);
		  cvReleaseImage(&iplimage);
		  iplimage=conved;
	  }

	  inline void newImage(int x, int y, int depth, int channels) {
		  release();
		  iplimage = cvCreateImage(cvSize(x,y),depth,channels);
	  }

	  inline void rotate(float cx, float cy, float theta) {
			IplImage *conved = cvCreateImage(cvSize(iplimage->width, iplimage->height),iplimage->depth, iplimage->nChannels);
			float m[6];
			CvMat M;
			M = cvMat(2, 3, CV_32F, m);
			m[0] = (float) (cos(-theta));
			m[1] = (float) (sin(-theta));
			m[3] = -m[1];
			m[4] = m[0];
			// and the translation (centre point from original image)
			m[2] = cx;
			m[5] = cy;

			cvGetQuadrangleSubPix(iplimage, conved, &M);
			cvReleaseImage(&iplimage);
			iplimage=conved;
	  }

	  inline OcvImage<T>* rotateCopy(float cx, float cy, float theta, OcvImage<T>* destination=NULL) {
			OcvImage<T> *conved;
			if (destination==NULL)
				conved = new OcvImage<T>(iplimage->width, iplimage->height,iplimage->depth, iplimage->nChannels);
			else
				conved=destination;

			float m[6];
			CvMat M;
			M = cvMat(2, 3, CV_32F, m);
			m[0] = (float) (cos(-theta));
			m[1] = (float) (sin(-theta));
			m[3] = -m[1];
			m[4] = m[0];
			// and the translation (centre point from original image)
			m[2] = cx;
			m[5] = cy;

			cvGetQuadrangleSubPix(iplimage, (*conved), &M);
			return conved;
	  }

	  inline void crop(float removeTop,float removeBottom,float removeLeft, float removeRight) {
		  IplImage *conved = cvCreateImage(cvSize(iplimage->width-removeLeft-removeRight, iplimage->height-removeTop-removeBottom),iplimage->depth, iplimage->nChannels);
		  float m[6];
		  CvMat M;
		  M = cvMat(2, 3, CV_32F, m);
		  m[0] = 1;
		  m[1] = 0;
		  m[3] = 0;
		  m[4] = 1;
		  // and the translation (centre point from original image)
		  m[2] = iplimage->width * 0.5f;
		  m[5] = iplimage->height * 0.5f;
		  cvGetQuadrangleSubPix(iplimage, conved, &M);
		  cvReleaseImage(&iplimage);
		  iplimage=conved;
	  }

	  inline void writeString(int x, int y, const char* text, CvScalar colour = CV_RGB(0,0,0), double scale=0.3, int thickness=1) {
		  CvFont font;
		  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,scale,scale,0,thickness);
		  cvPutText(iplimage,text,cvPoint(x,y),&font,colour);
	  }

	  inline void writeInt(int x, int y, int number, CvScalar colour = CV_RGB(0,0,0), double scale=0.3, int thickness=1) {
		  char buff[10];
		  sprintf(buff,"%d",number);
		  writeString(x,y,buff,colour,scale,thickness);
	  }

	  inline void writeFormatted(int x, int y, const char *format, CvScalar colour, double scale, int thickness, ...) {
		  char buff[50];
		  va_list vargs;
		  va_start(vargs,format); //10?va_start (args, format);

		  vsprintf(buff,format,vargs);
		  writeString(x,y,buff,colour,scale,thickness);
	  }

	  inline void loadImage(const char *imagefile) {
		  release();
		  iplimage = cvLoadImage(imagefile);
	  }

	  inline OcvImage<T> getROI(int x, int y, int width, int height) {
		  if (x+width > iplimage->width)
			  width=iplimage->width-x;
		  if (y+height > iplimage->height)
			  width=iplimage->height-y;
		  if (x<0) x=0;
		  if (y<0) y=0;
		  OcvImage<T> roi(cvCreateImageHeader(cvSize(width,height),iplimage->depth,iplimage->nChannels));

		  roi.iplimage->imageData=iplimage->imageData+y*iplimage->widthStep+x*iplimage->nChannels;
		  roi.iplimage->widthStep=iplimage->widthStep;

		  return roi;
	  }
};

template <class T> class BGRPixel {
	public:
		T r,g,b;
		BGRPixel() {

		}
		BGRPixel(int red, int green, int blue) {
			r=red; b=blue; g=green;
		}
		inline BGRPixel& operator=(BGRPixel p) {
			r=p.r; g=p.g; b=p.b;
			return *this;
		}
		inline void operator=(int v) {
			r=g=b=v;
		}

		inline T intensityAvg() {
			return (r+g+b)/3;
		}

		operator T() {
			  return intensityAvg;
		  }

};

template <class T> class RGBPixel {
	public:
		T b,g,r;
		RGBPixel() {

		}
		RGBPixel(int red, int green, int blue) {
			r=red; b=blue; g=green;
		}
		inline RGBPixel& operator=(RGBPixel p) {
			r=p.r; g=p.g; b=p.b;
			return *this;
		}
		inline void operator=(int v) {
			r=g=b=v;
		}

		inline T intensityAvg() {
			return (r+g+b)/3;
		}
		operator T() {
			  return intensityAvg;
		  }
};

//typedef Image<RgbPixel>       RgbImage;
//typedef Image<RgbPixelFloat>  RgbImageFloat;
//typedef Image<unsigned char>  BwImage;
//typedef Image<float>          BwImageFloat;


#endif /* OCVIMAGE_H_ */
