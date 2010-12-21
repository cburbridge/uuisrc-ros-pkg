/*
 *  CameraUW Player Driver
 *
 *  Copyright (C) 2010, Lorenzo Riano <lorenzo.riano@gmail.com>
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
 *	This program has been hacked around some of the source code of Player/Stage,
 *	"Player - One Hell of a Robot Server", Copyright (C) 2000  Brian Gerkey et al.
 */


/*
 * $Id: P2CMV.cc 4335 2008-01-28 02:26:32Z thjc $
 *
 * Uses CMVision to retrieve the blob data
 */

#include <cv.h>
#include <iostream>
#include <cstdlib>
#include <assert.h>
#include <stdio.h>
#include <unistd.h>  /* close(2),fcntl(2),getpid(2),usleep(3),execvp(3),fork(2)*/
#include <signal.h>  /* for kill(2) */
#include <fcntl.h>   /* for fcntl(2) */
#include <string.h>  /* for strncpy(3),memcpy(3) */
#include <stdlib.h>  /* for atexit(3),atoi(3) */
#include <pthread.h> /* for pthread stuff */
#include <math.h>    /* for rint */
#include <vector>

#include <libplayercore/playercore.h>
#include <libplayercore/error.h>
#include <libplayerjpeg/playerjpeg.h>

/********************************************************************/

using namespace cv;

#define cascadeName "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"

class FaceDetectionBlob: public Driver
{
  private:
    uint16_t         mWidth;
    uint16_t         mHeight;     // the image dimensions

    double scale;
    player_blobfinder_data_t   mData;
    unsigned int     allocated_blobs;

    player_devaddr_t mCameraAddr;
    Device*          mCameraDev;
    
    CascadeClassifier cascade;
    
    
    IplImage* mimage;
    IplImage* m_bw_image;
    IplImage* m_scale_image;

  public:
    int Setup();
    int Shutdown();
    // constructor
    FaceDetectionBlob(ConfigFile* cf, int section);
    virtual ~FaceDetectionBlob();
    // This method will be invoked on each incoming message
    virtual int ProcessMessage(QueuePointer & resp_queue,
                               player_msghdr * hdr,
                               void * data);
    virtual void Main();
    void ProcessImageData();
};

// a factory creation function
Driver*
FaceDetector_Init( ConfigFile* cf, int section)
{
  return((Driver*)(new FaceDetectionBlob( cf, section)));
}

// a driver registration function
void
FaceDetector_Register(DriverTable* table)
{
  table->AddDriver("facedetector", FaceDetector_Init);
}

FaceDetectionBlob::FaceDetectionBlob( ConfigFile* cf, int section)
  : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN,
           PLAYER_BLOBFINDER_CODE),
           mWidth(0),
           mHeight(0),
           mCameraDev(NULL)
{    
    // Must have an input camera    
    fflush(stdout);
    if (cf->ReadDeviceAddr(&mCameraAddr, section, "requires",
	PLAYER_CAMERA_CODE, -1, NULL) != 0)
    {
	PLAYER_ERROR("this driver requires a camera in the .cfg file");
	return;
  }
  scale = cf->ReadFloat(section,"scale",1.0);
  mimage = 0;

}

FaceDetectionBlob::~FaceDetectionBlob()
{
  
}

int
FaceDetectionBlob::Setup()
{
  printf("FaceDetectionBlob server initializing...");
  fflush(stdout);
  // Subscribe to the camera device
  if (!(this->mCameraDev = deviceTable->GetDevice(this->mCameraAddr)))
  {
    PLAYER_ERROR("unable to locate suitable camera device");
    return(-1);
  }
  if(0 != this->mCameraDev->Subscribe(this->InQueue))
  {
    PLAYER_ERROR("unable to subscribe to camera device");
    return(-1);
  }
  if( !cascade.load( cascadeName ) ) 
  {   
    PLAYER_ERROR("unable to load cascade file");
    return(-1);
  }

  // clean our data
  memset(&mData,0,sizeof(mData));
  puts("done.");

  StartThread();
  return(0);
}

int
FaceDetectionBlob::Shutdown()
{

  StopThread();

  // Unsubscribe from the camera
  this->mCameraDev->Unsubscribe(this->InQueue);

  puts("FaceDetectionBlob server has been shutdown");
  return(0);
}

void
FaceDetectionBlob::Main()
{
  // The main loop; interact with the device here
  for(;;)
  {
    // wait to receive a new message (blocking)
    Wait();

    // test if we are supposed to cancel
    pthread_testcancel();

    // Process incoming messages, and update outgoing data
    ProcessMessages();
  }
  return;
}

void
FaceDetectionBlob::ProcessImageData()
{
    // this shouldn't change often
    if ((mData.width != mWidth) || (mData.height != mHeight))
    {      
      mData.width      = mWidth;
      mData.height     = mHeight;
      printf("FaceDetectionBlob using camera: [w %d h %d]\n", mWidth, mHeight);
    }
    vector<Rect> faces;
//     cascade.detectMultiScale( m_bw_image, faces,
// 	1.1, 2, 0
//         //|CV_HAAR_FIND_BIGGEST_OBJECT
//         //|CV_HAAR_DO_ROUGH_SEARCH
//         |CV_HAAR_SCALE_IMAGE
//         ,
//         Size(30, 30) );
    cascade.detectMultiScale( m_scale_image, faces,
	1.1, 3, 0
        //|CV_HAAR_FIND_BIGGEST_OBJECT
        //|CV_HAAR_DO_ROUGH_SEARCH
        | CV_HAAR_SCALE_IMAGE
	| CV_HAAR_DO_CANNY_PRUNING
        ,
        Size(30, 30) );
   
    
    mData.blobs_count = faces.size();
    mData.blobs = (player_blobfinder_blob_t*)realloc(mData.blobs,sizeof(player_blobfinder_blob_t)*mData.blobs_count);
       
    for (int i = 0; i < faces.size(); i++)
    {	
        mData.blobs[i].color = uint32_t(255);
	Rect* r = &faces[i];
	
        // stage puts the range in here to simulate stereo mVision.
        mData.blobs[i].range = 0;
	uint32_t cx = cvRound((r->x + r->width*0.5)/scale);
	uint32_t cy = cvRound((r->y + r->height*0.5)/scale);

        // get the area first
        mData.blobs[i].area   = static_cast<uint32_t>(0);
        mData.blobs[i].x      = static_cast<uint32_t>(cx);
        mData.blobs[i].y      = static_cast<uint32_t>(cy);
        mData.blobs[i].left   = static_cast<uint32_t>( (r->x)/scale);
        mData.blobs[i].right  = static_cast<uint32_t>( (r->x + r->width)/scale);
        mData.blobs[i].top    = static_cast<uint32_t>( (r->y)/scale);
        mData.blobs[i].bottom = static_cast<uint32_t>( (r->y + r->height)/scale);
    }
    

    Publish(device_addr, 
          PLAYER_MSGTYPE_DATA, PLAYER_BLOBFINDER_DATA_BLOBS,
          reinterpret_cast<void*>(&mData));
}

int
FaceDetectionBlob::ProcessMessage(QueuePointer & resp_queue,
                           player_msghdr* hdr,
                           void* data)
{
  // Handle new data from the camera
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE,
                           this->mCameraAddr))
  {
    // Lock();
    // we can't quite do this so easily with camera data
    // because the images are different than the max size
    //assert(hdr->size == sizeof(player_camera_data_t));
    player_camera_data_t* camera_data = reinterpret_cast<player_camera_data_t *>(data);
    uint8_t* ptr;

    assert(camera_data);
    assert(camera_data->bpp == 24);
    if ((camera_data->width) && (camera_data->height))
    {
      if ((mWidth != camera_data->width) || (mHeight != camera_data->height) || (!mimage))
      {
        mWidth  = camera_data->width;
        mHeight = camera_data->height;        
        // we need to allocate some memory
	if (!mimage) 
	    cvReleaseImage(&mimage);
	if (!m_bw_image)
	    cvReleaseImage(&m_bw_image);
	if (!m_scale_image)
	    cvReleaseImage(&m_scale_image);
        
	mimage = cvCreateImage(cvSize(mWidth,mHeight),IPL_DEPTH_8U,3);
	m_bw_image = cvCreateImage(cvSize(mWidth,mHeight),IPL_DEPTH_8U,1);
	m_scale_image = cvCreateImage(cvSize(cvRound(mWidth*scale),cvRound(mHeight*scale)),IPL_DEPTH_8U,1);
      }
    
      if (camera_data->compression == PLAYER_CAMERA_COMPRESS_JPEG)
      {
	jpeg_decompress((unsigned char*)mimage->imageData, 
			mWidth*mHeight*3,
                        camera_data->image,
                        camera_data->image_count
                       );
      }
      else
      {
	  memcpy((unsigned char*)mimage->imageData, 
                        camera_data->image,
                        camera_data->image_count
                       );
      }
      cvCvtColor(mimage,m_bw_image,CV_BGR2GRAY);
      cvResize( m_bw_image, m_scale_image);
      cvEqualizeHist( m_scale_image, m_scale_image );
     
      // we have a new image,
      ProcessImageData();
    }
    // Unlock();
    return(0);
  }

  // Tell the caller that you don't know how to handle this message
  return(-1);
}

extern "C" {
  int player_driver_init(DriverTable* table)
  {
    FaceDetector_Register(table);
    return(0);
  }
}
