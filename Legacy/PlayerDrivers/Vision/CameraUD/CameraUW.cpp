/*
 *  CameraUW Player Driver
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
 *	This program has been hacked around some of the source code of Player/Stage,
 *	"Player - One Hell of a Robot Server", Copyright (C) 2000  Brian Gerkey et al.
 *  In particular, "jpeg compression and decompression routines",
 *  (c) Nate Koenig, Andrew Howard
 */

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <stdlib.h>       
#include <netinet/in.h>   
#include <math.h>
#include "..//OcvImage.h"

#include <highgui.h>
#include "../OmniUnwarp.h"

#include <libplayercore/playercore.h>
#include <libplayercore/error.h>
//#include <libplayerjpeg/playerjpeg.h>

class CameraUW : public Driver
{
  // Constructor
  public: CameraUW( ConfigFile* cf, int section);

  // Setup/shutdown routines.
  public: virtual int Setup();
  public: virtual int Shutdown();

  // This method will be invoked on each incoming message
  public: virtual int ProcessMessage(QueuePointer & resp_queue,
                                     player_msghdr * hdr,
                                     void * data);

  // Main function for device thread.
  private: virtual void Main();

  private: void ProcessImage(player_camera_data_t & rawdata);

  // Input camera device
  private:

    // Camera device info
    Device *camera;
    player_devaddr_t camera_id;
    double camera_time;
    bool camera_subscribed;

    // Output (unwarped) camera data
    private: player_camera_data_t data;

    // Unwarp parameters
    private: double z,k1,k2,k3;//1.50,6e-5,6e-10,2.4e-13

    
    private: int save;
    private: int frameno;

    int width,height;

   
    // Buffer for the image and the UW lookup table to point to
    OcvImage< BGRPixel<unsigned char> > *inputImage;
    OcvImage< BGRPixel<unsigned char>* > *unwarpedLookup;
};


Driver *CameraUW_Init(ConfigFile *cf, int section)
{
  return ((Driver*) (new CameraUW(cf, section)));
}

void CameraUW_Register(DriverTable *table)
{
  table->AddDriver("camerauw", CameraUW_Init);
}

CameraUW::CameraUW( ConfigFile *cf, int section)
  : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_CAMERA_CODE)
{
  this->data.image = NULL;
  this->frameno = 0;

  this->camera = NULL;
  // Must have a camera device
  if (cf->ReadDeviceAddr(&this->camera_id, section, "requires",
                       PLAYER_CAMERA_CODE, -1, NULL) != 0)
  {
    this->SetError(-1);
    return;
  }
  this->camera_time = 0.0;

  this->save = cf->ReadInt(section,"save",0);
  this->z = cf->ReadFloat(section, "z", 1.5);
  this->k1 = cf->ReadFloat(section, "k1", 6e-5);
  this->k2 = cf->ReadFloat(section, "k2", 6e-10);
  this->k2 = cf->ReadFloat(section, "k3", 6e-13);


  width=height=0;

  return;
}

int CameraUW::Setup()
{
  // Subscribe to the camera.
  if(Device::MatchDeviceAddress(this->camera_id, this->device_addr))
  {
    PLAYER_ERROR("attempt to subscribe to self");
    return(-1);
  }
  if(!(this->camera = deviceTable->GetDevice(this->camera_id)))
  {
    PLAYER_ERROR("unable to locate suitable camera device");
    return(-1);
  }
  if(this->camera->Subscribe(this->InQueue) != 0)
  {
    PLAYER_ERROR("unable to subscribe to camera device");
    return(-1);
  }

  // Start the driver thread.
  this->StartThread();

  return 0;
}

int CameraUW::Shutdown()
{
  // Stop the driver thread
  StopThread();

  camera->Unsubscribe(InQueue);

  if (this->data.image)
  {
    delete []this->data.image;
    this->data.image = NULL;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int CameraUW::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr,
                               void * data)
{
  assert(hdr);

  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, camera_id))
  {
	assert(data);
    player_camera_data_t * recv = reinterpret_cast<player_camera_data_t * > (data);
    ProcessImage(*recv);
    return 0;
  }

  return -1;
}

void CameraUW::Main()
{
  while (true)
  {
    // Let the camera driver update this thread
    InQueue->Wait();

    // Test if we are suppose to cancel this thread.
    pthread_testcancel();

    ProcessMessages();
  }
  return;
}

void CameraUW::ProcessImage(player_camera_data_t & rawdata)
{
  char filename[256];
  unsigned char * ptr, * ptr1;
  int i, l;
  unsigned char *buffer = NULL;
  bool willPublish=false;

  if ((rawdata.width != width) || (rawdata.height != height))
	{	// image size changed - should happen only once at the beginning.
	  // set up the input and unwarped images.
	  //TODO: Memory will leak if the size changes...
	  printf("Updating size!!!!\n");
	  width=rawdata.width;
	  height=rawdata.height;
	  inputImage = new OcvImage< BGRPixel<unsigned char> >(width,height,IPL_DEPTH_8U,3);
	  unwarpedLookup = createPlaneImageLookup< BGRPixel<unsigned char> > (inputImage,z,k1,k2,k3);


	} else if ((rawdata.width <= 0) || (rawdata.height <= 0))
  {	// no image?
    if (!(this->data.image)) return;
  } else if (rawdata.compression == PLAYER_CAMERA_COMPRESS_RAW)
  {	// not compressed
    switch (rawdata.bpp) {
    case 8:	// greyscale, duped into an 3 channel
      l = (rawdata.width) * (rawdata.height);
      ptr = buffer = new unsigned char[(rawdata.width) * (rawdata.height) * 3];
      assert(buffer);
      ptr1 = (unsigned char *)(rawdata.image);
      for (i = 0; i < l; i++)
      {
        ptr[0] = *ptr1;
        ptr[1] = *ptr1;
        ptr[2] = *ptr1;
        ptr += 3; ptr1++;
      }
      ptr = buffer;
      break;
    case 24:	// colour, no coppy needed
      ptr = (unsigned char *)(rawdata.image);
      break;
    case 32:	// extra channel, duped into removing 4th channel
      l = (rawdata.width) * (rawdata.height);
      ptr = buffer = new unsigned char[(rawdata.width) * (rawdata.height) * 3];
      assert(buffer);
      ptr1 = (unsigned char *)(rawdata.image);
      for (i = 0; i < l; i++)
      {
        ptr[0] = ptr1[0];
        ptr[1] = ptr1[1];
        ptr[2] = ptr1[2];
        ptr += 3; ptr1 += 4;
      }
      ptr = buffer;
      break;
    default:
      PLAYER_WARN("unsupported image depth (not good)");
      return;
    }

	// Move the data input into the input image space for the unwarp table to work...
	memcpy(inputImage->iplimage->imageData,ptr,width*height*3);

	// Process the unwarped image into the ouput space
    if (this->data.image) delete []this->data.image;
    this->data.image = new unsigned char[(unwarpedLookup->width()) * (unwarpedLookup->height())* 3];
    assert(this->data.image);
    int pt=0;
    for (int x=0;x<unwarpedLookup->height();x++) {
		for (int y=unwarpedLookup->width()-1;y>=0;y--) {
			
			data.image[pt]=(*((*unwarpedLookup)[y][x])).r;
			pt++;
			data.image[pt]=(*((*unwarpedLookup)[y][x])).g;
			pt++;
			data.image[pt]=(*((*unwarpedLookup)[y][x])).b;
			pt++;
			
		}
	}
    this->data.image_count = (unwarpedLookup->width()) * (unwarpedLookup->height()) * 3;
    this->data.width = unwarpedLookup->width();
    this->data.height = unwarpedLookup->height();
    this->data.bpp = 24;
    this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
    this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;
    this->data.image_count = (this->data.image_count);
    willPublish=true;
   
	  

  } else
  {	
	  PLAYER_WARN("(CameraUW) I think the subscribed camera is compressed, I need a raw image!");
  }


  // If the image has been duped into 3 channel then delete the dupe
  if (buffer) delete []buffer;
  buffer = NULL;
  if (willPublish)
	  Publish(device_addr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, (void*) &this->data, 0, &this->camera_time);
}


extern "C" {
int player_driver_init(DriverTable* table) {
	CameraUW_Register(table);
	return (0);
}
}
