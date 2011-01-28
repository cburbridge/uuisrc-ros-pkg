/*
 *  Camera Rotator Driver
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
#include "../OcvImage.h"
//#include "../../../opencv/include/cv.h"
#include <highgui.h> // include opencv highgui

#include <libplayercore/playercore.h>
#include <libplayercore/error.h>

class CameraRotate: public Driver {
	// Constructor
public:
	CameraRotate(ConfigFile* cf, int section);

	// Setup/shutdown routines.
	virtual int Setup();
	virtual int Shutdown();

	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr,void * data);

	// Main function for device thread.
private:
	virtual void Main();

	void ProcessImage(player_camera_data_t & rawdata);

	// Input camera device

	// Camera device info
	Device *camera;
	Device *position;
	player_devaddr_t camera_id;
	player_devaddr_t pos2d_id;

	float m[6];
	CvMat M;


	double camera_time;
	bool camera_subscribed;

	int cX,cY;

	// Output rotated camera data to be filled from ocv image
	player_camera_data_t data;
	int frameno;
	double theta;

	// Save image frames?
	int width, height;

	bool willPublish;


	// Buffer for the image and the UW lookup table to point to
	OcvImage< BGRPixel<unsigned char> > *inputImage;
	OcvImage< BGRPixel<unsigned char> > *outputImage;
};

Driver *CameraRotate_Init(ConfigFile *cf, int section) {
	return ((Driver*) (new CameraRotate(cf, section)));
}

void CameraRotate_Register(DriverTable *table) {
	table->AddDriver("camerarotate", CameraRotate_Init);
}

CameraRotate::CameraRotate(ConfigFile *cf, int section) :
	Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_CAMERA_CODE) {
	printf("CameraRotate driver starting....\n");
	this->data.image = NULL;
	this->frameno = 0;

	this->camera = NULL;
	// Must have a camera device
	printf("  (CameraRotate) checking for subscribed camera...\n");
	if (cf->ReadDeviceAddr(&this->camera_id, section, "requires",
			PLAYER_CAMERA_CODE, -1, NULL) != 0) {
		printf("     setting error!\n");this->SetError(-1);
		return;
	}
	// Must have a position device
	printf("  (CameraRotate) checking for subscribed pos2d...\n");
	if (cf->ReadDeviceAddr(&this->pos2d_id, section, "requires",
			PLAYER_POSITION2D_CODE, -1, NULL) != 0) {
		this->SetError(-1);
		return;
	}

	this->camera_time = 0.0;

	this->cX = cf->ReadInt(section, "cX", 320);
	this->cY = cf->ReadInt(section, "cY", 240);

	width = height = -1;
	M = cvMat(2, 3, CV_32F, m);

	printf("CameraRotate driver constructed.\n");fflush(stdout);

	willPublish = false;

	return;
}

int CameraRotate::Setup() {
	// Subscribe to the camera.
	if (Device::MatchDeviceAddress(this->camera_id, this->device_addr)) {
		PLAYER_ERROR("attempt to subscribe to self");
		return (-1);
	}
	if (!(this->camera = deviceTable->GetDevice(this->camera_id))) {
		PLAYER_ERROR("unable to locate suitable camera device");
		return (-1);
	}
	if (this->camera->Subscribe(this->InQueue) != 0) {
		PLAYER_ERROR("unable to subscribe to camera device");
		return (-1);
	}

	// Subscribe to the position2d.
	if (Device::MatchDeviceAddress(this->pos2d_id, this->device_addr)) {
		PLAYER_ERROR("attempt to subscribe to self");
		return (-1);
	}
	if (!(this->position = deviceTable->GetDevice(this->pos2d_id))) {
		PLAYER_ERROR("unable to locate suitable pos2d");
		return (-1);
	}
	if (this->position->Subscribe(this->InQueue) != 0) {
		PLAYER_ERROR("unable to subscribe to pos2d device");
		return (-1);
	}

	width = height = -1;
	willPublish=false;

	// Start the driver thread.
	this->StartThread();

	return 0;
}

int CameraRotate::Shutdown() {
	// Stop the driver thread
	StopThread();

	camera->Unsubscribe(InQueue);
	position->Unsubscribe(InQueue);

	if (this->data.image) {
		delete[] this->data.image;
		this->data.image = NULL;
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int CameraRotate::ProcessMessage(QueuePointer & resp_queue,
		player_msghdr * hdr, void * data) {
	assert(hdr);

	if (Message::MatchMessage(hdr,
			PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, camera_id)) {
		assert(data);
		player_camera_data_t * recv =
				reinterpret_cast<player_camera_data_t *> (data);
		ProcessImage(*recv);
		return 0;
	}

	if (Message::MatchMessage(hdr,
			PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE, pos2d_id)) {
		player_position2d_data_t * recv =
				reinterpret_cast<player_position2d_data_t *> (data);
		theta = recv->pos.pa;
		willPublish=true;

		return 0;
	}

	return -1;
}

void CameraRotate::Main() {
	while (true) {
		// Let the camera driver update this thread
		InQueue->Wait();

		// Test if we are suppose to cancel this thread.
		pthread_testcancel();

		ProcessMessages();
	}
	return;
}

void CameraRotate::ProcessImage(player_camera_data_t & rawdata) {
	char filename[256];
	unsigned char * ptr, *ptr1;
	int i, l;
	unsigned char *buffer = NULL;

	if ((rawdata.width != width) || (rawdata.height != height)) { // image size changed - should happen only once at the beginning.
		// set up the input and unwarped images.
		//TODO: Memory will leak if the size changes...
		printf("(-:CameraRotate:-) Updating size!!!!\n");
		width = rawdata.width;
		height = rawdata.height;
		inputImage = new OcvImage<BGRPixel<unsigned char> > (width, height,	IPL_DEPTH_8U,3);
		outputImage = new OcvImage<BGRPixel<unsigned char> > (width, height,	IPL_DEPTH_8U,3);
		if (this->data.image)
			delete[] this->data.image;
		this->data.image = new unsigned char[(outputImage->width()) * (outputImage->height()) * 3];

	} else if ((rawdata.width <= 0) || (rawdata.height <= 0)) { // no image?
		if (!(this->data.image))
			return;
	} else if (rawdata.compression == PLAYER_CAMERA_COMPRESS_RAW) { // not compressed
		switch (rawdata.bpp) {
		case 8: // greyscale, duped into an 3 channel

			break;
		case 24: // colour, no coppy needed
			ptr = (unsigned char *) (rawdata.image);
			break;
		case 32: // extra channel, duped into removing 4th channel

			break;
		default:
			PLAYER_WARN("unsupported image depth (not good)");
			return;
		}

		// Move the data input into the input image space for the unwarp table to work...
		memcpy(inputImage->iplimage->imageData, ptr, width * height * 3);

		// Process the input image into the ouput space
		// little rotation matrix
		m[0] = (float) (cos(theta));
		m[1] = (float) (sin(theta));
		m[3] = -m[1];
		m[4] = m[0];
		// and the translation (centre point from original image)
		m[2] = inputImage->width() * 0.5f;
		m[5] = inputImage->height() * 0.5f;

		cvGetQuadrangleSubPix((*inputImage), (*outputImage), &M);

		int pt = 0;
		for (int y = 0; y < outputImage->height(); y++) {
			for (int x = 0; x < outputImage->width(); x++) {
				data.image[pt] = ((*outputImage)[y][x]).r;
				pt++;
				data.image[pt] = (((*outputImage)[y][x])).g;
				pt++;
				data.image[pt] = (((*outputImage)[y][x])).b;
				pt++;
			}
		}
		//	  std::cout << "Camera sim almost ready...\n";
		this->data.image_count = (outputImage->width()) * (outputImage->height())
				* 3;
		this->data.width = outputImage->width();
		this->data.height = outputImage->height();
		this->data.bpp = 24;
		this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
		this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;
//		willPublish = true;



//		assert(this->data.image);
////		int pt = 0;
//		for (int x = 0; x < unwarpedLookup->height(); x++) {
//			for (int y = unwarpedLookup->width() - 1; y >= 0; y--) {
//				if (mario) {
//				} else {
//					data.image[pt] = (*((*unwarpedLookup)[y][x])).r;
//					pt++;
//					data.image[pt] = (*((*unwarpedLookup)[y][x])).g;
//					pt++;
//					data.image[pt] = (*((*unwarpedLookup)[y][x])).b;
//					pt++;
//				}
//			}
//		}
//		this->data.image_count = (unwarpedLookup->width())
//				* (unwarpedLookup->height()) * 3;
//		this->data.width = unwarpedLookup->width();
//		this->data.height = unwarpedLookup->height();
//		this->data.bpp = 24;
//		this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
//		this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;
//		this->data.image_count = (this->data.image_count);
		// This point do the compression


		//    this->data.image_count = jpeg_compress( (char *)(this->data.image),
		//                                            (char *)ptr,
		//                                            rawdata.width,
		//                                            rawdata.height,
		//                                            (rawdata.width) * (rawdata.height) * 3,
		//                                            (int)(this->quality*100));

	} else { // compressed so it is simply passed on?
		PLAYER_WARN("(CameraRotate) I think the subscribed camera is compressed, I need a raw image!");
	}


	if (willPublish){
		Publish(device_addr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE,
				(void*) &this->data, 0, &this->camera_time);
		willPublish=false;
	}
	// don't delete anything here! this->data.image is required and is deleted somewhere else
}


extern "C" {
int player_driver_init(DriverTable* table) {
	CameraRotate_Register(table);
	return (0);
}
}
