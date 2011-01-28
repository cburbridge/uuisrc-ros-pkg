/*
 *  Camera Simulator Player Driver
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

#include <highgui.h>

#include <libplayercore/playercore.h>
#include <libplayercore/error.h>

class CameraSimulator: public Driver {
		// Constructor
	public:
		CameraSimulator(ConfigFile* cf, int section);

		// Setup/shutdown routines.
	public:
		virtual int Setup();
	public:
		virtual int Shutdown();

		// This method will be invoked on each incoming message
	public:
		virtual int
				ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data);

		// Main function for device thread.
	private:
		virtual void Main();

	private:
		void ProcessImage();

		// Input camera device
	private:

		// Camera device info
		Device *camera;
		player_devaddr_t camera_id;
		double camera_time;
		bool camera_subscribed;

		// Output camera data
	private:
		player_camera_data_t data;

		int width, height;

		double x, y, th;

		float m[6];
		CvMat M;

		OcvImage<RGBPixel<unsigned char> > image;
		OcvImage<RGBPixel<unsigned char> > *imageOutput;
		Device* position_device;
		player_devaddr_t position_addr;
};

Driver *CameraUW_Init(ConfigFile *cf, int section) {
	return ((Driver*) (new CameraSimulator(cf, section)));
}

void CameraUW_Register(DriverTable *table) {
	table->AddDriver("camerasim", CameraUW_Init);
}

CameraSimulator::CameraSimulator(ConfigFile *cf, int section) :
			Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_CAMERA_CODE) {
	this->position_device = NULL;
	if (cf->ReadDeviceAddr(&this->position_addr, section, "requires", PLAYER_POSITION2D_CODE, -1, NULL)
			!= 0) {
		this->SetError(-1);
		return;
	}
	if (!(this->position_device = deviceTable->GetDevice(this->position_addr))) {
		PLAYER_ERROR("unable to locate suitable position device");
		return;
	}
	if (this->position_device->Subscribe(this->InQueue) != 0) {
		PLAYER_ERROR("unable to subscribe to position device");
		return;
	}

	this->data.image = NULL;
	camera_time = 0.0;
	width = cf->ReadInt(section, "width", 640);
	height = cf->ReadInt(section, "height", 480);
	// Load the image and prep it

	const char* imagename = cf->ReadString(section, "image", "image.png");
	std::cout << "Loading image " << imagename << std::endl;
	image = cvLoadImage(imagename);
	std::cout << "image loaded. size = " << image.width() << " X "
			<< image.height() << std::endl;
	;
	this->data.image = new unsigned char[width * height * 3];

	imageOutput
			= new OcvImage<RGBPixel<unsigned char> > (width, height, IPL_DEPTH_8U, 3);

	M = cvMat(2, 3, CV_32F, m);

}

int CameraSimulator::Setup() {
	// Start the driver thread.
	this->StartThread();

	return 0;
}

int CameraSimulator::Shutdown() {
	// Stop the driver thread
	StopThread();

	if (this->data.image) {
		delete[] this->data.image;
		this->data.image = NULL;
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int CameraSimulator::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data) {
	assert(hdr);
	//
	if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE, position_addr)) {
		assert(data);
		player_position2d_data_t * recv =
				reinterpret_cast<player_position2d_data_t *> (data);
		th = recv->pos.pa;
		x = recv->pos.px;
		y = recv->pos.py;
		ProcessImage();
		return 0;
	}

	return -1;
}

void CameraSimulator::Main() {
	while (true) {

		// Test if we are suppose to cancel this thread.
		pthread_testcancel();

		ProcessMessages();

		//    ProcessImage();

		//    usleep(10000);
	}
	return;
}

void CameraSimulator::ProcessImage() {
	// Copy the section of the image into thhe publish image
	float angle = th;
	// little rotation matrix
	m[0] = (float) (cos(-angle));
	m[1] = (float) (sin(-angle));
	m[3] = -m[1];
	m[4] = m[0];
	// and the translation (centre point from original image)
	m[2] = image.width() * 0.5f + x * 100.0;
	m[5] = image.height() * 0.5f + y * 100.0;

	cvGetQuadrangleSubPix(image, (*imageOutput), &M);

	int pt = 0;
	for (int y = 0; y < imageOutput->height(); y++) {
		for (int x = 0; x < imageOutput->width(); x++) {
			data.image[pt] = ((*imageOutput)[y][x]).r;
			pt++;
			data.image[pt] = (((*imageOutput)[y][x])).g;
			pt++;
			data.image[pt] = (((*imageOutput)[y][x])).b;
			pt++;
		}
	}
	//	  std::cout << "Camera sim almost ready...\n";
	this->data.image_count = (imageOutput->width()) * (imageOutput->height())
			* 3;
	this->data.width = imageOutput->width();
	this->data.height = imageOutput->height();
	this->data.bpp = 24;
	this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
	this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;

	// Process the unwarped image into the ouput space
	Publish(device_addr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, (void*) &this->data, 0, &this->camera_time);
}

extern "C" {
	int player_driver_init(DriverTable* table) {
		CameraUW_Register(table);
		return (0);
	}
}
