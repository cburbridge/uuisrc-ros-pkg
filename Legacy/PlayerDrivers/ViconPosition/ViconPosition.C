/*
 *  Vicon Player Driver - provides a position 2d interface giving the robot position
 *  as found using the vicon server.
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
 */


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

#include <libplayercore/playercore.h>
#include <libplayercore/error.h>

#include "../../../vicon/libvicon/include/ViconBaseStream.h"

/********************************************************************/

class ViconPosition: public Driver {
private:
	int port;
	const char* server;
	float cX,cY,cZ,fX,fY,fZ;	// Virtual marker positions
	const char *modelfile;
	const char *subjectName;
	bool rawcoordinates;
	bool stream;

	float odomX, odomY, odomTh;
	float odomDx,odomDy,odomDth;

	double dt, timestamp;

	float originX,originY,originTh;

	//    player_blobfinder_data_t   mData;
	//    unsigned int     allocated_blobs;

	player_position2d_data_t ViconInfo;
	player_position2d_data_t ViconInfoTransformed;

	player_devaddr_t Position2D_Address;
	Device* Position2D_Device;

	player_position2d_geom geometry;
	player_position2d_data_t* position_data;

	ViconBase *vicon;
	Marker *front;
	Marker *centre;


public:
	int Setup();
	int Shutdown();
	// constructor
	ViconPosition(ConfigFile* cf, int section);
	virtual ~ViconPosition();
	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr,
			void * data);
	virtual void Main();
	//    void ProcessImageData();
};

// a factory creation function
Driver* ViconPosition_Init(ConfigFile* cf, int section) {
	std::cout << "ViconPosition_Init();\n";
	return ((Driver*) (new ViconPosition(cf, section)));
}

// a driver registration function
void ViconPosition_Register(DriverTable* table) {
	table->AddDriver("viconposition", ViconPosition_Init);
}

ViconPosition::ViconPosition(ConfigFile* cf, int section) :
	Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_POSITION2D_CODE) ,
	port(800), server(NULL), modelfile(NULL),subjectName(NULL), cX(0),cY(0),cZ(0),fX(0),fY(0),fZ(1),rawcoordinates(false){

	server = cf->ReadString(section, "server", "192.168.1.3");
	port = cf->ReadInt(section, "port", 800);
	modelfile = cf->ReadString(section, "modelfile", "unknown");
	subjectName = cf->ReadString(section, "subject", "Subject 1");
	cX=cf->ReadTupleFloat(section,"robot_centre",0,0);
	cY=cf->ReadTupleFloat(section,"robot_centre",1,0);
	cZ=cf->ReadTupleFloat(section,"robot_centre",2,0);
	fX=cf->ReadTupleFloat(section,"robot_front",0,0);
	fY=cf->ReadTupleFloat(section,"robot_front",1,0);
	fZ=cf->ReadTupleFloat(section,"robot_front",2,1);
	rawcoordinates = cf->ReadBool(section,"raw_coordinates",false);
	stream = cf->ReadBool(section,"use_streaming",false);

	// Must have a position 2D
	if (cf->ReadDeviceAddr(&Position2D_Address, section, "requires", PLAYER_POSITION2D_CODE, -1, NULL) != 0) {
		PLAYER_ERROR("ViconPosition error: an underlying Position2d provided by the robot is not available.\n");
		return;
	}

	position_data=NULL;


	timestamp=0;
}

ViconPosition::~ViconPosition() {

}

int ViconPosition::Setup() {
	printf("ViconPosition plugin initializing...");
	fflush(stdout);
	// Subscribe to the camera device
	if (!(this->Position2D_Device = deviceTable->GetDevice(
			this->Position2D_Address))) {
		PLAYER_ERROR("ViconPosition: unable to locate suitable position 2d device");
		return (-1);
	}
	if (0 != this->Position2D_Device->Subscribe(this->InQueue)) {
		PLAYER_ERROR("ViconPosition: unable to subscribe to underlying position 2d device");
		return (-1);
	}

//	this->Position2D_Device->Request(this->InQueue,PLAYER_MSGTYPE_REQ,PLAYER_POSITION2D_REQ_GET_GEOM,NULL,0,NULL,false);
	this->Position2D_Device->PutMsg(this->InQueue,PLAYER_MSGTYPE_REQ,PLAYER_POSITION2D_REQ_GET_GEOM,NULL,0,NULL);


	if (stream)
		vicon = new ViconBaseStream(modelfile,subjectName,server,port);
	else
		vicon = new ViconBase(modelfile,subjectName,server,port);

	if (!vicon->isAlive()) {
		PLAYER_ERROR("ViconPosition error: Unable to work with vicon.\n");
		return -1;
	}


	if (vicon->Segments.size() > 1) {
		PLAYER_WARN("Multiple segments in vicon model, the first segment will be used!");
	}
	vicon->addVirtualMarker(cX,cY,cZ,&(vicon->Segments[0]),"Centre");
	vicon->addVirtualMarker(fX,fY,fZ,&(vicon->Segments[0]),"Front");
	centre=&vicon->Segments[0].VirtualMarkers[0];
	front=&vicon->Segments[0].VirtualMarkers[1];
	vicon->requestValuesUpdate();

	vicon->printStructure();
	ViconInfo.pos.px=ViconInfo.pos.py=ViconInfo.pos.pa=0;
	odomX=odomY=odomTh=0;
	originX=originY=originTh=0.0;

	puts("done.");
	StartThread();
	return (0);
}

int ViconPosition::Shutdown() {
//
	StopThread();

	// Unsubscribe from the camera
	this->Position2D_Device->Unsubscribe(this->InQueue);

	delete vicon;

	puts("ViconPosition plugin has been shutdown");
	return (0);
}

void ViconPosition::Main() {
	// The main loop; interact with the device here
	for (;;) {
		// wait to receive a new message (blocking)
		Wait();

		// test if we are supposed to cancel
		pthread_testcancel();

		// Process incoming messages, and update outgoing data
		ProcessMessages();
	}
	return;
}

int ViconPosition::ProcessMessage(QueuePointer & resp_queue, player_msghdr* hdr, void* data) {
	//  Handle the data from the position2d subscribed to and requests to this position2d

	if (Message::MatchMessage(hdr, -1, -1, this->Position2D_Address)) {	// A message related to the subscribed position 2D
		switch(hdr->type) {
			case PLAYER_MSGTYPE_DATA:
				switch (hdr->subtype) {
					case PLAYER_POSITION2D_DATA_GEOM:	//  Underlying position 2D has change geometry.
						printf("Underlying position geom changed\n");
						memcpy(&geometry, data, sizeof(player_position2d_geom));
						Publish(device_addr,PLAYER_MSGTYPE_DATA,PLAYER_POSITION2D_DATA_GEOM,data);
						return 0;
						break;

					case PLAYER_POSITION2D_DATA_STATE:	//	Underlying position 2D has change state
						// Calculate the change in odometry since last
						position_data =reinterpret_cast<player_position2d_data_t *> (data);
						odomDx = position_data->pos.px - odomX;
						odomDy = position_data->pos.py - odomY;
						odomDth = position_data->pos.pa - odomTh;

						odomX = position_data->pos.px;
						odomY = position_data->pos.py;
						odomTh = position_data->pos.pa;

						dt=hdr->timestamp-timestamp;
						timestamp=hdr->timestamp;

						// Check if the vicon can provide us with a new position
						vicon->requestValuesUpdate();

						// Add the velocity information to the vicon driver
						ViconInfo.vel.pa = position_data->vel.pa;
						ViconInfo.vel.px = position_data->vel.px;
						ViconInfo.vel.py = position_data->vel.py;

						//if (false) {
						if (vicon->Segments.at(0).visible) {
							ViconInfo.pos.px=centre->X/1000.0;
							ViconInfo.pos.py=centre->Y/1000.0;
							ViconInfo.pos.pa=atan2(front->Y-centre->Y,front->X-centre->X);
						} else {
							// Chris's way
							double dist = sqrt(odomDx*odomDx + odomDy*odomDy) ;
							ViconInfo.pos.px += dist*cos(ViconInfo.pos.pa);
							ViconInfo.pos.py += dist*sin(ViconInfo.pos.pa);
							ViconInfo.pos.pa += odomDth;
							if (ViconInfo.pos.pa > M_PI) {
								ViconInfo.pos.pa-=2.0*M_PI;
							} else if (ViconInfo.pos.pa<-M_PI) {
								ViconInfo.pos.pa+=2.0*M_PI;
							}

						}
						if (rawcoordinates){
							Publish(device_addr,PLAYER_MSGTYPE_DATA,PLAYER_POSITION2D_DATA_STATE,reinterpret_cast<void*>(&ViconInfo));
						}else {	// rotate translate to origin and publish the transformed vicon data
							double posx = ViconInfo.pos.px;
							double posy = ViconInfo.pos.py;
							double posa = ViconInfo.pos.pa;

							posa -= originTh;
							posx = posx*cos(-originTh) + posy*sin(-originTh) - originX;
							posy = posy*cos(-originTh) - posx*sin(-originTh) - originY;

//							posx = posx*cos(originTh) - posx*sin(originTh) - originX;
//							posy = posy*sin(originTh) + posy*cos(originTh) - originY;
//							posa = posa - originTh;

							ViconInfoTransformed.pos.px = posx;
							ViconInfoTransformed.pos.py = posy;
							ViconInfoTransformed.pos.pa = posa;

							Publish(device_addr,PLAYER_MSGTYPE_DATA,PLAYER_POSITION2D_DATA_STATE,reinterpret_cast<void*>(&ViconInfoTransformed));
						}

						return 0;
						break;

					default:
						printf("Another message type received?\n");
						return 0;
				}
				break;
			case PLAYER_MSGTYPE_RESP_ACK:
				switch (hdr->subtype) {
					case PLAYER_POSITION2D_REQ_GET_GEOM:	//  Our geometry reply from the underlying position driver
						memcpy(&geometry, data, sizeof(player_position2d_geom));
						printf("Geometry received: %f,%f,%f\n",geometry.pose.px,geometry.pose.py,geometry.pose.pz);
						return 0;
				}
		}

//
//		player_position2d_data_t* position_data =reinterpret_cast<player_position2d_data_t *> (data);
//		x = position_data->pos.px + 10;
//		y = position_data->pos.py + 10;
//		th = position_data->pos.pa;

	} else if (Message::MatchMessage(hdr, -1, -1, device_addr)) {		// A message related to this driver
		switch(hdr->type) {
			case PLAYER_MSGTYPE_CMD:
				switch (hdr->subtype) {
					case PLAYER_POSITION2D_CMD_CAR:
					case PLAYER_POSITION2D_CMD_POS:
					case PLAYER_POSITION2D_CMD_VEL:
					case PLAYER_POSITION2D_CMD_VEL_HEAD:
						this->Position2D_Device->PutMsg(resp_queue,hdr,data);

						return 0;
						break;
				}
				break;
			case PLAYER_MSGTYPE_REQ:
				switch (hdr->subtype) {
					case PLAYER_POSITION2D_REQ_RESET_ODOM:
						printf("Viconposition position2d has REQ_RESET_ODOM\n");
						originX=ViconInfo.pos.px;
						originY=ViconInfo.pos.py;
						originTh=ViconInfo.pos.pa;
						return 0;
						break;
					case PLAYER_POSITION2D_REQ_SET_ODOM:
						printf("Viconposition position2d has REQ_SET_ODOM\n");
						originX=ViconInfo.pos.px;
						originY=ViconInfo.pos.py;
						originTh=ViconInfo.pos.pa;
						return 0;
						break;
					case PLAYER_POSITION2D_REQ_GET_GEOM:
//						Publish(device_addr,resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_GET_GEOM, reinterpret_cast<void*>(&geometry));
//						return 0;
					case PLAYER_POSITION2D_REQ_MOTOR_POWER:
					case PLAYER_POSITION2D_REQ_POSITION_MODE:
					case PLAYER_POSITION2D_REQ_POSITION_PID:
					case PLAYER_POSITION2D_REQ_SPEED_PID:
					case PLAYER_POSITION2D_REQ_SPEED_PROF:
					case PLAYER_POSITION2D_REQ_VELOCITY_MODE:
//						this->Position2D_Device->PutMsg(resp_queue,hdr->type,hdr->subtype,data,0,NULL);
							Message* msg;

							if(!(msg = this->Position2D_Device->Request(this->InQueue, hdr->type, hdr->subtype,(void*)data,hdr->size, &hdr->timestamp)))
							{
								PLAYER_WARN1("failed to forward request with subtype: %d\n",	hdr->subtype);
								return(-1);
							}

							player_msghdr_t* rephdr = msg->GetHeader();
							void* repdata = msg->GetPayload();
							// Copy in our address and forward the response
							rephdr->addr = device_addr;
							this->Publish(resp_queue, rephdr, repdata);
							delete msg;
						// header contains the device addr which makes player confused when reply comes from wrong device
						return 0;
						break;
				}
				break;
		}

	}
	// Tell the caller that we don't know how to handle this message
	return (-1);
}

extern "C" {
int player_driver_init(DriverTable* table) {
	ViconPosition_Register(table);
	return (0);
}
}
