/***************************************************************************
 *   Copyright (C) 2007 by Lorenzo Riano   *
 *   lorenzo.riano@gmail.com   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "dynamicwindowcontroller.h"
#include <sys/time.h>
#include <ctime>

DynamicWindowController::DynamicWindowController(ConfigFile* cf, int section)
	: Driver(cf,section,true,PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_POSITION2D_CODE)
{
	srand(time(NULL));
	this->position_device = NULL;
	if (cf->ReadDeviceAddr(&this->position_addr, section, "requires",PLAYER_POSITION2D_CODE, -1, NULL) != 0) {
		this->SetError(-1);
		return;
	}

	this->laser_device = NULL;
	memset(&this->laser_addr,0,sizeof(player_devaddr_t));
	if (cf->ReadDeviceAddr(&this->laser_addr, section, "requires",PLAYER_LASER_CODE, -1, NULL) != 0) {
		this->SetError(-1);
		return;
	}
	
	double alpha = cf->ReadFloat(section,"alpha",0.5);
	dw.setAlpha(alpha);	
	double beta = cf->ReadFloat(section,"beta",0.2);
	dw.setBeta(beta);	
	double gamma = cf->ReadFloat(section,"gamma",0.3);
	dw.setGamma(gamma);	
	double wpmax = cf->ReadAngle(section,"wpmax",DynamicWindow::deg2rad(100));
	dw.setWpmax(wpmax);
	double wmin = cf->ReadAngle(section,"wmin",DynamicWindow::deg2rad(-90));
	dw.setWmin(wmin);
	double wmax = cf->ReadAngle(section,"wmax",DynamicWindow::deg2rad(90));
	dw.setWmax(wmax);
	double wpbrake = cf->ReadAngle(section,"wpbrake",DynamicWindow::deg2rad(100));
	dw.setWpbrake(wpbrake);
	double vmax = cf->ReadFloat(section,"vmax",0.5);
	dw.setVmax(vmax);
	m_vmax = vmax;
	double vpmax = cf->ReadFloat(section,"vpmax",1.0);
	dw.setVpmax(vpmax);
	double vpbrake = cf->ReadFloat(section,"vpbrake",0.3);
	dw.setVpbrake(vpbrake);
	m_tcmax = cf->ReadFloat(section,"time",0.2);
	dw.setTime_c(m_tcmax);
	
	// thresholds for deciding if goal reached (cjcb)
	m_dist_done = cf->ReadFloat(section,"distdone",0.5);	// distance in m
	m_theta_done = cf->ReadFloat(section,"angdone",5);		// angle in degrees

	m_active_goal = false;
	m_x_goal = 0;
	m_y_goal = 0;
	m_x_robot = 0;
	m_y_robot = 0;
	m_theta_robot = 0;
	m_v_robot = 0;
	m_w_robot = 0;
}


DynamicWindowController::~DynamicWindowController()
{
}

int DynamicWindowController::Setup () {
	
	if (this->setupPosition() != 0)
		return -1;

	if (this->laser_addr.interf && this->setupLaser() != 0)
		return -1;
	this->StartThread();

	return 0;
	
}
		
int DynamicWindowController::setupPosition() {
	
	if(!(this->position_device = deviceTable->GetDevice(this->position_addr)))
	{
		PLAYER_ERROR("POS1");
		PLAYER_ERROR("unable to locate suitable position device");
		return -1;
	}
	if(this->position_device->Subscribe(this->InQueue) != 0)
	{
		PLAYER_ERROR("POS2");
		PLAYER_ERROR("unable to subscribe to position device");
		return -1;
	}
	Message* msg;
	if(!(msg = this->position_device->Request(this->InQueue,PLAYER_MSGTYPE_REQ,PLAYER_POSITION2D_REQ_GET_GEOM,NULL, 0, NULL,false)) ||
		    (msg->GetHeader()->size != sizeof(player_position2d_geom_t)))
	{
		PLAYER_ERROR("POS3");
		PLAYER_ERROR("failed to get geometry of underlying position device");
		if(msg)
			delete msg;
		return -1;
	}
	player_position2d_geom_t* geom = (player_position2d_geom_t*)msg->GetPayload();

	float robot_radius = MAX(geom->size.sl,geom->size.sw);
	robot_radius /= 2.0;

	dw.setRadius(robot_radius);	
	delete msg;
	return 0;

}

int DynamicWindowController::setupLaser() {
	
	if(!(this->laser_device = deviceTable->GetDevice(this->laser_addr)))
	{
		PLAYER_ERROR("LAS1");
		PLAYER_ERROR("unable to locate suitable laser device");
		return -1;
	}
	if (this->laser_device->Subscribe(this->InQueue) != 0)
	{
		PLAYER_ERROR("LAS2");
		PLAYER_ERROR("unable to subscribe to laser device");
		return -1;
	}
	return 0;	
}

		
int DynamicWindowController::Shutdown () {
	
	StopThread();
	this->laser_device->Unsubscribe(this->InQueue);
	this->position_device->Unsubscribe(this->InQueue);
	putCommand(0,0);
	return 0;
}

void DynamicWindowController::processOdom(player_msghdr_t* hdr, player_position2d_data_t &data) {
	
	m_x_robot = data.pos.px;
	m_y_robot = data.pos.py;
	m_theta_robot = data.pos.pa;
	m_v_robot = data.vel.px;
	m_w_robot = data.vel.pa;
	
	//publish data for everyone
	player_msghdr_t newhdr = *hdr;
	newhdr.addr = this->device_addr;
	this->Publish(&newhdr, (void*)&data);
}

void DynamicWindowController::processLaser(player_laser_data_t &data) {
	
	dw.setLaser_count(data.ranges_count);
	dw.setLaser_max(data.max_range);
	float theta = data.min_angle;
	for (unsigned int i=0; i<data.ranges_count; i++) {
// 		std::cout<<data.ranges[i]<<" ";
		m_x_obstacle[i] = data.ranges[i] * cos(theta);
		m_y_obstacle[i] = data.ranges[i] * sin(theta);
		theta = theta + data.resolution;
	}
// 	std::cout<<"\n";
}

void DynamicWindowController::processCommand(player_position2d_cmd_pos_t &cmd) {
	
	m_active_goal = true;
	m_x_goal = cmd.pos.px;
	m_y_goal = cmd.pos.py;
	m_theta_goal = cmd.pos.pa;
	std::cout<<"Received new goal: ("<<m_x_goal<<","<<m_y_goal<<","<<DynamicWindow::rad2deg(m_theta_goal)<<")\n";
// 	std::cout<<"Received new goal: ("<<m_x_goal<<","<<m_y_goal<<m_theta_goal<<")\n";
}

int DynamicWindowController::ProcessMessage(QueuePointer & resp_queue, player_msghdr* hdr, void* data) {
    //  Handle the data from the position2d subscribed to and requests to this position2d
    
    if (Message::MatchMessage(hdr, -1, -1, this->position_addr)) {	// A message related to the subscribed position 2D
	float robot_radius = 0;
	player_position2d_geom_t* geom;
	switch(hdr->type) {
	    case PLAYER_MSGTYPE_DATA:
		switch (hdr->subtype) {
		    case PLAYER_POSITION2D_DATA_GEOM:	//  Underlying position 2D has change geometry.
			printf("Underlying position geom changed\n");
			geom = (player_position2d_geom_t*)data;
			robot_radius = MAX(geom->size.sl,geom->size.sw);
			robot_radius /= 2.0;
			dw.setRadius(robot_radius);
			Publish(device_addr,PLAYER_MSGTYPE_DATA,PLAYER_POSITION2D_DATA_GEOM,data);
			return 0;
			
		    case PLAYER_POSITION2D_DATA_STATE:	//	Underlying position 2D has change state
			assert(hdr->size == sizeof(player_position2d_data_t));
			processOdom(hdr, *reinterpret_cast<player_position2d_data_t *> (data));			
			return 0;
			
		    default:
			printf("Another message type received?\n");
			return 0;
		}
		break;
	    case PLAYER_MSGTYPE_RESP_ACK:
		switch (hdr->subtype) {
		    case PLAYER_POSITION2D_REQ_GET_GEOM:	//  Our geometry reply from the underlying position driver
			printf("DWC: receiced an ack, for what??\n");
			geom = (player_position2d_geom_t*)data;
			robot_radius = MAX(geom->size.sl,geom->size.sw);
			robot_radius /= 2.0;
			dw.setRadius(robot_radius);
			Publish(device_addr,PLAYER_MSGTYPE_DATA,PLAYER_POSITION2D_DATA_GEOM,data);
			return 0;
		    default:
			printf("I dont' know what is this, 1\n");
			return -1;
		}
	    default:
		printf("I dont' know what is this, 2\n");
		return -1;
	}
	
    } 
   
    //
    else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,PLAYER_LASER_DATA_SCAN, this->laser_addr)) 	{
	processLaser(*reinterpret_cast<player_laser_data_t *> (data));
	return 0;
    }
    
    else if (Message::MatchMessage(hdr, -1, -1, device_addr)) {		// A message related to this driver
	switch(hdr->type) {
	    case PLAYER_MSGTYPE_CMD:
		switch (hdr->subtype) {
		    case PLAYER_POSITION2D_CMD_CAR:
			printf("Dynamic window controller has CMD_CAR");
			return 0;
		    case PLAYER_POSITION2D_CMD_POS:			
			assert(hdr->size == sizeof(player_position2d_cmd_pos_t));
			processCommand(*reinterpret_cast<player_position2d_cmd_pos_t *> (data));
			return 0;
		    case PLAYER_POSITION2D_CMD_VEL:			
			assert(hdr->size == sizeof(player_position2d_cmd_vel_t));
			memcpy(&m_own_cmd,reinterpret_cast<player_position2d_cmd_pos_t *> (data),sizeof(m_own_cmd));
			this->m_active_goal = false;
			return 0;
		    case PLAYER_POSITION2D_CMD_VEL_HEAD:
			printf("Dynamic window controller has CMD_VEL_HEAD");
			return 0;
		    default:
			printf("I dont' know what is this, 3\n");
			return -1;
		}
		break;
	    case PLAYER_MSGTYPE_REQ:
		switch (hdr->subtype) {
		    void* repdata;
		    player_msghdr_t* rephdr;
		    case PLAYER_POSITION2D_REQ_RESET_ODOM:
			this->position_device->PutMsg(resp_queue,hdr,data);
			printf("Dynamic window controller has REQ_RESET_ODOM\n");
			return 0;
			break;
		    case PLAYER_POSITION2D_REQ_SET_ODOM:
			this->position_device->PutMsg(resp_queue,hdr,data);
			printf("Dynamic window controller has REQ_SET_ODOM\n");
			return 0;
			break;
		    case PLAYER_POSITION2D_REQ_GET_GEOM:
		    case PLAYER_POSITION2D_REQ_MOTOR_POWER:
		    case PLAYER_POSITION2D_REQ_POSITION_MODE:
		    case PLAYER_POSITION2D_REQ_POSITION_PID:
		    case PLAYER_POSITION2D_REQ_SPEED_PID:
		    case PLAYER_POSITION2D_REQ_SPEED_PROF:
		    case PLAYER_POSITION2D_REQ_VELOCITY_MODE:
			Message* msg;
			
			if(!(msg = this->position_device->Request(this->InQueue, hdr->type, hdr->subtype,(void*)data,hdr->size, &hdr->timestamp)))
			{
			    PLAYER_WARN1("Dynamic window controller failed to forward request with subtype: %d\n",	hdr->subtype);
			    return(-1);
			}
			
			rephdr = msg->GetHeader();
			repdata = msg->GetPayload();
			// Copy in our address and forward the response
			rephdr->addr = device_addr;
			this->Publish(resp_queue, rephdr, repdata);
			delete msg;
			// header contains the device addr which makes player confused when reply comes from wrong device
			return 0;
			break;
		    default:
			printf("I dont' know what is this, 4\n");
			return -1;
		}
		break;
	    default:
		printf("I dont' know what is this, 5\n");
		return -1;
	}    
    }
    // Tell the caller that we don't know how to handle this message
    printf("I really don't know what are you asking me\n");
    return (-1);
}


// int DynamicWindowController::ProcessMessage (QueuePointer &resp_queue, player_msghdr * hdr, void * data) {
// 	//Messages from/to underlying position device
// 	if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_DATA,PLAYER_POSITION2D_DATA_STATE,this->position_addr))
// 	{
// 		assert(hdr->size == sizeof(player_position2d_data_t));
// 		processOdom(hdr, *reinterpret_cast<player_position2d_data_t *> (data));
// 		return(0);
// 	}
// 	else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, -1, this->device_addr))
// 	{
// 		Message* msg;
// 
// 		if(!(msg = this->position_device->Request(this->InQueue,
// 		     hdr->type,hdr->subtype,(void*)data,hdr->size,&hdr->timestamp)))
// 		{
// 			PLAYER_WARN1("failed to forward config request with subtype: %d\n",
// 				     hdr->subtype);
// 			return(-1);
// 		}
// 
// 		player_msghdr_t* rephdr = msg->GetHeader();
// 		void* repdata = msg->GetPayload();
// 		rephdr->addr = this->device_addr;
// 		this->Publish(resp_queue, rephdr, repdata);
// 		delete msg;
// 		return(0);
// 	}
// 	else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,PLAYER_LASER_DATA_SCAN, this->laser_addr))
// 	{
// //		cout <<"hdr, PLAYER_MSGTYPE_DATA,PLAYER_LASER_DATA_SCAN, this->laser_addr)\n";
// 		processLaser(*reinterpret_cast<player_laser_data_t *> (data));
// 		return 0;
// 	}
// 	else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,PLAYER_POSITION2D_CMD_POS,this->device_addr))
// 	{
// //		cout <<"hdr, PLAYER_MSGTYPE_CMD,PLAYER_POSITION2D_CMD_POS,this->device_addr\n";
// 		assert(hdr->size == sizeof(player_position2d_cmd_pos_t));
// 		processCommand(*reinterpret_cast<player_position2d_cmd_pos_t *> (data));
// 		return 0;
// 	}
// 	else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,PLAYER_POSITION2D_CMD_VEL,this->device_addr))
// 	{
// //		cout <<"hdr, PLAYER_MSGTYPE_CMD,PLAYER_POSITION2D_CMD_VEL,this->device_addr\n";
// 		assert(hdr->size == sizeof(player_position2d_cmd_vel_t));
// 		memcpy(&m_own_cmd,reinterpret_cast<player_position2d_cmd_pos_t *> (data),sizeof(m_own_cmd));
// 		
// // 		player_msghdr_t newhdr = *hdr;
// // 		newhdr.addr = this->position_addr;
// // 		this->position_device->PutMsg(this->InQueue, &newhdr, (void*)data);
// 
// 		this->m_active_goal = false;
// 
// 		return 0;
// 	}
// 	else {
// 		std::cout<<"ma che diavolo mi sta arrivando?\n   translation: oh crap.\n";		
// 		return -1;
// 	}
// }

void DynamicWindowController::putCommand( double cmd_speed, double cmd_turnrate ) {
	
	player_position2d_cmd_vel_t cmd;
	memset(&cmd, 0, sizeof(cmd));
	
	cmd.vel.px =  cmd_speed;
	cmd.vel.py =  0;
	cmd.vel.pa =  cmd_turnrate;

	this->position_device->PutMsg(this->InQueue,PLAYER_MSGTYPE_CMD,PLAYER_POSITION2D_CMD_VEL,(void*)&cmd,sizeof(cmd),NULL);
}

void DynamicWindowController::Main() {

	struct timeval past;	
	gettimeofday(&past,NULL);

	while(true) {
		
		Wait();
		pthread_testcancel();
		ProcessMessages();
		
		double dist;
		double desired_theta;
		
		if(!this->m_active_goal) {
			//backward
			if (m_own_cmd.vel.px <= 0) {
				putCommand(m_own_cmd.vel.px,m_own_cmd.vel.pa);
				continue;
			}		       	
			dist = 2.0*m_dist_done;
			desired_theta = m_own_cmd.vel.pa;
			//std::cout<<"Received: "<<m_own_cmd.vel.px<<"\n"; 
			dw.setVmax(m_own_cmd.vel.px);
		}
		else {
			dw.setVmax(m_vmax);
			dist = hypot(m_x_goal-m_x_robot, m_y_goal - m_y_robot);
			double theta_goal = atan2(m_y_goal - m_y_robot,m_x_goal - m_x_robot);		
			DynamicWindow::fix_angle(theta_goal);
			desired_theta = theta_goal - m_theta_robot;
			DynamicWindow::fix_angle(desired_theta);
		}
		
		bool turn_in_place = false;
// 		std::cout<<"Distance to goal: "<<dist<<"\n";
// 		std::cout<<"Desired theta: "<<DynamicWindow::rad2deg(desired_theta)<<"\n";
		if (dist <= m_dist_done) {
			if (fabs(m_theta_robot - m_theta_goal) < DynamicWindow::deg2rad(m_theta_done))  {
				std::cout<<"Goal reached!\n";
				m_active_goal = false;
				bzero(&m_own_cmd,sizeof(m_own_cmd));
				putCommand(0,0);
				continue;
			}
			else {
				std::cout<<"Turning in place\n";
				turn_in_place = true;
			}
		}

		struct timeval present;
		gettimeofday(&present,NULL);
		double elapsed = present.tv_sec -past.tv_sec + double(present.tv_usec - past.tv_usec)/1.0e06;
		if (elapsed < 0.8*m_tcmax) {
			//std::cout<<"INFO: "<<elapsed<<"\n";
			continue;
		}
		if (elapsed > m_tcmax) {
			//PLAYER_WARN1("WARNING, TAKING TOO LONG: %f",elapsed);
		}
		gettimeofday(&past,NULL);
	
		double v,w;
		if (turn_in_place) {
			double tmp_theta = m_theta_goal - m_theta_robot;
			DynamicWindow::fix_angle(tmp_theta);
			dw.motion_model_sampling_time(m_tcmax-0.02,m_v_robot,m_w_robot,tmp_theta,m_x_obstacle,m_y_obstacle,v,w);
			v=0;
		}
		else {
			dw.motion_model_sampling_time(m_tcmax-0.02,m_v_robot,m_w_robot,desired_theta,m_x_obstacle,m_y_obstacle,v,w);
		}
		if ( (m_active_goal) && (!turn_in_place) && (v<=0.03) ){
			w = M_PI/3.0*(desired_theta>=0?1:-1);
			v = -0.3;
		}
		putCommand(v,w);
	}	
}

Driver* DynamicWindowController_Init(ConfigFile* cf, int section)
{
	return ((Driver*) (new DynamicWindowController( cf, section)));
}

void DynamicWindowController_Register(DriverTable* table)
{
	table->AddDriver("dwc",  DynamicWindowController_Init);
	return;
}

extern "C" {
	int player_driver_init(DriverTable* table) {
		DynamicWindowController_Register(table);
		return 0;
	}
}
