/*
 *  ViconBaseStream class extends ViconBase for streaming communication 
 *  with a vicon server.
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
 */

#include "libvicon/ViconBaseStream.h"

void* ViconStreamThreadStarter(void* vs) {
	((ViconBaseStream*)vs)->streamingThread();
	return NULL;
}


ViconBaseStream::ViconBaseStream(const char *ModelFile, const char *SubjectName, const char* viconpc , int port ) :
	ViconBase(ModelFile, SubjectName, viconpc, port ) {
	ready=false;
	if (!alive)
		return;
	std::cout << "Vicon object is a streamer!\n";
	if (alive) {
		if (initialiseMode()==-1) {
			std::cerr << "ERROR: Can't initialise mode\n";
			alive =false;
			return;
	//		throw
		}
	}
	pthread_mutex_init(&mutex,NULL);
	pthread_mutex_unlock(&mutex);

}

ViconBaseStream::~ViconBaseStream() {
	pthread_cancel(stream_thread);
}

int ViconBaseStream::requestValuesUpdate() {
	if (!ready)
		return -1;
	pthread_mutex_lock(&mutex);
//	std::cout << "Locked for value update.\n";
	if (updateTransformations() != 1) {
		std::cerr << "ERROR: Error updating transformations.\n";
		pthread_mutex_unlock(&mutex);
		return -1;
	}
	pthread_mutex_unlock(&mutex);
//	std::cout << "unocked after value update.\n";
	return 1;
}


int ViconBaseStream::initialiseMode() {
		// request stream
	command[0] = 3; // stream
	command[1] = 0; // request (3,0)
	if ((send(sockfd, command, 8, MSG_DONTROUTE)) != 8) {
		std::cerr << "ERROR: could not request data stream! \n";
		return 1;
	}

	// Start the thread
	pthread_create(&stream_thread, NULL, ViconStreamThreadStarter, (void*)this);

	return 1;
}

void ViconBaseStream::streamingThread(){
	unsigned int quant;
	while (true) {

		// Check the kind type
		if ((quant=recv(sockfd, command, 8, MSG_WAITALL)) != 8) {
			std::cerr << "ERROR: could not receive the data stream! no data arriving to socket?\n" <<
						 "       received  "<< quant << "bytes, we want 8.\n";
			return;
		}
		if (command[0] != 2 || command[1] != 1) { // data reply check (2,1)
			std::cerr << "ERROR: could not interpret the data stream! \n";
			std::cerr << "       stream kind identifier (2) =  "<< command[0] << "\n" <<
						 "       stream type identifier (1) =  " << command[1] << std::endl;

		}

		recv(sockfd, &_itemsCount, 4, MSG_WAITALL); // receive number of items
		if (_itemsCount!=numberValues) {
			std::cerr << "ERROR: wrong number of vicon items (changed?)! \n";
			return;
		}
		pthread_mutex_lock(&mutex);

		// receive data
		quant=0;
		while (quant!=sizeof(double)*_itemsCount){
			quant += recv(sockfd,values+(quant/sizeof(double)),(sizeof(double)*_itemsCount)-quant,0);
		}



		pthread_mutex_unlock(&mutex);
		ready=true;
		pthread_testcancel();
	}
}


