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

#ifndef VICONBASESTREAM_H_
#define VICONBASESTREAM_H_

#include "libvicon/ViconBase.h"

#include <pthread.h>

class ViconBaseStream: public  ViconBase {
public:
	ViconBaseStream(const char *ModelFile, const char *SubjectName, const char* viconpc = DEFAULT_IP, int port = DEFAULT_PORT);

	virtual ~ViconBaseStream();

	virtual int requestValuesUpdate();

private:
	pthread_mutex_t mutex;
	pthread_t stream_thread;

	/**
	 *  Initialise the connections as streaming
	 */
	int initialiseMode();


	/**
	 *  The thread that runs in the background to interpret vicon stream
	 */
	void streamingThread();
	friend void* ViconStreamThreadStarter(void* vs);
};

#endif /* VICONBASESTREAM_H_ */
