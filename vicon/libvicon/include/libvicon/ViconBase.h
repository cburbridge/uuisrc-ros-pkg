/*
 *  ViconBase class for Request-Reply communication with a vicon server
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
 */

#ifndef VICONBASE_H_
#define VICONBASE_H_

#define DEFAULT_IP	"192.168.1.1"
#define DEFAULT_PORT	800

#include <exception>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <boost/algorithm/string.hpp>
#include <libxml2/libxml/parser.h>
#include <libxml2/libxml/tree.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <math.h>


struct Marker {
	std::string Name;
	double X, Y, Z;
	bool visible;
	double modelX, modelY, modelZ;	// the marker pose in model co-ordinates

	int streamPosition;

	inline bool operator==(std::string& name) {
		if (name == Name)
			return true;
		return false;
	}
};

struct Segment {
	std::string Name;
	std::vector<Marker> Markers;
	std::vector<Marker> VirtualMarkers;
	double X, Y, Z;
	double QuaternionX, QuaternionY, QuaternionZ, QuaternionW;
	double AngleAxisX, AngleAxisY, AngleAxisZ;
	int streamPosition;
	bool visible;

	inline bool operator==(std::string& name) {
		if (name == Name)
			return true;
		return false;
	}

	Marker* getMarker(const char *name);

};

//class ViconError : std::exception {
//	public
//private:
//	char *exceptiontring[];
//}

class ViconBase {
	public:
		/**
		 *  Constructor sets up connection to vicon server.
		 *  The model file is loaded, and the marker names extracted. The server send the names of available makers
		 *  The identities of the ones we want are stored.
		 *  Calls initialiseMode() after completion to allow subclass to set up stream or Request-Reply
		 */
		ViconBase(const char *ModelFile, const char *SubjectName, const char* viconpc = DEFAULT_IP, int port = DEFAULT_PORT);

		/**
		 * 	Request the current values from the server be stored inside the class
		 *  The values are copied into the vector of segments by updateTransformations
		 */
		virtual int requestValuesUpdate();

		/**
		 *  Adds a virtual marker to a segment
		 *  Returns the number the marker
		 */
		Marker* addVirtualMarker(double X, double Y, double Z, Segment *segment, const char* name="Virtual");

		/**
		 *  Returns the segment with the name given
		 */
		Segment* getSegment(const char *name);

		/**
		 * 	Simply prints the structure of the model to the console.
		 */
		void printStructure() const;

		bool isAlive() { return alive;}

		std::string subjectName;
		std::vector<Segment> Segments;
		float timeStamp;

		/**
		 *  Initialise the connections. Streaming version start a thread, RR version just make a request
		 */
//		virtual int initialiseMode();



		virtual ~ViconBase();

		friend class ViconBaseStream;

	private:
		double *values;
		char **names;
		long numberValues;
		int i, j, k;
		int string_length;
		long command[3];
		int timeStampPosition;
		long _itemsCount;
		int sockfd;
		struct sockaddr_in dest_addr; // will hold the destination address
		bool alive;
		bool ready;

		ViconBase() {};

		/**
		 *  Reads in the model file.
		 *  Returns -1 if an error, 0 if OK.
		 */
		int readModel(const char *ModelFile);

		/**
		 * 	Function to calculate the Euler angles inside the segments and the world position of the virtual model points
		 *  Called by the requestValues method to copy the values from the raw stream into
		 *  the correct places.
		 *  returns -1 if error, 1 if OK.
		 */
		int updateTransformations();

		/**
		 *  Initialise the connection to the vicon machine, storing the names of the markers etc.
		 */
		int initialiseConnection(const char* viconpc , int port);




		/**
		 *  Read the values from the server - whichever mode.
		 */
		int readValues();

		/**
		 *  Apply the rotation stored in the segment to the passed vector
		 */
		void applyQuaternionRotation(Segment& segment, float *vector);

		/**
		 *  Check if the object is ready for value request....
		 */
		inline bool isReady() { return ready;}


};


#endif /* VICONBASE_H_ */
