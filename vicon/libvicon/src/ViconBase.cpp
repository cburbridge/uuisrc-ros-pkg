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

#include "libvicon/ViconBase.h"


Marker* Segment::getMarker(const char *name)  {
	std::vector<Marker>::iterator it;
	std::string markerString(name);
	for (it=Markers.begin();it!=Markers.end();it++) {
		if (*it==markerString) break;
	}
	if (it==Markers.end())
		return NULL;
	else
		return &(*it);
}

ViconBase::ViconBase(const char *ModelFile, const char *SubjectName, const char* viconpc , int port) {
	alive =true;
	ready=false;
	// Read the model
	if (readModel(ModelFile) != 1) {
		std::cerr << "ERROR: Construction of Vicon object failed.\n";
		alive =false;
		return;
//		throw ViconError("Error reading file.");
	}

	// Set up a connection to Vicon
	if (initialiseConnection(viconpc,port) !=1) {
		std::cerr << "ERROR: Construction of Vicon object failed.\n";
		alive =false;
		return;
//		throw
	}

	subjectName.assign(SubjectName);
	timeStampPosition=-1;

	// Look through the available data names and see which ones are desired from the model file
	std::string itemName;
	std::vector<std::string> splitItemName;	 // Split on the brackets
	std::vector<std::string> splitItemName2; // Split on the colon
	std::string bodyName;
	std::string markerSegmentName;
	std::string type;
	for (int i=0; i <numberValues;i++) {
		itemName.assign(names[i]);
		boost::split(splitItemName,itemName,boost::is_any_of("<>"));
		boost::split(splitItemName2,splitItemName[0],boost::is_any_of(":"));
		boost::trim(bodyName=splitItemName2[0]);
		boost::trim(markerSegmentName=splitItemName2[1]);
		boost::trim(type=splitItemName[1]);
//		std::cout << "Body name=" << bodyName << "   Marker = " << markerSegmentName << ";    type="<<type << std::endl;
		if (bodyName == subjectName) {	// This is for the desired body
//			std:: cout <<   "  Good body\n";
			// Check if this is a segment
			Segment *segment = getSegment(markerSegmentName.c_str());
			if (segment!=NULL) {
				if (segment->streamPosition==-1)
					segment->streamPosition=i;
			} else { // Not a segment so find the marker inside the segments
				for (std::vector<Segment>::iterator segment = Segments.begin(); segment != Segments.end(); segment++) {
					Marker *marker = segment->getMarker(markerSegmentName.c_str());
					if ((marker != NULL) && (marker->streamPosition==-1)) {
						marker->streamPosition = i;
						break;
					}
				}
			}
		} else {
			// Maybe a time stamp?
			boost::split(splitItemName,bodyName,boost::is_space());
			if (splitItemName[0].compare("Time") == 0) {
				timeStampPosition = i;
			}
		}
	}

	// Check that all the desired fields are present
	if (timeStampPosition==-1) {
		std::cerr << "ERROR: Can't find timestamp field in the vicon data\n";
		alive =false;
		return;
//		throw
	}
	std::vector<Segment>::iterator it;
	std::vector<Marker>::iterator itM;
	for (it=Segments.begin();it!=Segments.end();it++) {
		if (it->streamPosition == -1) {
			std::cerr << "ERROR: Data coming from vicon does not contain the segement " << it->Name << "\n";
			alive =false;
			return;
//			throw
		}
		for (itM=it->Markers.begin();itM!=it->Markers.end();itM++) {
			if (itM->streamPosition == -1) {
				std::cerr << "ERROR: Data coming from vicon does not contain the marker " << itM->Name << "\n";
				alive =false;
				return;
	//			throw
			}
		}
	}
	std::cout << "ViconBase contruction done....\n";
	ready=true;

}

int ViconBase::initialiseConnection(const char* viconpc , int port) {
	sockfd = socket(PF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		std::cerr << "ERROR: Can't create socket!\n";
		return -1;
	}

	// Set up the socket
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(port);
	dest_addr.sin_addr.s_addr = inet_addr(viconpc);
	memset(dest_addr.sin_zero, '\0', sizeof dest_addr.sin_zero);

	// Connect to server
	if (connect(sockfd, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr)) < 0) {
		std::cerr << "ERROR: Can't connect to Vicon server.\n";
		return -1;
	}

	// Request info from the TARSUS server
	command[0] = 1; // info
	command[1] = 0; // request (1,0)
	if (send(sockfd, command, 8, 0) < 0 ) {	// send 8 bytes
		std::cerr << "ERROR: Problem requesting the info from Vicon!\n";
		return -1;
	}

	// Receive a response
	if (recv(sockfd, command, 8, MSG_WAITALL) < 0) {
		std::cerr << "ERROR: Got no response to info request from Vicon!\n";
		return -1;
	}
	// Check the response is as it should be - 1:1
	if (command[0] != 1 || command[1] != 1) {
		std::cerr << "ERROR: Got a bad response to info request from Vicon!\n";
		return -1;
	}

	// Receive the info
	recv(sockfd, &numberValues, 4, MSG_WAITALL); // receive number of items

	names = new char* [(int)numberValues];
	values = new double[(int)numberValues];

	int string_length;

	for(int i=0; i<numberValues; i++) {
		recv(sockfd, &string_length, 4, MSG_WAITALL);
		names[i]=new char[string_length+1];
		recv(sockfd, names[i], string_length, MSG_WAITALL);
		names[i][string_length] = '\0';
	}
	return 1;
}



int ViconBase::readModel(const char *ModelFile) {
	xmlDocPtr doc = NULL; /* the resulting document tree */
	xmlNodePtr cur,cur2,cur3,cur4;
	int successLevel=1;
try {
	doc = xmlReadFile(ModelFile, NULL, 0);
	if (doc == NULL) {
	   std::cerr << "ERROR: Failed to parse Vicon model file " << ModelFile << std::endl;
	   throw(-1);
	}

	// Find the Segments
	cur = xmlDocGetRootElement(doc);
	cur = cur->xmlChildrenNode;

	while (cur != NULL ) {
		if ((!xmlStrcmp(cur->name, (const xmlChar *)"Skeleton"))){
			break;
		}
		cur = cur->next;
	}

	if (cur == NULL) {
		std::cerr << "ERROR: Failed to find the Skeleton in the model file.\n";
		throw(-1);
	}

	cur = cur->xmlChildrenNode;
	while (cur != NULL) {
		if ((!xmlStrcmp(cur->name, (const xmlChar *)"Segment"))){
			xmlChar *key;
			key = xmlGetProp(cur, (const xmlChar *)"NAME");
			Segment s;
			s.Name.assign((const char*)key);
			s.streamPosition = -1;
			Segments.push_back(s);
			xmlFree(key);

			// Second level segments
			cur2=cur->xmlChildrenNode;
			while (cur2 != NULL) {
				if ((!xmlStrcmp(cur2->name, (const xmlChar *)"Segment"))){
					xmlChar *key;
					key = xmlGetProp(cur2, (const xmlChar *)"NAME");
					Segment s;
					s.Name.assign((const char*)key);
					s.streamPosition = -1;
					Segments.push_back(s);
					xmlFree(key);

					// Third level segments
					cur3=cur2->xmlChildrenNode;
					while (cur3 != NULL) {
						if ((!xmlStrcmp(cur3->name, (const xmlChar *)"Segment"))){
							xmlChar *key;
							key = xmlGetProp(cur3, (const xmlChar *)"NAME");
							Segment s;
							s.Name.assign((const char*)key);
							s.streamPosition = -1;
							Segments.push_back(s);
							xmlFree(key);

							// Fourth level segments
							cur4=cur3->xmlChildrenNode;
							while (cur4 != NULL) {
								if ((!xmlStrcmp(cur4->name, (const xmlChar *)"Segment"))){
									xmlChar *key;
									key = xmlGetProp(cur4, (const xmlChar *)"NAME");
									Segment s;
									s.Name.assign((const char*)key);
									s.streamPosition = -1;
									Segments.push_back(s);
									xmlFree(key);
								}
								cur4 = cur4->next;
							}

						}
						cur3 = cur3->next;
					}

				}
				cur2 = cur2->next;
			}

		}
		cur = cur->next;
	}
	//TODO: Only four levels of segment are considered by parsing this way. 


	// Find the marker names
	cur = xmlDocGetRootElement(doc);
	cur = cur->xmlChildrenNode;

	while (cur != NULL ) {
		if ((!xmlStrcmp(cur->name, (const xmlChar *)"MarkerSet"))){
			break;
		}
		cur = cur->next;
	}
	if (cur == NULL) {
		std::cerr << "ERROR: Failed to find the MarkerSet in the model file.\n";
		throw(-1);
	}

	cur = cur->xmlChildrenNode;
	while (cur != NULL ) {
		if ((!xmlStrcmp(cur->name, (const xmlChar *)"Markers"))){
			break;
		}
		cur = cur->next;
	}
	if (cur == NULL) {
		std::cerr << "ERROR: Failed to find the Markers section of the model file.\n";
		throw(-1);
	}

	cur = cur->xmlChildrenNode;
	while (cur != NULL) {
		if ((!xmlStrcmp(cur->name, (const xmlChar *)"Marker"))){
			xmlChar *pose;
			xmlChar *name;
			xmlChar *segment;
			pose = xmlGetProp(cur, (const xmlChar *)"POSITION");
			name = xmlGetProp(cur, (const xmlChar *)"NAME");
			segment = xmlGetProp(cur, (const xmlChar *)"SEGMENT");

			std::string positionString((const char*)pose);
			std::vector<std::string> valueStrings;
			boost::split(valueStrings,positionString,boost::is_space());
			if (valueStrings.size()!=3) {
				std::cerr << "ERROR: Found a marker in the model with a bad position!";
				throw( -1);
			}

			std::vector<Segment>::iterator it;
			std::string segmentString((const char*)segment);
			for (it=Segments.begin();it!=Segments.end();it++) {
				if (*it==segmentString) break;
			}
//			it = std::find(Segments.begin(),Segments.end(),segmentString);
			if (it == Segments.end()) {
				std::cerr << "ERROR: Can't handle marker in Model file, not sure which segment it goes to!" << std::endl;
				std::cerr << "       Says it wants segment: " << segmentString << std::endl;
				throw(-1);
			}

			Marker m;
			m.streamPosition=-1;
			m.Name.assign((const char*)name);
			m.modelX=atof(valueStrings.at(0).c_str());
			m.modelY=atof(valueStrings.at(1).c_str());
			m.modelZ=atof(valueStrings.at(2).c_str());
			it->Markers.push_back(m);

			xmlFree(pose);
			xmlFree(name);
			xmlFree(segment);


		}

	cur = cur->next;
	}

} catch (int i) {
	if (i<0) {
		std::cerr << "ERROR: Vicon class error reading the config file.\n";
	}
	successLevel=i;
}
//	doc.

	xmlFreeDoc(doc);

	/*
	 * Cleanup function for the XML library.
	 */
	xmlCleanupParser();
	return successLevel;
}

Marker* ViconBase::addVirtualMarker(double X, double Y, double Z, Segment *segment, const char* name) {
	if (segment==NULL) {
		std::cerr << "ERROR: Adding virtual marker to NULL segment!\n";
		return NULL;
	}
	Marker m;
	m.modelX=X;
	m.modelY=Y;
	m.modelZ=Z;
	m.Name.assign(name);
	m.visible=true;
	segment->VirtualMarkers.push_back(m);
	return &(segment->VirtualMarkers.back());
}

Segment* ViconBase::getSegment(const char *name) {
	std::vector<Segment>::iterator it;
	std::string segmentString(name);
	for (it=Segments.begin();it!=Segments.end();it++) {
		if (*it==segmentString) break;
	}
	if (it==Segments.end())
		return NULL;
	else
		return &(*it);
}

void ViconBase::printStructure() const {
	for (unsigned int i=0;i<Segments.size();i++) {
		std::cout << Segments.at(i).Name << std::endl;
		std::cout << "  " << "stream position=" << Segments.at(i).streamPosition << std::endl;
		for (unsigned int j=0;j<Segments.at(i).Markers.size();j++) {
			std::cout << "  |-----" << Segments.at(i).Markers.at(j).Name;
			std::cout << "  (";
			(Segments.at(i).Markers.at(j).visible)? std::cout << "Visible)\n":std::cout << "Ocluded)\n";
			std::cout << "\t" << "stream position=" << Segments.at(i).Markers.at(j).streamPosition << std::endl;
			std::cout << "\t|-----X=" << Segments.at(i).Markers.at(j).modelX <<
							":" << Segments.at(i).Markers.at(j).X << std::endl;
			std::cout << "\t|-----Y=" << Segments.at(i).Markers.at(j).modelY <<
								":" << Segments.at(i).Markers.at(j).Y << std::endl;
			std::cout << "\t|-----Z=" << Segments.at(i).Markers.at(j).modelZ <<
								":" << Segments.at(i).Markers.at(j).Z << std::endl;
		}
		for (unsigned int j=0;j<Segments.at(i).VirtualMarkers.size();j++) {
			std::cout << "  |-----" << Segments.at(i).VirtualMarkers.at(j).Name << "(Virtual)" << std::endl;
			std::cout << "\t|-----X=" << Segments.at(i).VirtualMarkers.at(j).modelX <<
							":" << Segments.at(i).VirtualMarkers.at(j).X << std::endl;
			std::cout << "\t|-----Y=" << Segments.at(i).VirtualMarkers.at(j).modelY <<
							":" << Segments.at(i).VirtualMarkers.at(j).Y << std::endl;
			std::cout << "\t|-----Z=" << Segments.at(i).VirtualMarkers.at(j).modelZ <<
							":" << Segments.at(i).VirtualMarkers.at(j).Z << std::endl;
		}
	}
}


int ViconBase::updateTransformations() {
	// Copy the data from the values member variable into the correct locations
	// Locking of values should be performed outside this function if threading!
	std::vector<Segment>::iterator itSegment;
	std::vector<Marker>::iterator itMarker;
	int segmentVisibleCount=0;
	for (itSegment=Segments.begin();itSegment!=Segments.end();itSegment++) {
		// Copy the segment transforms data
		itSegment->AngleAxisX = values[itSegment->streamPosition];
		itSegment->AngleAxisY = values[itSegment->streamPosition+1];
		itSegment->AngleAxisZ = values[itSegment->streamPosition+2];
		itSegment->X = values[itSegment->streamPosition+3];
		itSegment->Y = values[itSegment->streamPosition+4];
		itSegment->Z = values[itSegment->streamPosition+5];

		// Copy each marker
		segmentVisibleCount=0;
		for (itMarker=itSegment->Markers.begin();itMarker!=itSegment->Markers.end();itMarker++) {
			itMarker->X=values[itMarker->streamPosition];
			itMarker->Y=values[itMarker->streamPosition+1];
			itMarker->Z=values[itMarker->streamPosition+2];
			itMarker->visible = (values[itMarker->streamPosition+3] < 0.5);
			if (itMarker->visible)
				segmentVisibleCount++;
		}
		itSegment->visible=(segmentVisibleCount>4);

	}
	timeStamp = values[timeStampPosition];

	// Calculate the Quaternions from the rotations provided by Vicon for each segment
	// Apply the transformation to virtual markers
	double length, tmp;
	float virtualMarker[3];
	for (itSegment=Segments.begin();itSegment!=Segments.end();itSegment++) {
		length = sqrt(  itSegment->AngleAxisX * itSegment->AngleAxisX +
						itSegment->AngleAxisY * itSegment->AngleAxisY +
						itSegment->AngleAxisZ * itSegment->AngleAxisZ);
		itSegment->QuaternionW = cos( length / 2.0);
		tmp = sin(length / 2.0);
		if (length < 1e-10) {
			itSegment->QuaternionX = itSegment->AngleAxisX;
			itSegment->QuaternionY = itSegment->AngleAxisY;
			itSegment->QuaternionZ = itSegment->AngleAxisZ;
		}
		else {
			itSegment->QuaternionX = itSegment->AngleAxisX * tmp / length;
			itSegment->QuaternionY = itSegment->AngleAxisY * tmp / length;
			itSegment->QuaternionZ = itSegment->AngleAxisZ * tmp / length;
		}

		// Apply it to any virtual markers on body
		for (itMarker=itSegment->VirtualMarkers.begin();itMarker!=itSegment->VirtualMarkers.end();itMarker++) {
			virtualMarker[0]=itMarker->modelX;
			virtualMarker[1]=itMarker->modelY;
			virtualMarker[2]=itMarker->modelZ;
			applyQuaternionRotation(*itSegment,virtualMarker);
			virtualMarker[0]+=itSegment->X;
			virtualMarker[1]+=itSegment->Y;
			virtualMarker[2]+=itSegment->Z;
			itMarker->X=virtualMarker[0];
			itMarker->Y=virtualMarker[1];
			itMarker->Z=virtualMarker[2];
		}
	}


	// TESTING
	// Output the transformed of all model markers and show vicon stated values
//	std::cout << "-----------------------------------------------\n\n";
//	float testVec[3];
//	for (itMarker=Segments[0].Markers.begin();itMarker!=Segments[0].Markers.end();itMarker++) {
//		testVec[0]=itMarker->modelX;
//		testVec[1]=itMarker->modelY;
//		testVec[2]=itMarker->modelZ;
//		applyQuaternionRotation(Segments[0],testVec);
//		testVec[0]+=Segments[0].X;
//		testVec[1]+=Segments[0].Y;
//		testVec[2]+=Segments[0].Z;
//		std::cout << itMarker->X << ", " << itMarker->Y << ", " << itMarker->Z << "    ==   " <<
//			testVec[0] << ", " << testVec[1] << ", " << testVec[2] << "\n";
//	}
	////////

	return 1;	//TODO: Add checking for errors :-/
}

//int ViconBase::initialiseMode() {
//	// Default is to not need initialising....
//	std::cout << "Setting mode...(base)\n";
//	return 1;
//}

int ViconBase::requestValuesUpdate() {
	// Default behaviour is request-reply
	if (!ready) {
		std::cerr << "WARNING: Can't update vicon values as there are not any yet.\n";
		return -1;
	}

	// Request the server to send values...
	command[0] = 2; // data
	command[1] = 0; // request (2,0)
	if (send(sockfd, command, 8, 0) !=8) {
		std::cerr << "ERROR: Can't send request to Vicon server for values.\n";
		return -1;
	}

	// Receive a response
	if (recv(sockfd, command, 8, MSG_WAITALL ) < 0) {
		std::cerr << "ERROR: No response from Vicon server.\n";
		return -1;
	}
	if (command[0] != 2 || command[1] != 1) { 	// data reply check (2,1)
		std::cerr << "ERROR: Vicon to server data reply invalid.\n";
		return -1;
	}

	if (readValues() !=1 ) {
		std::cerr << "ERROR: Problem reading values from server.\n";
		return -1;
	}

	if (updateTransformations() != 1) {
		std::cerr << "ERROR: Error updating transformations.\n";
		return -1;
	}
	return 1;
}

int ViconBase::readValues() {
	static unsigned int quant;
	// Receive item count
	if (recv(sockfd, &_itemsCount, 4, MSG_WAITALL) < 0) {
		std::cerr << "ERROR: Number of values not received from Vicon\n";
		return -1;
	}
	// If the number of items != previous, maybe problem....

	// Receive the values
	// receive data
	quant=0;
	while (quant!=sizeof(double)*_itemsCount){
		quant += recv(sockfd,values+(quant/sizeof(double)),(sizeof(double)*_itemsCount)-quant,0);
	}

	return 1;
}

void ViconBase::applyQuaternionRotation(Segment& segment, float *vector) {
	//TODO: Make more efficient and nicer!!!
#define a segment.QuaternionW
#define b segment.QuaternionX
#define c segment.QuaternionY
#define d segment.QuaternionZ
#define v1 vector[0]
#define v2 vector[1]
#define v3 vector[2]

	float t2 =   a*b;
	float t3 =   a*c;
	float t4 =   a*d;
	float t5 =  -b*b;
	float t6 =   b*c;
	float t7 =   b*d;
	float t8 =  -c*c;
	float t9 =   c*d;
	float t10 = -d*d;
	float v1new = 2.0 * ( (t8 + t10)*v1 + (t6 -  t4)*v2 + (t3 + t7)*v3 ) + v1;
	float v2new = 2.0 * ( (t4 +  t6)*v1 + (t5 + t10)*v2 + (t9 - t2)*v3 ) + v2;
	float v3new = 2.0 * ( (t7 -  t3)*v1 + (t2 +  t9)*v2 + (t5 + t8)*v3 ) + v3;
	vector[0]=v1new;
	vector[1]=v2new;
	vector[2]=v3new;
}


ViconBase::~ViconBase() {
	shutdown(sockfd,SHUT_RDWR);
	close(sockfd);
	for (int i=0; i<numberValues; i++) {
		delete names[i];
	}
	delete names;
	delete values;
}
