g++ -Wall -pedantic ViconPosition.C ../../../vicon/libvicon/src/ViconBase.cpp ../../../vicon/libvicon/src/ViconBaseStream.cpp  `pkg-config  --cflags --libs libxml-2.0 ` -I/usr/include/player-2.1  -lplayerdrivers -lplayercore -lpthread -lplayererror   -fPIC -shared -o libviconposition.so

#
