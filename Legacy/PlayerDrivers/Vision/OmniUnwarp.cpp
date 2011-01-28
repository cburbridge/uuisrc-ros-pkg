/*
 * OmniUnwarp.cpp
 *
 *  Created on: 17-Mar-2009
 *      Author: chris
 */
#include "OmniUnwarp.h"

char*** createUnwarpTable(IplImage *image, int number_rows, int first_r, int uwidth, int cX, int cY) {
	// Initialiser the unwarper look up table
	int i,j;
	int imX,imY;
	float r;
	float theta;
	static char *nullspace = new char;
	*nullspace='c';
	char ***unwarped = new char**[number_rows];
	for (i=0;i<number_rows; i++) {
		unwarped[i]=new char*[uwidth];
	}
	for (i=0;i<number_rows; i++) {
		r=first_r+i; // (i/(float)number_rows)*(last_r-first_r) + first_r;
		for (j=0;j<uwidth;j++) {
			theta=j/(float)uwidth * 2*M_PI;
			imX=(int) round(cX+r*sin(theta));
			imY=(int) round(cY+r*cos(theta));
			if ((imY<0) || (imY>image->height) || (imX<0) || (imX>image->width)) {
				unwarped[i][j]= nullspace;
			} else {
				unwarped[i][j]=  &((image->imageData + imY*image->widthStep)[imX]);
			}
		}
	}
	return unwarped;
}

void deleteUnwarpTable(char ***table, int width, int height) {
	for (int i=0;i<height; i++) {
		delete table[i];
	}
	delete **table;
}


char*** createPlaneImageLookup(IplImage *image,float scale, double k1, double k2, double k3 ) {
	int i,j;
	int x,y,xd,yd;
	double mult,xu,yu;
	static char *nullspace = new char;
	*nullspace='c';
	// Create a square lookup image the size of the height of the input
	char ***unwarped = new char**[image->height];
	for (i=0;i<image->height; i++) {
		unwarped[i]=new char*[image->height];
	}
	// Default each location to being empty
	for (i=0;i<image->height;i++) {
		for (j=0;j<image->height;j++) {
			unwarped[i][j]=nullspace;
		}
	}
	// Map the image pixels to the location on the image
	// The mapping is a mapping from distorted pixel to undistorted pixel. The function used is equation (6) in "Straight lines have to be straight"
	// Three coefficients are used - k1, k2 and k3
	for (x=0;x<image->width;x++) {
		for (y=0;y<image->height;y++) {
			xd=(image->width/2)-x;
			yd=(image->height/2)-y;
			mult=1+k1*((xd*xd + yd*yd))+k2*((xd*xd + yd*yd))*((xd*xd + yd*yd)) +	k3*((xd*xd + yd*yd))*((xd*xd + yd*yd))*((xd*xd + yd*yd));
			xu=round(((mult*xd)/scale)+(image->height/2));
			yu=round(((mult*yd)/scale)+(image->height/2));

			if ( (xu>=0) && (xu<image->height) && (yu>=0) && (yu<image->height) ) {
				unwarped[(int)round(yu)][(int)round(xu)]=(image->imageData + y*image->widthStep +x);
			}
		}
	}
	// Clean up the pixels that have no associated value
	for (x=0;x<image->height;x++) {
		for (y=0;y<image->height;y++) {
			if (unwarped[y][x]==nullspace) {
				// bad point...

				// direction flag says which direction moving in :
				//  0: positive x
				//  1: positive y
				//  2: negative x
				//  3: negative y
				int direction=0;

				// width size is the current side width
				int widthsize=1;

				// width done = the number of pixels of the side checked so far
				int widthdone=0;

				int current_x=x,current_y=y;	// current co-ordinates
				bool searching=true;
				while (searching) {
					switch(direction) {
						case 0:
							current_x++;
							break;
						case 1:
							current_y++;
							break;
						case 2:
							current_x--;
							break;
						case 3:
							current_y--;
							break;
					}
					widthdone++;
					if (widthdone==widthsize) {
						widthdone=0;
						if ((direction==1) || (direction==3))
							widthsize++;

						direction++;
						if (direction>3) direction=0;
					}
					if ((current_x>0) && (current_x<image->height) && (current_y>0) && (current_y<image->height) )
						if (unwarped[current_y][current_x]!=nullspace) {
							searching=false;
						}
				}
				unwarped[y][x] = unwarped[current_y][current_x];

			}
		}
	}
	return unwarped;
}

void deletePlaneImageLookup(char ***table, int width, int height){
	for (int i=0;i<height; i++) {
		delete table[i];
	}
	delete **table;
}


char *** createPolarMappingLookup(char ***input, int cx, int cy, int dest_width, int dest_height) {
	// Initialiser the unwarper look up table
	int i,j;
	int imX,imY;
	float r;
	float theta;
//	static char *nullspace = new char;
//	*nullspace=0;
	char ***unwarped = new char**[dest_height];
	for (i=0;i<dest_height; i++) {
		unwarped[i]=new char*[dest_width];
	}
	for (i=0;i<dest_height; i++) {
		r=i; // (i/(float)number_rows)*(last_r-first_r) + first_r;
		for (j=0;j<dest_width;j++) {
			theta=j/(float)dest_width * 2*M_PI;
			imX=(int) round(cx+r*sin(theta));
			imY=(int) round(cy+r*cos(theta));
//			if ((imY<0) || (imY>image->height) || (imX<0) || (imX>image->width)) {
//				unwarped[i][j]= nullspace;
//			} else {
				unwarped[i][j]=  input[imY][imX];//&((image->imageData + imY*image->widthStep)[imX]);
//			}
		}
	}
	return unwarped;
}

void deletePolarMappingLookup(char ***table, int width, int height){
	for (int i=0;i<height; i++) {
		delete table[i];
	}
	delete **table;
}

