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
#include "dynamicwindow.h"
#include <cmath>
#include <iostream>
#include <assert.h>
#include <sys/time.h>
#include <ctime>
#include <cstdlib>

DynamicWindow::DynamicWindow()
{
	srand(time(NULL));
}


DynamicWindow::~DynamicWindow()
{
}


/*!
    \fn DynamicWindow::circle_circle_intersection(double x0, double y0, double r0,double x1, double y1, double r1,double *xi, double *yi,double& xi_prime, double& yi_prime) const
 */
int DynamicWindow::circle_circle_intersection(double x0, double y0, double r0,double x1, double y1, double r1,double& xi, double&  yi,double& xi_prime, double& yi_prime) const
{
	double a, dx, dy, d, h, rx, ry;
	double x2, y2;

  	/* dx and dy are the vertical and horizontal distances between
	* the circle centers.
	*/
	dx = x1 - x0;
	dy = y1 - y0;

	/* Determine the straight-line distance between the centers. */
	d = hypot(dx,dy); // Suggested by Keith Briggs

	/* Check for solvability. */
	if (d > (r0 + r1))
	{
		/* no solution. circles do not intersect. */
		return 0;
	}
	if (d < fabs(r0 - r1))
	{
		/* no solution. one circle is contained in the other */
		return 0;
	}

  	/* 'point 2' is the point where the line through the circle
	* intersection points crosses the line between the circle
	* centers.  
	*/

	/* Determine the distance from point 0 to point 2. */
	a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

	/* Determine the coordinates of point 2. */
	x2 = x0 + (dx * a/d);
	y2 = y0 + (dy * a/d);

  	/* Determine the distance from point 2 to either of the
	* intersection points.
	*/
	h = sqrt((r0*r0) - (a*a));

  	/* Now determine the offsets of the intersection points from
	* point 2.
	*/
	rx = -dy * (h/d);
	ry = dx * (h/d);

	/* Determine the absolute intersection points. */
	xi = x2 + rx;
	xi_prime = x2 - rx;
	yi = y2 + ry;
	yi_prime = y2 - ry;

	return 1;
}


/*!
    \fn DynamicWindow::circle_xaxis_intersection(double x,double y,double r,double& xi,double& xi_prime,double& yi,double& yi_prime) const
 */
int DynamicWindow::circle_xaxis_intersection(double x,double y,double r,double& xi,double& xi_prime,double& yi,double& yi_prime) const
{
	if ( (r*r) < (y*y)) {
		return 0;
	}
	else {
		xi = x + sqrt(r*r - y*y);
		xi_prime = x - sqrt(r*r - y*y);
		yi = 0;
		yi_prime = 0;
		return 1;
	}
}


/*!
    \fn DynamicWindow::circle_circle_dist(double x0,double y0,double r0,double x1,double y1,double r1) const
 */
double DynamicWindow::circle_circle_dist(double x0,double y0,double r0,double x1,double y1,double r1) const
{
	double dx = x0 - x1;
	double dy = y0 - y1;
	double d = hypot(dy,dx);
	
	if (d > (r0 + r1))
		return d - (r0 + r1);
	else if (d < fabs(r0 - r1))
		return fabs(r0 - r1) - d;
	else
		return 0;
}

/*!
    \fn DynamicWindow::curvature_dist(v,w,x_obs,y_obs,r_obs) const
 */
double DynamicWindow::curvature_dist(double v,double w,double x_obs,double y_obs,double r_obs) const
{
	double xi, xi_prime, yi, yi_prime;
	double r;
	double y_c;
	double x_c = 0;
	
	
	if (w != 0) {
		r = fabs(v/w);
		y_c = v/w;
		if (!circle_circle_intersection(x_c,y_c,r,x_obs,y_obs,r_obs,xi,yi,xi_prime,yi_prime))
			return 1;
	}
	else {
		if (!circle_xaxis_intersection(x_obs,y_obs,r_obs,xi,yi,xi_prime,yi_prime))
			return 1;
	}
	
	double theta_p1 = atan2(yi-y_c,xi);
	double theta1;
	if (w >= 0)
		theta1 = M_PI/2.0 + theta_p1;
	else if (w < 0)
		theta1 = M_PI/2.0 - theta_p1;
	double dist1 = r*theta1;
	
	double theta_p2 = atan2(yi_prime-y_c,xi_prime);
	double theta2;
	if (w >= 0)
		theta2 = M_PI/2.0 + theta_p2;
	else if (w < 0)
		theta2 = M_PI/2.0 - theta_p2;
	double dist2 = r*theta2;
	
	if (dist1 <= dist2)
		return dist1/m_laser_max;
	else
		return dist2/m_laser_max;
}


/*!
    \fn DynamicWindow::target_heading_score(double v, double w, double theta_goal)
 */
double DynamicWindow::target_heading_score(double v, double w, double theta_goal) const
{
	double delta = 2.0;
	double score = (1.0 - fabs(theta_goal - w*1.0)/M_PI);
	double val = 1.0 + delta*pow(fabs(theta_goal/M_PI),0.5);
	return val*score;
// 	return (1 - fabs(theta_goal - w*2.0)/M_PI);
	
}

/*!
    \fn DynamicWindow::speed_score(double v, double w) const
 */
double DynamicWindow::speed_score(double v, double w) const
{
	return v/m_vmax;
}

void DynamicWindow::motion_model_sampling_time(double maxtime,double currv, double currw,double theta_goal,float* x_obs,float* y_obs, double& v_found, double& w_found) const
{
    	if (m_vmax == 0) {
	    v_found = 0;
	    w_found = theta_goal;
	    return;
	}
	else if (m_vmax < 0) {
	    v_found = m_vmax;
	    w_found = theta_goal;	    
	    std::cout<<"BACK!!\n";
	    return;
	}
	
	double minv,minw,maxv,maxw;
		
	compute_window(currv,currw,minv,maxv,minw,maxw);
	

	
	
	assert(maxv >= minv);
	assert(maxw >= minw);
	double f_found = 0;
	
	double v = 0;
	double w = 0;
	unsigned int samples=0;
	
	struct timeval past,present;
	gettimeofday(&past,NULL);
	gettimeofday(&present,NULL);
	double elapsed = present.tv_sec -past.tv_sec + double(present.tv_usec - past.tv_usec)/1.0e06;
	while(elapsed < maxtime) {
		gettimeofday(&present,NULL);
		elapsed = present.tv_sec -past.tv_sec + double(present.tv_usec - past.tv_usec)/1.0e06;
		samples++;
		
		v = randab(minv,maxv);
		w = randab(minw,maxw);
		double dist_score = 1.0;
		if (v != 0) {
			for (unsigned int o=0; o<m_laser_count; o++) {
				if ( (sqrt(x_obs[o]*x_obs[o] + y_obs[o]*y_obs[o]) >= m_laser_max) ) 
					continue;
				if (x_obs[o]*x_obs[o] + y_obs[o]*y_obs[o] <= m_radius*m_radius){
					continue;
				}
				double dist = curvature_dist(v,w,x_obs[o],y_obs[o],m_radius);
				if (dist <=0) {
					dist = 1.0;
				}
				if (dist < dist_score) {
					dist_score = dist;
				}
			}
		}
		if ( (v>0.5*sqrt(2.0*dist_score*m_laser_max*m_vpbrake)) || (w>0.5*sqrt(2.0*dist_score*m_laser_max*m_wpbrake))) {
			//std::cerr<<"Discarding v:"<<v<<"\tw:"<<w<<"\n";
			continue;
			;
		}
		else {
			double vel_score = speed_score(v,w);
			double h_score = target_heading_score(v,w,theta_goal);
			double tmp_val = m_alpha*h_score + m_beta*dist_score + m_gamma*vel_score;;
			if ( tmp_val > f_found) {
				f_found = tmp_val;
				v_found = v;
				w_found = w;
			}
		}		
	}
// 	std::cout<<"Samples: "<<samples<<"\n";
	if (f_found == 0) {
		std::cerr<<"Warning, no value found!\n";
		v_found = 0;
		w_found = 0;
	}
	
//  	std::cout<<"Vmin Vmax Wmin Wmax: "<<minv<<" "<<maxv<<" "<<DynamicWindow::rad2deg(minw)<<" "<<DynamicWindow::rad2deg(maxw)<<"\n";
// 	std::cout<<"V W "<<v_found<<" "<<w_found<<"\n";
}

/*!
    \fn DynamicWindow::compute_window(double v_curr, double w_curr,double& v_min,double& v_max,double& w_min,double& wmax) const
 */
void DynamicWindow::compute_window(double v_curr, double w_curr,double& minv,double& maxv,double& minw,double& maxw) const
{
	
	if (v_curr < 0)
		v_curr = 0;
	minv = v_curr - m_vpmax*m_time_c;
	if (minv < 0)
		minv = 0;
	maxv = v_curr + m_vpmax*m_time_c;
	if (maxv > m_vmax)
		maxv = m_vmax;
	
	minw = w_curr - m_wpmax*m_time_c;
	if (minw < m_wmin)
		minw = m_wmin;
	maxw = w_curr + m_wpmax*m_time_c;
	if (maxw > m_wmax)
		maxw = m_wmax;
	if (maxw < m_wmin)
		maxw = m_wmin;
		
	if (minv > maxv) {
		minv = 0;
		maxv = 0;
		minw = m_wmin;
		maxw = m_wmax;
	}
	if (minw > maxw) {
		minv = 0;
		maxv = 0;
		minw = m_wmin;
		maxw = m_wmax;
	}
	
	minv = 0;
// 	std::cout<<"Vmin Vmax Wmin Wmax: "<<minv<<" "<<maxv<<" "<<DynamicWindow::rad2deg(minw)<<" "<<DynamicWindow::rad2deg(maxw)<<"\n";
}

void DynamicWindow::setAlpha ( double val )
{
	m_alpha = val;
}

void DynamicWindow::setBeta ( double val )
{
	m_beta = val;
}


void DynamicWindow::setGamma ( double val )
{
	m_gamma = val;
}


void DynamicWindow::setLaser_count ( unsigned int val )
{
	m_laser_count = val;
}

void DynamicWindow::setLaser_max ( double val )
{
	m_laser_max = val;
}

void DynamicWindow::setRadius ( double val )
{
	m_radius = val;
}


void DynamicWindow::setTime_c ( double val )
{
	m_time_c = val;
}


void DynamicWindow::setV_res ( double val )
{
	m_v_res = val;
}


void DynamicWindow::setVmax ( double val )
{
	m_vmax = val;
}


void DynamicWindow::setVpmax ( double val )
{
	m_vpmax = val;
}


void DynamicWindow::setW_res ( double val )
{
	m_w_res = val;
}


void DynamicWindow::setWmax ( double val )
{
	m_wmax = val;
}


void DynamicWindow::setWmin ( double val )
{
	m_wmin = val;
}


void DynamicWindow::setWpmax ( double val )
{
	m_wpmax = val;
}


/*!
    \fn DynamicWindow::rad2deg(double val)
 */
double DynamicWindow::rad2deg(double val)
{
	return val*180.0/M_PI;
}


/*!
    \fn DynamicWindow::deg2rad(double val)
 */
double DynamicWindow::deg2rad(double val)
{
	return val*M_PI/180.0;
}

void DynamicWindow::setVpbrake ( double val )
{
	m_vpbrake = val;
}


void DynamicWindow::setWpbrake ( double val )
{
	m_wpbrake = val;
}


/*!
    \fn DynamicWindow::fix_angle(double)
 */
void DynamicWindow::fix_angle(double& angle)
{
	angle = rad2deg(angle);
	if (angle >= 360)
		angle = angle - 360.0 * (double)((int)angle / 360);
	if (angle < -360)
		angle = angle + 360.0 * (double)((int)angle / -360);
	if (angle <= -180)
		angle = + 180.0 + (angle + 180.0);
	if (angle > 180)
		angle = - 180.0 + (angle - 180.0);
	angle = deg2rad(angle);
}


/*!
\fn DynamicWindow::rand(double a, double b)
 */
double DynamicWindow::randab(double min, double max)
{
	double r = double(rand())/double(RAND_MAX);
	return (max-min)*r + min;
	
}


void DynamicWindow::setGoal_x ( double val )
{
	m_goal_x = val;
}


void DynamicWindow::setGoal_y ( double val )
{
	m_goal_y = val;
}


void DynamicWindow::setDelta ( double val )
{
	m_delta = val;
}



