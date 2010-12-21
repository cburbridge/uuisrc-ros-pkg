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
#ifndef DYNAMICWINDOW_H
#define DYNAMICWINDOW_H

#include <libplayerc++/playerc++.h>

/**
	@author Lorenzo Riano <lorenzo.riano@gmail.com>
 */
class DynamicWindow{
	public:
		DynamicWindow();
		~DynamicWindow();
		void motion_model_sampling_time(double maxtime,double currv, double currw,double theta_goal,float* ranges,float* bearings, double& v_found, double& w_found) const;
		void setLaser_count ( unsigned int val );
		void setGamma ( double val );
		void setBeta ( double val );
		void setAlpha ( double val );
		void setWpmax ( double val );
		void setWmin ( double val );	
		void setWmax ( double val );
		void setW_res ( double val );
		void setVpmax ( double val );
		void setVmax ( double val );
		void setV_res ( double val );
		void setTime_c ( double val );
		void setRadius ( double val );
		void setLaser_max ( double val );		
		static double rad2deg(double val);
		static double deg2rad(double val);
		void setVpbrake ( double val );
		void setWpbrake ( double val );
		static void fix_angle(double& angle);
    		static double randab(double a, double b);
		void setGoal_x ( double val );
		void setGoal_y ( double val );

	void setDelta ( double val );
	
	
	

	protected:
		int circle_circle_intersection(double x0, double y0, double r0,double x1, double y1, double r1,double& xi, double& yi,double& xi_prime, double& yi_prime) const;
		int circle_xaxis_intersection(double x,double y,double r,double& xi,double& xi_prime,double& yi,double& yi_prime) const;
		double target_heading_score(double v, double w, double theta_goal) const;
		double speed_score(double v, double w) const;
		void compute_window(double v_curr, double w_curr,double& v_min,double& v_max,double& w_min,double& w_max) const;
    		double curvature_dist(double v,double w,double x_obs,double y_obs,double r_obs) const;
    double circle_circle_dist(double x0,double y0,double r0,double x1,double y1,double r1) const;


	protected:
		double m_time_c;
		double m_laser_max;
		double m_vmax;
		double m_wmax;
		double m_wmin;
		double m_v_res;
		double m_w_res;
		double m_vpmax;
		double m_wpmax;
		unsigned int m_laser_count;
		double m_alpha, m_beta, m_gamma, m_delta;
		double m_radius;
		double m_vpbrake;
		double m_wpbrake;
		double m_goal_x;
		double m_goal_y;

};

#endif
