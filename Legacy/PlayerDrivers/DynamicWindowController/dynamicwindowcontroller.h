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
#ifndef DYNAMICWINDOWCONTROLLER_H
#define DYNAMICWINDOWCONTROLLER_H
#include <libplayercore/playercore.h>
#include <libplayercore/error.h>

#include "dynamicwindow.h"
#define 	PLAYER_LASER_MAX_SAMPLES   1024
/**
	@author Lorenzo Riano <lorenzo.riano@gmail.com>
 */
class DynamicWindowController : public Driver
{
	public:
		DynamicWindowController(ConfigFile* cf, int section);

		virtual ~DynamicWindowController();
		virtual int Setup ();
		virtual int Shutdown ();
		int ProcessMessage (QueuePointer &resp_queue, player_msghdr * hdr, void * data);
		virtual void Main();
    
	protected:
		
		int setupPosition();
		int setupLaser();
		void processOdom(player_msghdr_t* hdr, player_position2d_data_t &data);
		void processLaser(player_laser_data_t &data);
		void processCommand(player_position2d_cmd_pos_t &cmd);
		void putCommand( double cmd_speed, double cmd_turnrate );
		
		Device* position_device;
		    player_devaddr_t position_addr;
		Device* laser_device;
		player_devaddr_t laser_addr;
		DynamicWindow dw;
		
		float m_x_obstacle[PLAYER_LASER_MAX_SAMPLES];
		float m_y_obstacle[PLAYER_LASER_MAX_SAMPLES];
		bool m_active_goal;
		double m_x_goal;
		double m_y_goal;
		double m_x_robot;
		double m_y_robot;
		double m_theta_robot;
		double m_v_robot;
		double m_w_robot;
		double m_tcmax;
		double m_theta_goal;
		double m_dist_done;
		double m_theta_done;
		double m_vmax;
		
		player_position2d_cmd_vel_t m_own_cmd;

};

#endif
