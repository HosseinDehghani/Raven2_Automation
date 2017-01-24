/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington BioRobotics Laboratory
 *
 * This file is part of Raven 2 Control.
 *
 * Raven 2 Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */


/**\file mapping.cpp
 * \brief contains functions involving mapping from master to slave
 * \author Mitch Lum
 * \author BioRobotics Lab
 * \date July 25, 2006
 */

#include "mapping.h"
#include "log.h"
#include <iostream>

#include <typeinfo>
const int USE_ITP = 1;

/** \fn void fromITP(struct position *delpos, btQuaternion &delrot, int armserial)
 * \brief Transform a position increment and an orientation increment from ITP coordinate frame into local robot coordinate frame.
 *        Do this using inv(R)*C*R : R= transform, C= increment
 * \param delpos - a pointer points to a position struct
 * \param delrot - a reference of a btQuanternion class
 * \param armserial - an integer number of of mechanisam id
 * \question why post multiply with R inverse?
*/
void fromITP(struct position *delpos, btQuaternion &delrot, int armserial)
{
ROS_ERROR("*********mapping_ delpos->x  verybefore: %d", delpos->x);
//ROS_ERROR("*********mapping_ delrot type: %s", typeid(delrot).name());
//ROS_ERROR("*********mapping_ delrot1 before: %f", delrot[0]);
//ROS_ERROR("*********mapping_ delrot2 before: %f", delrot[1]);
//ROS_ERROR("*********mapping_ delrot3 before: %f", delrot[2]);
//ROS_ERROR("*********mapping_ delrot4 before: %f", delrot[3]);
    const btTransform ITP2Gold ( btMatrix3x3 (0,0,-1,  -1,0,0,  0,1,0), btVector3 (0,0,0) );
    const btTransform ITP2Green( btMatrix3x3 (0,0,-1,  1,0,0,  0,-1,0), btVector3 (0,0,0) );
    btTransform incr (delrot, btVector3(delpos->x, delpos->y, delpos->z));
//ROS_ERROR("*********mapping_ incrGOLD: %f", ITP2Gold.getOrigin()[0]);
//ROS_ERROR("*********mapping_ incrGREEN: %f", ITP2Green.getOrigin()[0]);
//ROS_ERROR("*********mapping_ incr0: %f", incr.getOrigin()[0]);
    if (armserial == GOLD_ARM_SERIAL)
    {
        incr = ITP2Gold  * incr * ITP2Gold.inverse();
//ROS_ERROR("*********mapping_ incr0GOLDinv: %f", ITP2Gold.inverse().getOrigin()[0]);
//ROS_ERROR("*********mapping_ incr0GOLD: %f", incr.getOrigin()[0]);
    }
    else
    {
        incr = ITP2Green * incr * ITP2Green.inverse();
//_ERROR("*********mapping_ incr0GREENinv: %f", ITP2Green.inverse().getOrigin()[0]);
//ROS_ERROR("*********mapping_ incr0GREEN: %f", incr.getOrigin()[0]);
    }
//ROS_ERROR("*********mapping_ incr1: %f", incr.getOrigin()[0]);
    delrot = incr.getRotation();
//ROS_ERROR("*********mapping_ delrot1 after: %f", delrot[0]);
//ROS_ERROR("*********mapping_ delrot2 after: %f", delrot[1]);
//ROS_ERROR("*********mapping_ delrot3 after: %f", delrot[2]);
//ROS_ERROR("*********mapping_ delrot4 after: %f", delrot[3]);
//ROS_ERROR("*********mapping_ type: %s", typeid(incr.getOrigin()[0]).name());
ROS_ERROR("*********mapping_ delpos->x before: %d", delpos->x);
    delpos->x = (int)(incr.getOrigin()[0]);
    delpos->y = (int)(incr.getOrigin()[1]);
    delpos->z = (int)(incr.getOrigin()[2]);
ROS_ERROR("*********mapping_ delpos->x very before: %d", delpos->x);



//ROS_ERROR("*********mapping_ type: %s", typeid(delpos->x).name());
ROS_ERROR("*********mapping_ delpos->x after: %d", delpos->x);
ROS_ERROR("*********mapping_ incr2: %f", incr.getOrigin()[0]);
}
