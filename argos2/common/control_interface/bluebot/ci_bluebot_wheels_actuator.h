/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**
 * @file common/control_interface/bluebot/ci_bluebot_wheels_actuator.h
 *
 * @brief This file provides the definition of the bluebot wheels actuator.
 *
 * @author Ali Emre Turgut - <ali.turgut@gmail.com>
 */

#ifndef CCI_BLUEBOT_WHEELS_ACTUATOR_H
#define CCI_BLUEBOT_WHEELS_ACTUATOR_H

/* To avoid dependency problems when including */
namespace argos {
    class CCI_BluebotWheelsActuator;
}

#include <argos2/common/control_interface/ci_actuator.h>

namespace argos {

    class CCI_BluebotWheelsActuator : virtual public CCI_Actuator {

    public:

        virtual void SetLinearVelocity(Real f_left_velocity,
                                       Real f_right_velocity) = 0;

    protected:

        Real m_fCurrentVelocity[2];
    };

}

#endif
