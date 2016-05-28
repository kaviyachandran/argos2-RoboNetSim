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
 * @file common/control_interface/e-puck/ci_epuck_micro_sensor.h
 *
 * @brief This file provides the implementation of the epuck micro sensor.
 *
 * This file provides the definition of the epuck micro sensor.
 * The sensors are evenly spaced on the left, right and back to the base of the epuck.
 * The micros are placed as follow:
 *
 *              front
 *   l                  		r
 *   e   MIC_L        MIC_R	i
 *   f           .            g
 *   t                    		h
 *             MIC_B     		t
 *
 *              back
 *
 *
 * @author Lorenzo Garattoni <lorenzo.garattoni3@studio.unibo.it>
 * @author Matteo Amaducci <matteo.amaducci2@studio.unibo.it>
 */

#include "ci_epuck_micro_sensor.h"

namespace argos {

/****************************************/
/****************************************/

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_EPuckMicroSensor::SReading& s_reading) {
      c_os << "SReading("
           << s_reading.Right
           << ","
           << s_reading.Left
			  << ","
           << s_reading.Back
           << ")";
      return c_os;
   }

   /****************************************/
   /****************************************/
}
