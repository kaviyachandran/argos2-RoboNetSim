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
 *              MIC_B     		t
 *
 *              back
 *
 *
 * @author Lorenzo Garattoni <lorenzo.garattoni3@studio.unibo.it>
 * @author Matteo Amaducci <matteo.amaducci2@studio.unibo.it>
 */

#ifndef CCI_EPUCK_MICRO_SENSOR_H
#define CCI_EPUCK_MICRO_SENSOR_H

/* To avoid dependency problems when including */
namespace argos {
   class CCI_EPuckMicroSensor;
}

#include <argos2/common/control_interface/ci_sensor.h>

namespace argos {

   class CCI_EPuckMicroSensor : virtual public CCI_Sensor {

   public:

      struct SReading {
         Real Right;
         Real Left;
         Real Back;

         SReading() : Right(0), Left(0), Back(0) {}

         SReading( Real un_right, Real un_left, Real un_back) :
            Right(un_right),
            Left(un_left),
            Back(un_back) {}

      };

      virtual ~CCI_EPuckMicroSensor() {
      }

		/** Returns the last micros reading (all 3 sensors) **/
      inline virtual const SReading& GetReading() const {
         return m_sReading;
      }

   protected:

      SReading m_sReading;

   };

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_EPuckMicroSensor::SReading& s_reading);

}

#endif
