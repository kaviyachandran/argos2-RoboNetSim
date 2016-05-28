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
 * @file common/control_interface/e-puck/ci_epuck_light_sensor.h
 *
 * @brief This file provides the implementation of the epuck light sensor.
 *
 * This file provides the definition of the epuck light sensor.
 * The sensors are evenly spaced on a ring around the base of the ropuck.
 * Therefore, they do not turn with the turret. The readings are in the
 * following order (seeing the ropuck from TOP, battery socket is the BACK):
 *
 *              front
 *
 *              0   7
 *     l     1         6    r
 *     e                    i
 *     f    2           5   g
 *     t                    h
 *             3     4      t
 *
 *              back
 *
 *
 * @author Lorenzo Garattoni <lorenzo.garattoni3@studio.unibo.it>
 * @author Matteo Amaducci <matteo.amaducci2@studio.unibo.it>
 */

#ifndef CCI_EPUCK_ACCELEROMETER_SENSOR_H
#define CCI_EPUCK_ACCELEROMETER_SENSOR_H

/* To avoid dependency problems when including */
namespace argos {
   class CCI_EPuckAccelerometerSensor;
}

#include <argos2/common/control_interface/ci_sensor.h>

namespace argos {

   class CCI_EPuckAccelerometerSensor : virtual public CCI_Sensor {

   public:

      struct SReading {
         Real X;
         Real Y;
         Real Z;

         SReading() : X(0.0f), Y(0.0f), Z(0.0f) {}

         SReading(Real un_x, Real un_y, Real un_z) :
            X(un_x),
            Y(un_y),
            Z(un_z) {}

      };

      virtual ~CCI_EPuckAccelerometerSensor() {
      }

      inline virtual const SReading& GetReading() const {
         return m_sReading;
      }

   protected:

      SReading m_sReading;

   };

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_EPuckAccelerometerSensor::SReading& s_reading);

}

#endif
