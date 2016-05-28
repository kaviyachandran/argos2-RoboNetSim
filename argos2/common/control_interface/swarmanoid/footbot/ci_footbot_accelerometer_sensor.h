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
 * @file <common/control_interface/footbot/ci_footbot_gyroscopic_sensor.h>
 *
 * @brief This file provides the common interface definition of the footbot accelerometer.
 *
 * @author Nithin MATHEWS - <nmathews@ulb.ac.be>
 */

#ifndef CCI_FOOTBOT_ACCELEROMETER_SENSOR_H
#define CCI_FOOTBOT_ACCELEROMETER_SENSOR_H

namespace argos {
   class CCI_FootBotAccelerometerSensor;
}

#include <argos2/common/control_interface/ci_sensor.h>
#include <argos2/common/utility/math/vector2.h>

namespace argos {

   class CCI_FootBotAccelerometerSensor : public CCI_Sensor {

   public:

      struct SReading {
         SInt16 AxisX;
         SInt16 AxisY;
         SInt16 AxisZ;

         SReading(SInt16 n_value_x_axis, SInt16 n_value_y_axis, SInt16  n_value_z_axis) :
        	 AxisX(n_value_x_axis),
        	 AxisY(n_value_y_axis),
        	 AxisZ(n_value_z_axis){
         }
      };


      CCI_FootBotAccelerometerSensor() :
         m_sReading(0,0,0) {
      }

      virtual ~CCI_FootBotAccelerometerSensor() {
      }

      inline virtual const SReading& GetReading() const {
         return m_sReading;
      }

   protected:

      SReading m_sReading;

   };

}

#endif
