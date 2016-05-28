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
 * @brief This file provides the common interface definition of the footbot gyroscopes.
 *
 * The current implementation returns only the difference in rotation along the Z axis
 * with respect to the previous step.
 *
 * WARNING:
 *
 * AT THE CURRENT STATE ONLY THE ROTATION ALONG THE Z AXIS IS SUPPORTED.
 *
 *
 * @author Giovanni Pini - <gpini@ulb.ac.be>
 */

#ifndef CCI_FOOTBOT_GYROSCOPIC_SENSOR_H
#define CCI_FOOTBOT_GYROSCOPIC_SENSOR_H

namespace argos {
   class CCI_FootBotGyroscopicSensor;
}

#include <argos2/common/control_interface/ci_sensor.h>
#include <argos2/common/utility/math/vector2.h>

namespace argos {

   class CCI_FootBotGyroscopicSensor : public CCI_Sensor {

   public:

      struct SReading {
         CRadians DeltaRotationZ;
      };

      virtual ~CCI_FootBotGyroscopicSensor() {
      }

      inline virtual const SReading& GetReading() const {
         return m_sReading;
      }

   protected:

      SReading m_sReading;

   };

}

#endif
