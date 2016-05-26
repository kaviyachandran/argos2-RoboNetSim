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
 * @file common/control_interface/booth/ci_booth_presence_sensor.h
 *
 * @brief This file provides the definition of the booth presence sensor.
 *
 * This file provides the definition of the booth presence sensor.
 * The sensors returns a boolean that is TRUE if there is a robot inside the booth
 * and FALSE otherwise.
 *
 * @author Manuele Brambilla - <mbrambilla@iridia.ulb.ac.be>
 */

#ifndef CCI_BOOTH_PRESENCE_SENSOR_H
#define CCI_BOOTH_PRESENCE_SENSOR_H

/* To avoid dependency problems when including */
namespace argos {
   class CCI_BoothPresenceSensor;
}

#include <argos2/common/control_interface/ci_sensor.h>
#include <argos2/common/utility/math/angles.h>

namespace argos {

   class CCI_BoothPresenceSensor : virtual public CCI_Sensor {

   public:

	  CCI_BoothPresenceSensor() {};
      virtual ~CCI_BoothPresenceSensor() {}

      inline virtual const bool SensePresence() const {
         return m_bPresence;
      }

   protected:
      
      bool m_bPresence;

   };

}

#endif
