/* -*- Mode: C++ -*-
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
 * @file <argos2/simulator/generic_sensors/bluebot/bluebot_sensor.h>
 *
 * @brief This file provides the definition of
 *
 * Ali Emre Turgut - <ali.turgut@gmail.com>
*/

#ifndef BLUEBOT_SENSOR_H
#define BLUEBOT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CBluebotSensor;
}

#include <argos2/simulator/sensors/simulated_sensor.h>
#include <argos2/simulator/space/entities/bluebot_entity.h>

namespace argos {

   class CBluebotSensor : public CSimulatedSensor {

   public:

      CBluebotSensor() :
         m_pcBluebotEntity(NULL) {}
      virtual ~CBluebotSensor() {}

      inline virtual CBluebotEntity& GetEntity() {
         return *m_pcBluebotEntity;
      }
      inline virtual void SetEntity(CEntity& c_entity) {
         m_pcBluebotEntity = dynamic_cast<CBluebotEntity*>(&c_entity);
         if(m_pcBluebotEntity == NULL) {
            THROW_ARGOSEXCEPTION("Cannot associate a bluebot sensor to a robot of type \"" << c_entity.GetTypeDescription() << "\"");
         }
      }

   private:

      CBluebotEntity* m_pcBluebotEntity;

   };

}

#endif
