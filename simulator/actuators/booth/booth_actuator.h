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
 * @file <argos2/simulator/generic_actuators/booth/epuck_actuator.h>
 *
 * @brief This file provides the definition of
 *
 * @author Manuele Brambilla - <mbrambilla@iridia.ulb.ac.be>
 */

#ifndef BOOTH_ACTUATOR_H
#define BOOTH_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CBoothActuator;
}

#include <argos2/simulator/actuators/simulated_actuator.h>
#include <argos2/simulator/space/entities/booth_entity.h>

namespace argos {

   class CBoothActuator : public CSimulatedActuator {

   public:

      CBoothActuator() :
         m_pcBoothEntity(NULL) {}
      virtual ~CBoothActuator() {}

      inline virtual CBoothEntity& GetEntity() {
         return *m_pcBoothEntity;
      }
      inline virtual void SetEntity(CEntity& c_entity) {
         m_pcBoothEntity = dynamic_cast<CBoothEntity*>(&c_entity);
         if(m_pcBoothEntity == NULL) {
            THROW_ARGOSEXCEPTION("Cannot associate a booth actuator to a robot of type \"" << c_entity.GetTypeDescription() << "\"");
         }
      }

   private:

      CBoothEntity* m_pcBoothEntity;

   };

}

#endif
