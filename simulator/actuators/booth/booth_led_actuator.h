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
 * @file <argos2/simulator/actuators/booth/booth_led_actuator.h>
 *
 * @brief This file provides the definition of the booth LED actuator.
 *
 * @author Manuele Brambilla - <mbrambilla@iridia.ulb.ac.be>
 */

#ifndef BOOTH_LED_ACTUATOR_H
#define BOOTH_LED_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CBoothLedActuator;
}

#include <argos2/common/control_interface/booth/ci_booth_led_actuator.h>
#include <argos2/simulator/actuators/booth/booth_actuator.h>
#include <argos2/simulator/space/entities/booth_entity.h>
#include <argos2/simulator/space/entities/led_equipped_entity.h>

namespace argos {

   class CBoothLedActuator : public CBoothActuator,
                              public CCI_BoothLedActuator {

   public:

      CBoothLedActuator() :
         m_pcLedEquippedEntity(NULL) {}
      virtual ~CBoothLedActuator() {}

      virtual void Init(TConfigurationNode& t_tree) {}
      virtual void Destroy() {}

      inline virtual void SetEntity(CEntity& c_entity) {
         CBoothActuator::SetEntity(c_entity);
         m_pcLedEquippedEntity = &(GetEntity().GetLEDEquippedEntity());
      }

      virtual void SetColor(const CColor& c_color);

      virtual void Update();
      virtual void Reset();

   private:

      CLedEquippedEntity* m_pcLedEquippedEntity;

   };

}

#endif
