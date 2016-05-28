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
 * @file <argos2/simulator/sensors/booth/booth_proximity_sensor.h>
 *
 * @author Manuele Brambilla - <mbrambilla@iridia.ulb.ac.be>
 */

#ifndef BOOTH_PROXIMITY_SENSOR_H
#define BOOTH_PROXIMITY_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CBoothPresenceSensor;
}

#include <argos2/common/control_interface/booth/ci_booth_presence_sensor.h>
#include <argos2/simulator/sensors/booth/booth_sensor.h>
#include <argos2/simulator/space/entities/booth_entity.h>
#include <argos2/simulator/space/space.h>

namespace argos {
   
   class CBoothPresenceSensor : public CBoothSensor,
                                 public CCI_BoothPresenceSensor {

   public:

      CBoothPresenceSensor();
      virtual ~CBoothPresenceSensor() {}

      virtual void Init(TConfigurationNode& t_tree);

      inline virtual void SetEntity(CEntity& c_entity) {
         CBoothSensor::SetEntity(c_entity);
         m_pcEmbodiedEntity = &(GetEntity().GetEmbodiedEntity());
         m_cBoothPosition = m_pcEmbodiedEntity->GetPosition();
      }

      virtual void Update();
      virtual void Reset();

   protected:

      CSpace& m_cSpace;
      CEmbodiedEntity* m_pcEmbodiedEntity;
      CVector3 m_cBoothPosition;

   };

}

#endif
