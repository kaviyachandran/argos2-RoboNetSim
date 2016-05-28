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
 * @file <argos2/simulator/sensors/bluebot/bluebot_proximity_sensor.h>
 *
 * Ali Emre Turgut - <ali.turgut@gmail.com>
 */

#ifndef BLUEBOT_PROXIMITY_SENSOR_H
#define BLUEBOT_PROXIMITY_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CBluebotProximitySensor;
}

#include <argos2/common/control_interface/bluebot/ci_bluebot_proximity_sensor.h>
#include <argos2/common/utility/math/range.h>
#include <argos2/simulator/sensors/bluebot/bluebot_sensor.h>
#include <argos2/simulator/space/entities/bluebot_entity.h>
#include <argos2/simulator/space/space.h>
#include <argos2/common/utility/argos_random.h>

namespace argos {
   
   class CBluebotProximitySensor : public CBluebotSensor,
                                 public CCI_BluebotProximitySensor {

   public:

      CBluebotProximitySensor();
      virtual ~CBluebotProximitySensor() {}

      virtual void Init(TConfigurationNode& t_tree);

      inline virtual void SetEntity(CEntity& c_entity) {
         CBluebotSensor::SetEntity(c_entity);
         m_pcEmbodiedEntity = &(GetEntity().GetEmbodiedEntity());
      }

      virtual void Update();
      virtual void Reset();

   protected:

      CSpace& m_cSpace;
      CSpaceHash<CEmbodiedEntity, CEmbodiedEntitySpaceHashUpdater>& m_cEmbodiedSpaceHash;
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /* Flag for showing rays in the simulator */
      bool m_bShowRays;

      /* Random number generator */
      CARGoSRandom::CRNG* m_pcRNG;

      /* Uniform noise parameter */
      Real m_fNoiseLevel;

      /* Radings default range */
      static const CRange<Real> BLUEBOT_PROXIMITY_SENSORS_READINGS_RANGE;

      /* noise level in in the proximity range */
      Real m_fRangeNoiseLevel;
    
      /* Adds noise to the sensor reading */
      virtual void AddNoise(UInt16 un_sensor_index);

   };

}

#endif
