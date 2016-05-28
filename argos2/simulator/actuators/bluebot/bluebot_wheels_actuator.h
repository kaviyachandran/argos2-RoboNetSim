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
 * @file <argos2/simulator/actuators/bluebot/bluebot_wheels_actuator.h>
 *
 * @brief This file provides the definition of the bluebot wheels actuator
 *
 * @author Ali Emre Turgut - <ali.turgut@gmail.com>
 */

#ifndef BLUEBOT_WHEELS_ACTUATOR_H
#define BLUEBOT_WHEELS_ACTUATOR_H

#include <string>
#include <map>
#include <argos2/common/utility/argos_random.h>

namespace argos {
   class CBluebotWheelsActuator;
}

#include <argos2/common/control_interface/bluebot/ci_bluebot_wheels_actuator.h>
#include <argos2/simulator/actuators/bluebot/bluebot_actuator.h>
#include <argos2/simulator/space/entities/bluebot_entity.h>
#include <argos2/simulator/space/entities/wheeled_entity.h>

namespace argos {

   class CBluebotWheelsActuator : public CBluebotActuator,
                                  public CCI_BluebotWheelsActuator {

   public:

      enum BLUEBOT_WHEELS {
         BLUEBOT_LEFT_WHEEL = 0,
         BLUEBOT_RIGHT_WHEEL = 1
      };

      CBluebotWheelsActuator();
      virtual ~CBluebotWheelsActuator() {}

      inline virtual void SetEntity(CEntity& c_entity) {
         CBluebotActuator::SetEntity(c_entity);
         m_pcWheeledEntity = &(GetEntity().GetWheeledEntity());
      }

      virtual void Init(TConfigurationNode& t_tree);

      virtual void SetLinearVelocity(Real f_left_velocity,
                                     Real f_right_velocity);

      virtual void Update();
      virtual void Reset();

   private:

      /** Random number generator */
      CARGoSRandom::CRNG* m_pcRNG;
      CWheeledEntity<2>* m_pcWheeledEntity;

      /** Noise parameters, at the moment noise is Gaussian */
      Real m_fNoiseStdDeviation;
      
      /** Adds noise to the wheels velocity */
      virtual void AddGaussianNoise();
   };

}

#endif
