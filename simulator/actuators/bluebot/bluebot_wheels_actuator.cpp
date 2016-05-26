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

#include "bluebot_wheels_actuator.h"
#include <argos2/common/utility/logging/argos_log.h>
#include <argos2/common/utility/argos_random.h>

namespace argos {

   /****************************************/
   /****************************************/

   CBluebotWheelsActuator::CBluebotWheelsActuator() :
      m_pcWheeledEntity(NULL),
      m_fNoiseStdDeviation(0.0f) {
      m_fCurrentVelocity[BLUEBOT_LEFT_WHEEL] = 0.0f;
      m_fCurrentVelocity[BLUEBOT_RIGHT_WHEEL] = 0.0f;
   }

   /****************************************/
   /****************************************/

   void CBluebotWheelsActuator::Init(TConfigurationNode& t_tree) {
      try {
         CCI_BluebotWheelsActuator::Init(t_tree);
         /* Parse noise std deviation */
         GetNodeAttributeOrDefault<Real>(t_tree, "noise_std_dev", m_fNoiseStdDeviation, 0.0f);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in bluebot wheels actuator.", ex);
      }
      /* Thread safe random number generator*/
      m_pcRNG = CARGoSRandom::CreateRNG("argos");
   }

   /****************************************/
   /****************************************/

   void CBluebotWheelsActuator::SetLinearVelocity(Real f_left_velocity,
                                                Real f_right_velocity) {
      /* The speed is passed in cm/sec, so we have to transform it into m/sec */
      m_fCurrentVelocity[BLUEBOT_LEFT_WHEEL] = f_left_velocity * 0.01f;
      m_fCurrentVelocity[BLUEBOT_RIGHT_WHEEL] = f_right_velocity * 0.01f;

      if(m_fNoiseStdDeviation != 0.0f) {
         AddGaussianNoise();
      }
   }

   /****************************************/
   /****************************************/

   void CBluebotWheelsActuator::Update() {
      m_pcWheeledEntity->SetSpeed(m_fCurrentVelocity);
   }

   /****************************************/
   /****************************************/

   void CBluebotWheelsActuator::Reset() {
      m_fCurrentVelocity[BLUEBOT_LEFT_WHEEL]  = 0.0f;
      m_fCurrentVelocity[BLUEBOT_RIGHT_WHEEL] = 0.0f;
   }
   /****************************************/
   /****************************************/

   void CBluebotWheelsActuator::AddGaussianNoise() {
      m_fCurrentVelocity[BLUEBOT_LEFT_WHEEL]  += m_fCurrentVelocity[BLUEBOT_LEFT_WHEEL] * m_pcRNG->Gaussian(m_fNoiseStdDeviation);
      m_fCurrentVelocity[BLUEBOT_RIGHT_WHEEL] += m_fCurrentVelocity[BLUEBOT_RIGHT_WHEEL] * m_pcRNG->Gaussian(m_fNoiseStdDeviation);

   }
   /****************************************/
   /****************************************/

   REGISTER_ACTUATOR(CBluebotWheelsActuator,
                     "bluebot_wheels", "default",
                     "The bluebot wheels actuator",
                     "Ali Emre Turgut [ali.turgut@gmail.com]",
                     "This actuator controls the bluebot wheels. For a complete description of its\n"
                     "usage, refer to the common interface.\n\n"
                     "REQUIRED XML CONFIGURATION\n\n"
                     "  <controllers>\n"
                     "    ...\n"
                     "    <my_controller ...>\n"
                     "      ...\n"
                     "      <actuators>\n"
                     "        ...\n"
                     "        <bluebot_wheels implementation=\"default\" />\n"
                     "        ...\n"
                     "      </actuators>\n"
                     "      ...\n"
                     "    </my_controller>\n"
                     "    ...\n"
                     "  </controllers>\n\n"
                     "OPTIONAL XML CONFIGURATION\n\n"
                     "It is possible to specify noisy speed in order to match the characteristics\n"
                     "of the real ropuck. This can be done with the xml parameter: 'noise_std_dev',\n"
                     "which indicates the standard deviation of a gaussian noise applied to the\n"
                     "desired velocity of the wheels.",
                     "Under development"
      );

}
