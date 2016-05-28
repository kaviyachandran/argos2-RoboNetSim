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
 * @file <argos2/simulator/generic_actuators/booth/booth_leds_actuator.cpp>
 *
 * @brief This file provides the implementation of the booth LEDs actuator.
 *
 * @author Manuele Brambilla - <mbrambilla@iridia.ulb.ac.be>
 */

#include "booth_led_actuator.h"
#include <argos2/common/utility/logging/argos_log.h>

namespace argos {

   /****************************************/
   /****************************************/

   void CBoothLedActuator::SetColor(const CColor& c_color) {
      m_cLedColor = c_color;
      Update();
   }

   /****************************************/
   /****************************************/

   void CBoothLedActuator::Update() {
      m_pcLedEquippedEntity->SetLedColor(0, m_cLedColor);
   }

   /****************************************/
   /****************************************/

   void CBoothLedActuator::Reset() {
      m_pcLedEquippedEntity->SetLedColor(0,CColor::BLACK);
   }

   /****************************************/
   /****************************************/

   REGISTER_ACTUATOR(CBoothLedActuator,
                     "booth_led", "default",
                     "The booth LED actuator",
                     "Manuele Brambilla [mbrambilla@iridia.ulb.ac.be]",
                     "This actuator controls the booth LED. For a complete description of its\n"
                     "usage, refer to the common interface.\n\n"
                     "REQUIRED XML CONFIGURATION\n\n"
                     "  <controllers>\n"
                     "    ...\n"
                     "    <my_controller ...>\n"
                     "      ...\n"
                     "      <actuators>\n"
                     "        ...\n"
                     "        <booth_led implementation=\"default\" />\n"
                     "        ...\n"
                     "      </actuators>\n"
                     "      ...\n"
                     "    </my_controller>\n"
                     "    ...\n"
                     "  </controllers>\n\n"
                     "OPTIONAL XML CONFIGURATION\n\n"
                     "None for the time being.\n",
                     "Under development"
      );

}
