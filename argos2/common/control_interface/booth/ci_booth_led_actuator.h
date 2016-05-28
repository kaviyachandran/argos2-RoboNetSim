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
 * @file common/control_interface/booth/ci_epuck_leds_actuator.h
 *
 * @brief This file provides the definition of the booth LED actuator.
 *
 * This file provides the definition of the booth LED actuator.
 *
 * The user can switch on/off the LED panel of the booth.
 *
 *  @author Manuele Brambilla - <mbrambilla@iridia.ulb.ac.be>
 */

#ifndef CCI_BOOTH_LED_ACTUATOR_H
#define CCI_BOOTH_LED_ACTUATOR_H

/* To avoid dependency problems when including */
namespace argos {
   class CCI_BoothLedActuator;
}

#include <argos2/common/control_interface/ci_actuator.h>
#include <argos2/common/utility/datatypes/color.h>

namespace argos {

   class CCI_BoothLedActuator : virtual public CCI_Actuator {

   public:

      CCI_BoothLedActuator(){}

      virtual ~CCI_BoothLedActuator() {}

      /**
       * Set the color of the LED panel.
       */
      virtual void SetColor(const CColor& c_color) = 0;

   protected:

      CColor m_cLedColor;

   };

}

#endif
