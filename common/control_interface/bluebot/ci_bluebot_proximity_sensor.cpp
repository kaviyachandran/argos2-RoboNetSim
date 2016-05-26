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
 * @file common/control_interface/bluebot/ci_bluebot_proximity_sensor.h
 *
 * @brief This file provides the definition of the bluebot proximity sensor.
 *
 * This file provides the definition of the bluebot proximity sensor.
 * The sensors are spaced on a ring around the base of the bluebot.
 * Therefore, they do not turn with the turret. The readings are in the
 * following order (seeing the bluebot from TOP, battery socket is the BACK):
 *
 *              front
 *
 *              0   7
 *     l    1           6   r
 *     e                    i
 *     f    2           5   g
 *     t                    h
 *             3     4      t
 *
 *              back
 *
 * @author Ali Emre Turgut - <ali.turgut@gmail.com>
 */

#include "ci_bluebot_proximity_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   CCI_BluebotProximitySensor::CCI_BluebotProximitySensor() :
      m_tReadings(8) {
      CRadians cSlice = CRadians::TWO_PI / m_tReadings.size();
      for(size_t i = 0; i < m_tReadings.size(); ++i) {
         m_tReadings[i].Angle = i * cSlice + cSlice * 0.5f;
      }
   }

   /****************************************/
   /****************************************/

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_BluebotProximitySensor::SReading& s_reading) {
      c_os << "SReading("
           << s_reading.Value
           << ","
           << s_reading.Angle
           << ")";
      return c_os;
   }

   /****************************************/
   /****************************************/

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_BluebotProximitySensor::TReadings& t_readings) {
      if(! t_readings.empty()) {
         c_os << t_readings[0];
         for(size_t i = 1; i < t_readings.size(); ++i) {
            c_os << " " << t_readings[i];
         }
      }
      return c_os;
   }

   /****************************************/
   /****************************************/

}
