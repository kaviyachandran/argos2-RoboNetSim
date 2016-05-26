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
 * @file <argos2/simulator/sensors/eye-bot/eyebot_altitude_sensor.cpp>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#include "eyebot_altitude_sensor.h"
#include <argos2/simulator/space/entities/eyebot_entity.h>
#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/space.h>

namespace argos {

   /****************************************/
   /****************************************/

   CEyeBotAltitudeSensor::CEyeBotAltitudeSensor() :
      m_cSpace(CSimulator::GetInstance().GetSpace()) {
      /* Ignore the sensing robot when checking for occlusions */
      m_tIgnoreEntities.insert(&GetEntity().GetEmbodiedEntity());
   }

   /****************************************/
   /****************************************/

   void CEyeBotAltitudeSensor::Update() {
      /* Set the occlusion check ray between the bottom of the eye-bot
         and the projection on the floor */
      m_cOcclusionCheckRay.SetStart(
         CVector3(GetEntity().GetEmbodiedEntity().GetPosition().GetX(),
                  GetEntity().GetEmbodiedEntity().GetPosition().GetY(),
                  GetEntity().GetEmbodiedEntity().GetPosition().GetZ()));
      m_cOcclusionCheckRay.SetEnd(
         CVector3(GetEntity().GetEmbodiedEntity().GetPosition().GetX(),
                  GetEntity().GetEmbodiedEntity().GetPosition().GetY(),
                  0.0f));
      /* Buffer to store the intersection data */
      CSpace::SEntityIntersectionItem<CEmbodiedEntity> sIntersectionData;
      if(m_cSpace.GetClosestEmbodiedEntityIntersectedByRay(sIntersectionData,
                                                           m_cOcclusionCheckRay,
                                                           m_tIgnoreEntities)) {
         /* There is an object under the eye-bot */
         m_fAltitude = m_cOcclusionCheckRay.GetDistance(sIntersectionData.TOnRay);
      }
      else {
         /* No object under the eye-bot */
         m_fAltitude = GetEntity().GetEmbodiedEntity().GetPosition().GetZ();
      }
   }

   /****************************************/
   /****************************************/

   void CEyeBotAltitudeSensor::Reset() {
      m_fAltitude = 0.0f;
   }
   
   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CEyeBotAltitudeSensor,
                   "eyebot_altitude", "default",
                   "The eye-bot altitude sensor",
                   "Carlo Pinciroli [cpinciro@ulb.ac.be]",
                   "This sensor returns the altitude of the eye-bot, that is its distance from the\n"
                   "floor. If an object is between the floor and the eye-bot, the altitude is\n"
                   "calculated as the distance from the eye-bot to the top of the object.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <eyebot_altitude implementation=\"default\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "None.\n",
                   "Ready for use, although the sensor is currently noiseless."
      );

}
