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
 * @file <argos2/simulator/sensors/e-puck/epuck_encoder_sensor.cpp>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#include "epuck_encoder_sensor.h"
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/space/entities/epuck_entity.h>
#include <argos2/simulator/space/entities/embodied_entity.h>
#include <argos2/simulator/space/entities/wheeled_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real EPUCK_INTERWHEEL_DISTANCE      = 0.053f;
   static const Real EPUCK_HALF_INTERWHEEL_DISTANCE = EPUCK_INTERWHEEL_DISTANCE * 0.5f;

   /****************************************/
   /****************************************/

   CEPuckEncoderSensor::CEPuckEncoderSensor() {
   }

   /****************************************/
   /****************************************/

   void CEPuckEncoderSensor::SetEntity(CEntity& c_entity) {
      CEPuckSensor::SetEntity(c_entity);
      m_pcWheeledEntity  = &(GetEntity().GetWheeledEntity());
      m_pcEmbodiedEntity = &(GetEntity().GetEmbodiedEntity());
      m_cOldPosition     = m_pcEmbodiedEntity->GetPosition();
      m_cOldOrientation  = m_pcEmbodiedEntity->GetOrientation();
      CalculateWheelPosition(m_cOldWheelPosition, m_cOldPosition, m_cOldOrientation);
   }

   /****************************************/
   /****************************************/

   void CEPuckEncoderSensor::Update() {
      /* Get new position and orientation */
      m_cNewPosition    = m_pcEmbodiedEntity->GetPosition();
      m_cNewOrientation = m_pcEmbodiedEntity->GetOrientation();
      /* Calculate the new position of the wheels */
      CalculateWheelPosition(m_cNewWheelPosition,
                             m_cNewPosition,
                             m_cNewOrientation);
      /* Calculate readings in cm */
      m_sReading.CoveredDistanceLeftWheel  = (m_cNewWheelPosition[LEFT_WHEEL]  - m_cOldWheelPosition[LEFT_WHEEL]).Length() * 100.0f;
      m_sReading.CoveredDistanceRightWheel = (m_cNewWheelPosition[RIGHT_WHEEL] - m_cOldWheelPosition[RIGHT_WHEEL]).Length() * 100.0f;
      /* Save current info for the next step */
      m_cOldPosition = m_cNewPosition;
      m_cOldOrientation = m_cNewOrientation;
      m_cOldWheelPosition[LEFT_WHEEL]  = m_cNewWheelPosition[LEFT_WHEEL];
      m_cOldWheelPosition[RIGHT_WHEEL] = m_cNewWheelPosition[RIGHT_WHEEL];
   }

   /****************************************/
   /****************************************/

   void CEPuckEncoderSensor::Reset() {
      m_sReading.CoveredDistanceLeftWheel  = 0.0f;
      m_sReading.CoveredDistanceRightWheel = 0.0f;
   }

   /****************************************/
   /****************************************/

   void CEPuckEncoderSensor::CalculateWheelPosition(CVector2* pc_wheel_pos,
                                                    const CVector3& c_robot_pos,
                                                    const CQuaternion& c_robot_orient) {
      CRadians cXAngle, cYAngle, cZAngle;
      c_robot_orient.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      pc_wheel_pos[LEFT_WHEEL ].SetX(EPUCK_HALF_INTERWHEEL_DISTANCE * Cos(cZAngle + CRadians::PI_OVER_TWO));
      pc_wheel_pos[LEFT_WHEEL ].SetY(EPUCK_HALF_INTERWHEEL_DISTANCE * Sin(cZAngle + CRadians::PI_OVER_TWO));
      pc_wheel_pos[RIGHT_WHEEL].SetX(EPUCK_HALF_INTERWHEEL_DISTANCE * Cos(cZAngle - CRadians::PI_OVER_TWO));
      pc_wheel_pos[RIGHT_WHEEL].SetY(EPUCK_HALF_INTERWHEEL_DISTANCE * Sin(cZAngle - CRadians::PI_OVER_TWO));
      pc_wheel_pos[LEFT_WHEEL ] += CVector2(c_robot_pos.GetX(), c_robot_pos.GetY());
      pc_wheel_pos[RIGHT_WHEEL] += CVector2(c_robot_pos.GetX(), c_robot_pos.GetY());
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CEPuckEncoderSensor,
                   "epuck_encoder", "default",
                   "The e-puck wheel encoder sensor",
                   "Carlo Pinciroli [cpinciro@ulb.ac.be]",
                   "This sensor accesses the e-puck wheel encoder sensor. For a complete\n"
                   "description of its usage, refer to the common interface.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <epuck_encoder implementation=\"default\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "None for the time being.\n",
                   "Under development"
      );
   
}
