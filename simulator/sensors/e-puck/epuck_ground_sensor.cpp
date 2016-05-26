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
 * @file <argos2/simulator/sensors/foot-bot/footbot_base_ground_sensor.cpp>
 *
 * @author Laurent Compere - <lcompere@ulb.ac.be>
 */

#include "epuck_ground_sensor.h"
#include <argos2/simulator/space/entities/embodied_entity.h>
#include <argos2/simulator/space/entities/epuck_entity.h>
#include <argos2/simulator/space/entities/floor_entity.h>
#include <argos2/simulator/space/space.h>

namespace argos {

    /****************************************/
    /****************************************/

#define CALCULATE_READING(IDX, POSITION)                                      \
   cSensorPos = m_cSensorPosOffset[IDX];                                      \
   cSensorPos.Rotate(cRotZ);                                                  \
   cSensorPos += cCenterPos;                                                  \
   m_sReadings.POSITION =                                                     \
      cFloorEntity.GetColorAtPoint(cSensorPos.GetX(),                         \
                                   cSensorPos.GetY()).ToGrayScale() / 255.0f;

    /****************************************/

    /****************************************/

    void CEPuckGroundSensor::Init(TConfigurationNode& t_tree) {
        try {
            /* Parse noise level */
            GetNodeAttributeOrDefault(t_tree, "noise_level", m_fNoiseLevel, m_fNoiseLevel);
        } catch (CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Initialization error in e-puck ground sensor", ex);
        }

        /* Random number generator*/
        m_pcRNG = CARGoSRandom::CreateRNG("argos");
    }
    /****************************************/

    /****************************************/

    void CEPuckGroundSensor::Update() {
        /* We make the assumption that the e-puck is rotated only wrt to Z */
        CFloorEntity& cFloorEntity = m_cSpace.GetFloorEntity();
        const CVector3& cEntityPos = GetEntity().GetEmbodiedEntity().GetPosition();
        const CQuaternion& cEntityRot = GetEntity().GetEmbodiedEntity().GetOrientation();
        CRadians cRotZ, cRotY, cRotX;
        cEntityRot.ToEulerAngles(cRotZ, cRotY, cRotX);
        CVector2 cCenterPos(cEntityPos.GetX(), cEntityPos.GetY());

        CVector2 cSensorPos;
        CALCULATE_READING(0, Right);
        CALCULATE_READING(1, Center);
        CALCULATE_READING(2, Left);
        if (m_fNoiseLevel > 0.0) {
            AddNoise(0);
            AddNoise(1);
            AddNoise(2);

        }
        

    }
    /****************************************/

        /****************************************/

    void CEPuckGroundSensor::AddNoise(UInt32 un_sensor_index) {
        if (un_sensor_index == 0) {
            m_sReadings.Right += m_pcRNG->Uniform(CRange<Real > (-m_fNoiseLevel, m_fNoiseLevel));
            if (m_sReadings.Right < 0) {
                m_sReadings.Right = 0;
            } else if (m_sReadings.Right > 1) {
                m_sReadings.Right = 1;
            }
        } else if (un_sensor_index == 1) {
            m_sReadings.Center += m_pcRNG->Uniform(CRange<Real > (-m_fNoiseLevel, m_fNoiseLevel));
            if (m_sReadings.Center < 0) {
                m_sReadings.Center = 0;
            } else if (m_sReadings.Center > 1) {
                m_sReadings.Center = 1;
            }
        } else if (un_sensor_index == 2) {
            m_sReadings.Left += m_pcRNG->Uniform(CRange<Real > (-m_fNoiseLevel, m_fNoiseLevel));
            if (m_sReadings.Left < 0) {
                m_sReadings.Left = 0;
            } else if (m_sReadings.Left > 1) {
                m_sReadings.Left = 1;
            }
        }

    }
    /****************************************/

    /****************************************/

    void CEPuckGroundSensor::Reset() {
        m_sReadings.Reset();
    }

    /****************************************/
    /****************************************/

    REGISTER_SENSOR(CEPuckGroundSensor,
            "epuck_ground", "rot_z_only",
            "The e-puck base ground sensor (optimized for 2D)",
            "Carlo Pinciroli [cpinciro@ulb.ac.be]",
            "This sensor accesses the e-puck ground sensor. For a complete description of\n"
            "its usage, refer to the common interface.\n"
            "In this implementation, the readings are calculated under the assumption that\n"
            "the e-puck is always parallel to the XY plane, i.e., it rotates only around\n"
            "the Z axis. This implementation is faster than a 3D one and should be used\n"
            "only when the assumption about the e-puck rotation holds.\n\n"
            "REQUIRED XML CONFIGURATION\n\n"
            "  <controllers>\n"
            "    ...\n"
            "    <my_controller ...>\n"
            "      ...\n"
            "      <sensors>\n"
            "        ...\n"
            "        <epuck_ground implementation=\"rot_z_only\" />\n"
            "        ...\n"
            "      </sensors>\n"
            "      ...\n"
            "    </my_controller>\n"
            "    ...\n"
            "  </controllers>\n\n"
            "OPTIONAL XML CONFIGURATION\n\n"
            "It is possible to specify noisy sensors in order to match the characteristics\n"
                   "of the real robot. This can be done with the xml parameter 'noise_level'\n"
                   "It indicates a percentage of the sensor range that is added to the actual sensor\n"
                   "reading. The added noise value is uniformly random on this range." 
            "",
            "Under development"
            );

}
